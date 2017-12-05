#include <tuna.h>

#include "simple.hpp"

#include "bi3_plus_lcd.h"

#include <math.h>

#include "configuration_store.h"

// TODO FIXME remove floats after initial testing. They're here to establish sane ranges to help with
// the fixed-point math for exponents.

using namespace Tuna::Thermal::Manager;

// TODO Establish a global logging system like this.
namespace Tuna::Log
{
  template <uint8 tabs = 0, typename ...Args>
  inline void d(arg_type<flash_string> tag, arg_type<flash_string> format, Args... args)
  {
    critical_section log_critsec;
    Serial.print(tag.fsh());
    Serial.print(": "_p.fsh());
    for (uint8 i = 0; i < tabs; ++i)
    {
      Serial.print("  "_p.fsh());
    }
    char buffer[128];
    sprintf_P(buffer, format.c_str(), args...);
    Serial.println(buffer);
  }
}

namespace
{
  constexpr const auto Tag = "SimpleManager"_p;

  using preferred_table_type = uint8;
  constexpr const uint8 integer_swing = 16;
  constexpr const uint8 integer_swing_bits = constant::log2<integer_swing>;
  constexpr const uint8 relevant_fraction_bits = min(temp_t::fractional_bits, (sizeof(preferred_table_type) * 8_u8) - integer_swing_bits);
  constexpr const uint8 relevant_total_bits = (relevant_fraction_bits + integer_swing_bits);
  constexpr const auto numTableEntries = make_uintsz<1_u64 << relevant_total_bits>;
  using tableidx_t = uintsz<1_u64 << relevant_total_bits>;

  // Validations
  static_assert(constant::is_pow2<integer_swing>, "integer_swing must be power-of-two.");

  // SRAM
  uint8 pwm_table[numTableEntries] = {};
  Simple::calibration pwm_calibration;

  static pair<bool, tableidx_t> get_diff_idx(arg_type<temp_t> current, arg_type<temp_t> target)
  {
    temp_t difference = target - current;
    auto difference_raw = difference.raw();
    difference_raw >>= (temp_t::fractional_bits - relevant_fraction_bits); // shift away fraction bits we don't consider.
    // If the value is larger than the total number of entries, it is out of range.
    if (__unlikely(difference_raw >= numTableEntries))
    {
      return { false };
    }
    // Otherwise it's valid.
    return { true, tableidx_t(difference_raw) };
  }
}

const Simple::calibration & Simple::GetCalibration()
{
  return pwm_calibration;
}

void Simple::SetCalibration(arg_type<calibration> value)
{
  Log::d(Tag, "Current Calibration: %.6f %u"_p, value.Exponent_, uint16(value.Scalar_));

  pwm_calibration = value;

  for (tableidx_t i = 0; i < numTableEntries; ++i)
  {
    pwm_table[i] = uint8(roundf(powf(float(i) / float(numTableEntries), value.Exponent_) * 255.5f));
  }
}

bool Simple::calibrate(arg_type<temp_t> target)
{
  Log::d(Tag, "Starting Calibration"_p);

  constexpr const auto test_max_time = 60000_ms16; // Otherwise, test goes this long.

  constexpr const float exponent_epsilon = 0.00625f;
  constexpr const float exponent_base = 0.60000f;

  constexpr const float test_exponents[] =
  {
    exponent_base + (exponent_epsilon * -4),
    exponent_base + (exponent_epsilon * -3),
    exponent_base + (exponent_epsilon * -2),
    exponent_base + (exponent_epsilon * -1),
    exponent_base + (exponent_epsilon * 0),
    exponent_base + (exponent_epsilon * +1),
    exponent_base + (exponent_epsilon * +2),
    exponent_base + (exponent_epsilon * +3),
    exponent_base + (exponent_epsilon * +4),
  };

  constexpr const uint8 test_scalars[] =
  {
    1_u8,
    2_u8,
    3_u8,
    //4_u8,
    //5_u8
  };

  using tempavg_t = type_trait<temp_t::type>::larger_type;
  constexpr const uint16 TempAvgSampleCount = 256;

  struct oscillation final
  {
    bool valid = false;

    temp_t high = { 0 }; // how far it oscillates above target
    temp_t low = { 0 }; // how far it oscillates below target

    tempavg_t tempMean;
  };

  const auto average_append = [](auto & __restrict sum, const auto & __restrict new_val, const auto & __restrict N)
  {
    using sum_t = decltype(sum);
    tempavg_t mean = sum / N;
    sum -= mean;
    sum += new_val;
  };

  const auto extract_average = [](const auto & __restrict sum, const auto & __restrict N) -> auto
  {
    return sum / N;
  };

  const auto init_average = [](auto & __restrict sum, const auto & __restrict cur, const auto & __restrict N)
  {
    sum = cur;
    sum *= N;
  };

  tempavg_t tempAvg;
  init_average(tempAvg, Temperature::degHotend().raw(), TempAvgSampleCount);

  const auto set_temp_target = [&](arg_type<temp_t> temp, bool await)
  {
    Temperature::setTargetHotend(temp);

    if (!await)
    {
      return;
    }

    for (;;)
    {
      if (__unlikely(Temperature::manage_heater()))
      {
        const temp_t current_temperature = Temperature::degHotend();

        average_append(tempAvg, current_temperature.raw(), TempAvgSampleCount);

        const Temperature::Trend temp_trend = Temperature::get_temperature_trend();
        //Log::d(TagGraph, "%.6f, %.6f, %S%.6f"_p, float(current_temperature), float(temp_t::from(extract_average(tempAvg, TempAvgSampleCount))), temp_trend.second ? "+"_p : "-"_p, float(temp_trend.first));

        lcd::update_graph();

        // We only care when we are above the temp target during an upswing. This guarantees we aren't polluting the test due to going 'down' first.
        if (__unlikely(temp_trend == Temperature::Trend::Up && current_temperature >= Temperature::degTargetHotend()))
        {
          return;
        }
      }
    }
  };

  const auto populate_pwm_table_const = [](uint8 value)
  {
    pwm_calibration.Exponent_ = -1.0f;

    for (tableidx_t i = 0; i < numTableEntries; ++i)
    {
      pwm_table[i] = value;
    }
  };

  Temperature::disable_all_heaters();

  // First, we set the target temperature to a value minus double the expected swing.
  constexpr const temp_t double_swing = temp_t{ integer_swing * 2 };
  const temp_t low_target = target - double_swing;

  Log::d(Tag, "Calibration Parameters:"_p);
  Log::d<1>(Tag, "test_max_time: %u"_p, test_max_time.raw());
  Log::d<1>(Tag, "TempAvgSampleCount: %u"_p, TempAvgSampleCount);
  Log::d<1>(Tag, "integer_swing: %u"_p, integer_swing);
  Log::d<1>(Tag, "low_target: %.6f"_p, float(low_target));
  Log::d<1>(Tag, "table entries: %u"_p, numTableEntries);
  Log::d<1>(Tag, "pwm integer bits: %u"_p, integer_swing_bits);
  Log::d<1>(Tag, "pwm fracion bits: %u"_p, relevant_fraction_bits);
  Log::d<1>(Tag, "temp_t integer bits: %u"_p, temp_t::integer_bits);
  Log::d<1>(Tag, "temp_t fraction bits: %u"_p, temp_t::fractional_bits);

  const auto execute_test = [&, target](arg_type<float> exponent, oscillation & __restrict error, uint8 scalar)
  {
    Log::d<1>(Tag, "Exponent: %.6f"_p, exponent);

    // Reset the PWM table to 0xFF for low-target setting.
    populate_pwm_table_const(0xFF);
    // Reset the target temperature to the lower target.
    Log::d(Tag, "Reaching Low Target"_p);
    set_temp_target(low_target, true);
    Log::d(Tag, "Low Target Reached, beginning test"_p);

    // Populate the PWM table.
    SetCalibration({ exponent, scalar });

    // Set target temperature.
    set_temp_target(target, false);

    const auto start_time = chrono::time_ms<uint24>::get();

    // Once we pass the target temperature (defines as going from below to above the target temperature)
    // we begin our analysis.
    for (;;)
    {
      // manage_heater handles managing PWM values, reading temperature values
      // and implements safeties.
      // It returns true when new temperature data is available.
      if (__unlikely(Temperature::manage_heater()))
      {
        // TODO FIXME use a running average (even though temperature is already an average)
        // otherwise this will be quite noisy.

        const temp_t current_temperature = Temperature::degHotend();
        average_append(tempAvg, current_temperature.raw(), TempAvgSampleCount);

        //const Temperature::Trend temp_trend = Temperature::get_temperature_trend();
        //Log::d(TagGraph, "%.6f, %.6f, %S%.6f"_p, float(current_temperature), float(temp_t::from(extract_average(tempAvg, TempAvgSampleCount))), temp_trend.second ? "+"_p : "-"_p, float(temp_trend.first));

        lcd::update_graph();

        if (current_temperature >= target)
        {
          error.valid = true;

          if (current_temperature > target)
          {
            const temp_t differential = current_temperature - target;
            error.high = max(error.high, differential);
          }
        }
        else if (current_temperature < target)
        {
          const temp_t differential = target - current_temperature;
          // We only track 'low' if we actually every break our temperature goal.
          if (error.valid)
          {
            error.low = max(error.low, differential);
          }
        }
      }

      // Regardless, we also check the current time, to see if we've timed out.
      if (__unlikely(start_time.elapsed(test_max_time)))
      {
        break;
      }
    }

    error.tempMean = tempAvg;
  };

  constexpr const uint8 num_exponents = array_size(test_exponents);
  constexpr const uint8 num_scalars = array_size(test_scalars);
  constexpr const uint8 num_tests = num_exponents * num_scalars;

  //temp_t best_high = temp_t{ printer_max_temperature };
  //uint8 best_high_idx = 0;
  //temp_t best_low = temp_t{ printer_max_temperature };
  //uint8 best_low_idx = 0;

  const auto errorCalculate = [&](arg_type<oscillation> error)->uint32
  {
    const auto highMean = error.high;
    const auto tempMean = error.tempMean;
    const auto lowMean = error.low;

    const tempavg_t targetTemp = (tempavg_t(target.raw()) * TempAvgSampleCount);
    const auto tempMeanDifferential = [&]()->tempavg_t
    {
      if (targetTemp >= tempMean)
      {
        return targetTemp - tempMean;
      }
      return tempMean - targetTemp;
    }();

    return (uint32(highMean.raw()) * 5) + (uint32(lowMean.raw()) * 2);
  };

  uint32 bestError = type_trait<uint32>::max;
  uint8 bestIndex = 0;

  constexpr const auto total_tests = num_tests;

  for (uint8 i = 0; i < num_tests; ++i)
  {
    const uint8 exponent_idx = i / num_exponents;
    const uint8 scalar_idx = i % num_exponents;

    const auto & __restrict exponent = test_exponents[exponent_idx];
    const uint8 scalar = test_scalars[scalar_idx];
    oscillation error;

    Log::d(Tag, "Executing Test: %u / %u *****************"_p, i, total_tests);
    execute_test(exponent, error, scalar);

    Log::d(Tag, "Test Complete: %u / %u"_p, i, total_tests);
    Log::d<1>(Tag, "valid: %u"_p, error.valid ? 1 : 0);

    if (__likely(error.valid))
    {
      Log::d<1>(Tag, "high: %.6f"_p, float(error.high));
      Log::d<1>(Tag, "low: %.6f"_p, float(error.low));
      Log::d<1>(Tag, "tempMean: %lu"_p, uint32(error.tempMean));

      if constexpr (true)
      {
        const uint32 errorVal = errorCalculate(error);
        Log::d<1>(Tag, "error: %lu"_p, uint32(errorVal));
        if (errorVal < bestError)
        {
          Log::d<1>(Tag, "New Best &&&&&&&&&&&&&&"_p);
          bestError = errorVal;
          bestIndex = i;
        }
      }
      else
      {
        const tempavg_t errorVal = tempavg_t(error.high.raw()) + tempavg_t(error.low.raw());
        Log::d<1>(Tag, "error: %lu"_p, uint32(errorVal));
        Log::d<1>(Tag, "New Best &&&&&&&&&&&&&&"_p);
        if (errorVal < bestError)
        {
          bestError = errorVal;
          bestIndex = i;
        }
      }
    }
  }

  Temperature::disable_all_heaters();

  // For now, choose the best high.
  Log::d(Tag, "Best Index: %u"_p, bestIndex);

  {
    const uint8 exponent_idx = bestIndex / num_exponents;
    const uint8 scalar_idx = bestIndex % num_exponents;

    const auto & __restrict exponent = test_exponents[exponent_idx];
    const uint8 scalar = test_scalars[scalar_idx];

    Log::d<1>(Tag, "Exponent: %.6f  Scalar: %u"_p, float(exponent), scalar);

    SetCalibration({ exponent, scalar });
  }

  lcd::show_page(lcd::Page::PID_Finished);
  enqueue_and_echo_command("M107");
  lcd::update();

  settings.save();

  return true;
}

uint8 Simple::get_power(arg_type<temp_t> current, arg_type<temp_t> target) 
{
  constexpr const bool tempLog = false;

  constexpr const temp_t degrees_above_pwm = 1_C;
  const temp_t high_target = target + degrees_above_pwm;

  static constexpr const uint8 none = 0x00_u8;
  static constexpr const uint8 full = 0xFF_u8;

  uint8 out_temp = none;

  if (__likely(current < high_target))
  {
    const auto diff_idx = get_diff_idx(current, high_target);

    if (__unlikely(!diff_idx))
    {
      out_temp = full;
    }
    // Otherwise, do a lookup in the pwm table.
    else if (__likely(current >= target))
    {
      if (Temperature::get_temperature_trend() == Temperature::Trend::Down)
      {
        out_temp = uint8(min(uint16(pwm_table[diff_idx.second]) * pwm_calibration.Scalar_, full));
      }
      else
      {
        out_temp = none;
      }
    }
    else if (__likely(Temperature::get_temperature_trend() == Temperature::Trend::Up))
    {
      out_temp = pwm_table[diff_idx.second];
    }
    else
    {
      //return pwm_table_down[diff_idx.second];
      out_temp = full;
    }
  }

  if constexpr (tempLog)
  {
    Log::d(Tag, "%.6f, %u"_p, float(current), full);
  }

  return out_temp;
}

void Simple::debug_dump()
{
}