#import <tuna.h>

#import "simple.hpp"

#import "bi3_plus_lcd.h"

#import <math.h>

#import "configuration_store.h"

// TODO FIXME remove floats after initial testing. They're here to establish sane ranges to help with
// the fixed-point math for exponents.

using namespace Tuna::Thermal::Manager;

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
  constexpr const auto TagGraph = "SimpleManager[GRAPH]"_p;

  constexpr const bool shortcut_results = false;
  constexpr const uint8 integer_swing = 16; // must be POW2. TODO add pow2 check.
  constexpr const uint8 integer_swing_bits = ce_log2<integer_swing>;
  constexpr const uint8 relevant_fraction_bits = min(temp_t::fractional_bits, 4);
  constexpr const uint8 relevant_total_bits = (relevant_fraction_bits + integer_swing_bits);
  constexpr const auto numTableEntries = make_uintsz<1 << relevant_total_bits>;
  using tableidx_t = uintsz<1 << relevant_total_bits>;

  bool calibrating = false;
  uint8 pwm_table[numTableEntries] = {};
  float pwm_exponent = -1.0f;

  static pair<bool, tableidx_t> get_diff_idx(arg_type<temp_t> current, arg_type<temp_t> target)
  {
    temp_t difference = target - current;
    auto difference_raw = difference.raw();
    difference_raw >>= (temp_t::fractional_bits - relevant_fraction_bits); // shift away fraction bits we don't consider.
    // If the value is larger than the total number of entries, it is out of range.
    if (difference_raw >= numTableEntries)
    {
      return { false };
    }
    // Otherwise it's valid.
    return { true, tableidx_t(difference_raw) };
  }
}

pair<float, float> Simple::GetCalibration()
{
  return { pwm_exponent, 0.0f };
}

void Simple::SetCalibration(arg_type<pair<float, float>> exponent)
{
  Log::d<>(Tag, "SetCalibration: %.6f"_p, exponent.first);

  pwm_exponent = exponent.first;

  for (tableidx_t i = 0; i < numTableEntries; ++i)
  {
    pwm_table[i] = uint8(roundf(powf(float(i) / float(numTableEntries), exponent.first) * 255.5f));
  }
}

bool Simple::calibrate(arg_type<temp_t> target)
{
  Log::d<>(Tag, "Starting Calibration"_p);

  constexpr const uint8 test_cycles = 25; // how many full cycles to wait for while testing exponents.
  constexpr const auto test_max_time = 120000_ms24; // Otherwise, test goes this long.

  constexpr const float test_exponents[] =
  {
    0.4f,
    0.425f,
    0.45f,
    0.475f,
    0.4875f,
    0.5f,
    0.5125f,
    0.525f,
    0.55f,
  };

  using tempavg_t = type_trait<temp_t::type>::larger_type;
  constexpr const uint16 TempAvgSampleCount = 256;

  struct oscillation final
  {
    bool valid = false;

    temp_t high = { 0 }; // how far it oscillates above target
    temp_t low = { 0 }; // how far it oscillates below target

    tempavg_t highMean;
    tempavg_t tempMean;
    tempavg_t lowMean;
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
      if (Temperature::manage_heater())
      {
        const temp_t current_temperature = Temperature::degHotend();

        average_append(tempAvg, current_temperature.raw(), TempAvgSampleCount);

        const auto temp_trend = Temperature::get_temperature_trend();
        //Log::d<>(TagGraph, "%.6f, %.6f, %S%.6f"_p, float(current_temperature), float(temp_t::from(extract_average(tempAvg, TempAvgSampleCount))), temp_trend.second ? "+"_p : "-"_p, float(temp_trend.first));

        lcd::update_graph();

        // We only care when we are above the temp target during an upswing. This guarantees we aren't polluting the test due to going 'down' first.
        if (current_temperature >= Temperature::degTargetHotend() && temp_trend.second)
        {
          return;
        }
      }
    }
  };

  const auto populate_pwm_table_const = [](uint8 value)
  {
    pwm_exponent = 0.0f;

    for (tableidx_t i = 0; i < numTableEntries; ++i)
    {
      pwm_table[i] = value;
      //pwm_table_down[i] = value;
    }
  };

  Temperature::disable_all_heaters();

  // First, we set the target temperature to a value minus double the expected swing.
  constexpr const temp_t double_swing = temp_t{ integer_swing * 2 };
  const temp_t low_target = target - double_swing;

  Log::d<>(Tag, "Calibration Parameters:"_p);
  Log::d<1>(Tag, "target: %.6f"_p, float(target));
  Log::d<1>(Tag, "test_cycles: %u"_p, test_cycles);
  Log::d<1>(Tag, "test_max_time: %u"_p, test_max_time.raw());
  Log::d<1>(Tag, "TempAvgSampleCount: %u"_p, TempAvgSampleCount);
  Log::d<1>(Tag, "integer_swing: %u"_p, integer_swing);
  Log::d<1>(Tag, "low_target: %.6f"_p, float(low_target));
  Log::d<1>(Tag, "table entries: %u"_p, numTableEntries);
  Log::d<1>(Tag, "pwm integer bits: %u"_p, integer_swing_bits);
  Log::d<1>(Tag, "pwm fracion bits: %u"_p, relevant_fraction_bits);
  Log::d<1>(Tag, "temp_t integer bits: %u"_p, temp_t::integer_bits);
  Log::d<1>(Tag, "temp_t fraction bits: %u"_p, temp_t::fractional_bits);

  const auto execute_test = [&, target](arg_type<float> exponent, oscillation & __restrict error)
  {
    Log::d<1>(Tag, "Exponent: %.6f"_p, exponent);

    // Reset the PWM table to 0xFF for low-target setting.
    populate_pwm_table_const(0xFF);
    // Reset the target temperature to the lower target.
    Log::d<>(Tag, "Reaching Low Target"_p);
    set_temp_target(low_target, true);
    Log::d<>(Tag, "Low Target Reached, beginning test"_p);

    // Populate the PWM table.
    SetCalibration({ exponent, 0.0f });

    // Set target temperature.
    set_temp_target(target, false);

    // This tracks if temperature last was going upwards or downwards.
    bool upswing = true;

    // This tracks how many cycles we've been through.
    uint8 num_cycles = 0;

    // called when we cross the target again, indicating one half of a cycle.
    const auto half_cycle = [&]() -> bool
    {
      upswing = !upswing;
      ++num_cycles;
      //return num_cycles >= test_cycles;
      return false;
    };

    tempavg_t highAvg = 0;
    tempavg_t lowAvg;
    init_average(lowAvg, (target - Temperature::degHotend()).raw(), TempAvgSampleCount);

    const auto start_time = chrono::time_ms<uint24>::get();

    // Once we pass the target temperature (defines as going from below to above the target temperature)
    // we begin our analysis.
    for (;;)
    {
      // manage_heater handles managing PWM values, reading temperature values
      // and implements safeties.
      // It returns true when new temperature data is available.
      if (Temperature::manage_heater())
      {
        // TODO FIXME use a running average (even though temperature is already an average)
        // otherwise this will be quite noisy.

        const temp_t current_temperature = Temperature::degHotend();
        average_append(tempAvg, current_temperature.raw(), TempAvgSampleCount);

        const auto temp_trend = Temperature::get_temperature_trend();
        //Log::d<>(TagGraph, "%.6f, %.6f, %S%.6f"_p, float(current_temperature), float(temp_t::from(extract_average(tempAvg, TempAvgSampleCount))), temp_trend.second ? "+"_p : "-"_p, float(temp_trend.first));

        lcd::update_graph();

        if (current_temperature >= target)
        {
          error.valid = true;
        }
        if (current_temperature > target)
        {
          const temp_t differential = current_temperature - target;
          average_append(highAvg, differential.raw(), TempAvgSampleCount);
          error.high = max(error.high, differential);
          if (!upswing)
          {
            if (half_cycle())
            {
              break;
            }
          }
        }
        else if (current_temperature < target)
        {
          const temp_t differential = target - current_temperature;
          average_append(lowAvg, differential.raw(), TempAvgSampleCount);
          // We only track 'low' if we actually every break our temperature goal.
          if (error.valid)
          {
            error.low = max(error.low, differential);
            if (upswing)
            {
              if (half_cycle())
              {
                break;
              }
            }
          }
        }
      }

      // Regardless, we also check the current time, to see if we've timed out.
      if (start_time.elapsed(test_max_time))
      {
        break;
      }

      error.highMean = highAvg;
      error.tempMean = tempAvg;
      error.lowMean = lowAvg;
    }

    // We had zero cycles for this test...
    // TODO we should probably do something here.
    if (num_cycles == 0)
    {
    }
  };

  constexpr const uint8 num_tests = array_size(test_exponents);

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

    return (uint32(highMean.raw()) * 3) + (uint32(lowMean.raw()) * 2);
  };

  uint32 bestError = type_trait<uint32>::max;
  uint16 bestIndex = 0;

  constexpr const auto total_tests = num_tests;

  for (uint8 i = 0; i < num_tests; ++i)
  {
    const auto & __restrict exponent = test_exponents[i];
    oscillation error;

    Log::d<>(Tag, "Executing Test: %u / %u *****************"_p, i, total_tests);
    execute_test(exponent, error);

    Log::d<>(Tag, "Test Complete: %u / %u"_p, i, total_tests);
    Log::d<1>(Tag, "valid: %u"_p, error.valid ? 1 : 0);

    if (error.valid)
    {
      Log::d<1>(Tag, "high: %.6f"_p, float(error.high));
      Log::d<1>(Tag, "low: %.6f"_p, float(error.low));
      Log::d<1>(Tag, "highMean: %lu"_p, uint32(error.highMean));
      Log::d<1>(Tag, "tempMean: %lu"_p, uint32(error.tempMean));
      Log::d<1>(Tag, "lowMean: %lu"_p, uint32(error.lowMean));
    }

    if (error.valid)
    {
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
  Log::d<>(Tag, "Best Index: %u"_p, bestIndex);

  const float & __restrict best_exp = test_exponents[bestIndex];

  Log::d<1>(Tag, "Exponent: %.6f"_p, float(best_exp));

  SetCalibration({ best_exp, 0.0f });

  lcd::show_page(lcd::Page::PID_Finished);
  enqueue_and_echo_command("M106 S0");
  lcd::update();

  settings.save();

  return true;
}

bool Simple::calibrating()
{
  return calibrating;
}

uint8 Simple::get_power(arg_type<temp_t> current, arg_type<temp_t> target) 
{
  constexpr const bool tempLog = false;

  constexpr const temp_t degrees_above_pwm = 1_C;
  const temp_t high_target = target + degrees_above_pwm;

  if (current > high_target)
  {
    if constexpr (tempLog) Log::d<>(Tag, "%.6f, %u"_p, float(current), 0);
    return 0;
  }

  const auto diff_idx = get_diff_idx(current, high_target);

  if (!diff_idx)
  {
    if constexpr (tempLog) Log::d<>(Tag, "%.6f, %u"_p, float(current), 0xFF);
    return 0xFF;
  }

  // Otherwise, do a lookup in the pwm table.
  if (current >= target)
  {
    if (!Temperature::get_temperature_trend().second)
    {
      if constexpr (tempLog) Log::d<>(Tag, "%.6f, %u"_p, float(current), pwm_table[diff_idx.second]);

      // We want to intensify this value a bit, in order to prevent undershooting too much.
      constexpr const uint8 multiplier = 3_u8;
      return uint8(min(uint16(pwm_table[diff_idx.second]) * multiplier, 255_u8));
    }
    else
    {
      if constexpr (tempLog)Log::d<>(Tag, "%.6f, %u"_p, float(current), pwm_table[diff_idx.second]);

      return 0x00;
    }
  }
  else if (Temperature::get_temperature_trend().second)
  {
    if constexpr (tempLog) Log::d<>(Tag, "%.6f, %u"_p, float(current), pwm_table[diff_idx.second]);
    return pwm_table[diff_idx.second];
  }
  else
  {
    //return pwm_table_down[diff_idx.second];
    if constexpr (tempLog) Log::d<>(Tag, "%.6f, %u"_p, float(current), 0xFF);
    return 0xFF;
  }
}

void Simple::debug_dump()
{
}