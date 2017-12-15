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
  using namespace Tuna;
  using namespace Tuna::Thermal::Manager;

  constexpr const auto Tag = "SimpleManager"_p;

  using scalar_t = Simple::scalar_t;
  using exponent_t = Simple::exponent_t;

  // user-defined literals for internal types
  template <char... Chars> constexpr scalar_t operator "" _scalar()
  {
    static_assert(type_trait<scalar_t>::is_integral, "scalar_t must be integral");

    constexpr const uint64 value = _internal::expand_string_to_int<Chars...>();
    static_assert(value <= type_trait<scalar_t>::max, "out of range u8 literal");
    return scalar_t(value);
  }

  constexpr exponent_t operator "" _exponent(long double value)
  {
    return value;
  }

  using preferred_table_type = uint8;
  constexpr const uint8 integer_swing = 16;
  constexpr const uint8 integer_swing_bits = constant::log2<integer_swing>;
  constexpr const uint8 relevant_fraction_bits = min(temp_t::fractional_bits, (sizeof(preferred_table_type) * 8_u8) - integer_swing_bits);
  constexpr const uint8 relevant_total_bits = (relevant_fraction_bits + integer_swing_bits);
  constexpr const auto numTableEntries = make_uintsz<1_u64 << relevant_total_bits>;
  constexpr const auto test_max_time = 100000_ms24; // Otherwise, test goes this long.
  using tableidx_t = uintsz<1_u64 << relevant_total_bits>;

  // Validations
  static_assert(constant::is_pow2<integer_swing>, "integer_swing must be power-of-two.");

  // SRAM
  uint8 pwm_table[numTableEntries] = {};
  Simple::calibration pwm_calibration;

  static pair<bool, tableidx_t> __forceinline __flatten get_diff_idx(arg_type<temp_t> current, arg_type<temp_t> target)
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

const __forceinline __flatten Simple::calibration & Simple::GetCalibration()
{
  return pwm_calibration;
}

void Simple::SetCalibration(arg_type<calibration> value)
{
  Log::d(Tag, "Current Calibration: %.6f %u"_p, value.Exponent_, uint16(value.Scalar_));

  pwm_calibration = value;

  const exponent_t inv_numTableEntries = 1.0_exponent / exponent_t(numTableEntries);
  for (tableidx_t i = 0; i < numTableEntries; ++i)
  {
    const exponent_t fraction = exponent_t(i) * inv_numTableEntries;
    __assume(fraction >= 0.0_exponent && fraction <= 1.0_exponent);
    const float exponentiated = pow(fraction, value.Exponent_) * 255.5_exponent;
    __assume(exponentiated >= 0.0_exponent && exponentiated <= 255.5f);
    pwm_table[i] = uint8(exponentiated);
    //Log::d<1>(Tag, "%u"_p, pwm_table[i]);
  }
}

namespace
{
  // This will return multiplier_t if it's given an exponent_t, and an exponent_t if given a multiplier_t
  template <typename T>
  struct _alternate_type;

  template <>
  struct _alternate_type<scalar_t> final : trait::ce_only
  {
    using type = exponent_t;
  };

  template <>
  struct _alternate_type<exponent_t> final : trait::ce_only
  {
    using type = scalar_t;
  };

  template <typename T>
  using alternate_type = typename _alternate_type<T>::type;


  template <typename T, uint8 TN>
  uint8 execute_test(arg_type<temp_t> target, const T (& __restrict test_array) [TN], arg_type<alternate_type<T>> test_alternate)
  {
    using namespace Tuna::Thermal::Manager;

    // If T is exponent_t, we are still processing exponents,
    // otherwise we are now trying to find a multiplier.
    constexpr const bool is_exponent_test = is_same<T, exponent_t>;
    
    using tempavg_t = type_trait<temp_t::type>::larger_type;
    constexpr const uint16 TempAvgSampleCount = 256;

    struct oscillation final
    {
      bool valid = false;

      temp_t high = { 0 }; // how far it oscillates above target
      temp_t low = { 0 }; // how far it oscillates below target
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

          //average_append(tempAvg, current_temperature.raw(), TempAvgSampleCount);

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

    const auto execute_test = [&, target](arg_type<exponent_t> exponent, oscillation & __restrict error, scalar_t scalar)
    {
      Log::d<1>(Tag, "Exponent: %.6f"_p, exponent);

      // Reset the PWM table to 0xFF for low-target setting.
      populate_pwm_table_const(0xFF);
      // Reset the target temperature to the lower target.
      Log::d(Tag, "Reaching Low Target"_p);
      set_temp_target(low_target, true);
      Log::d(Tag, "Low Target Reached, beginning test"_p);

      // Populate the PWM table.
      Simple::SetCalibration({ exponent, scalar });

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
          //average_append(tempAvg, current_temperature.raw(), TempAvgSampleCount);

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
    };

    constexpr const uint8 num_tests = TN;

    //temp_t best_high = temp_t{ printer_max_temperature };
    //uint8 best_high_idx = 0;
    //temp_t best_low = temp_t{ printer_max_temperature };
    //uint8 best_low_idx = 0;

    const auto errorCalculate = [&](arg_type<oscillation> error)->uint32
    {
      const auto highMean = error.high;
      const auto lowMean = error.low;

      return (uint32(highMean.raw()) * 5) + (uint32(lowMean.raw()) * 2);
    };

    uint32 bestError = type_trait<uint32>::max;
    uint8 bestIndex = 0;

    constexpr const auto total_tests = num_tests;

    for (uint8 i = 0; i < num_tests; ++i)
    {
      const exponent_t & __restrict exponent = is_exponent_test ? test_array[i] : test_alternate;
      const scalar_t scalar = is_exponent_test ? test_alternate : test_array[i];
      oscillation error;

      Log::d(Tag, "Executing Test: %u / %u *****************"_p, i + 1, total_tests);
      execute_test(exponent, error, scalar);

      Log::d(Tag, "Test Complete: %u / %u"_p, i + 1, total_tests);
      Log::d<1>(Tag, "valid: %u"_p, error.valid ? 1 : 0);

      if (__likely(error.valid))
      {
        Log::d<1>(Tag, "high: %.6f"_p, float(error.high));
        Log::d<1>(Tag, "low: %.6f"_p, float(error.low));

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

    // For now, choose the best high.
    Log::d(Tag, "Local Best Index: %u"_p, bestIndex);

    {
      const exponent_t & __restrict exponent = is_exponent_test ? test_array[bestIndex] : test_alternate;
      const scalar_t scalar = is_exponent_test ? test_alternate : test_array[bestIndex];

      Log::d<1>(Tag, "Exponent: %.6f  Scalar: %u"_p, float(exponent), scalar);

      return bestIndex;
    }
  }
}

bool Simple::calibrate(arg_type<temp_t> target)
{
  Log::d(Tag, "Starting Calibration"_p);

  constexpr const exponent_t test_exponents[] =
  {
    0.0_exponent,
    0.1_exponent,
    0.2_exponent,
    0.3_exponent,
    0.4_exponent,
    0.5_exponent,
    0.6_exponent,
    0.7_exponent,
    0.8_exponent,
    0.9_exponent,
    //1.0_exponent,
    //1.1_exponent,
    //1.2_exponent,
    //1.3_exponent,
    //1.4_exponent,
    //1.5_exponent,
    //1.6_exponent,
    //1.7_exponent,
    //1.8_exponent,
    //1.9_exponent,
    //2.0_exponent,
  };

  constexpr const scalar_t test_scalars[] =
  {
    1_scalar,
    2_scalar,
    3_scalar,
    4_scalar,
    5_scalar
  };

  constexpr const uint8 exponent_levels = 3;

  exponent_t exponent;
  exponent_t epsilon = 0.05_exponent;

  for (uint8 cur_level = 0; cur_level < exponent_levels; ++cur_level)
  {
    constexpr const scalar_t test_scalar = 1_scalar;

    if (cur_level == 0)
    {
      // test only the base exponents.
      const uint8 best_idx = execute_test(target, test_exponents, test_scalar);
      exponent = test_exponents[best_idx];
    }
    else
    {
      const exponent_t exponents[] =
      {
        exponent - (epsilon * 2),
        exponent - (epsilon * 1),
        exponent,
        exponent + (epsilon * 1),
        exponent + (epsilon * 2),
      };

      epsilon *= 0.5_exponent;

      const uint8 best_idx = execute_test(target, exponents, test_scalar);
      exponent = exponents[best_idx];
    }
  }

  // Now test scalars.

  const uint8 best_scalar_idx = execute_test(target, test_scalars, exponent);
  const scalar_t scalar = test_scalars[best_scalar_idx];

  Temperature::disable_all_heaters();

  Log::d<1>(Tag, "Best Exponent: %.6f  Scalar: %u"_p, float(exponent), scalar);

  SetCalibration({ exponent, scalar });

  lcd::show_page(lcd::Page::PID_Finished);
  enqueue_and_echo_command("M107");
  lcd::update();

  settings.save();

  return true;
}

uint8 __forceinline __flatten Simple::get_power(arg_type<temp_t> current, arg_type<temp_t> target)
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
    Log::d(Tag, "%u :: %.6f, %u, trend: %s"_p, 0, float(current), out_temp, (Temperature::get_temperature_trend() == Temperature::Trend::Up) ? "up" : "down");
  }

  return out_temp;
}

void Simple::debug_dump()
{
}