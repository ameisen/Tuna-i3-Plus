/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

 // OVERSAMPLENR __flashmem

#include "ThermistorUtils.hpp"

namespace Tuna::Thermistor
{
  using TablePair = _ThermistorUtils::TablePairBase<uint16>;

  constexpr const __flashmem TablePair temp_table[]{
    { 23, 300 },
    { 24, 295 },
    { 26, 290 },
    { 28, 285 },
    { 30, 280 },
    { 32, 275 },
    { 35, 270 },
    { 37, 265 },
    { 40, 260 },
    { 43, 255 },
    { 47, 250 },
    { 51, 245 },
    { 55, 240 },
    { 60, 235 },
    { 65, 230 },
    { 70, 225 },
    { 76, 220 },
    { 83, 215 },
    { 90, 210 },
    { 99, 205 },
    { 108, 200 },
    { 118, 195 },
    { 129, 190 },
    { 141, 185 },
    { 154, 180 },
    { 168, 175 },
    { 184, 170 },
    { 202, 165 },
    { 221, 160 },
    { 242, 155 },
    { 265, 150 },
    { 289, 145 },
    { 316, 140 },
    { 344, 135 },
    { 375, 130 },
    { 407, 125 },
    { 441, 120 },
    { 476, 115 },
    { 512, 110 },
    { 550, 105 },
    { 587, 100 },
    { 625, 95 },
    { 663, 90 },
    { 699, 85 },
    { 734, 80 },
    { 768, 75 },
    { 800, 70 },
    { 829, 65 },
    { 856, 60 },
    { 881, 55 },
    { 903, 50 },
    { 922, 45 },
    { 939, 40 },
    { 954, 35 },
    { 966, 30 },
    { 977, 25 },
    { 986, 20 },
    { 994, 15 },
    { 1000, 10 },
    { 1005, 5 },
    { 1009, 0 }
  };
  constexpr uint8_t temp_table_size = array_size(temp_table);

  constexpr const bool IsFixedStepTable_Temperature = true;
  constexpr const auto FixedStepTable_Temperature = make_uintsz<5 << temp_t::fractional_bits>;
  constexpr const bool IsFixedStepTable_ADC = false;

  template <typename functor, uint8_t i = 0, uint8_t end = temp_table_size - 1>
  constexpr uint16 table_search(functor predicate, uint16 best = 0)
  {
    const uint16 current_best = predicate(temp_table[i], best);

    if constexpr (i == end)
    {
      return current_best;
    }
    return table_search<functor, i + 1, end>(predicate, current_best);
  }

  constexpr const auto max_adc_uncast = table_search([](auto &pair, auto best) {
    return max(as<decltype(best)>(pair.Adc), best);
  });
  constexpr const auto min_adc_uncast = table_search([](auto &pair, auto best) {
    return min(as<decltype(best)>(pair.Adc), best);
  }, 0xFFFF);

  constexpr const auto max_temp_uncast = table_search([](auto &pair, auto best) {
    return max(as<decltype(best)>(pair.Temperature), best);
  });
  constexpr const auto min_temp_uncast = table_search([](auto &pair, auto best) {
    return min(as<decltype(best)>(pair.Temperature), best);
  }, 0xFFFF);

  constexpr const auto max_adc = make_uintsz<max_adc_uncast>;
  constexpr const auto min_adc = make_uintsz<min_adc_uncast>;
  constexpr const temp_t max_temperature = temp_t::from(max_temp_uncast);
  constexpr const temp_t min_temperature = temp_t::from(min_temp_uncast);
  constexpr const auto max_temperature_integer = make_uintsz<max_temp_uncast>;
  constexpr const auto min_temperature_integer = make_uintsz<min_temp_uncast>;

  constexpr inline uint16 __forceinline __flatten clamp_adc(uint16 adc)
  {
    return clamp(adc, uint16(min_adc), max_adc);
  }

  static_assert(Thermistor::max_temperature_integer >= printer_max_temperature, "the system max temperature must be representable in the temperature table.");

  template <uint8_t i = 1, uint8_t end = temp_table_size - 1>
  constexpr uint16_t get_max_adc_delta(uint16_t delta = 0)
  {
    if constexpr (i == 0)
    {
      return get_max_adc_delta<1, end>();
    }
    else if constexpr (i >= end)
    {
      constexpr uint16_t local_delta = (temp_table[i].Adc > temp_table[i - 1].Adc) ?
        temp_table[i].Adc - temp_table[i - 1].Adc :
        temp_table[i - 1].Adc - temp_table[i].Adc;

      return (local_delta > delta) ? local_delta : delta;
    }
    else
    {
      constexpr uint16_t local_delta = (temp_table[i].Adc > temp_table[i - 1].Adc) ?
        temp_table[i].Adc - temp_table[i - 1].Adc :
        temp_table[i - 1].Adc - temp_table[i].Adc;
      return get_max_adc_delta<i + 1, end>((local_delta > delta) ? local_delta : delta);
    }
  }

  constexpr const auto max_adc_delta = make_uintsz<get_max_adc_delta()>;

  template <uint8_t i = 0>
  constexpr uint16_t get_max_adc_uint8_temp(uint16_t max_adc = 0)
  {
    if constexpr (i == temp_table_size)
    {
      return max_adc;
    }
    else
    {
      return get_max_adc_uint8_temp<i + 1>(temp_table[i].Temperature <= 0xFF ? (temp_table[i].Adc > max_adc ? as(temp_table[i].Adc) : max_adc) : max_adc);
    }
  }

  template <uint8_t i = 0, uint16_t adc = 0>
  constexpr uint16_t get_max_adc_uint8_temp_idx(uint8_t idx = 0)
  {
    if constexpr (i == temp_table_size)
    {
      return idx;
    }
    else
    {
      constexpr bool set_new_adc = (temp_table[i].Temperature <= 0xFF) && (temp_table[i].Adc > adc);
      return get_max_adc_uint8_temp_idx<i + 1, set_new_adc ? as(temp_table[i].Adc) : adc>(set_new_adc ? i : idx);
    }
  }

  constexpr const auto max_adc_uint8_temp = make_uintsz<get_max_adc_uint8_temp()>;
  constexpr const auto max_adc_uint8_temp_idx = make_uintsz<get_max_adc_uint8_temp_idx()>;

  template <uint8_t i = 0>
  constexpr uint16_t get_min_adc_uint8_temp(uint16_t min_adc = 0xFFFF)
  {
    if constexpr (i == temp_table_size)
    {
      return min_adc;
    }
    else
    {
      return get_min_adc_uint8_temp<i + 1>(temp_table[i].Temperature <= 0xFF ? (temp_table[i].Adc <= min_adc ? as(temp_table[i].Adc) : min_adc) : min_adc);
    }
  }

  template <uint8_t i = 0, uint16_t adc = 0xFFFF, uint8_t idx = 0>
  constexpr uint16_t get_min_adc_uint8_temp_idx()
  {
    if constexpr (i == temp_table_size)
    {
      return idx;
    }
    else
    {
      constexpr bool set_new_adc = (temp_table[i].Temperature <= 0xFF) && (temp_table[i].Adc <= adc);
      return get_min_adc_uint8_temp_idx<i + 1, set_new_adc ? as(temp_table[i].Adc) : adc, set_new_adc ? i : idx>();
    }
  }

  constexpr const auto min_adc_uint8_temp = make_uintsz<get_min_adc_uint8_temp()>;
  constexpr const auto min_adc_uint8_temp_idx = make_uintsz<get_min_adc_uint8_temp_idx()>;

  template <uint8_t i = 0>
  constexpr uint16_t get_max_adc_uint8(uint8_t adc = 0)
  {
    if constexpr (i == temp_table_size)
    {
      return adc;
    }
    else
    {
      return get_max_adc_uint8<i + 1>(((temp_table[i].Adc >= adc) && temp_table[i].Adc <= 0xFF) ? as(temp_table[i].Adc) : adc);
    }
  }

  template <uint8_t i = 0, uint8_t adc = 0, uint8_t idx = 0>
  constexpr uint16_t get_max_adc_uint8_idx()
  {
    if constexpr (i == temp_table_size)
    {
      return idx;
    }
    else
    {
      constexpr bool set_new_adc = ((temp_table[i].Adc >= adc) && temp_table[i].Adc <= 0xFF);
      return get_max_adc_uint8_idx<i + 1, set_new_adc ? as(temp_table[i].Adc) : adc, set_new_adc ? i : idx>();
    }
  }

  constexpr const auto max_adc_uint8 = make_uintsz<get_max_adc_uint8()>;
  constexpr const auto max_adc_uint8_idx = make_uintsz<get_max_adc_uint8_idx()>;

  using delta_t = decltype(max_adc_delta);

  constexpr uint8 low_temp_idx = temp_table_size - 1;
  constexpr uint8 hi_temp_idx = 0;
  constexpr uint8 low_adc_idx = 0;
  constexpr uint8 hi_adc_idx = temp_table_size - 1;
  constexpr int8 higher_temp_idx = (low_temp_idx < hi_temp_idx) ? 1 : -1;
  constexpr int8 higher_adc_idx = (low_adc_idx < hi_adc_idx) ? 1 : -1;

  template <typename T>
  struct interpolator final
  {
    const T delta;
    const T max;
  };

  template <uint16 x, uint16 a, uint16 b>
  constexpr inline interpolator<uint16> __forceinline __flatten interpoland()
  {
    uint16 delta = x - a;
    uint16 max_diff = b - a;

    return { delta, max_diff };
  }

  template <uint16 delta, uint16 max, uint16 b, uint16 a>
  constexpr inline uint16 __forceinline __flatten interpolate()
  {
    uint32 a_lambda = (uint32(a) * uint32(delta));
    uint32 b_lambda = (uint32(b) * uint32(max - delta));
    return (a_lambda + b_lambda) / max;
  }

  template <uint16 temperature, bool first = true, uint8 cur_idx = 0, uint8 best_le = low_temp_idx>
  constexpr uint16 ce_convert_temp_to_adc() // TODO convert to a uintsz somehow.
  {
    static_assert(temperature <= max_temperature_integer && temperature >= min_temperature_integer, "temperature is out of range.");

    if constexpr (cur_idx == temp_table_size)
    {
      if constexpr (best_le == low_temp_idx)
      {
        return as(temp_table[best_le].Adc);
      }
      else
      {
        constexpr auto best_temp = as(temp_table[best_le].Temperature);
        constexpr auto next_temp = as(temp_table[best_le + higher_temp_idx].Temperature);
        constexpr auto best_adc = as(temp_table[best_le].Adc);
        constexpr auto next_adc = as(temp_table[best_le + higher_temp_idx].Adc);

        constexpr auto interp = interpoland<temperature, best_temp, next_temp>();
        return interpolate<
          interp.delta, interp.max, best_adc, next_adc
        >();
      }
    }
    else
    {
      constexpr auto cur_temp = as(temp_table[cur_idx].Temperature);
      constexpr auto best_temp = as(temp_table[best_le].Temperature);
      return ce_convert_temp_to_adc<
        temperature, false, cur_idx + 1,
        (cur_temp <= temperature && cur_temp > best_temp) ?
          cur_idx : best_le
      >();
    }
  }

  template <uint16 adc, uint8 cur_idx = 0, uint8 best_le = low_temp_idx>
  constexpr temp_t ce_convert_adc_to_temp()
  {
    static_assert(adc <= max_adc && adc >= min_adc, "adc is out of range.");

    if constexpr (cur_idx == temp_table_size)
    {
      if constexpr (best_le == temp_table_size - 1)
      {
        return temp_table[best_le].Temperature;
      }
      else
      {
        constexpr auto best_value = temp_table[best_le];
        constexpr auto next_value = temp_table[best_le + higher_adc_idx];
        constexpr auto interp = interpoland<adc, best_value.Adc, next_value.Adc>();
        return temp_t::from(interpolate<
          interp.delta, interp.max, best_value.Temperature, next_value.Temperature
        >());
      }
    }
    else
    {
      constexpr auto cur_value = temp_table[cur_idx];
      constexpr auto best_value = temp_table[best_le];
      return ce_convert_adc_to_temp<
        adc, cur_idx + 1,
        (cur_value.Adc <= adc && cur_value.Adc > best_value.Adc) ?
        cur_idx : best_le
  >();
    }
  }

  template <bool small_adc, bool small_temp>
  inline temp_t __forceinline __flatten binsearch_temp_get_branched(arg_type<uintsz<small_adc ? 0xFF : 0xFFFF>> adc)
  {
    constexpr const uint8_t max_idx = small_adc ? max_adc_uint8_idx : (temp_table_size - 1);
    constexpr const uint8_t min_idx = small_temp ? min_adc_uint8_temp_idx : 0;
    constexpr const auto max_adc_delta_local = make_uintsz<get_max_adc_delta<min_idx ? min_idx - 1 : 0, max_idx>()>;
    static_assert(uint16_t(max_idx * 2) <= 0xFF, "otherwise halving won't work, and I don't want to add extra code for it.");

    using adc_t = uintsz<small_adc ? 0xFF : 0xFFFF>;
    using deltatemp_t = uintsz<small_temp ? 0xFF : 0xFFFF>;

    uint8_t L = min_idx;
    uint8_t R = max_idx;
    do
    {
      uint8_t m = (L + R) / 2;
      adc_t A = as<adc_t>(temp_table[m].Adc);
      if (A < adc)
      {
        L = m + 1;
      }
      else if (A > adc)
      {
        R = m - 1;
      }
      else
      {
        return { temp_t::from(as<deltatemp_t>(temp_table[m].Temperature)) };
      }
    } while (L <= R);

    // L is either equal to or less than our real value. We need to interpolate between this value and L+1.

    adc_t le_value = as<adc_t>(temp_table[L - 1].Adc);
    adc_t g_value = as<adc_t>(temp_table[L].Adc);

    const auto __forceinline __flatten get_interpoland = [](arg_type<adc_t> a, arg_type<adc_t> b, arg_type<adc_t> x) -> interpolator<delta_t>
    {
      delta_t delta = x - a;
      delta_t max_diff = b - a;

      return { delta, max_diff };
    };

    const auto __forceinline __flatten interpolate = [](arg_type<deltatemp_t> b, arg_type<deltatemp_t> a, arg_type<interpolator<delta_t>> delta) -> deltatemp_t
    {
      constexpr const auto MaxTemp = make_uintsz<Thermistor::max_temperature.raw()>;
      constexpr const auto DeltaTimesTemp = make_uintsz<uint64(max_adc_delta_local) * MaxTemp>;

      static_assert(DeltaTimesTemp <= type_trait<uint32>::max, "ridiculous temperature values in table.");

      using operand_t = uintsz<DeltaTimesTemp>;
      using operation_t = uintsz<(DeltaTimesTemp * 2)>;

      operand_t a_lambda = (operand_t(a) * (delta.delta));
      operand_t b_lambda = (operand_t(b) * (delta.max - delta.delta));

      return (operation_t(a_lambda) + b_lambda) / delta.max;
    };

    auto interpoland = get_interpoland(le_value, g_value, adc);

    const deltatemp_t g_value_t = as<deltatemp_t>(temp_table[L].Temperature);
    const deltatemp_t __forceinline __flatten le_value_t = [&, L]()->deltatemp_t
    {
      // Save on a load when we can derive it from something already loaded.
      if constexpr (IsFixedStepTable_Temperature)
      {
        return g_value_t + FixedStepTable_Temperature;
      }
      else
      {
        return as<deltatemp_t>(temp_table[L - 1].Temperature);
      }
    }();

    return temp_t::from(interpolate(le_value_t, g_value_t, interpoland));
  }

  inline temp_t __forceinline __flatten binsearch_temp_get(arg_type<uint16_t> adc)
  {
    static constexpr bool branched_binsearch = true;

    if constexpr (!branched_binsearch)
    {
      return binsearch_temp_get_branched<false, false>(adc);
    }
    else
    {
      if constexpr (min_adc <= 0xFF)
      {
        const bool small_adc = (adc <= max_adc_uint8);
        constexpr const uint8 small_adc_bit = 0b01;
        const bool small_temp = (adc >= min_adc_uint8_temp);
        constexpr const uint8 small_temp_bit = 0b10;

        const uint8 bit = uint8(small_adc) | (uint8(small_temp) << 1);

        switch (bit)
        {
        case 0b00:
          return binsearch_temp_get_branched<false, false>(adc);
        case small_adc_bit:
          return binsearch_temp_get_branched<true, false>(adc);
        case small_temp_bit:
          return binsearch_temp_get_branched<false, true>(adc);
        case small_adc_bit | small_temp_bit:
          return binsearch_temp_get_branched<true, true>(adc);
        }
      }
      else
      {
        if constexpr (min_adc_uint8_temp == max_adc)
        {
          return binsearch_temp_get_branched<false, false>(adc);
        }
        else
        {
          const bool small_temp = (adc >= min_adc_uint8_temp);
          if (small_temp)
          {
            return binsearch_temp_get_branched<false, true>(adc);
          }
          else
          {
            return binsearch_temp_get_branched<false, false>(adc);
          }
        }
      }
    }
  }

  inline temp_t __forceinline __flatten adc_to_temperature(arg_type<uint16_t> adc)
  {
    return binsearch_temp_get(adc);
  }
}
