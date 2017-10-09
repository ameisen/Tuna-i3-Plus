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

// OVERSAMPLENR PROGMEM

struct TablePair final
{
	uint16_t Adc = 0;
	uint16_t Temperature = 0;

	constexpr TablePair() = default;
	constexpr TablePair(uint16_t _ADC, uint16_t _Temperature) :
		Adc(_ADC * OVERSAMPLENR), Temperature(_Temperature)
	{}

	constexpr TablePair & operator = (const TablePair &pair)
	{
		Adc = pair.Adc;
		Temperature = pair.Temperature;
		return *this;
	}
};

constexpr const PROGMEM TablePair temp_table[]{
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
constexpr uint8_t temp_table_size = sizeof(temp_table) / sizeof(temp_table[0]);

constexpr const bool IsFixedStepTable_Temperature = true;
constexpr const uint8_t FixedStepTable_Temperature = 5;
constexpr const bool IsFixedStepTable_ADC = false;

template <uint8_t i = 1, uint8_t end = temp_table_size>
constexpr uint16_t get_max_adc_delta(uint16_t delta = 0)
{
	if constexpr (i == 0)
	{
		return get_max_adc_delta<1, end>();
	}
	else if constexpr (i == end)
	{
		return delta;
	}
	else
	{
		constexpr uint16_t local_delta = (temp_table[i].Adc > temp_table[i - 1].Adc) ?
			temp_table[i].Adc - temp_table[i - 1].Adc :
			temp_table[i - 1].Adc - temp_table[i].Adc;
		return get_max_adc_delta<i + 1, end>((local_delta > delta) ? local_delta : delta);
	}
}

constexpr uint16_t max_adc_delta = get_max_adc_delta();

template <uint8_t i = 0>
constexpr uint16_t get_max_adc_uint8_temp(uint16_t max_adc = 0)
{
	if constexpr (i == temp_table_size)
	{
		return max_adc;
	}
	else
	{
		return get_max_adc_uint8_temp<i + 1>(temp_table[i].Temperature <= 0xFF ? (temp_table[i].Adc > max_adc ? temp_table[i].Adc : max_adc) : max_adc);
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
		return get_max_adc_uint8_temp_idx<i + 1, set_new_adc ? temp_table[i].Adc : adc>(set_new_adc ? i : idx);
	}
}

constexpr uint16_t max_adc_uint8_temp = get_max_adc_uint8_temp();
constexpr uint16_t max_adc_uint8_temp_idx = get_max_adc_uint8_temp_idx();

template <uint8_t i = 0>
constexpr uint16_t get_min_adc_uint8_temp(uint16_t min_adc = 0xFFFF)
{
	if constexpr (i == temp_table_size)
	{
		return min_adc;
	}
	else
	{
		return get_min_adc_uint8_temp<i + 1>(temp_table[i].Temperature <= 0xFF ? (temp_table[i].Adc <= min_adc ? temp_table[i].Adc : min_adc) : min_adc);
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
		return get_min_adc_uint8_temp_idx<i + 1, set_new_adc ? temp_table[i].Adc : adc, set_new_adc ? i : idx>();
	}
}

constexpr uint16_t min_adc_uint8_temp = get_min_adc_uint8_temp();
constexpr uint16_t min_adc_uint8_temp_idx = get_min_adc_uint8_temp_idx();

template <uint8_t i = 0>
constexpr uint16_t get_max_adc_uint8(uint8_t adc = 0)
{
	if constexpr (i == temp_table_size)
	{
		return adc;
	}
	else
	{
		return get_max_adc_uint8<i + 1>(((temp_table[i].Adc >= adc) && temp_table[i].Adc <= 0xFF) ? temp_table[i].Adc : adc);
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
		return get_max_adc_uint8_idx<i + 1, set_new_adc ? temp_table[i].Adc : adc, set_new_adc ? i : idx>();
	}
}

constexpr uint16_t max_adc_uint8 = get_max_adc_uint8();
constexpr uint16_t max_adc_uint8_idx = get_max_adc_uint8_idx();

template <bool bits8>
struct delta_type;

template <>
struct delta_type<true> final
{
	using type = uint8_t;
};

template <>
struct delta_type<false> final
{
	using type = uint16_t;
};

using delta_t = delta_type<max_adc_delta <= 0xFF>::type;

template <typename T, typename U = T>
inline T pgm_read(const U &var);

template <>
inline uint8_t pgm_read<uint8_t>(const uint8 &var)
{
	return pgm_read_byte((uint16_t)&var);
}

template <>
inline uint8_t pgm_read<uint8_t, uint16_t>(const uint16 &var)
{
	return pgm_read_byte((uint16_t)&var);
}

template <>
inline uint16_t pgm_read<uint16_t>(const uint16_t &var)
{
	return pgm_read_word((uint16_t)&var);
}

constexpr uint8 low_temp_idx = temp_table_size - 1;
constexpr uint8 hi_temp_idx = 0;
constexpr uint16 min_temperature = temp_table[low_temp_idx].Temperature;
constexpr uint16 max_temperature = temp_table[hi_temp_idx].Temperature;
constexpr uint8 low_adc_idx = 0;
constexpr uint8 hi_adc_idx = temp_table_size - 1;
constexpr uint16 min_adc = temp_table[low_adc_idx].Adc;
constexpr uint16 max_adc = temp_table[hi_temp_idx].Adc;
constexpr int8 higher_temp_idx = (low_temp_idx < hi_temp_idx) ? 1 : -1;
constexpr int8 higher_adc_idx = (low_adc_idx < hi_adc_idx) ? 1 : -1;

namespace thermistor
{
	template <typename T>
	struct interpolator
	{
		const T delta;
		const T max;
	};

	template <uint16 x, uint16 a, uint16 b>
	constexpr interpolator<uint16> interpoland()
	{
		uint16 delta = x - a;
		uint16 max_diff = b - a;

		return { delta, max_diff };
	}

	template <uint16 delta, uint16 max, uint16 b, uint16 a>
	constexpr uint16 interpolate()
	{
		uint32_t a_lambda = (uint32_t(a) * uint32_t(delta));
		uint32_t b_lambda = (uint32_t(b) * uint32_t(max - delta));
		return (a_lambda + b_lambda) / max;
	}

	template <uint16 temperature, uint8 cur_idx = 0, uint8 best_le = low_temp_idx>
	constexpr uint16_t ce_convert_temp_to_adc()
	{
		static_assert(temperature <= max_temperature && temperature >= min_temperature, "temperature is out of range.");

		if constexpr (cur_idx == temp_table_size)
		{
			if constexpr (best_le == temp_table_size - 1)
			{
				return temp_table[best_le].Adc;
			}
			else
			{
				constexpr auto best_value = temp_table[best_le];
				constexpr auto next_value = temp_table[best_le + higher_temp_idx];
				constexpr auto interp = interpoland<temperature, best_value.Temperature, next_value.Temperature>();
				return interpolate<
					interp.delta, interp.max, best_value.Adc, next_value.Adc
				>();
			}
		}
		else
		{
			constexpr auto cur_value = temp_table[cur_idx];
			constexpr auto best_value = temp_table[best_le];
			return ce_convert_temp_to_adc<
				temperature, cur_idx + 1,
				(cur_value.Temperature <= temperature && cur_value.Temperature > best_value.Temperature) ?
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
				return temp_t::_from(interpolate<
					interp.delta, interp.max, best_value.Temperature << temp_t::frac_bits, next_value.Temperature << temp_t::frac_bits
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
	inline temp_t binsearch_temp_get_branched(typename delta_type<small_adc>::type adc)
	{
		constexpr const uint8_t max_idx = small_adc ? max_adc_uint8_idx : (temp_table_size - 1);
		constexpr const uint8_t min_idx = small_temp ? min_adc_uint8_temp_idx : 0;
		constexpr const uint16_t max_adc_delta_local = get_max_adc_delta<min_idx ? min_idx - 1 : 0, max_idx + 1>();
		static_assert(uint16_t(max_idx * 2) <= 0xFF, "otherwise halving won't work, and I don't want to add extra code for it.");

		using adc_t = typename delta_type<small_adc>::type;
		using deltatemp_t = typename delta_type<small_temp>::type;

		uint8_t L = min_idx;
		uint8_t R = max_idx;
		uint8_t m;
		do
		{
			m = (L + R) / 2;
			adc_t A = pgm_read<adc_t>(temp_table[m].Adc);
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
				return { pgm_read<deltatemp_t>(temp_table[m].Temperature) };
			}
		} while (L <= R);

		// L is either equal to or less than our real value. We need to interpolate between this value and L+1.

		adc_t le_value = pgm_read<adc_t>(temp_table[L - 1].Adc);
		adc_t g_value = pgm_read<adc_t>(temp_table[L].Adc);

		//assert(le_value <= adc);
		//assert(g_value > adc);

		const auto get_interpoland = [](adc_t a, adc_t b, adc_t x) -> interpolator<delta_t>
		{
			delta_t delta = x - a;
			delta_t max_diff = b - a;

			return { delta, max_diff };
		};



		const auto interpolate = [](deltatemp_t b, deltatemp_t a, const interpolator<delta_t> &delta) -> deltatemp_t
		{
			//lambda = interpolation_value - lambda;
			// 16 bits is a safe interpolation size.
			// (A*(255-x)+B*x)/255 orig

			constexpr const uint16_t MaxTemp = temp_table[0].Temperature;
			constexpr const uint64_t DeltaTimesTemp = uint64(max_adc_delta_local) * MaxTemp;

			static_assert(DeltaTimesTemp <= tuna::type_trait<uint32>::max, "ridiculous temperature values in table.");
			if constexpr (DeltaTimesTemp <= tuna::type_trait<uint8>::max)
			{
				uint8_t a_lambda = (a * (delta.delta));
				uint8_t b_lambda = (b * (delta.max - delta.delta));
				if constexpr ((DeltaTimesTemp * 2) <= tuna::type_trait<uint8>::max)
				{
					return (a_lambda + b_lambda) / delta.max;
				}
				else
				{
					return (uint16_t(a_lambda) + b_lambda) / delta.max;
				}
			}
			else if constexpr (DeltaTimesTemp <= tuna::type_trait<uint16>::max)
			{
				uint16_t a_lambda = (uint16_t(a) * (delta.delta));
				uint16_t b_lambda = (uint16_t(b) * (delta.max - delta.delta));
				static_assert((DeltaTimesTemp * 2) <= tuna::type_trait<uint8>::max, "ridiculous temperature values in table.");
				if constexpr ((DeltaTimesTemp * 2) <= tuna::type_trait<uint16>::max)
				{
					return (uint16_t(a_lambda) + b_lambda) / delta.max;
				}
				else
				{
					return (uint32_t(a_lambda) + b_lambda) / delta.max;
				}
			}
			else
			{
				uint32_t a_lambda = (uint32_t(a) * (delta.delta));
				uint32_t b_lambda = (uint32_t(b) * (delta.max - delta.delta));
				static_assert((DeltaTimesTemp * 2) <= tuna::type_trait<uint32>::max, "ridiculous temperature values in table.");
				if constexpr ((DeltaTimesTemp * 2) <= tuna::type_trait<uint32>::max)
				{
					return (a_lambda + b_lambda) / delta.max;
				}
			}
		};

		interpolator interpoland = get_interpoland(le_value, g_value, adc);

		const deltatemp_t g_value_t = pgm_read<deltatemp_t>(temp_table[L].Temperature);
		const deltatemp_t le_value_t = [&, L]()->deltatemp_t
		{
			// Save on a load when we can derive it from something already loaded.
			if constexpr (IsFixedStepTable_Temperature)
			{
				return g_value_t + FixedStepTable_Temperature;
			}
			else
			{
				return pgm_read<deltatemp_t>(temp_table[L - 1].Temperature);
			}
		}();

		return interpolate(le_value_t, g_value_t, interpoland);
	}

	inline temp_t binsearch_temp_get(uint16_t adc)
	{
		static constexpr bool branched_binsearch = false;

		if constexpr (!branched_binsearch)
		{
			return binsearch_temp_get_branched<false, false>(adc);
		}
		else
		{
			const bool small_adc = (adc <= max_adc_uint8);
			constexpr const uint8 small_adc_bit = 0b01;
			const bool small_temp = (adc >= min_adc_uint8_temp);
			constexpr const uint8 small_temp_bit = 0b10;

			const uint8 bit = (small_adc ? small_adc_bit : 0b00) | (small_temp ? small_temp_bit : 0b00);

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
	}

	inline temp_t adc_to_temperature(uint16_t adc)
	{
		return binsearch_temp_get(adc);
	}
}

// 100k bed thermistor
#if 1
constexpr short temptable_1[][2] PROGMEM = {
  {   23 * OVERSAMPLENR, 300 },
  {   25 * OVERSAMPLENR, 295 },
  {   27 * OVERSAMPLENR, 290 },
  {   28 * OVERSAMPLENR, 285 },
  {   31 * OVERSAMPLENR, 280 },
  {   33 * OVERSAMPLENR, 275 },
  {   35 * OVERSAMPLENR, 270 },
  {   38 * OVERSAMPLENR, 265 },
  {   41 * OVERSAMPLENR, 260 },
  {   44 * OVERSAMPLENR, 255 },
  {   48 * OVERSAMPLENR, 250 },
  {   52 * OVERSAMPLENR, 245 },
  {   56 * OVERSAMPLENR, 240 },
  {   61 * OVERSAMPLENR, 235 },
  {   66 * OVERSAMPLENR, 230 },
  {   71 * OVERSAMPLENR, 225 },
  {   78 * OVERSAMPLENR, 220 },
  {   84 * OVERSAMPLENR, 215 },
  {   92 * OVERSAMPLENR, 210 },
  {  100 * OVERSAMPLENR, 205 },
  {  109 * OVERSAMPLENR, 200 },
  {  120 * OVERSAMPLENR, 195 },
  {  131 * OVERSAMPLENR, 190 },
  {  143 * OVERSAMPLENR, 185 },
  {  156 * OVERSAMPLENR, 180 },
  {  171 * OVERSAMPLENR, 175 },
  {  187 * OVERSAMPLENR, 170 },
  {  205 * OVERSAMPLENR, 165 },
  {  224 * OVERSAMPLENR, 160 },
  {  245 * OVERSAMPLENR, 155 },
  {  268 * OVERSAMPLENR, 150 },
  {  293 * OVERSAMPLENR, 145 },
  {  320 * OVERSAMPLENR, 140 },
  {  348 * OVERSAMPLENR, 135 },
  {  379 * OVERSAMPLENR, 130 },
  {  411 * OVERSAMPLENR, 125 },
  {  445 * OVERSAMPLENR, 120 },
  {  480 * OVERSAMPLENR, 115 },
  {  516 * OVERSAMPLENR, 110 },
  {  553 * OVERSAMPLENR, 105 },
  {  591 * OVERSAMPLENR, 100 },
  {  628 * OVERSAMPLENR,  95 },
  {  665 * OVERSAMPLENR,  90 },
  {  702 * OVERSAMPLENR,  85 },
  {  737 * OVERSAMPLENR,  80 },
  {  770 * OVERSAMPLENR,  75 },
  {  801 * OVERSAMPLENR,  70 },
  {  830 * OVERSAMPLENR,  65 },
  {  857 * OVERSAMPLENR,  60 },
  {  881 * OVERSAMPLENR,  55 },
  {  903 * OVERSAMPLENR,  50 },
  {  922 * OVERSAMPLENR,  45 },
  {  939 * OVERSAMPLENR,  40 },
  {  954 * OVERSAMPLENR,  35 },
  {  966 * OVERSAMPLENR,  30 },
  {  977 * OVERSAMPLENR,  25 },
  {  985 * OVERSAMPLENR,  20 },
  {  993 * OVERSAMPLENR,  15 },
  {  999 * OVERSAMPLENR,  10 },
  { 1004 * OVERSAMPLENR,   5 },
  { 1008 * OVERSAMPLENR,   0 },
  { 1012 * OVERSAMPLENR,  -5 },
  { 1016 * OVERSAMPLENR, -10 },
  { 1020 * OVERSAMPLENR, -15 }
};
#endif
