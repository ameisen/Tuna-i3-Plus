#pragma once

#include <math.h>

#ifdef round
#	define ARDUINO_ROUND 1
#	undef round
#endif

namespace Tuna
{

	template <typename T, uint8 N>
	inline fixed<T, N> round(arg_type<fixed<T, N>> value)
	{
		return value.rounded();
	}

	inline float round(arg_type<float> x)
	{
#if ARDUINO_ROUND
		return ((x) >= 0 ? (long)((x)+0.5) : (long)((x)-0.5)); // Arduino SDK implementation.
#else
		return roundf(x);
#endif
	}

  template <typename T>
  constexpr inline __forceinline __flatten T round(arg_type<float> value)
  {
    if constexpr (is_same<T, float> || is_same<T, double>)
    {
      // We are doing float->float rounding.
      return ::roundf(value);
    }
    else if constexpr(type_trait<T>::is_unsigned)
    {
      // We are doing float->unsigned rounding.
      __assume(T(value) >= 0 && T(value + 0.5f) <= type_trait<T>::max);
      __assume(value >= 0.0 && value <= type_trait<T>::max);
      const auto result = lround(value);
      __assume(result >= 0 && result <= type_trait<T>::max);
      return result;
    }
    else
    {
      // We are doing float->signed rounding.
      __assume(T(value) >= type_trait<T>::min && T(value + 0.5f) <= type_trait<T>::max);
      __assume(value >= type_trait<T>::min && value <= type_trait<T>::max);
      const auto result = lround(value);
      __assume(result >= type_trait<T>::min && result <= type_trait<T>::max);
      return result;
    }
  }
}
