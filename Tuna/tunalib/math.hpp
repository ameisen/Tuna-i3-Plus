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

#define M_PI_C 3.14159265358979323846_C
}
