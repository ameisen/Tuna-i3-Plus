#pragma once

#include <stdint.h>

#include "HardwareSerial.h"
#include "avr/interrupt.h"

#undef min
#undef max
#undef clamp
#undef cli
#undef sei

//#define __flash PROGMEM

namespace Tuna
{
	using uint8 = uint8_t;
	using uint16 = uint16_t;
	using uint24 = __uint24;
	using uint32 = uint32_t;
	using uint64 = uint64_t;
	using int8 = int8_t;
	using int16 = int16_t;
	using int24 = __int24;
	using int32 = int32_t;
	using int64 = int64_t;

  // sanity checks
  static_assert(sizeof(uint8) == 1);
  static_assert(sizeof(int8) == 1);
  static_assert(sizeof(uint16) == 2);
  static_assert(sizeof(int16) == 2);
  static_assert(sizeof(uint24) == 3);
  static_assert(sizeof(int24) == 3);
  static_assert(sizeof(uint32) == 4);
  static_assert(sizeof(int32) == 4);
  static_assert(sizeof(uint64) == 8);
  static_assert(sizeof(int64) == 8);

  static_assert(alignof(uint8) == 1);
  static_assert(alignof(int8) == 1);
  static_assert(alignof(uint16) == 1);
  static_assert(alignof(int16) == 1);
  static_assert(alignof(uint24) == 1);
  static_assert(alignof(int24) == 1);
  static_assert(alignof(uint32) == 1);
  static_assert(alignof(int32) == 1);
  static_assert(alignof(uint64) == 1);
  static_assert(alignof(int64) == 1);

#if !defined(__INTELLISENSE__)
	static_assert(sizeof(int) == 2, "atmega int is 2 bytes. Utils need to be rewritten for other sizes.");
#endif
	using uint = uint16;

	constexpr uint8 operator"" _u8(unsigned long long int value)
	{
		// static_assert(value > uint8(-1), "out of range u8 literal");
		return value;
	}

	constexpr int8 operator"" _i8(unsigned long long int value)
	{
		// static_assert(value > uint8(-1), "out of range u8 literal");
		return value;
	}

	constexpr uint16 operator"" _u16(unsigned long long int value)
	{
		// static_assert(value > uint8(-1), "out of range u8 literal");
		return value;
	}

	constexpr int16 operator"" _i16(unsigned long long int value)
	{
		// static_assert(value > uint8(-1), "out of range u8 literal");
		return value;
	}

	constexpr uint24 operator"" _u24(unsigned long long int value)
	{
		// static_assert(value > uint8(-1), "out of range u8 literal");
		return value;
	}

	constexpr int24 operator"" _i24(unsigned long long int value)
	{
		// static_assert(value > uint8(-1), "out of range u8 literal");
		return value;
	}

	constexpr uint32 operator"" _u32(unsigned long long int value)
	{
		// static_assert(value > uint8(-1), "out of range u8 literal");
		return value;
	}

	constexpr int32 operator"" _i32(unsigned long long int value)
	{
		// static_assert(value > uint8(-1), "out of range u8 literal");
		return value;
	}

	constexpr uint64 operator"" _u64(unsigned long long int value)
	{
		// static_assert(value > uint8(-1), "out of range u8 literal");
		return value;
	}

	constexpr int64 operator"" _i64(unsigned long long int value)
	{
		// static_assert(value > uint8(-1), "out of range u8 literal");
		return value;
	}
}

namespace Tuna::utils
{
	struct ce_only { ce_only() = delete; };

	template <typename T, T v>
	struct integral_constant : ce_only
	{
		static constexpr const T value = v;
	};
	struct false_type : integral_constant<bool, false> {};
	struct true_type : integral_constant<bool, true> {};

	template <typename T> struct _is_void final { static constexpr const bool value = false; };;
	template<> struct _is_void<void> final { static constexpr const bool value = true; };

	template <typename T, typename U>
	struct _is_same final : false_type {};
	template <typename T>
	struct _is_same<T, T> final : true_type {};
	template <typename T, typename U> constexpr const bool is_same = _is_same<T, U>::value;

  template <typename T, typename U>
  class _larger_type final : ce_only
  {
    static constexpr auto typer()
    {
      if constexpr (sizeof(U) > sizeof(T))
      {
        return U{};
      }
      return T{};
    }
  public:
    using type = decltype(typer());
  };
  template <typename T, typename U> using larger_type = typename _larger_type<T, U>::type;

  template <typename T, typename U>
  class _smaller_type final : ce_only
  {
    static constexpr auto typer()
    {
      if constexpr (sizeof(U) < sizeof(T))
      {
        return U{};
      }
      return T{};
    }
  public:
    using type = decltype(typer());
  };
  template <typename T, typename U> using smaller_type = typename _smaller_type<T, U>::type;

  template <typename T, bool by_value>
  struct _arg_type;

  template <typename T>
  struct _arg_type<T, true> final : ce_only
  {
    using type = const T;
  };

  template <typename T>
  struct _arg_type<T, false> final : ce_only
  {
    using type = const T & __restrict;
  };

  //template <typename T> using arg_type = typename _arg_type<T, sizeof(T) <= sizeof(void *)>::type;

  template <typename T> using arg_type = const T & __restrict;

	using namespace Tuna;

	constexpr inline __attribute__((always_inline)) void sei()
	{
		__builtin_avr_sei();
	}

	constexpr inline __attribute__((always_inline)) void cli()
	{
		__builtin_avr_cli();
	}

	constexpr inline __attribute__((always_inline)) void nop()
	{
		__builtin_avr_nop();
	}

	constexpr inline __attribute__((always_inline)) void sleep()
	{
		__builtin_avr_sleep();
	}

	constexpr inline __attribute__((always_inline)) void wdr()
	{
		__builtin_avr_wdr();
	}

	constexpr inline __attribute__((always_inline)) uint8 swap(uint8 val)
	{
		return __builtin_avr_swap(val);
	}

	constexpr inline __attribute__((always_inline)) uint16 fmul(uint8 val0, uint8 val1)
	{
		return __builtin_avr_fmul(val0, val1);
	}

	constexpr inline __attribute__((always_inline)) int16 fmuls(int8 val0, int8 val1)
	{
		return __builtin_avr_fmuls(val0, val1);
	}

	constexpr inline __attribute__((always_inline)) int16 fmulsu(int8 val0, uint8 val1)
	{
		return __builtin_avr_fmulsu(val0, val1);
	}

	constexpr inline __attribute__((always_inline)) void delay_cycles(arg_type<uint32> ticks)
	{
		__builtin_avr_delay_cycles(ticks);
	}

	constexpr inline __attribute__((always_inline)) uint8 insert_bits(arg_type<uint32> map, uint8 bits, uint8 val)
	{
		return __builtin_avr_insert_bits(map, bits, val);
	}

#if 0
	template <typename T, uint SZ>
	alignas(T) class array final
	{
		T m_Data[SZ]={};
	public:
		using type = T;
		constexpr static uint size = SZ;

		const T * __restrict data() const __restrict { return m_Data; }
		T * __restrict data() __restrict { return m_Data; }

		constexpr T operator [] (uint index) const __restrict
		{
			static_assert(index < size);
			return m_Data[index];
		}

		T & operator [] (uint index) __restrict
		{
			return m_Data[index];
		}
	};
#endif

	template <typename T>
	constexpr inline T max(arg_type<T> a, arg_type<T> b)
	{
		return (a >= b) ? a : b;
	}

	template <typename T>
	constexpr inline T min(arg_type<T> a, arg_type<T> b)
	{
		return (a < b) ? a : b;
	}

	template <typename T>
	constexpr inline T clamp(arg_type<T> val, arg_type<T> _min, arg_type<T> _max)
	{
		return min(max(val, _min), _max);
	}

	template <typename T>
  class type_trait : ce_only
  {
    // Default
    constexpr static const char name[] = "unknown";

    using type = T;

    constexpr static uint8 size = sizeof(T);
    constexpr static uint8 bits = sizeof(T) * 8;

    constexpr static bool is_signed = false;
    constexpr static bool is_unsigned = false;
    constexpr static bool is_integral = false;
    constexpr static bool is_float = false;
    constexpr static bool is_primitive = false;
    constexpr static bool is_array = false;
    constexpr static bool is_pointer = false;
    constexpr static bool is_reference = false;
    constexpr static bool is_atomic = false;
    constexpr static bool is_emulated = false;
  };

  template <>
  struct type_trait<void> final : ce_only
  {
    constexpr static const char name[] = "void";

    using type = void;

    constexpr static uint8 size = 0;
    constexpr static uint8 bits = 0;

    constexpr static bool is_signed = false;
    constexpr static bool is_unsigned = false;
    constexpr static bool is_integral = false;
    constexpr static bool is_float = false;
    constexpr static bool is_primitive = false;
    constexpr static bool is_array = false;
    constexpr static bool is_pointer = false;
    constexpr static bool is_reference = false;
    constexpr static bool is_atomic = false;
    constexpr static bool is_emulated = false;
  };

	template <>
	struct type_trait<bool> final : ce_only
	{
		constexpr static const char name[] = "bool";

		using type = bool;
		using signed_type = void;
		using unsigned_type = bool; // I mean, I guess 'bool' can be considered unsigned...?
		using larger_type = void;
		using smaller_type = void;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = false;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = true;
		constexpr static bool is_float = false;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = true;
		constexpr static bool is_emulated = false;

		constexpr inline static unsigned_type as_unsigned(arg_type<type> val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(arg_type<type> val) { return { val }; }

		// We really don't want to be able to generate bools with values that aren't 0x00 and 0x01... it's
		// perfectly possible but is likely to confuse the runtime due to UB.
		constexpr static type max = { true };
		constexpr static type min = { false };
		constexpr static type ones = { true };
		constexpr static type zeros = { false };
	};

	template <>
	struct type_trait<uint8> final : ce_only
	{
		constexpr static const char name[] = "uint8";

		using type = uint8;
		using signed_type = int8;
		using unsigned_type = type;
		using larger_type = uint16;
		using smaller_type = void;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = false;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = true;
		constexpr static bool is_float = false;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = true;
		constexpr static bool is_emulated = false;

		constexpr inline static unsigned_type as_unsigned(arg_type<type> val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(arg_type<type> val) { return { val }; }
		constexpr inline static signed_type as_signed(arg_type<type> val) { return val; }

		constexpr static type max = { 0xFF_u8 };
		constexpr static type min = { 0x00_u8 };
		constexpr static type ones = { 0xFF_u8 };
		constexpr static type zeros = { 0x00_u8 };
	};

	template <>
	struct type_trait<int8> final : ce_only
	{
		constexpr static const char name[] = "int8";

		using type = int8;
		using signed_type = type;
		using unsigned_type = uint8;
		using larger_type = int16;
		using smaller_type = void;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = true;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = true;
		constexpr static bool is_float = false;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = true;
		constexpr static bool is_emulated = false;

		constexpr inline static unsigned_type as_unsigned(arg_type<type> val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(arg_type<type> val) { return val; }
		constexpr inline static signed_type as_signed(arg_type<type> val) { return { val }; }

		constexpr static type max = { 0x7F_i8 };
		constexpr static type min = { 0x80_i8 };
		constexpr static type ones = { 0xFF_i8 };
		constexpr static type zeros = { 0x00_i8 };
	};

	template <>
	struct type_trait<uint16> final : ce_only
	{
		constexpr static const char name[] = "uint16";

		using type = uint16;
		using signed_type = int16;
		using unsigned_type = type;
		using larger_type = uint24;
		using smaller_type = uint8;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = false;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = true;
		constexpr static bool is_float = false;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = false;
		constexpr static bool is_emulated = true;

		constexpr inline static unsigned_type as_unsigned(arg_type<type> val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(arg_type<type> val) { return { val }; }
		constexpr inline static signed_type as_signed(arg_type<type> val) { return val; }

		constexpr static type max = { 0xFFFF_u16 };
		constexpr static type min = { 0x0000_u16 };
		constexpr static type ones = { 0xFFFF_u16 };
		constexpr static type zeros = { 0x0000_u16 };
	};

	template <>
	struct type_trait<int16> final : ce_only
	{
		constexpr static const char name[] = "int16";

		using type = int16;
		using signed_type = type;
		using unsigned_type = uint16;
		using larger_type = int24;
		using smaller_type = int8;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = true;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = true;
		constexpr static bool is_float = false;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = false;
		constexpr static bool is_emulated = true;

		constexpr inline static unsigned_type as_unsigned(arg_type<type> val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(arg_type<type> val) { return val; }
		constexpr inline static signed_type as_signed(arg_type<type> val) { return { val }; }

		constexpr static type max = { 0x7FFF_i16 };
		constexpr static type min = { 0x8000_i16 };
		constexpr static type ones = { 0xFFFF_i16 };
		constexpr static type zeros = { 0x0000_i16 };
	};

	template <>
	struct type_trait<uint24> final : ce_only
	{
		constexpr static const char name[] = "uint24";

		using type = uint24;
		using signed_type = int24;
		using unsigned_type = type;
		using larger_type = uint32;
		using smaller_type = uint16;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = false;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = true;
		constexpr static bool is_float = false;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = false;
		constexpr static bool is_emulated = true;

		constexpr inline static unsigned_type as_unsigned(arg_type<type> val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(arg_type<type> val) { return { val }; }
		constexpr inline static signed_type as_signed(arg_type<type> val) { return val; }

		constexpr static type max = { 0xFFFFFF_u24 };
		constexpr static type min = { 0x000000_u24 };
		constexpr static type ones = { 0xFFFFFF_u24 };
		constexpr static type zeros = { 0x000000_u24 };
	};

	template <>
	struct type_trait<int24> final : ce_only
	{
		constexpr static const char name[] = "int24";

		using type = int24;
		using signed_type = type;
		using unsigned_type = uint24;
		using larger_type = int32;
		using smaller_type = int16;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = true;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = true;
		constexpr static bool is_float = false;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = false;
		constexpr static bool is_emulated = true;

		constexpr inline static unsigned_type as_unsigned(arg_type<type> val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(arg_type<type> val) { return val; }
		constexpr inline static signed_type as_signed(arg_type<type> val) { return { val }; }

		constexpr static type max = { 0x7FFFFF_i24 };
		constexpr static type min = { 0x800000_i24 };
		constexpr static type ones = { 0xFFFFFF_i24 };
		constexpr static type zeros = { 0x000099_i24 };
	};

	template <>
	struct type_trait<uint32> final : ce_only
	{
		constexpr static const char name[] = "uint32";

		using type = uint32;
		using signed_type = int32;
		using unsigned_type = type;
		using larger_type = uint64;
		using smaller_type = uint16;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = false;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = true;
		constexpr static bool is_float = false;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = false;
		constexpr static bool is_emulated = true;

		constexpr inline static unsigned_type as_unsigned(arg_type<type> val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(arg_type<type> val) { return { val }; }
		constexpr inline static signed_type as_signed(arg_type<type> val) { return val; }

		constexpr static type max = { 0xFFFFFFFF_u32 };
		constexpr static type min = { 0x00000000_u32 };
		constexpr static type ones = { 0xFFFFFFFF_u32 };
		constexpr static type zeros = { 0x00000000_u32 };
	};

	template <>
	struct type_trait<int32> final : ce_only
	{
		constexpr static const char name[] = "int32";

		using type = int32;
		using signed_type = type;
		using unsigned_type = uint32;
		using larger_type = int64;
		using smaller_type = int16;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = true;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = true;
		constexpr static bool is_float = false;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = false;
		constexpr static bool is_emulated = true;

		constexpr inline static unsigned_type as_unsigned(arg_type<type> val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(arg_type<type> val) { return val; }
		constexpr inline static signed_type as_signed(arg_type<type> val) { return { val }; }

		constexpr static type max = { 0x7FFFFFFF_i32 };
		constexpr static type min = { 0x80000000_i32 };
		constexpr static type ones = { 0xFFFFFFFF_i32 };
		constexpr static type zeros = { 0x00000000_i32 };
	};

	template <>
	struct type_trait<uint64> final : ce_only
	{
		constexpr static const char name[] = "uint64";

		using type = uint64;
		using signed_type = int64;
		using unsigned_type = type;
		using larger_type = void;
		using smaller_type = uint32;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = false;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = true;
		constexpr static bool is_float = false;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = false;
		constexpr static bool is_emulated = true;

		constexpr inline static unsigned_type as_unsigned(arg_type<type> val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(arg_type<type> val) { return { val }; }
		constexpr inline static signed_type as_signed(arg_type<type> val) { return val; }

		constexpr static type max = { 0xFFFFFFFFFFFFFFFF_u64 };
		constexpr static type min = { 0x0000000000000000_u64 };
		constexpr static type ones = { 0xFFFFFFFFFFFFFFFF_u64 };
		constexpr static type zeros = { 0x0000000000000000_u64 };
	};

	template <>
	struct type_trait<int64> final : ce_only
	{
		constexpr static const char name[] = "int64";

		using type = int64;
		using signed_type = type;
		using unsigned_type = uint64;
		using larger_type = void;
		using smaller_type = int32;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = true;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = true;
		constexpr static bool is_float = false;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = false;
		constexpr static bool is_emulated = true;

		constexpr inline static unsigned_type as_unsigned(arg_type<type> val) { return val; }
		constexpr inline static signed_type as_signed(arg_type<type> val) { return { val }; }

		constexpr static type max = { 0x7FFFFFFFFFFFFFFF_i64 };
		constexpr static type min = { 0x8000000000000000_i64 };
		constexpr static type ones = { 0xFFFFFFFFFFFFFFFF_i64 };
		constexpr static type zeros = { 0x0000000000000000_i64 };
	};

	template <>
	struct type_trait<float> : ce_only
	{
		constexpr static const char name[] = "float";

		using type = float;
		using signed_type = type;
		using unsigned_type = void;
		using larger_type = void;
		using smaller_type = void;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 4;

		constexpr static bool is_signed = true;
		constexpr static bool is_unsigned = !is_signed;
		constexpr static bool is_integral = false;
		constexpr static bool is_float = true;
		constexpr static bool is_primitive = true;
		constexpr static bool is_array = false;
		constexpr static bool is_pointer = false;
		constexpr static bool is_reference = false;
		constexpr static bool is_atomic = false;
		constexpr static bool is_emulated = true;

		constexpr inline static signed_type as_signed(arg_type<type> val) { return { val }; }

		constexpr static type max = { __builtin_huge_valf() };
		constexpr static type min = { 1.175494e-38f };
    constexpr static type epsilon = { 1.192093e-07f };
	};
	// On AVR, float == double == long double, though the compiler doesn't always agree due to strict typing.
	template <> struct type_trait<double> final : type_trait<float> {};
	template <> struct type_trait<long double> final : type_trait<float> {};

	template <typename T, typename R = typename type_trait<typename type_trait<T>::unsigned_type>::smaller_type>
	constexpr inline R hi(arg_type<T> value)
	{
		constexpr uint8 shift = (sizeof(R) * 8);

		return type_trait<T>::as_unsigned(value) >> shift;
	}

	template <typename T, typename R = typename type_trait<typename type_trait<T>::unsigned_type>::smaller_type>
	constexpr inline R lo(arg_type<T> value)
	{
		return type_trait<T>::as_unsigned(value);
	}

	class alignas(uint8) critical_section final
	{
		const uint8 m_sReg = SREG;
	public:
		critical_section(const critical_section &) = delete;
		critical_section(critical_section &&) = delete;
		critical_section & operator = (const critical_section &) = delete;
		critical_section & operator = (critical_section &&) = delete;

		inline critical_section()
		{
			cli();
		}
		inline ~critical_section()
		{
			SREG = m_sReg;
		}
	};

  class critical_section_not_isr final
  {
  public:
    critical_section_not_isr(const critical_section_not_isr &) = delete;
    critical_section_not_isr(critical_section_not_isr &&) = delete;
    critical_section_not_isr & operator = (const critical_section_not_isr &) = delete;
    critical_section_not_isr & operator = (critical_section_not_isr &&) = delete;

    inline critical_section_not_isr()
    {
      cli();
    }
    inline ~critical_section_not_isr()
    {
      sei();
    }
  };

	template <uint64 v, bool _canary = false, uint8 r = 0, bool pow2 = true>
	constexpr uint8 _ce_log2()
	{
		static_assert(!_canary, "Please do not use parameters past the first in ce_log2. The rest are used internally.");
		if constexpr (v != 0)
		{
			constexpr uint64 new_v = v >> 1;
			return _ce_log2<new_v, _canary, r + 1, (new_v != 0 && (v & 1) != 0) ? false : pow2>();
		}
		else
		{
			return r - (pow2 ? 1 : 0);
		}
	}

	template <uint64 v> constexpr uint8 ce_log2 = _ce_log2<v>();

	template <uint64 value>
	class _uintsz final : ce_only
	{
		constexpr static auto typer()
		{
			if constexpr (value <= type_trait<uint8>::max)
			{
				return uint8{};
			}
			else if constexpr (value <= type_trait<uint16>::max)
			{
				return uint16{};
			}
			else if constexpr (value <= type_trait<uint24>::max)
			{
				return uint24{};
			}
			else if constexpr (value <= type_trait<uint32>::max)
			{
				return uint32{};
			}
			else if constexpr (value <= type_trait<uint64>::max)
			{
				return uint64{};
			}
		}
	public:
		using type = decltype(typer());
	};

	template <uint64 value> using uintsz = typename _uintsz<value>::type;

  template <uint64 value> constexpr auto make_uintsz = uintsz<value>{ value };

  extern uint32 millis32();
	extern uint24 millis24();
	extern uint16 millis16();
  extern uint8 millis8();

  template <typename T>
  struct flash_ptr final
  {
  public:
    using ptr_t = uint16; // TODO handle > 16-bit ptrs
  private:
    const uint16  m_Ptr;
  public:
    constexpr flash_ptr(arg_type<T> value) : m_Ptr(uint16(&value)) {}

    constexpr inline operator uint16 () const __restrict
    {
      return m_Ptr;
    }
  };

  template <typename U>
  static inline U read_pgm_ptr(uint16 ptr)
  {
    U retValue;// = pgm_read_word(ptr);
    //return retValue;

    if constexpr (sizeof(U) == 1)
    {
      __asm__ __volatile__
      (
        "lpm %0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 2)
    {
      __asm__ __volatile__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 3)
    {
      __asm__ __volatile__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 4)
    {
      __asm__ __volatile__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z+" "\n\t"
        "lpm %D0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 5)
    {
      __asm__ __volatile__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z+" "\n\t"
        "lpm %D0, Z+" "\n\t"
        "lpm %E0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 6)
    {
      __asm__ __volatile__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z+" "\n\t"
        "lpm %D0, Z+" "\n\t"
        "lpm %E0, Z+" "\n\t"
        "lpm %F0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 7)
    {
      __asm__ __volatile__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z+" "\n\t"
        "lpm %D0, Z+" "\n\t"
        "lpm %E0, Z+" "\n\t"
        "lpm %F0, Z+" "\n\t"
        "lpm %G0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 8)
    {
      __asm__ __volatile__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z+" "\n\t"
        "lpm %D0, Z+" "\n\t"
        "lpm %E0, Z+" "\n\t"
        "lpm %F0, Z+" "\n\t"
        "lpm %G0, Z+" "\n\t"
        "lpm %H0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) > 8)
    {
      uint8 *retValuePtr = (uint8 *)&retValue;
      constexpr uint8 type_size = sizeof(U);
      // TODO : we should hand-optimize this routine, as this is suboptimal by far.

      constexpr uint8 qwords = type_size / 8;

      for (uint8 i = 0; i < qwords; ++i)
      {
        auto &val = *(uint64 *)retValuePtr;
        __asm__ __volatile__
        (
          "lpm %A0, Z+" "\n\t"
          "lpm %B0, Z+" "\n\t"
          "lpm %C0, Z+" "\n\t"
          "lpm %D0, Z+" "\n\t"
          "lpm %E0, Z+" "\n\t"
          "lpm %F0, Z+" "\n\t"
          "lpm %G0, Z+" "\n\t"
          "lpm %H0, Z" "\n\t"
          : "=r" (val), "=z" (ptr)
          : "1" (ptr)
        );
        retValuePtr += 8;
        ptr += 8;
      }
      if constexpr (type_size & 4)
      {
        auto &val = *(uint32 *)retValuePtr;
        __asm__ __volatile__
        (
          "lpm %A0, Z+" "\n\t"
          "lpm %B0, Z+" "\n\t"
          "lpm %C0, Z+" "\n\t"
          "lpm %D0, Z" "\n\t"
          : "=r" (val), "=z" (ptr)
          : "1" (ptr)
        );
        retValuePtr += 4;
        ptr += 4;
      }
      if constexpr (type_size & 2)
      {
        auto &val = *(uint16 *)retValuePtr;
        __asm__ __volatile__
        (
          "lpm %A0, Z+" "\n\t"
          "lpm %B0, Z" "\n\t"
          : "=r" (val), "=z" (ptr)
          : "1" (ptr)
        );
        retValuePtr += 2;
        ptr += 2;
      }
      if constexpr (type_size & 1)
      {
        auto &val = *(uint8 *)retValuePtr;
        __asm__ __volatile__
        (
          "lpm %A0, Z" "\n\t"
          : "=r" (val), "=z" (ptr)
          : "1" (ptr)
        );
      }
    }
    return retValue;
  }

  template <typename U, typename T>
  static inline U read_pgm(arg_type<flash_ptr<T>> value)
  {
    return read_pgm_ptr<U>(uint16(value));
  }

  template <typename U, typename T>
  static inline U read_pgm(arg_type<T> value)
  {
    return read_pgm_ptr<U>(uint16(&value));
  }


  template <typename T>
  class alignas(T) flash final
  {
    const T m_Value = T{};

    static inline T _read_pgm(arg_type<T> value)
    {
      return read_pgm<T>(value);
    }

  public:
    using type = T;

    constexpr flash() = default;
    constexpr flash(arg_type<flash> data) : m_Value(data.m_Value) {}
    constexpr flash(arg_type<T> value) : m_Value(value) {}

    template <typename U = T>
    constexpr T get() const __restrict
    {
      if (__builtin_constant_p(m_Value))
      {
        return m_Value;
      }
      else
      {
        return _read_pgm(m_Value);
      }
    }

    //constexpr operator T () const
    //{
    //  return get();
    //}

    template <typename U>
    constexpr operator U () const __restrict
    {
      static_assert(sizeof(T) <= sizeof(U), "Cannot extract pgm value larger than declared storage.");
      return get<U>();
    }

    constexpr bool operator == (arg_type<flash> other) const __restrict
    {
      return get() == other.get();
    }

    constexpr bool operator != (arg_type<flash> other) const __restrict
    {
      return get() != other.get();
    }

    constexpr bool operator > (arg_type<flash> other) const __restrict
    {
      return get() > other.get();
    }

    constexpr bool operator >= (arg_type<flash> other) const __restrict
    {
      return get() >= other.get();
    }

    constexpr bool operator < (arg_type<flash> other) const __restrict
    {
      return get() < other.get();
    }

    constexpr bool operator <= (arg_type<flash> other) const __restrict
    {
      return get() <= other.get();
    }

    template <typename U>
    constexpr bool operator == (arg_type<U> other) const __restrict
    {
      return get() == other;
    }

    template <typename U>
    constexpr bool operator != (arg_type<U> other) const __restrict
    {
      return get() != other;
    }

    template <typename U>
    constexpr bool operator > (arg_type<U> other) const __restrict
    {
      return get() > other;
    }

    template <typename U>
    constexpr bool operator >= (arg_type<U> other) const __restrict
    {
      return get() >= other;
    }

    template <typename U>
    constexpr bool operator < (arg_type<U> other) const __restrict
    {
      return get() < other;
    }

    template <typename U>
    constexpr bool operator <= (arg_type<U> other) const __restrict
    {
      return get() <= other;
    }

    constexpr T operator + (arg_type<flash> other) const __restrict
    {
      return get() + other.get();
    }

    constexpr T operator - (arg_type<flash> other) const __restrict
    {
      return get() - other.get();
    }

    constexpr T operator / (arg_type<flash> other) const __restrict
    {
      return get() / other.get();
    }

    constexpr T operator * (arg_type<flash> other) const __restrict
    {
      return get() * other.get();
    }

    constexpr T operator % (arg_type<flash> other) const __restrict
    {
      return get() % other.get();
    }

    constexpr T operator >> (arg_type<flash> other) const __restrict
    {
      return get() >> other.get();
    }

    constexpr T operator << (arg_type<flash> other) const __restrict
    {
      return get() << other.get();
    }

    template <typename U>
    constexpr T operator + (arg_type<U> other) const __restrict
    {
      return get() + other;
    }

    template <typename U>
    constexpr T operator - (arg_type<U> other) const __restrict
    {
      return get() - other;
    }

    template <typename U>
    constexpr T operator / (arg_type<U> other) const __restrict
    {
      return get() / other;
    }

    template <typename U>
    constexpr T operator * (arg_type<U> other) const __restrict
    {
      return get() * other;
    }

    template <typename U>
    constexpr T operator % (arg_type<U> other) const __restrict
    {
      return get() % other;
    }

    template <typename U>
    constexpr T operator >> (arg_type<U> other) const __restrict
    {
      return get() >> other;
    }

    template <typename U>
    constexpr T operator << (arg_type<U> other) const __restrict
    {
      return get() << other;
    }
  };

  template<typename T, size_t N>
  constexpr inline size_t array_size(T(&)[N])
  {
    return N;
  }

  // Simple static-cast like routine. Also makes extracting sub-types easier from encapsulations like 'flash'.
  template <typename T = void, typename U>
  constexpr inline auto as(arg_type<U> value)
  {
    if constexpr (is_same<T, void>)
    {
      if constexpr (type_trait<U>::is_primitive)
      {
        return U(value);
      }
      else
      {
        using type = typename U::type;
        return type(value);
      }
    }
    else
    {
      return T(value);
    }
  }

  class flash_string final
  {
    const char *m_Str = nullptr;
  public:
    constexpr flash_string() = default;
    constexpr flash_string(const char * __restrict str) : m_Str(str) {}
    constexpr flash_string(arg_type<flash_string> str) : m_Str(str.m_Str) {}

    constexpr flash_string & __restrict operator = (const char * __restrict str) __restrict
    {
      m_Str = str;
      return *this;
    }

    constexpr flash_string & __restrict operator = (arg_type<flash_string> str) __restrict
    {
      m_Str = str.m_Str;
      return *this;
    }

    constexpr operator bool() const __restrict
    {
      return m_Str != nullptr;
    }

    constexpr bool operator == (arg_type<flash_string> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    constexpr bool operator != (arg_type<flash_string> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    constexpr const char * __restrict c_str() const __restrict
    {
      return m_Str;
    }

    // This implicit conversion exists for Arduino SDK support.
    constexpr operator const __FlashStringHelper * __restrict () const __restrict
    {
      return (const __FlashStringHelper * __restrict)m_Str;
    }

    constexpr auto fsh() const __restrict
    {
      return (const __FlashStringHelper * __restrict)m_Str;
    }
  };

  template <size_t LEN>
  class flash_char_array final
  {
    const char * const m_Str;

  public:
    static constexpr const auto length = make_uintsz<LEN>;

    constexpr flash_char_array(const char * const __restrict str) : m_Str(str) {}

    constexpr operator flash_string () const __restrict
    {
      return { m_Str };
    }

    constexpr operator bool() const __restrict
    {
      return m_Str != nullptr;
    }

    constexpr bool operator == (arg_type<flash_string> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    constexpr bool operator != (arg_type<flash_string> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    template <size_t U>
    constexpr bool operator == (arg_type<flash_char_array<U>> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    template <size_t U>
    constexpr bool operator != (arg_type<flash_char_array<U>> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    constexpr const char * __restrict c_str() const __restrict
    {
      return m_Str;
    }

    // This implicit conversion exists for Arduino SDK support.
    constexpr operator const __FlashStringHelper * __restrict () const __restrict
    {
      return (const __FlashStringHelper * __restrict)m_Str;
    }

    constexpr auto fsh() const __restrict
    {
      return (const __FlashStringHelper * __restrict)m_Str;
    }
  };

  namespace _internal
  {
    template <char... Chars>
    struct progmem_str_store final : ce_only
    {
      static constexpr const char str[] PROGMEM = { Chars..., '\0' };
    };
  }

  template <typename T, T... Chars>
  constexpr flash_char_array<sizeof...(Chars)> operator "" _p()
  {
    static_assert(is_same<T, char>, "_p must be used with 'char'");
    //static const char str[] PROGMEM = { Chars..., '\0' };
    return { _internal::progmem_str_store<Chars...>::str };
  }

  template <uint8 SerialNumber = 0>
  struct serial final
  {
#if defined(SerialUSB)
    constexpr static uint8 USB = type_trait<uint8>::max;
#endif

    serial() = delete;

    constexpr static uint8 number = SerialNumber;

    constexpr static auto & __restrict get_serial_device()
    {
      switch (number)
      {
      case 0:
        return Serial;
      case 1:
        return Serial1;
      case 2:
        return Serial2;
      case 3:
        return Serial3;
#if defined(SerialUSB)
      case USB:
        return SerialUSB;
#endif
      }
    }

    template <uint32 baud>
    static inline void begin()
    {
      get_serial_device().begin(baud);
    }

    template <typename T>
    constexpr static inline uint8 write_struct(arg_type<T> obj)
    {
      return get_serial_device().write((const uint8 * __restrict)&obj, sizeof(T));
    }

    template <typename T, uint8 N>
    constexpr static inline uint8 write(const T(&__restrict buffer)[N])
    {
      return get_serial_device().write((const uint8 * __restrict)buffer, N);
    }

    template <size_t N>
    constexpr static inline uint8 write(const flash_char_array<N> & __restrict str)
    {
      return get_serial_device().write((const uint8 * __restrict)str.c_str(), N + 1);
    }

    constexpr static inline uint8 write(arg_type<flash_string> str, const uint8 length)
    {
      return get_serial_device().write((const uint8 * __restrict)str.c_str(), length);
    }

    template <typename T>
    constexpr static inline uint8 write(arg_type<T> buffer, const uint8 length)
    {
      return get_serial_device().write((const uint8 * __restrict)buffer, length);
    }

    template <typename T, uint8 N>
    constexpr static inline uint8 read_bytes(T(&__restrict buffer)[N])
    {
      return get_serial_device().readBytes((uint8 * __restrict)buffer, N);
    }

    template <typename T>
    constexpr static inline uint8 read_bytes(T * __restrict buffer, const uint8 length)
    {
      return get_serial_device().readBytes((uint8 * __restrict)buffer, length);
    }

    static inline uint available()
    {
      return get_serial_device().available();
    }

    static inline bool available(uint8 length)
    {
      return uint8(min(get_serial_device().available(), 255)) >= length;
    }

    // TODO handle -1. Didn't want this to be an int.
    static inline uint8 read()
    {
      return get_serial_device().read();
    }
  };

  template <typename T, typename U>
  struct pair final
  {
    T first;
    U second;

    constexpr pair() = default;
    constexpr pair(const T & __restrict _1) : first(_1) {}
    constexpr pair(const T & __restrict _1, const U & __restrict _2) : first(_1), second(_2) {}
    constexpr pair(const pair & __restrict other) : first(other.first), second(other.second) {}

    constexpr operator bool() const __restrict
    {
      return bool(first);
    }
  };

  template <typename T, typename U>
  constexpr inline pair<T, U> make_pair(const T & __restrict first, const U & __restrict second)
  {
    return { first, second };
  }

  template <typename T, typename U>
  struct ref_pair final
  {
    const T & __restrict first;
    const U & __restrict second;

    constexpr ref_pair() = default;
    constexpr ref_pair(const T & __restrict _1) : first(_1) {}
    constexpr ref_pair(const T & __restrict _1, const U & __restrict _2) : first(_1), second(_2) {}
    constexpr ref_pair(const ref_pair & __restrict other) : first(other.first), second(other.second) {}

    constexpr operator bool() const __restrict
    {
      return bool(first);
    }
  };

  template <typename T, typename U>
  constexpr inline ref_pair<T, U> make_ref_pair(const T & __restrict first, const U & __restrict second)
  {
    return { first, second };
  }

	// WIP
#if 0
	template <typename T>
	alignas(T) class __flash final
	{
		T PROGMEM m_Data;
	public:
		constexpr __flash() = default;
		constexpr __flash(const __flash &data) : m_Data(data.m_Data) {}
		constexpr __flash(const T &data) : m_Data(data) {}

		constexpr operator T () const
		{
			if constexpr (sizeof(T) == 1)
			{
				return (T &)pgm_read_byte((uint16)&m_Data);
			}
			else if constexpr (sizeof(T) == 2)
			{
				return (T &)pgm_read_word((uint16)&m_Data);
			}
			else if constexpr (sizeof(T) == 3)
			{
				return pgm_read_word((uint16)&m_Data);
			}
		}
	};
#endif
}

namespace Tuna
{
	using namespace utils;
}

// TODO remove when done
using namespace Tuna;

