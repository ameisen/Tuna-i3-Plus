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

namespace tuna
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

namespace tuna::utils
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

	using namespace tuna;

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

	constexpr inline __attribute__((always_inline)) void delay_cycles(uint32 ticks)
	{
		__builtin_avr_delay_cycles(ticks);
	}

	constexpr inline __attribute__((always_inline)) uint8 insert_bits(uint32 map, uint8 bits, uint8 val)
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
	constexpr inline const T & __restrict max(const T & __restrict a, const T & __restrict b)
	{
		return (a >= b) ? a : b;
	}

	template <typename T>
	constexpr inline const T & __restrict min(const T & __restrict a, const T & __restrict b)
	{
		return (a < b) ? a : b;
	}

	template <typename T>
	constexpr inline T clamp(const T val, const T _min, const T _max)
	{
		return min(max(val, _min), _max);
	}

	template <typename T>
	class type_trait;

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

		constexpr inline static unsigned_type as_unsigned(type val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(type val) { return { val }; }

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

		constexpr inline static unsigned_type as_unsigned(type val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(type val) { return { val }; }
		constexpr inline static signed_type as_signed(type val) { return val; }

		constexpr static type max = { 0xFF };
		constexpr static type min = { 0x00 };
		constexpr static type ones = { 0xFF };
		constexpr static type zeros = { 0x00 };
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

		constexpr inline static unsigned_type as_unsigned(type val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(type val) { return val; }
		constexpr inline static signed_type as_signed(type val) { return { val }; }

		constexpr static type max = { 127 };
		constexpr static type min = { -128 };
		constexpr static type ones = { unsigned_type{0xFF} };
		constexpr static type zeros = { unsigned_type{0x00} };
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

		constexpr inline static unsigned_type as_unsigned(type val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(type val) { return { val }; }
		constexpr inline static signed_type as_signed(type val) { return val; }

		constexpr static type max = { 0xFFFF };
		constexpr static type min = { 0x0000 };
		constexpr static type ones = { 0xFFFF };
		constexpr static type zeros = { 0x0000 };
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

		constexpr inline static unsigned_type as_unsigned(type val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(type val) { return val; }
		constexpr inline static signed_type as_signed(type val) { return { val }; }

		constexpr static type max = { 32767 };
		constexpr static type min = { -32768 };
		constexpr static type ones = { unsigned_type{ 0xFFFF } };
		constexpr static type zeros = { unsigned_type{ 0x0000 } };
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

		constexpr inline static unsigned_type as_unsigned(type val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(type val) { return { val }; }
		constexpr inline static signed_type as_signed(type val) { return val; }

		constexpr static type max = { 0xFFFFFF };
		constexpr static type min = { 0x000000 };
		constexpr static type ones = { 0xFFFFFF };
		constexpr static type zeros = { 0x000000 };
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

		constexpr inline static unsigned_type as_unsigned(type val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(type val) { return val; }
		constexpr inline static signed_type as_signed(type val) { return { val }; }

		constexpr static type max = { 16777215 };
		constexpr static type min = { -16777216 };
		constexpr static type ones = { unsigned_type{ 0xFFFFFF } };
		constexpr static type zeros = { unsigned_type{ 0x000099 } };
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

		constexpr inline static unsigned_type as_unsigned(type val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(type val) { return { val }; }
		constexpr inline static signed_type as_signed(type val) { return val; }

		constexpr static type max = { 0xFFFFFFFF };
		constexpr static type min = { 0x00000000 };
		constexpr static type ones = { 0xFFFFFFFF };
		constexpr static type zeros = { 0x00000000 };
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

		constexpr inline static unsigned_type as_unsigned(type val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(type val) { return val; }
		constexpr inline static signed_type as_signed(type val) { return { val }; }

		constexpr static type max = { 2147483647 };
		constexpr static type min = { -2147483648 };
		constexpr static type ones = { unsigned_type{ 0xFFFFFFFF } };
		constexpr static type zeros = { unsigned_type{ 0x00000000 } };
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

		constexpr inline static unsigned_type as_unsigned(type val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(type val) { return { val }; }
		constexpr inline static signed_type as_signed(type val) { return val; }

		constexpr static type max = { 0xFFFFFFFFFFFFFFFF };
		constexpr static type min = { 0x0000000000000000 };
		constexpr static type ones = { 0xFFFFFFFFFFFFFFFF };
		constexpr static type zeros = { 0x0000000000000000 };
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

		constexpr inline static unsigned_type as_unsigned(type val) { return val; }
		constexpr inline static signed_type as_signed(type val) { return { val }; }

		constexpr static type max = { 9'223'372'036'854'775'807 };
		constexpr static type min = { -9'223'372'036'854'775'808 };
		constexpr static type ones = { 0xFFFFFFFFFFFFFFFF };
		constexpr static type zeros = { 0x0000000000000000 };
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

		constexpr inline static signed_type as_signed(type val) { return { val }; }

		//constexpr static type max = { FLT_MAX };
		//constexpr static type min = { FLT_MIN };
	};
	// On AVR, float == double == long double, though the compiler doesn't always agree due to strict typing.
	template <> struct type_trait<double> final : type_trait<float> {};
	template <> struct type_trait<long double> final : type_trait<float> {};

	template <typename T, typename R = typename type_trait<typename type_trait<T>::unsigned_type>::smaller_type>
	constexpr inline R hi(T value)
	{
		constexpr uint8 shift = (sizeof(R) * 8);

		return type_trait<T>::as_unsigned(value) >> shift;
	}

	template <typename T, typename R = typename type_trait<typename type_trait<T>::unsigned_type>::smaller_type>
	constexpr inline R lo(T value)
	{
		return type_trait<T>::as_unsigned(value);
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
		constexpr static inline uint8 write_struct(const T & __restrict obj)
		{
			return get_serial_device().write((const uint8 * __restrict)&obj, sizeof(T));
		}

		template <typename T, uint8 N>
		constexpr static inline uint8 write(const T(&__restrict buffer)[N])
		{
			return get_serial_device().write((const uint8 * __restrict )buffer, N);
		}

		template <typename T>
		constexpr static inline uint8 write(const T * __restrict buffer, const uint8 length)
		{
			return get_serial_device().write((const uint8 * __restrict)buffer, length);
		}

		template <typename T, uint8 N>
		constexpr static inline uint8 read_bytes(T (&__restrict buffer)[N])
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

	extern uint24 millis24();
	extern uint16 millis16();

  template <typename T>
  alignas(T) class flash final
  {
    T PROGMEM m_Value = T{};

    // Put ASM into another function... which works for some reason.
    T rt_getter() const
    {
      uint16 ptr = (uint16)&m_Value;

      if constexpr (sizeof(T) == 1)
      {
        return (const T &)pgm_read_byte(ptr);
      }
      else if constexpr (sizeof(T) == 2)
      {
        return (const T &)pgm_read_word(ptr);
      }
      else if constexpr (sizeof(T) > 2)
      {
        T retValue;
        uint8 *retValuePtr = (uint8 *)&retValue;
        constexpr uint8 type_size = sizeof(T);
        // TODO : we should hand-optimize this routine, as this is suboptimal by far.

        constexpr uint8 dwords = type_size / 4;

        for (uint8 i = 0; i < dwords; ++i)
        {
          *(uint32 *)retValuePtr = pgm_read_dword(ptr);
          retValuePtr += 4;
          ptr += 4;
        }

        if constexpr (type_size & 2)
        {
          *(uint16 *)retValuePtr = pgm_read_word(ptr);
          retValuePtr += 2;
          ptr += 2;
        }
        if constexpr (type_size & 1)
        {
          *(uint8 *)retValuePtr = pgm_read_byte(ptr);
        }

        return retValue;
      }
    }

  public:
    constexpr flash() = default;
    constexpr flash(const flash &data) : m_Value(data.m_Value) {}
    constexpr flash(const T &value) : m_Value(value) {}

    constexpr operator T () const
    {
      if (__builtin_constant_p(m_Value))
      {
        return m_Value;
      }
      else
      {
        return rt_getter();
      }
    }
  };

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

namespace tuna
{
	using namespace utils;
}

// TODO remove when done
using namespace tuna;

