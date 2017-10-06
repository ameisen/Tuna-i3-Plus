#pragma once

#include <stdint.h>

#include "HardwareSerial.h"
#include "avr/interrupt.h"

#ifdef min
#	undef min
#endif
#ifdef max
#	undef max
#endif
#ifdef clamp
#	undef clamp
#endif

namespace tuna
{
	using uint8 = uint8_t;
	using uint16 = uint16_t;
	using uint32 = uint32_t;
	using uint64 = uint64_t;
	using int8 = int8_t;
	using int16 = int16_t;
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
	using namespace tuna;

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
	struct type_trait<uint8> final
	{
		type_trait() = delete;

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

		constexpr inline static unsigned_type as_unsigned(type val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(type val) { return { val }; }
		constexpr inline static signed_type as_signed(type val) { return val; }

		constexpr static type max = { 0xFF };
		constexpr static type min = { 0x00 };
		constexpr static type ones = { 0xFF };
		constexpr static type zeros = { 0x00 };
	};

	template <>
	struct type_trait<int8> final
	{
		type_trait() = delete;

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

		constexpr inline static unsigned_type as_unsigned(type val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(type val) { return val; }
		constexpr inline static signed_type as_signed(type val) { return { val }; }

		constexpr static type max = { 127 };
		constexpr static type min = { -128 };
		constexpr static type ones = { unsigned_type{0xFF} };
		constexpr static type zeros = { unsigned_type{0x00} };
	};

	template <>
	struct type_trait<uint16> final
	{
		type_trait() = delete;

		constexpr static const char name[] = "uint16";

		using type = uint16;
		using signed_type = int16;
		using unsigned_type = type;
		using larger_type = uint32;
		using smaller_type = uint8;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = false;
		constexpr static bool is_unsigned = !is_signed;

		constexpr inline static unsigned_type as_unsigned(type val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(type val) { return { val }; }
		constexpr inline static signed_type as_signed(type val) { return val; }

		constexpr static type max = { 0xFFFF };
		constexpr static type min = { 0x0000 };
		constexpr static type ones = { 0xFFFF };
		constexpr static type zeros = { 0x0000 };
	};

	template <>
	struct type_trait<int16> final
	{
		type_trait() = delete;

		constexpr static const char name[] = "int16";

		using type = int16;
		using signed_type = type;
		using unsigned_type = uint16;
		using larger_type = int32;
		using smaller_type = int8;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = true;
		constexpr static bool is_unsigned = !is_signed;

		constexpr inline static unsigned_type as_unsigned(type val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(type val) { return val; }
		constexpr inline static signed_type as_signed(type val) { return { val }; }

		constexpr static type max = { 32767 };
		constexpr static type min = { -32768 };
		constexpr static type ones = { unsigned_type{ 0xFFFF } };
		constexpr static type zeros = { unsigned_type{ 0x0000 } };
	};

	template <>
	struct type_trait<const int16> final
	{
		type_trait() = delete;

		constexpr static const char name[] = "int16";

		using type = int16;
		using signed_type = type;
		using unsigned_type = uint16;
		using larger_type = int32;
		using smaller_type = int8;

		constexpr static uint8 size = sizeof(type);
		constexpr static uint8 bits = sizeof(type) * 8;

		constexpr static bool is_signed = true;
		constexpr static bool is_unsigned = !is_signed;

		constexpr inline static unsigned_type as_unsigned(type val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(type val) { return val; }
		constexpr inline static signed_type as_signed(type val) { return { val }; }

		constexpr static type max = { 32767 };
		constexpr static type min = { -32768 };
		constexpr static type ones = { unsigned_type{ 0xFFFF } };
		constexpr static type zeros = { unsigned_type{ 0x0000 } };
	};

	template <>
	struct type_trait<uint32> final
	{
		type_trait() = delete;

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

		constexpr inline static unsigned_type as_unsigned(type val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(type val) { return { val }; }
		constexpr inline static signed_type as_signed(type val) { return val; }

		constexpr static type max = { 0xFFFFFFFF };
		constexpr static type min = { 0x00000000 };
		constexpr static type ones = { 0xFFFFFFFF };
		constexpr static type zeros = { 0x00000000 };
	};

	template <>
	struct type_trait<int32> final
	{
		type_trait() = delete;

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

		constexpr inline static unsigned_type as_unsigned(type val) { return val; }
		constexpr inline static typename type_trait<unsigned_type>::larger_type as_safe_unsigned(type val) { return val; }
		constexpr inline static signed_type as_signed(type val) { return { val }; }

		constexpr static type max = { 2147483647 };
		constexpr static type min = { -2147483648 };
		constexpr static type ones = { unsigned_type{ 0xFFFFFFFF } };
		constexpr static type zeros = { unsigned_type{ 0x00000000 } };
	};

	template <>
	struct type_trait<uint64> final
	{
		type_trait() = delete;

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

		constexpr inline static unsigned_type as_unsigned(type val) { return { val }; }
		constexpr inline static unsigned_type as_safe_unsigned(type val) { return { val }; }
		constexpr inline static signed_type as_signed(type val) { return val; }

		constexpr static type max = { 0xFFFFFFFFFFFFFFFF };
		constexpr static type min = { 0x0000000000000000 };
		constexpr static type ones = { 0xFFFFFFFFFFFFFFFF };
		constexpr static type zeros = { 0x0000000000000000 };
	};

	template <>
	struct type_trait<int64> final
	{
		type_trait() = delete;

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

		constexpr inline static unsigned_type as_unsigned(type val) { return val; }
		constexpr inline static signed_type as_signed(type val) { return { val }; }

		constexpr static type max = { 9'223'372'036'854'775'807 };
		constexpr static type min = { -9'223'372'036'854'775'808 };
		constexpr static type ones = { 0xFFFFFFFFFFFFFFFF };
		constexpr static type zeros = { 0x0000000000000000 };
	};

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

}

// TODO remove when done
using namespace tuna;
using namespace tuna::utils;
