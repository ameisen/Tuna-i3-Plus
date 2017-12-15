#pragma once

namespace Tuna
{
  namespace _internal
  {
    template <typename T>
    struct base_type_trait : trait::ce_only
    {
      // Default
      constexpr static const char name[] = "unknown";

      using type = T;
      using underlying_type = T;

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
      constexpr static bool is_enuum = false;
    };

    template <typename T>
    struct enum_type_trait;

    template <typename T, bool is_enum = __is_enum(T)>
    struct base_type_trait_selector;

    template <typename T>
    struct base_type_trait_selector<T, true> final : trait::ce_only
    {
      using type = enum_type_trait<T>;
    };

    template <typename T>
    struct base_type_trait_selector<T, false> final : trait::ce_only
    {
      using type = base_type_trait<T>;
    };
  }

  template <typename T>
  struct type_trait final : _internal::base_type_trait_selector<T>::type
  {
  };

  template <>
  struct type_trait<void> final : _internal::base_type_trait<void>
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
  struct type_trait<bool> final : _internal::base_type_trait<bool>
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

    constexpr inline static unsigned_type __forceinline __flatten as_unsigned(arg_type<type> val) { return { val }; }
    constexpr inline static unsigned_type __forceinline __flatten as_safe_unsigned(arg_type<type> val) { return { val }; }

    // We really don't want to be able to generate bools with values that aren't 0x00 and 0x01... it's
    // perfectly possible but is likely to confuse the runtime due to UB.
    constexpr static type max = { true };
    constexpr static type min = { false };
    constexpr static type ones = { true };
    constexpr static type zeros = { false };
  };

  template <>
  struct type_trait<uint8> final : _internal::base_type_trait<uint8>
  {
    constexpr static const char name[] = "uint8";

    using type = uint8;
    using signed_type = int8;
    using unsigned_type = type;
    using larger_type = uint16;
    using smaller_type = void;
    using faster_type = type;

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

    constexpr inline static unsigned_type __forceinline __flatten as_unsigned(arg_type<type> val) { return { val }; }
    constexpr inline static unsigned_type __forceinline __flatten as_safe_unsigned(arg_type<type> val) { return { val }; }
    constexpr inline static signed_type __forceinline __flatten as_signed(arg_type<type> val) { return val; }

    constexpr static type max = { 0xFF_u8 };
    constexpr static type min = { 0x00_u8 };
    constexpr static type ones = { 0xFF_u8 };
    constexpr static type zeros = { 0x00_u8 };

    template <uint8 base>
    constexpr static usize string_length =
      (base == 2) ? 8 :
      (base == 8) ? 3 :
      (base == 10) ? 3 :
      (base == 16) ? 2 :
      0;
  };

  template <>
  struct type_trait<int8> final : _internal::base_type_trait<int8>
  {
    constexpr static const char name[] = "int8";

    using type = int8;
    using signed_type = type;
    using unsigned_type = uint8;
    using larger_type = int16;
    using smaller_type = void;
    using faster_type = type;

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

    constexpr inline static unsigned_type __forceinline __flatten as_unsigned(arg_type<type> val) { return val; }
    constexpr inline static typename type_trait<unsigned_type>::larger_type __forceinline __flatten as_safe_unsigned(arg_type<type> val) { return val; }
    constexpr inline static signed_type __forceinline __flatten as_signed(arg_type<type> val) { return { val }; }

    constexpr static type max = { 0x7F_i8 };
    constexpr static type min = { 0x80_i8 };
    constexpr static type ones = { 0xFF_i8 };
    constexpr static type zeros = { 0x00_i8 };

    template <uint8 base>
    constexpr static usize string_length =
      (base == 2) ? 8 :
      (base == 8) ? 3 :
      (base == 10) ? 4 :
      (base == 16) ? 2 :
      0;
  };

  template <>
  struct type_trait<uint16> : _internal::base_type_trait<uint16>
  {
    constexpr static const char name[] = "uint16";

    using type = uint16;
    using signed_type = int16;
    using unsigned_type = type;
    using larger_type = uint24;
    using smaller_type = uint8;
    using faster_type = type;

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

    constexpr inline static unsigned_type __forceinline __flatten as_unsigned(arg_type<type> val) { return { val }; }
    constexpr inline static unsigned_type __forceinline __flatten as_safe_unsigned(arg_type<type> val) { return { val }; }
    constexpr inline static signed_type __forceinline __flatten as_signed(arg_type<type> val) { return val; }

    constexpr static type max = { 0xFFFF_u16 };
    constexpr static type min = { 0x0000_u16 };
    constexpr static type ones = { 0xFFFF_u16 };
    constexpr static type zeros = { 0x0000_u16 };

    template <uint8 base>
    constexpr static usize string_length =
      (base == 2) ? 16 :
      (base == 8) ? 6 :
      (base == 10) ? 5 :
      (base == 16) ? 4 :
      0;
  };
  // Alias, AVR, TODO
  c_static_assert(sizeof(unsigned int) == sizeof(uint16), "AVR error");
  template <>
  struct type_trait<unsigned int> final : type_trait<uint16> {};

  template <>
  struct type_trait<int16> : _internal::base_type_trait<int16>
  {
    constexpr static const char name[] = "int16";

    using type = int16;
    using signed_type = type;
    using unsigned_type = uint16;
    using larger_type = int24;
    using smaller_type = int8;
    using faster_type = type;

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

    constexpr inline static unsigned_type __forceinline __flatten as_unsigned(arg_type<type> val) { return val; }
    constexpr inline static typename type_trait<unsigned_type>::larger_type __forceinline __flatten as_safe_unsigned(arg_type<type> val) { return val; }
    constexpr inline static signed_type __forceinline __flatten as_signed(arg_type<type> val) { return { val }; }

    constexpr static type max = { 0x7FFF_i16 };
    constexpr static type min = { 0x8000_i16 };
    constexpr static type ones = { 0xFFFF_i16 };
    constexpr static type zeros = { 0x0000_i16 };

    template <uint8 base>
    constexpr static usize string_length =
      (base == 2) ? 16 :
      (base == 8) ? 6 :
      (base == 10) ? 6 :
      (base == 16) ? 4 :
      0;
  };
  // Alias, AVR, TODO
  c_static_assert(sizeof(int) == sizeof(int16), "AVR error");
  template <>
  struct type_trait<int> final : type_trait<int16> {};


  template <>
  struct type_trait<uint24> final : _internal::base_type_trait<uint24>
  {
    constexpr static const char name[] = "uint24";

    using type = uint24;
    using signed_type = int24;
    using unsigned_type = type;
    using larger_type = uint32;
    using smaller_type = uint16;
    using faster_type = type;

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

    constexpr inline static unsigned_type __forceinline __flatten as_unsigned(arg_type<type> val) { return { val }; }
    constexpr inline static unsigned_type __forceinline __flatten as_safe_unsigned(arg_type<type> val) { return { val }; }
    constexpr inline static signed_type __forceinline __flatten as_signed(arg_type<type> val) { return val; }

    constexpr static type max = { 0xFFFFFF_u24 };
    constexpr static type min = { 0x000000_u24 };
    constexpr static type ones = { 0xFFFFFF_u24 };
    constexpr static type zeros = { 0x000000_u24 };

    template <uint8 base>
    constexpr static usize string_length =
      (base == 2) ? 24 :
      (base == 8) ? 8 :
      (base == 10) ? 8 :
      (base == 16) ? 6 :
      0;
  };

  template <>
  struct type_trait<int24> final : _internal::base_type_trait<int24>
  {
    constexpr static const char name[] = "int24";

    using type = int24;
    using signed_type = type;
    using unsigned_type = uint24;
    using larger_type = int32;
    using smaller_type = int16;
    using faster_type = type;

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

    constexpr inline static unsigned_type __forceinline __flatten as_unsigned(arg_type<type> val) { return val; }
    constexpr inline static typename type_trait<unsigned_type>::larger_type __forceinline __flatten as_safe_unsigned(arg_type<type> val) { return val; }
    constexpr inline static signed_type __forceinline __flatten as_signed(arg_type<type> val) { return { val }; }

    constexpr static type max = { 0x7FFFFF_i24 };
    constexpr static type min = { 0x800000_i24 };
    constexpr static type ones = { 0xFFFFFF_i24 };
    constexpr static type zeros = { 0x000099_i24 };

    template <uint8 base>
    constexpr static usize string_length =
      (base == 2) ? 24 :
      (base == 8) ? 8 :
      (base == 10) ? 9 :
      (base == 16) ? 6 :
      0;
  };

  template <>
  struct type_trait<uint32> final : _internal::base_type_trait<uint32>
  {
    constexpr static const char name[] = "uint32";

    using type = uint32;
    using signed_type = int32;
    using unsigned_type = type;
    using larger_type = uint64;
    using smaller_type = uint16;
    using faster_type = type;

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

    constexpr inline static unsigned_type __forceinline __flatten as_unsigned(arg_type<type> val) { return { val }; }
    constexpr inline static unsigned_type __forceinline __flatten as_safe_unsigned(arg_type<type> val) { return { val }; }
    constexpr inline static signed_type __forceinline __flatten as_signed(arg_type<type> val) { return val; }

    constexpr static type max = { 0xFFFFFFFF_u32 };
    constexpr static type min = { 0x00000000_u32 };
    constexpr static type ones = { 0xFFFFFFFF_u32 };
    constexpr static type zeros = { 0x00000000_u32 };

    template <uint8 base>
    constexpr static usize string_length =
      (base == 2) ? 32 :
      (base == 8) ? 11 :
      (base == 10) ? 10 :
      (base == 16) ? 8 :
      0;
  };

  template <>
  struct type_trait<int32> final : _internal::base_type_trait<int32>
  {
    constexpr static const char name[] = "int32";

    using type = int32;
    using signed_type = type;
    using unsigned_type = uint32;
    using larger_type = int64;
    using smaller_type = int16;
    using faster_type = type;

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

    constexpr inline static unsigned_type __forceinline __flatten as_unsigned(arg_type<type> val) { return val; }
    constexpr inline static typename type_trait<unsigned_type>::larger_type __forceinline __flatten as_safe_unsigned(arg_type<type> val) { return val; }
    constexpr inline static signed_type __forceinline __flatten as_signed(arg_type<type> val) { return { val }; }

    constexpr static type max = { 0x7FFFFFFF_i32 };
    constexpr static type min = { 0x80000000_i32 };
    constexpr static type ones = { 0xFFFFFFFF_i32 };
    constexpr static type zeros = { 0x00000000_i32 };

    template <uint8 base>
    constexpr static usize string_length =
      (base == 2) ? 32 :
      (base == 8) ? 11 :
      (base == 10) ? 11 :
      (base == 16) ? 8 :
      0;
  };

  template <>
  struct type_trait<uint64> final : _internal::base_type_trait<uint64>
  {
    constexpr static const char name[] = "uint64";

    using type = uint64;
    using signed_type = int64;
    using unsigned_type = type;
    using larger_type = void;
    using smaller_type = uint32;
    using faster_type = type;

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

    constexpr inline static unsigned_type __forceinline __flatten as_unsigned(arg_type<type> val) { return { val }; }
    constexpr inline static unsigned_type __forceinline __flatten as_safe_unsigned(arg_type<type> val) { return { val }; }
    constexpr inline static signed_type __forceinline __flatten as_signed(arg_type<type> val) { return val; }

    constexpr static type max = { 0xFFFFFFFFFFFFFFFF_u64 };
    constexpr static type min = { 0x0000000000000000_u64 };
    constexpr static type ones = { 0xFFFFFFFFFFFFFFFF_u64 };
    constexpr static type zeros = { 0x0000000000000000_u64 };

    template <uint8 base>
    constexpr static usize string_length =
      (base == 2) ? 64 :
      (base == 8) ? 22 :
      (base == 10) ? 20 :
      (base == 16) ? 16 :
      0;
  };

  template <>
  struct type_trait<int64> final : _internal::base_type_trait<int64>
  {
    constexpr static const char name[] = "int64";

    using type = int64;
    using signed_type = type;
    using unsigned_type = uint64;
    using larger_type = void;
    using smaller_type = int32;
    using faster_type = type;

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

    constexpr inline static unsigned_type __forceinline __flatten as_unsigned(arg_type<type> val) { return val; }
    constexpr inline static signed_type __forceinline __flatten as_signed(arg_type<type> val) { return { val }; }

    constexpr static type max = { 0x7FFFFFFFFFFFFFFF_i64 };
    constexpr static type min = { 0x8000000000000000_i64 };
    constexpr static type ones = { 0xFFFFFFFFFFFFFFFF_i64 };
    constexpr static type zeros = { 0x0000000000000000_i64 };

    template <uint8 base>
    constexpr static usize string_length =
      (base == 2) ? 64 :
      (base == 8) ? 22 :
      (base == 10) ? 20 :
      (base == 16) ? 16 :
      0;
  };

  template <>
  struct type_trait<float> : _internal::base_type_trait<float>
  {
    constexpr static const char name[] = "float";

    using type = float;
    using signed_type = type;
    using unsigned_type = void;
    using larger_type = void;
    using smaller_type = void;
    using faster_type = type;

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

    constexpr inline static signed_type __forceinline __flatten as_signed(arg_type<type> val) { return { val }; }

    constexpr static type max = { __builtin_huge_valf() };
    constexpr static type min = { 1.175494e-38f };
    constexpr static type epsilon = { 1.192093e-07f };

    template <uint8 base>
    constexpr static usize string_length =
      (base == 2) ? 32 :
      (base == 8) ? 11 :
      (base == 10) ? 32 :
      (base == 16) ? 8 :
      0;
  };
  // On AVR, float == double == long double, though the compiler doesn't always agree due to strict typing.
  template <> struct type_trait<double> final : type_trait<float> {};
  template <> struct type_trait<long double> final : type_trait<float> {};

  namespace _internal
  {
    template <typename T>
    struct enum_type_trait : _internal::base_type_trait<T>
    {
      // Default
      constexpr static const char name[] = "enumerator";

      using type = T;
      using underlying_type = __underlying_type(T);

      constexpr static uint8 size = sizeof(T);
      constexpr static uint8 bits = sizeof(T) * 8;

      constexpr static bool is_signed = type_trait<underlying_type>::is_signed;
      constexpr static bool is_unsigned = type_trait<underlying_type>::is_unsigned;
      constexpr static bool is_integral = type_trait<underlying_type>::is_integral;
      constexpr static bool is_float = type_trait<underlying_type>::is_float;
      constexpr static bool is_primitive = type_trait<underlying_type>::is_primitive;
      constexpr static bool is_array = type_trait<underlying_type>::is_array;
      constexpr static bool is_pointer = type_trait<underlying_type>::is_pointer;
      constexpr static bool is_reference = type_trait<underlying_type>::is_reference;
      constexpr static bool is_atomic = type_trait<underlying_type>::is_atomic;
      constexpr static bool is_emulated = type_trait<underlying_type>::is_emulated;
      constexpr static bool is_enum = true;
    };
  }

  // Shortened versions
  template <typename T>
  using underlying_type = typename type_trait<T>::underlying_type;
}
