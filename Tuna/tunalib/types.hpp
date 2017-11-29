#pragma once

namespace Tuna
{
  // A few definitions just to help Intellisense along.
#if !__compiling
  using __uint24 = unsigned int;
  using __int24 = signed int;
#endif
  // ~Intellisense

  // TODO everything that's not AVR8
  using uint8 = unsigned char;
  using uint16 = unsigned short;
  using uint24 = __uint24;
  using uint32 = unsigned long;
  using uint64 = unsigned long long;

  using int8 = signed char;
  using int16 = signed short;
  using int24 = __int24;
  using int32 = signed long;
  using int64 = signed long long;

  using uint = uint16;

  using usize = uint16;

  // sanity checks
  c_static_assert(sizeof(uint8) == 1);
  c_static_assert(sizeof(int8) == 1);
  c_static_assert(sizeof(uint16) == 2);
  c_static_assert(sizeof(int16) == 2);
  c_static_assert(sizeof(uint24) == 3);
  c_static_assert(sizeof(int24) == 3);
  c_static_assert(sizeof(uint32) == 4);
  c_static_assert(sizeof(int32) == 4);
  c_static_assert(sizeof(uint64) == 8);
  c_static_assert(sizeof(int64) == 8);

  c_static_assert(alignof(uint8) == 1);
  c_static_assert(alignof(int8) == 1);
  c_static_assert(alignof(uint16) == 1);
  c_static_assert(alignof(int16) == 1);
  c_static_assert(alignof(uint24) == 1);
  c_static_assert(alignof(int24) == 1);
  c_static_assert(alignof(uint32) == 1);
  c_static_assert(alignof(int32) == 1);
  c_static_assert(alignof(uint64) == 1);
  c_static_assert(alignof(int64) == 1);

  c_static_assert(sizeof(int) == 2, "atmega int is 2 bytes. Utils need to be rewritten for other sizes.");

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
