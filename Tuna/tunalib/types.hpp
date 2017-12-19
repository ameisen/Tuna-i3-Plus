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

  using real = float; // TODO change me once we're done stripping out floats.

  using user_int = uint64;
  using user_real = long double;

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

  namespace _internal
  {
    template <uint64 value, uint64 exponent, uint64 times> constexpr static const uint64 pow = times ? pow<value * exponent, exponent, times - 1> : value;
    template <uint64 value, uint64 exponent> constexpr static const uint64 pow<value, exponent, 0> = value;

    template <typename T, T c = 0, T...>
    constexpr static const T get_front = c;

    // doesn't work right.
    // template <typename T, T c, T... pack>
    // constexpr static const auto pop_front = (pack, ...);


    template <char c>
    constexpr static const bool is_numeral = (c >= '0' && c <= '9');

    template <uint64 in_value, uint64 multiplicand, uint64 _base, char c>
    constexpr uint64 expand_string_to_int_char_impl()
    {
      // switch statement was throwing static_asserts when it shouldn't have.
      if constexpr (_base == 2)
      {
        static_assert(c >= '0' && c <= '1', "input string is not a valid integer");
        return in_value + (uint64(c - '0') * multiplicand);
      }
      if constexpr (_base == 8)
      {
        static_assert(c >= '0' && c <= '8', "input string is not a valid integer");
        return in_value + (uint64(c - '0') * multiplicand);
      }
      if constexpr (_base == 10)
      {
        static_assert(c >= '0' && c <= '9', "input string is not a valid integer");
        return in_value + (uint64(c - '0') * multiplicand);
      }
      if constexpr (_base == 16)
      {
        static_assert(
          (c >= '0' && c <= '9') ||
          (c >= 'a' && c <= 'f') ||
          (c >= 'A' && c <= 'F')
          , "input string is not a valid integer");
        if constexpr (c >= '0' && c <= '9')
        {
          return in_value + (uint64(c - '0') * multiplicand);
        }
        else if constexpr (c >= 'a' && c <= 'f')
        {
          return in_value + ((uint64(c - 'a') + 10) * multiplicand);
        }
        else if constexpr (c >= 'A' && c <= 'F')
        {
          return in_value + ((uint64(c - 'A') + 10) * multiplicand);
        }
        else
        {
          //static_assert(false, "input string is not a valid integer");
          return 0;
        }
      }
      else
      {
        //return _base;
        //static_assert(false, "unknown literal type");
        return 0;
      }
    }

    template <uint64 in_value, uint64 multiplicand, uint64 _base, char c, char... characters>
    constexpr uint64 expand_string_to_int_impl()
    {
      if constexpr (sizeof...(characters) == 0)
      {
        return expand_string_to_int_char_impl<in_value, multiplicand, _base, c>();
      }
      else
      {
        return expand_string_to_int_impl<
          expand_string_to_int_char_impl<in_value, multiplicand, _base, c>(),
          multiplicand / _base,
          _base,
          characters...
        >();
      }
    }

    template <uint64 in_value, uint64 multiplicand, uint64 _base, char c, char... characters>
    constexpr uint64 expand_string_to_int_impl_consume()
    {
      return expand_string_to_int_impl<in_value, multiplicand, _base, characters...>();
    }

    // Seperate implementation function to kick off, for organization and also because of constexpr issues.
    template <char c, char... characters>
    constexpr uint64 expand_string_to_int()
    {
      constexpr const char next_c = get_front<char, characters...>;

#if 1
      if constexpr (
        (sizeof...(characters) >= 2) && // 0t requires at least 2 characters.
        (next_c != 0) &&                // 0t requires at least 2 characters.
        (c == '0') &&                     // first element must be 0.
        !is_numeral<next_c>             // 0t cannot have 't' be a numeral.
        )
      {
        // If we got here, then we received a string in the format 0t..., which implies that it is non-decimal.
        // Also doesn't like the switch statement.
        if constexpr (next_c == 'b')
        {
          return expand_string_to_int_impl_consume<0, pow<1, 2, sizeof...(characters)-2>, 2, characters...>();
        }
        else if constexpr (next_c == 'o')
        {
          return expand_string_to_int_impl_consume<0, pow<1, 8, sizeof...(characters)-2>, 8, characters...>();
        }
        else if constexpr (next_c == 'x' || next_c == 'X')
        {
          return expand_string_to_int_impl_consume<0, pow<1, 16, sizeof...(characters)-2>, 16, characters...>();
        }
        else
        {
          static_assert(
            next_c != 'b' &&
            next_c != 'o' &&
            next_c != 'x' &&
            next_c != 'X'
            , "unknown literal type");
        }
      }
      else
      {
        return expand_string_to_int_impl<0, pow<1, 10, sizeof...(characters)>, 10, c, characters...>();
      }

#else
      if constexpr (
        (sizeof...(characters) >= 2) && // 0t requires at least 2 characters.
        (next_c != 0) &&                // 0t requires at least 2 characters.
        (c == '0') &&                     // first element must be 0.
        !is_numeral<next_c>             // 0t cannot have 't' be a numeral.
        )
      {
        // If we got here, then we received a string in the format 0t..., which implies that it is non-decimal.
        // Also doesn't like the switch statement.
        if constexpr (next_c == 'b')
        {
          return expand_string_to_int_impl<0, pow<1, 2, sizeof...(characters)-2>, 2, pop_front<char, characters...>>();
        }
        else if constexpr (next_c == 'o')
        {
          return expand_string_to_int_impl<0, pow<1, 8, sizeof...(characters)-2>, 8, pop_front<char, characters...>>();
        }
        else if constexpr (next_c == 'x' || next_c == 'X')
        {
          return expand_string_to_int_impl<0, pow<1, 16, sizeof...(characters)-2>, 16, pop_front<char, characters...>>();
        }
        else
        {
          static_assert(
            next_c != 'b' &&
            next_c != 'o' &&
            next_c != 'x' &&
            next_c != 'X'
            , "unknown literal type");
        }
      }
      else
      {
        return expand_string_to_int_impl<0, pow<1, 10, sizeof...(characters)>, 10, c, characters...>();
      }
#endif
    }
  }

  constexpr uint8 operator"" _u8(user_int value)
  {
    // static_assert(value > uint8(-1), "out of range u8 literal");
    return value;
  }

  template <char... Chars> constexpr uint8 operator "" _u8()
  {
    constexpr const uint64 value = _internal::expand_string_to_int<Chars...>();
    static_assert(value <= uint64(-1), "out of range u8 literal");
    return uint8(value);
  }

  constexpr int8 operator"" _i8(user_int value)
  {
    // static_assert(value > uint8(-1), "out of range u8 literal");
    return value;
  }

  constexpr uint16 operator"" _u16(user_int value)
  {
    // static_assert(value > uint8(-1), "out of range u8 literal");
    return value;
  }

  constexpr int16 operator"" _i16(user_int value)
  {
    // static_assert(value > uint8(-1), "out of range u8 literal");
    return value;
  }

  constexpr uint24 operator"" _u24(user_int value)
  {
    // static_assert(value > uint8(-1), "out of range u8 literal");
    return value;
  }

  constexpr int24 operator"" _i24(user_int value)
  {
    // static_assert(value > uint8(-1), "out of range u8 literal");
    return value;
  }

  constexpr uint32 operator"" _u32(user_int value)
  {
    // static_assert(value > uint8(-1), "out of range u8 literal");
    return value;
  }

  constexpr int32 operator"" _i32(user_int value)
  {
    // static_assert(value > uint8(-1), "out of range u8 literal");
    return value;
  }

  constexpr uint64 operator"" _u64(user_int value)
  {
    // static_assert(value > uint8(-1), "out of range u8 literal");
    return value;
  }

  constexpr int64 operator"" _i64(user_int value)
  {
    // static_assert(value > uint8(-1), "out of range u8 literal");
    return value;
  }

  namespace _internal
  {
    template <typename T>
    struct remove_reference
    {
      using type = T;
    };

    template <typename T>
    struct remove_reference<T &>
    {
      using type = T;
    };

    template <typename T>
    struct remove_reference<T &&>
    {
      using type = T;
    };
  }

  template <typename T> using remove_reference = typename _internal::remove_reference<T>::type;
}
