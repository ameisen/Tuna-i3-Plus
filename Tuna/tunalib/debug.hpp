#pragma once

namespace Tuna::debug
{
  namespace _internal
  {
    template <usize N1, usize N2>
    char * string_copy(char (& __restrict buffer) [N1], arg_type<flash_char_array<N2>> str)
    {
      memcpy_P(buffer, str.c_str(), N2);
      return buffer + N2;
    }

    template <usize N2>
    char * string_copy(char * __restrict buffer, arg_type<flash_char_array<N2>> str)
    {
      memcpy_P(buffer, str.c_str(), N2);
      return buffer + N2;
    }

    template <typename T, uint8 base = 10>
    char * int_to_string(char * __restrict buffer, arg_type<T> in_value)
    {
      typename type_trait<T>::unsigned_type value = in_value;
      constexpr const bool is_signed = type_trait<T>::is_signed;

      static constexpr const char lookup[] = R"(0123456789ABCDEF)";

      static_assert(base <= 16, "unsupported base");

      if (in_value == 0)
      {
        buffer[0] = '0';
        buffer[1] = '\0';
        return buffer + 1;
      }

      uint8 written = 0;

      const auto reverse = [buffer, &written]
      {
        for (uint8 i = 0; i < (written >> 1); ++i)
        {
          const char temp = buffer[i];
          buffer[i] = buffer[written - (i + 1)];
          buffer[written - (i + 1)] = temp;
        }
      };

      while (value)
      {
        const uint8 idx = (value % base);
        value /= base;

        buffer[written++] = lookup[idx];
      }

      if constexpr (is_signed && base == 10)
      {
        if (in_value < 0)
        {
          buffer[written++] = '-';
        }
      }

      reverse();
      buffer[written] = '\0';
      return &buffer[written];
    }
  }

  template <uint8 base = 10, typename T, usize N>
  void dump(arg_type<flash_char_array<N>> name, arg_type<T> value)
  {
    // TODO this can be done faster.
    char buffer[N + 32];
    char *writer = buffer;
    writer = _internal::string_copy(writer, name);
    writer = _internal::string_copy(writer, ": "_p);

    if constexpr (
      is_same<T, uint8> ||
      is_same<T, uint16> ||
      is_same<T, uint24> ||
      is_same<T, uint32> ||
      is_same<T, uint64> ||
      is_same<T, int8> ||
      is_same<T, int16> ||
      is_same<T, int24> ||
      is_same<T, int32> ||
      is_same<T, int64>
    )
    {
      _internal::int_to_string(writer, value);
    }
    else if constexpr (is_same<T, float>)
    {
      sprintf_P(writer, "%f"_p.c_str(), value);
    }

    Serial.println(buffer);
  }
}