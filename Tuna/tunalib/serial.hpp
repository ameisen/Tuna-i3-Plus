#pragma once

#include "HardwareSerial.h"

// Get rid of any macros that someone already may have defined.
#undef cli
#undef sei
#undef min
#undef max
#undef clamp

namespace Tuna
{
  template <uint8 SerialNumber = 0>
  struct serial final : trait::ce_only
  {
#if defined(SerialUSB)
    constexpr static const uint8 USB = type_trait<uint8>::max;
#endif

    constexpr static const uint8 number = SerialNumber;

    constexpr static __forceinline __flatten auto & __restrict get_serial_device()
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
    static inline __forceinline __flatten void begin()
    {
      get_serial_device().begin(baud);
    }

    template <typename T>
    constexpr static inline __forceinline __flatten uint8 write_struct(arg_type<T> obj)
    {
      return get_serial_device().write((const uint8 * __restrict)&obj, sizeof(T));
    }

    template <typename T, usize N>
    constexpr static inline __forceinline __flatten uint8 write(const T(&__restrict buffer)[N])
    {
      return get_serial_device().write((const uint8 * __restrict)buffer, N);
    }

    template <usize N>
    constexpr static inline __forceinline __flatten uint8 write(const flash_char_array<N> & __restrict str)
    {
      return get_serial_device().write((const uint8 * __restrict)str.c_str(), N + 1);
    }

    constexpr static inline __forceinline __flatten uint8 write(arg_type<flash_string> str, const uint8 length)
    {
      return get_serial_device().write((const uint8 * __restrict)str.c_str(), length);
    }

    template <typename T>
    constexpr static inline __forceinline __flatten uint8 write(arg_type<T> buffer, const uint8 length)
    {
      return get_serial_device().write((const uint8 * __restrict)buffer, length);
    }

    template <typename T, usize N>
    constexpr static inline __forceinline __flatten uint8 read_bytes(T(&__restrict buffer)[N])
    {
      return get_serial_device().readBytes((uint8 * __restrict)buffer, N);
    }

    template <typename T>
    constexpr static inline __forceinline __flatten uint8 read_bytes(T * __restrict buffer, const uint8 length)
    {
      return get_serial_device().readBytes((uint8 * __restrict)buffer, length);
    }

    static inline __forceinline __flatten uint available()
    {
      return get_serial_device().available();
    }

    static inline __forceinline __flatten bool available(uint8 length)
    {
      return uint8(min(get_serial_device().available(), 255)) >= length;
    }

    // TODO handle -1. Didn't want this to be an int.
    static inline __forceinline __flatten uint8 read()
    {
      return get_serial_device().read();
    }
  };
}
