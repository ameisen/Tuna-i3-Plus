#pragma once

namespace Tuna::chrono
{
  template <typename T> class time_ms;

  // TODO FIXME Change this later
  template <typename T> using duration_ms = time_ms<T>;

  template <typename T>
  class time_ms final
  {
    static_assert(!is_same<T, uint64>, "time_ms type cannot be uint64");

    T m_Value = {};

  public:
    constexpr time_ms() = default;
    constexpr time_ms(arg_type<T> value) : m_Value(value) {}
    constexpr time_ms(arg_type<time_ms> value) : m_Value(value.m_Value) {}

    constexpr time_ms & operator = (arg_type<time_ms> value) __restrict
    {
      m_Value = value.m_Value;
      return *this;
    }

    T raw() const __restrict
    {
      return m_Value;
    }

    static time_ms get()
    {
      if constexpr (is_same<T, uint8>)
      {
        return { millis8() };
      }
      else if constexpr (is_same<T, uint16>)
      {
        return { millis16() };
      }
      else if constexpr (is_same<T, uint24>)
      {
        return { millis24() };
      }
      else if constexpr (is_same<T, uint32>)
      {
        return { millis32() };
      }
    }

    constexpr time_ms operator - (arg_type<time_ms> other) const __restrict
    {
      return T(m_Value) - T(other.m_Value);
    }

    constexpr bool operator == (arg_type<time_ms> other) const __restrict
    {
      return m_Value == other.m_Value;
    }

    constexpr bool operator != (arg_type<time_ms> other) const __restrict
    {
      return m_Value != other.m_Value;
    }

    constexpr bool operator > (arg_type<time_ms> other) const __restrict
    {
      return m_Value > other.m_Value;
    }

    constexpr bool operator >= (arg_type<time_ms> other) const __restrict
    {
      return m_Value >= other.m_Value;
    }

    constexpr bool operator < (arg_type<time_ms> other) const __restrict
    {
      return m_Value < other.m_Value;
    }

    constexpr bool operator <= (arg_type<time_ms> other) const __restrict
    {
      return m_Value <= other.m_Value;
    }

    template <typename U>
    bool elapsed(arg_type<time_ms> current_time, arg_type<duration_ms<U>> time) const __restrict
    {
      // TODO FIXME use duration type instead of raw integers.

      const T duration = T(current_time.m_Value) - T(m_Value);

      return duration >= time.m_Value;
    }

    template <typename U>
    pair<bool, duration_ms<larger_type<T, U>>> elapsed_over(arg_type<time_ms> current_time, arg_type<duration_ms<U>> time) const __restrict
    {
      // TODO FIXME use duration type instead of raw integers.

      using larger_t = larger_type<T, U>;

      // We reverse the operand order depending on if an overflow has occured.
      const T duration = T(current_time.m_Value) - T(m_Value);

      if (duration >= time.m_Value)
      {
        return { true, larger_t(duration) - larger_t(time.m_Value) };
      }
      return { false };
    }

    template <typename U>
    bool elapsed(arg_type<duration_ms<U>> time) const __restrict
    {
      return elapsed<U>(get(), time);
    }

    template <typename U>
    pair<bool, duration_ms<larger_type<T, U>>> elapsed_over(arg_type<duration_ms<U>> time) const __restrict
    {
      return elapsed_over<U>(get(), time);
    }
  };

  using time_ms8 = time_ms<uint8>;
  using time_ms16 = time_ms<uint16>;
  using time_ms24 = time_ms<uint24>;
  using time_ms32 = time_ms<uint32>;
}

namespace Tuna
{
  constexpr chrono::duration_ms<uint8> operator "" _ms8(uint64 value)
  {
    return { uint8(value) };
  }

  constexpr chrono::duration_ms<uint16> operator "" _ms16(uint64 value)
  {
    return { uint16(value) };
  }

  constexpr chrono::duration_ms<uint24> operator "" _ms24(uint64 value)
  {
    return { uint24(value) };
  }

  constexpr chrono::duration_ms<uint32> operator "" _ms32(uint64 value)
  {
    return { uint32(value) };
  }

  //constexpr chrono::duration_ms<uint64> operator "" _ms64(uint64 value)
  //{
  //  return { uint64(value) };
  //}
}
