#pragma once

namespace Tuna
{
  template <typename T>
  class memory final
  {
    T value_ = {};
  public:
    constexpr memory() = default;
    constexpr memory(arg_type<T> value) : value_(value) {}
    constexpr memory(arg_type<memory> value) : value_(value.value_) {}

    inline constexpr memory & __forceinline __flatten operator = (arg_type<T> value) __restrict
    {
      value_ = value;
      return *this;
    }

    inline constexpr memory & __forceinline __flatten operator = (arg_type<memory> value) __restrict
    {
      __assume(this != &value);
      value_ = value.value_;
      return *this;
    }

    inline constexpr __forceinline __flatten operator T & () __restrict
    {
      return value_;
    }

    inline constexpr __forceinline __flatten operator const T & () const __restrict
    {
      return value_;
    }

    inline constexpr void __forceinline __flatten write_through(arg_type<T> value) __restrict
    {
      ((volatile T &)value_) = value;
    }

    inline constexpr void __forceinline __flatten write_through(arg_type<memory> value) __restrict
    {
      ((volatile T &)value_) = value;
    }

    inline constexpr T __forceinline __flatten read_through() const __restrict
    {
      return ((volatile T &)value_);
    }

    inline constexpr void __forceinline __flatten write_read_through(arg_type<memory> value) __restrict
    {
      ((volatile T &)value_) = value.read_through();
    }

    inline constexpr void __forceinline __flatten flush() __restrict
    {
      ((volatile T &)value_) = value_;
    }

    // Non-read through. Cached-access.
    inline constexpr T * __forceinline __flatten operator -> () __restrict
    {
      return &value_;
    }

    inline constexpr const T * __forceinline __flatten operator -> () const __restrict
    {
      return &value_;
    }
  };
}
