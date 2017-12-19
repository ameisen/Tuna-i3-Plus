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

    constexpr memory & operator = (arg_type<T> value) __restrict
    {
      value_ = value;
      return *this;
    }

    constexpr memory & operator = (arg_type<memory> value) __restrict
    {
      __assume(this != &value);
      value_ = value.value_;
      return *this;
    }

    operator T & () __restrict
    {
      return value_;
    }

    operator const T & () const __restrict
    {
      return value_;
    }

    void write_through(arg_type<T> value) __restrict
    {
      ((volatile T &)value_) = value;
    }

    void write_through(arg_type<memory> value) __restrict
    {
      ((volatile T &)value_) = value;
    }

    T read_through() const __restrict
    {
      return ((volatile T &)value_);
    }

    void write_read_through(arg_type<memory> value) __restrict
    {
      ((volatile T &)value_) = value.read_through();
    }

    void flush() __restrict
    {
      ((volatile T &)value_) = value_;
    }
  };
}
