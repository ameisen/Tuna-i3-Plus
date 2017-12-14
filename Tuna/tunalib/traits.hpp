#pragma once

namespace Tuna::trait
{
  struct ce_only { ce_only() = delete; };

  struct getter {};

  struct setter {};

  template <typename T>
  struct empty_setter : setter
  {
    static void __forceinline __flatten set(const T & __restrict) {}
  };

  template <typename T>
  struct empty_getter : getter
  {
    static const T & __restrict __forceinline __flatten get()
    {
      static constexpr const T const_val{};
      return const_val;
    }
  };

  template <typename T = void>
  struct empty : empty_getter<T>, empty_setter<T> {};

  template <>
  struct empty<void> {};
}
