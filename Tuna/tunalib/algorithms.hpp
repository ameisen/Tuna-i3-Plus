#pragma once

// Get rid of any macros that someone already may have defined.
#undef min
#undef max
#undef clamp

namespace Tuna
{
  template <typename T>
  constexpr inline __forceinline __flatten T max(arg_type<T> a, arg_type<T> b)
  {
    return (a >= b) ? a : b;
  }

  template <typename T>
  constexpr inline __forceinline __flatten T min(arg_type<T> a, arg_type<T> b)
  {
    return (a < b) ? a : b;
  }

  template <typename T>
  constexpr inline __forceinline __flatten T clamp(arg_type<T> val, arg_type<T> _min, arg_type<T> _max)
  {
    return min(max(val, _min), _max);
  }

  template <typename T, typename R = typename type_trait<typename type_trait<T>::unsigned_type>::smaller_type>
  constexpr inline __forceinline __flatten R hi(arg_type<T> value)
  {
    constexpr uint8 shift = (sizeof(R) * 8);

    return type_trait<T>::as_unsigned(value) >> shift;
  }

  template <typename T, typename R = typename type_trait<typename type_trait<T>::unsigned_type>::smaller_type>
  constexpr inline __forceinline __flatten R lo(arg_type<T> value)
  {
    return type_trait<T>::as_unsigned(value);
  }

  template <typename T, typename U>
  constexpr inline __forceinline __flatten bool is_within(arg_type<T> value, arg_type<U> min, arg_type<U> max)
  {
    return value >= min && value <= max;
  }

  // Simple static-cast like routine. Also makes extracting sub-types easier from encapsulations like 'flash'.
  template <typename T = void, typename U>
  constexpr inline __forceinline __flatten auto as(arg_type<U> value);
}
