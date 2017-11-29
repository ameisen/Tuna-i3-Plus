#pragma once

namespace Tuna
{
  // Simple static-cast like routine. Also makes extracting sub-types easier from encapsulations like 'flash'.
  template <typename T = void, typename U>
  constexpr inline __forceinline __flatten auto as(arg_type<U> value)
  {
    if constexpr (is_same<T, void>)
    {
      if constexpr (type_trait<U>::is_primitive)
      {
        return U(value);
      }
      else
      {
        using type = typename U::type;
        return type(value);
      }
    }
    else
    {
      return T(value);
    }
  }
}
