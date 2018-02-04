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

  template <typename T, typename U>
  constexpr inline __forceinline __flatten T bitwise_as(arg_type<U> value)
  {
    static_assert(sizeof(T) == sizeof(U), "bitwise_as can only be used on same-sized types");

    if constexpr (is_same<T, U>)
    {
      return value;
    }
    else
    {
      union
      {
        U in;
        T out;
      };
      in = { value };
      return out;
    }
  }

  // TODO add more helpers to constrain the range, which can reduce the type-size of lambda_t.
  template <typename T, typename U>
  constexpr U linear_interpolate(arg_type<T> value, arg_type<T> min, arg_type<T> max, arg_type<U> res_min, arg_type<U> res_max)
  {
    using unsigned_T = typename type_trait<T>::unsigned_type;
    using unsigned_U = typename type_trait<U>::unsigned_type;

    const unsigned_T delta = bitwise_as<unsigned_T>(value - min);
    const unsigned_T max_diff = bitwise_as<unsigned_T>(max - min);

    using lambda_t = uintsz<type_trait<unsigned_U>::max * type_trait<unsigned_T>::max>;
    using add_t = uintsz<type_trait<lambda_t>::max + type_trait<lambda_t>::max>;

    lambda_t b_lambda = (lambda_t(res_max) * delta);
    lambda_t a_lambda = (lambda_t(res_min) * max_diff);

    return bitwise_as<U>(unsigned_U((add_t(b_lambda) + a_lambda) / max));
  }
}
