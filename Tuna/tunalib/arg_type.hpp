#pragma once

namespace Tuna
{
  template <typename T, bool by_value>
  struct _arg_type;

  template <typename T>
  struct _arg_type<T, true> final : trait::ce_only
  {
    using type = const T;
  };

  template <typename T>
  struct _arg_type<T, false> final : trait::ce_only
  {
    using type = const T & __restrict;
  };

  //template <typename T> using arg_type = typename _arg_type<T, sizeof(T) <= sizeof(void *)>::type;

  template <typename T> using arg_type = const T & __restrict;
}
