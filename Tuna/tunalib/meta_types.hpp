#pragma once

namespace Tuna
{
  template <typename T, T v>
  struct integral_constant : trait::ce_only
  {
    static constexpr const T value = v;
  };
  struct false_type : integral_constant<bool, false> {};
  struct true_type : integral_constant<bool, true> {};

  template <typename T> struct _is_void final { static constexpr const bool value = false; };;
  template<> struct _is_void<void> final { static constexpr const bool value = true; };

  template <typename T, typename U>
  struct _is_same final : false_type {};
  template <typename T>
  struct _is_same<T, T> final : true_type {};
  template <typename T, typename U> constexpr const bool is_same = _is_same<T, U>::value;

  template <typename T, typename U>
  class _larger_type final : trait::ce_only
  {
    static constexpr auto typer()
    {
      if constexpr (sizeof(U) > sizeof(T))
      {
        return U{};
      }
      return T{};
    }
  public:
    using type = decltype(typer());
  };
  template <typename T, typename U> using larger_type = typename _larger_type<T, U>::type;

  template <typename T, typename U>
  class _smaller_type final : trait::ce_only
  {
    static constexpr auto typer()
    {
      if constexpr (sizeof(U) < sizeof(T))
      {
        return U{};
      }
      return T{};
    }
  public:
    using type = decltype(typer());
  };
  template <typename T, typename U> using smaller_type = typename _smaller_type<T, U>::type;

  template <uint64 value>
  class _uintsz final : trait::ce_only
  {
    constexpr static auto typer()
    {
      if constexpr (value <= type_trait<uint8>::max)
      {
        return uint8{};
      }
      else if constexpr (value <= type_trait<uint16>::max)
      {
        return uint16{};
      }
      else if constexpr (value <= type_trait<uint24>::max)
      {
        return uint24{};
      }
      else if constexpr (value <= type_trait<uint32>::max)
      {
        return uint32{};
      }
      else if constexpr (value <= type_trait<uint64>::max)
      {
        return uint64{};
      }
    }
  public:
    using type = decltype(typer());
  };

  template <uint64 value> using uintsz = typename _uintsz<value>::type;

  template <uint64 value> constexpr auto make_uintsz = uintsz<value>{ value };

  template <int64 value>
  class _intsz final : trait::ce_only
  {
    constexpr static auto typer()
    {
      if constexpr (is_within(value, type_trait<int8>::min, type_trait<int8>::max))
      {
        return int8{};
      }
      else if constexpr (is_within(value, type_trait<int16>::min, type_trait<int16>::max))
      {
        return int16{};
      }
      else if constexpr (is_within(value, type_trait<int24>::min, type_trait<int24>::max))
      {
        return int24{};
      }
      else if constexpr (is_within(value, type_trait<int32>::min, type_trait<int32>::max))
      {
        return int32{};
      }
      else if constexpr (is_within(value, type_trait<int64>::min, type_trait<int64>::max))
      {
        return int64{};
      }
    }
  public:
    using type = decltype(typer());
  };

  template <int64 value> using intsz = typename _intsz<value>::type;

  template <int64 value> constexpr auto make_intsz = intsz<value>{ value };

  template <typename T, typename U>
  struct pair final
  {
    T first;
    U second;

    constexpr __forceinline __flatten pair() = default;
    constexpr __forceinline __flatten pair(arg_type<T> _1) : first(_1) {}
    constexpr __forceinline __flatten pair(arg_type<T> _1, arg_type<U> _2) : first(_1), second(_2) {}
    constexpr __forceinline __flatten pair(arg_type<pair> other) : first(other.first), second(other.second) {}

    constexpr __forceinline __flatten operator bool() const __restrict
    {
      return bool(first);
    }
  };

  template <typename T, typename U>
  constexpr inline pair<T, U> make_pair(arg_type<T> first, arg_type<U> second)
  {
    return { first, second };
  }

  template <typename T, typename U>
  struct ref_pair final
  {
    const T & __restrict first;
    const U & __restrict second;

    constexpr __forceinline __flatten ref_pair() = default;
    constexpr __forceinline __flatten ref_pair(arg_type<T> _1) : first(_1) {}
    constexpr __forceinline __flatten ref_pair(arg_type<T> _1, arg_type<U> _2) : first(_1), second(_2) {}
    constexpr __forceinline __flatten ref_pair(arg_type<ref_pair> other) : first(other.first), second(other.second) {}

    constexpr __forceinline __flatten operator bool() const __restrict
    {
      return bool(first);
    }
  };

  template <typename T, typename U>
  constexpr inline ref_pair<T, U> make_ref_pair(arg_type<T> first, arg_type<U> second)
  {
    return { first, second };
  }

  template <typename T, bool enabled>
  struct conditional_type
  {
    T value_ = {};

    conditional_type(const T & __restrict value) : value_(value) {}
    conditional_type & __restrict operator = (const T & __restrict value)
    {
      value_ = value;
      return *this;
    }

    operator T & __restrict () __restrict { return value_; }
    operator const T & __restrict () __restrict const { return value_; }
  };

  template <typename T>
  struct conditional_type<T, false>
  {
    conditional_type(const T &) {}
    conditional_type & __restrict operator = (const T &) { return *this; }

    operator T () __restrict const
    {
      return {};
    }
  };

  template <typename T, bool enabled>
  struct volatile_conditional_type
  {
    volatile T value_ = {};

    volatile_conditional_type(const T & __restrict value) : value_(value) {}
    volatile_conditional_type & __restrict operator = (const T & __restrict value)
    {
      value_ = value;
      return *this;
    }

    //operator T () __restrict { return value_; }
    operator const T () __restrict const { return value_; }
  };

  template <typename T>
  struct volatile_conditional_type<T, false>
  {
    volatile_conditional_type(const T &) {}
    volatile_conditional_type & __restrict operator = (const T &) { return *this; }

    operator T () __restrict const
    {
      return {};
    }
  };
}
