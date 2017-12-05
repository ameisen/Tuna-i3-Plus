#pragma once

// This exists in the global namespace for Arduino support.
class __FlashStringHelper;

namespace Tuna
{
  template <typename T>
  struct flash_ptr final
  {
    using ptr_t = uint16; // TODO handle > 16-bit ptrs
  private:
    const uint16  m_Ptr;
  public:
    constexpr __forceinline __flatten flash_ptr(arg_type<T> value) : m_Ptr(uint16(&value)) {}

    constexpr inline __forceinline __flatten operator uint16 () const __restrict
    {
      return m_Ptr;
    }
  };

  template <typename U>
  static inline __forceinline __flatten U read_pgm_ptr(uint16 ptr)
  {
    U retValue;// = pgm_read_word(ptr);
               //return retValue;

    if constexpr (sizeof(U) == 1)
    {
      __asm__
      (
        "lpm %0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 2)
    {
      __asm__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 3)
    {
      __asm__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 4)
    {
      __asm__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z+" "\n\t"
        "lpm %D0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 5)
    {
      __asm__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z+" "\n\t"
        "lpm %D0, Z+" "\n\t"
        "lpm %E0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 6)
    {
      __asm__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z+" "\n\t"
        "lpm %D0, Z+" "\n\t"
        "lpm %E0, Z+" "\n\t"
        "lpm %F0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 7)
    {
      __asm__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z+" "\n\t"
        "lpm %D0, Z+" "\n\t"
        "lpm %E0, Z+" "\n\t"
        "lpm %F0, Z+" "\n\t"
        "lpm %G0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) == 8)
    {
      __asm__
      (
        "lpm %A0, Z+" "\n\t"
        "lpm %B0, Z+" "\n\t"
        "lpm %C0, Z+" "\n\t"
        "lpm %D0, Z+" "\n\t"
        "lpm %E0, Z+" "\n\t"
        "lpm %F0, Z+" "\n\t"
        "lpm %G0, Z+" "\n\t"
        "lpm %H0, Z" "\n\t"
        : "=r" (retValue), "=z" (ptr)
        : "1" (ptr)
      );
    }
    else if constexpr (sizeof(U) > 8)
    {
      uint8 *retValuePtr = (uint8 *)&retValue;
      constexpr uint8 type_size = sizeof(U);
      // TODO : we should hand-optimize this routine, as this is suboptimal by far.

      constexpr uint8 qwords = type_size / 8;

      for (uint8 i = 0; i < qwords; ++i)
      {
        auto &val = *(uint64 *)retValuePtr;
        __asm__
        (
          "lpm %A0, Z+" "\n\t"
          "lpm %B0, Z+" "\n\t"
          "lpm %C0, Z+" "\n\t"
          "lpm %D0, Z+" "\n\t"
          "lpm %E0, Z+" "\n\t"
          "lpm %F0, Z+" "\n\t"
          "lpm %G0, Z+" "\n\t"
          "lpm %H0, Z" "\n\t"
          : "=r" (val), "=z" (ptr)
          : "1" (ptr)
        );
        retValuePtr += 8;
        ptr += 8;
      }
      if constexpr (type_size & 4)
      {
        auto &val = *(uint32 *)retValuePtr;
        __asm__
        (
          "lpm %A0, Z+" "\n\t"
          "lpm %B0, Z+" "\n\t"
          "lpm %C0, Z+" "\n\t"
          "lpm %D0, Z" "\n\t"
          : "=r" (val), "=z" (ptr)
          : "1" (ptr)
        );
        retValuePtr += 4;
        ptr += 4;
      }
      if constexpr (type_size & 2)
      {
        auto &val = *(uint16 *)retValuePtr;
        __asm__
        (
          "lpm %A0, Z+" "\n\t"
          "lpm %B0, Z" "\n\t"
          : "=r" (val), "=z" (ptr)
          : "1" (ptr)
        );
        retValuePtr += 2;
        ptr += 2;
      }
      if constexpr (type_size & 1)
      {
        auto &val = *(uint8 *)retValuePtr;
        __asm__
        (
          "lpm %A0, Z" "\n\t"
          : "=r" (val), "=z" (ptr)
          : "1" (ptr)
        );
      }
    }
    return retValue;
  }

  template <typename U, typename T>
  static inline __forceinline __flatten U read_pgm(arg_type<flash_ptr<T>> value)
  {
    return read_pgm_ptr<U>(uint16(value));
  }

  template <typename U, typename T>
  static inline __forceinline __flatten U read_pgm(arg_type<T> value)
  {
    return read_pgm_ptr<U>(uint16(&value));
  }


  template <typename T>
  class alignas(T) flash final
  {
    const T m_Value = T{};

    static inline __forceinline __flatten T _read_pgm(arg_type<T> value)
    {
      return read_pgm<T>(value);
    }

  public:
    using type = T;

    constexpr __forceinline __flatten flash() = default;
    constexpr __forceinline __flatten flash(arg_type<flash> data) : m_Value(data.m_Value) {}
    constexpr __forceinline __flatten flash(arg_type<T> value) : m_Value(value) {}

    template <typename U = T>
    constexpr __forceinline __flatten T get() const __restrict
    {
      if (__builtin_constant_p(m_Value))
      {
        return m_Value;
      }
      else
      {
        return _read_pgm(m_Value);
      }
    }

    //constexpr operator T () const
    //{
    //  return get();
    //}

    template <typename U>
    constexpr __forceinline __flatten operator U () const __restrict
    {
      static_assert(sizeof(T) <= sizeof(U), "Cannot extract pgm value larger than declared storage.");
      return get<U>();
    }

    constexpr __forceinline __flatten bool operator == (arg_type<flash> other) const __restrict
    {
      return get() == other.get();
    }

    constexpr __forceinline __flatten bool operator != (arg_type<flash> other) const __restrict
    {
      return get() != other.get();
    }

    constexpr __forceinline __flatten bool operator > (arg_type<flash> other) const __restrict
    {
      return get() > other.get();
    }

    constexpr __forceinline __flatten bool operator >= (arg_type<flash> other) const __restrict
    {
      return get() >= other.get();
    }

    constexpr __forceinline __flatten bool operator < (arg_type<flash> other) const __restrict
    {
      return get() < other.get();
    }

    constexpr __forceinline __flatten bool operator <= (arg_type<flash> other) const __restrict
    {
      return get() <= other.get();
    }

    template <typename U>
    constexpr __forceinline __flatten bool operator == (arg_type<U> other) const __restrict
    {
      return get() == other;
    }

    template <typename U>
    constexpr __forceinline __flatten bool operator != (arg_type<U> other) const __restrict
    {
      return get() != other;
    }

    template <typename U>
    constexpr __forceinline __flatten bool operator > (arg_type<U> other) const __restrict
    {
      return get() > other;
    }

    template <typename U>
    constexpr __forceinline __flatten bool operator >= (arg_type<U> other) const __restrict
    {
      return get() >= other;
    }

    template <typename U>
    constexpr __forceinline __flatten bool operator < (arg_type<U> other) const __restrict
    {
      return get() < other;
    }

    template <typename U>
    constexpr __forceinline __flatten bool operator <= (arg_type<U> other) const __restrict
    {
      return get() <= other;
    }

    constexpr __forceinline __flatten T operator + (arg_type<flash> other) const __restrict
    {
      return get() + other.get();
    }

    constexpr __forceinline __flatten T operator - (arg_type<flash> other) const __restrict
    {
      return get() - other.get();
    }

    constexpr __forceinline __flatten T operator / (arg_type<flash> other) const __restrict
    {
      return get() / other.get();
    }

    constexpr __forceinline __flatten T operator * (arg_type<flash> other) const __restrict
    {
      return get() * other.get();
    }

    constexpr __forceinline __flatten T operator % (arg_type<flash> other) const __restrict
    {
      return get() % other.get();
    }

    constexpr __forceinline __flatten T operator >> (arg_type<flash> other) const __restrict
    {
      return get() >> other.get();
    }

    constexpr __forceinline __flatten T operator << (arg_type<flash> other) const __restrict
    {
      return get() << other.get();
    }

    template <typename U>
    constexpr __forceinline __flatten T operator + (arg_type<U> other) const __restrict
    {
      return get() + other;
    }

    template <typename U>
    constexpr __forceinline __flatten T operator - (arg_type<U> other) const __restrict
    {
      return get() - other;
    }

    template <typename U>
    constexpr __forceinline __flatten T operator / (arg_type<U> other) const __restrict
    {
      return get() / other;
    }

    template <typename U>
    constexpr __forceinline __flatten T operator * (arg_type<U> other) const __restrict
    {
      return get() * other;
    }

    template <typename U>
    constexpr __forceinline __flatten T operator % (arg_type<U> other) const __restrict
    {
      return get() % other;
    }

    template <typename U>
    constexpr __forceinline __flatten T operator >> (arg_type<U> other) const __restrict
    {
      return get() >> other;
    }

    template <typename U>
    constexpr __forceinline __flatten T operator << (arg_type<U> other) const __restrict
    {
      return get() << other;
    }
  };

  template<typename T, usize N>
  constexpr inline __forceinline __flatten usize array_size(T(& __restrict)[N])
  {
    return N;
  }

  class flash_string final
  {
    const char *m_Str = nullptr;
  public:
    constexpr __forceinline __flatten flash_string() = default;
    constexpr __forceinline __flatten flash_string(const char * __restrict str) : m_Str(str) {}
    constexpr __forceinline __flatten flash_string(arg_type<flash_string> str) : m_Str(str.m_Str) {}

    constexpr __forceinline __flatten flash_string & __restrict operator = (const char * __restrict str) __restrict
    {
      m_Str = str;
      return *this;
    }

    constexpr __forceinline __flatten flash_string & __restrict operator = (arg_type<flash_string> str) __restrict
    {
      m_Str = str.m_Str;
      return *this;
    }

    constexpr __forceinline __flatten operator bool() const __restrict
    {
      return m_Str != nullptr;
    }

    constexpr __forceinline __flatten bool operator == (arg_type<flash_string> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    constexpr __forceinline __flatten bool operator != (arg_type<flash_string> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    constexpr __forceinline __flatten const char * __restrict c_str() const __restrict
    {
      return m_Str;
    }

    // This implicit conversion exists for Arduino SDK support.
    constexpr __forceinline __flatten operator const __FlashStringHelper * __restrict () const __restrict
    {
      return (const __FlashStringHelper * __restrict)m_Str;
    }

    constexpr __forceinline __flatten auto fsh() const __restrict
    {
      return (const __FlashStringHelper * __restrict)m_Str;
    }
  };

  template <usize LEN>
  class flash_char_array final
  {
    const char * const m_Str;

  public:
    static constexpr const auto length = make_uintsz<LEN>;

    constexpr __forceinline __flatten flash_char_array(const char * const __restrict str) : m_Str(str) {}

    constexpr __forceinline __flatten operator flash_string () const __restrict
    {
      return { m_Str };
    }

    constexpr __forceinline __flatten operator bool() const __restrict
    {
      return m_Str != nullptr;
    }

    constexpr __forceinline __flatten bool operator == (arg_type<flash_string> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    constexpr __forceinline __flatten bool operator != (arg_type<flash_string> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    template <usize U>
    constexpr __forceinline __flatten bool operator == (arg_type<flash_char_array<U>> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    template <usize U>
    constexpr __forceinline __flatten bool operator != (arg_type<flash_char_array<U>> str) const __restrict
    {
      return m_Str == str.m_Str;
    }

    constexpr __forceinline __flatten const char * __restrict c_str() const __restrict
    {
      return m_Str;
    }

    // This implicit conversion exists for Arduino SDK support.
    constexpr __forceinline __flatten operator const __FlashStringHelper * __restrict () const __restrict
    {
      return (const __FlashStringHelper * __restrict)m_Str;
    }

    constexpr __forceinline __flatten auto fsh() const __restrict
    {
      return (const __FlashStringHelper * __restrict)m_Str;
    }
  };

  namespace _internal
  {
    template <char... Chars>
    struct progmem_str_store final : trait::ce_only
    {
      static constexpr const char str[] __flashmem = { Chars..., '\0' };
    };
  }

  template <typename T, T... Chars>
  constexpr flash_char_array<sizeof...(Chars)> operator "" _p()
  {
    static_assert(is_same<T, char>, "_p must be used with 'char'");
    //static const char str[] __flashmem = { Chars..., '\0' };
    return { _internal::progmem_str_store<Chars...>::str };
  }
}
