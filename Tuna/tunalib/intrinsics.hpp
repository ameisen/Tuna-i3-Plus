#pragma once

// TODO add other platforms.
#include "arch/avr.hpp"

namespace Tuna::intrinsic
{
  enum class access : uint8
  {
    none,
    read,
    write,
    read_write
    // TODO implement execute?
  };
  enum class locality : uint8
  {
    none = 0,
    low = 1,
    moderate = 2,
    high = 3
  };

  // Prefetch data. If the architecture does not support this, this has no effect.
  // Not really a macro, obviously, but is best kept in here because of similarity to other functions.
  template <typename T, access _access = access::read, locality _locality = locality::high>
  constexpr inline __forceinline __flatten void prefetch(const T * __restrict addr)
  {
    static_assert(_access != access::none, "prefetch access level cannot be 'none'");
    // I am unsure if static_assert also implies __assume, so I am going to be explicit.
    __assume(_access != access::none);
    if constexpr (_access == access::read)
    {
      __builtin_prefetch(addr, 0, uint8(_locality));
    }
    else if constexpr (_access == access::write)
    {
      __builtin_prefetch(addr, 1, uint8(_locality));
    }
    else if constexpr (_access == access::read_write)
    {
      __builtin_prefetch(addr, 0, uint8(_locality));
      __builtin_prefetch(addr, 1, uint8(_locality));
    }
    else
    {
      __unreachable;
    }
  }

  // Flush the CPU's instruction cache. If the architecture does not support this, this has no effect.
  template <typename T>
  constexpr inline __forceinline __flatten void flush_icache(const T * const begin, const T * const end)
  {
    __builtin___clear_cache(begin, end);
  }
}
