#pragma once

namespace Tuna
{
#pragma GCC diagnostic ignored "-Wattributes" // annoying warning about __forceinline
  uint32 __forceinline __flatten millis32();
#pragma GCC diagnostic ignored "-Wattributes" // annoying warning about __forceinline
  uint24 __forceinline __flatten millis24();
#pragma GCC diagnostic ignored "-Wattributes" // annoying warning about __forceinline
  uint16 __forceinline __flatten millis16();
#pragma GCC diagnostic ignored "-Wattributes" // annoying warning about __forceinline
  uint8 __forceinline __flatten millis8();
}
