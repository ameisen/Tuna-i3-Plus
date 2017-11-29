#pragma once

// Get rid of any macros that someone already may have defined.
#undef cli
#undef sei

namespace Tuna::intrinsic
{
  constexpr inline __forceinline __flatten void sei()
  {
    __builtin_avr_sei();
  }

  constexpr inline __forceinline __flatten void cli()
  {
    __builtin_avr_cli();
  }

  constexpr inline __forceinline __flatten void nop()
  {
    __builtin_avr_nop();
  }

  constexpr inline __forceinline __flatten void sleep()
  {
    __builtin_avr_sleep();
  }

  constexpr inline __forceinline __flatten void wdr()
  {
    __builtin_avr_wdr();
  }

  constexpr inline __forceinline __flatten uint8 nibble_swap(uint8 val)
  {
    return __builtin_avr_swap(val);
  }

  constexpr inline __forceinline __flatten uint16 fmul(uint8 val0, uint8 val1)
  {
    return __builtin_avr_fmul(val0, val1);
  }

  constexpr inline __forceinline __flatten int16 fmuls(int8 val0, int8 val1)
  {
    return __builtin_avr_fmuls(val0, val1);
  }

  constexpr inline __forceinline __flatten int16 fmulsu(int8 val0, uint8 val1)
  {
    return __builtin_avr_fmulsu(val0, val1);
  }

  constexpr inline __forceinline __flatten void delay_cycles(arg_type<uint32> ticks)
  {
    __builtin_avr_delay_cycles(ticks);
  }

  constexpr inline __forceinline __flatten uint8 insert_bits(arg_type<uint32> map, uint8 bits, uint8 val)
  {
    return __builtin_avr_insert_bits(map, bits, val);
  }
}
