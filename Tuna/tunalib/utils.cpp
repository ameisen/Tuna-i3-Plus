#include <tuna.h>

/* Taken from Arduino wiring.c, reworked for 24 and 16-bit usage. */
extern volatile uint32 timer0_millis;

namespace Tuna
{
#pragma GCC diagnostic ignored "-Wattributes" // annoying warning about __forceinline
  uint32 __forceinline __flatten millis32()
  {
    extern volatile uint32 timer0_millis;
    // disable interrupts while we read timer0_millis or we might get an
    // inconsistent value (e.g. in the middle of a write to timer0_millis)
    critical_section _critsec;
    return (volatile uint32 &)::timer0_millis;
  }

#pragma GCC diagnostic ignored "-Wattributes" // annoying warning about __forceinline
	uint24 __forceinline __flatten millis24()
	{
		// disable interrupts while we read timer0_millis or we might get an
		// inconsistent value (e.g. in the middle of a write to timer0_millis)
		critical_section _critsec;
		return (volatile uint24 &)::timer0_millis;
	}

#pragma GCC diagnostic ignored "-Wattributes" // annoying warning about __forceinline
	uint16 __forceinline __flatten millis16()
	{
		// disable interrupts while we read timer0_millis or we might get an
		// inconsistent value (e.g. in the middle of a write to timer0_millis)
		critical_section _critsec;
		return (volatile uint16 &)::timer0_millis;
	}

#pragma GCC diagnostic ignored "-Wattributes" // annoying warning about __forceinline
  uint8 __forceinline __flatten millis8()
  {
    // disable interrupts while we read timer0_millis or we might get an
    // inconsistent value (e.g. in the middle of a write to timer0_millis)
    // critical_section _critsec;
    return (volatile uint8 & __restrict)::timer0_millis;
  }
}
