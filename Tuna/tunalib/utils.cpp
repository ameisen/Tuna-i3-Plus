#import <tuna.h>

/* Taken from Arduino wiring.c, reworked for 24 and 16-bit usage. */
extern "C" volatile uint32 timer0_millis;

namespace Tuna::utils
{
  uint32 millis32()
  {
    // disable interrupts while we read timer0_millis or we might get an
    // inconsistent value (e.g. in the middle of a write to timer0_millis)
    critical_section _critsec;
    return (volatile uint32 &)timer0_millis;
  }

	uint24 millis24()
	{
		// disable interrupts while we read timer0_millis or we might get an
		// inconsistent value (e.g. in the middle of a write to timer0_millis)
		critical_section _critsec;
		return (volatile uint24 &)timer0_millis;
	}

	uint16 millis16()
	{
		// disable interrupts while we read timer0_millis or we might get an
		// inconsistent value (e.g. in the middle of a write to timer0_millis)
		critical_section _critsec;
		return (volatile uint16 &)timer0_millis;
	}

  uint8 millis8()
  {
    // disable interrupts while we read timer0_millis or we might get an
    // inconsistent value (e.g. in the middle of a write to timer0_millis)
    critical_section _critsec;
    return (volatile uint8 &)timer0_millis;
  }
}
