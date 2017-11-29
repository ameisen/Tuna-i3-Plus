#pragma once

#include "avr/interrupt.h"

// Get rid of any macros that someone already may have defined.
#undef cli
#undef sei
#undef min
#undef max
#undef clamp

namespace Tuna
{
  class alignas(uint8)critical_section final
  {
    const uint8 m_sReg = SREG;
  public:
    critical_section(const critical_section &) = delete;
    critical_section(critical_section &&) = delete;
    critical_section & operator = (const critical_section &) = delete;
    critical_section & operator = (critical_section &&) = delete;

    inline __forceinline __flatten critical_section()
    {
      intrinsic::cli();
    }
    inline __forceinline __flatten ~critical_section()
    {
      SREG = m_sReg;
    }
  };

  class critical_section_not_isr final
  {
  public:
    critical_section_not_isr(const critical_section_not_isr &) = delete;
    critical_section_not_isr(critical_section_not_isr &&) = delete;
    critical_section_not_isr & operator = (const critical_section_not_isr &) = delete;
    critical_section_not_isr & operator = (critical_section_not_isr &&) = delete;

    inline __forceinline __flatten critical_section_not_isr()
    {
      intrinsic::cli();
    }
    inline __forceinline __flatten ~critical_section_not_isr()
    {
      intrinsic::sei();
    }
  };
}
