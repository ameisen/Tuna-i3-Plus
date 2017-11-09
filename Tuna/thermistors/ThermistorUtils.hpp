#pragma once

namespace Tuna::_ThermistorUtils
{
  template <typename T>
  struct TablePairBase final
  {
    const flash<T> Adc;
    const flash<T> Temperature;

    constexpr TablePairBase() = default;
    constexpr TablePairBase(T _ADC, T _Temperature) :
      Adc(_ADC * OVERSAMPLENR), Temperature(_Temperature << temp_t::fractional_bits)
    {}
  };
}
