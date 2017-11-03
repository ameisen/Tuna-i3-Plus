#pragma once

namespace Tuna::_ThermistorUtils
{
  template <typename T>
  struct TablePairBase final
  {
    flash<T> Adc = 0;
    flash<T> Temperature = 0;

    constexpr TablePairBase() = default;
    constexpr TablePairBase(T _ADC, T _Temperature) :
      Adc(_ADC * OVERSAMPLENR), Temperature(_Temperature << temp_t::fractional_bits)
    {}
  };
}
