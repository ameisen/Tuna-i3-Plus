#pragma once

#include "thermal/thermal.hpp"

namespace Tuna::Thermal::Manager
{
  // This is a struct and not a namespace only as classes are easier to inject into templates.
  struct Simple final : trait::ce_only
  {
    static bool calibrate(arg_type<temp_t> target);
    static uint8 get_power(arg_type<temp_t> current, arg_type<temp_t> target);
    static __pure void debug_dump();

    static __pure float GetCalibration();
    static void SetCalibration(arg_type<float> val);
  };
}
