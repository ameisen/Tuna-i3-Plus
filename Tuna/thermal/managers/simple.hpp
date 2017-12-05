#pragma once

#include "thermal/thermal.hpp"

namespace Tuna::Thermal::Manager
{
  // This is a struct and not a namespace only as classes are easier to inject into templates.
  struct Simple final : trait::ce_only
  {
    struct calibration final
    {
      float Exponent_ = -1.0f;
      uint8 Scalar_ = 3_u8;
    };

    static bool calibrate(arg_type<temp_t> target);
    static uint8 get_power(arg_type<temp_t> current, arg_type<temp_t> target);
    static __pure void debug_dump();

    static __pure const calibration & GetCalibration();
    static void SetCalibration(arg_type<calibration> val);
  };
}
