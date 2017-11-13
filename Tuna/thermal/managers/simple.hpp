#pragma once

#include <tuna.h>

#include "thermal/thermal.hpp"

namespace Tuna::Thermal::Manager
{
  // This is a struct and not a namespace only as classes are easier to inject into templates.
  struct Simple final : ce_only
  {
    static bool calibrate(arg_type<temp_t> target);
    static bool calibrating();
    static uint8 get_power(arg_type<temp_t> current, arg_type<temp_t> target);
    static void debug_dump();

    static pair<float, float> GetCalibration();
    static void SetCalibration(arg_type<pair<float, float>> val);
  };
}
