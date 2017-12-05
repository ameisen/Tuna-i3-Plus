#pragma once

namespace Tuna::VM
{
  template <typename T> constexpr const T NONE;
  template<> constexpr const float NONE<float> = type_trait<float>::max;
  template<> constexpr const int32 NONE<int32> = type_trait<int32>::max;

  extern void await_queue();

  template <MovementType speed, MovementMode move_mode>
  void linear_move(arg_type<float> X, arg_type<float> Y, arg_type<float> Z, arg_type<float> E = NONE<float>, arg_type<float> FeedRate = NONE<float>);

  template <MovementType speed, MovementMode move_mode>
  void extrude(arg_type<float> E, arg_type<float> FeedRate = NONE<float>);
}

inline void Tuna::VM::await_queue()
{
}

template <MovementType speed, MovementMode move_mode>
inline void Tuna::VM::linear_move(arg_type<float> X, arg_type<float> Y, arg_type<float> Z, arg_type<float> E, arg_type<float> FeedRate)
{
  if (__unlikely(!is_running()))
  {
    return;
  }

  constexpr const auto none = NONE<float>;
  constexpr const bool feed_param = (speed != MovementType::Rapid);

  float max_feedrate = 0.0f;

  // TODO Fix for this:
#if ENABLED(INCH_MODE_SUPPORT)
  static_assert(false, "Need to add inch mode support.");
#endif

  const auto axial_move = [&](AxisEnum axis, arg_type<float> value)
  {
    if (value != none)
    {
      MovementMode move_mode_local = move_mode;
      if (move_mode == MovementMode::Modal)
      {
        move_mode_local = (axis_relative_modes[axis] || relative_mode) ? MovementMode::Relative : MovementMode::Absolute;
      }

      if (move_mode_local == MovementMode::Relative)
      {
        destination[axis] = value + current_position[axis];
      }
      else // Absolute
      {
        destination[axis] = value;
      }

      if constexpr (!feed_param)
      {
        max_feedrate = max(max_feedrate, planner.max_feedrate_mm_s[axis]);
      }
    }
    else
    {
      destination[uint8(axis)] = current_position[uint8(axis)];
    }
  };

  axial_move(X_AXIS, X);
  axial_move(Y_AXIS, Y);
  axial_move(Z_AXIS, Z);
  axial_move(E_AXIS, E);

  if constexpr (feed_param)
  {
    if (FeedRate != none)
    {
      feedrate_mm_s = MMM_TO_MMS(FeedRate);
    }
  }
  else
  {
    feedrate_mm_s = max_feedrate;
  }

  if (E != none)
  {
    if (!DEBUGGING(DRYRUN))
    {
      if constexpr (move_mode == MovementMode::Relative)
      {
        print_job_timer.incFilamentUsed(E);
      }
      else
      {
        print_job_timer.incFilamentUsed(destination[E_AXIS] - current_position[E_AXIS]);
      }
    }
  }

  // perform move.
  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();

  if (E != none && !DEBUGGING(DRYRUN)) {
    if (Temperature::is_coldextrude()) {
      current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
      SERIAL_ECHO_START();
      SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
    }
#if ENABLED(PREVENT_LENGTHY_EXTRUDE)
    if (destination[E_AXIS] - current_position[E_AXIS] > EXTRUDE_MAXLENGTH) {
      current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
      SERIAL_ECHO_START();
      SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
    }
#endif
  }

  if (prepare_move_to_destination_cartesian())
    return;

  set_current_to_destination();
}

template <MovementType speed, MovementMode move_mode>
inline void Tuna::VM::extrude(arg_type<float> E, arg_type<float> FeedRate)
{
  if (__unlikely(!is_running()))
  {
    return;
  }

  constexpr const auto none = NONE<float>;
  constexpr const bool feed_param = (speed != MovementType::Rapid);

  float max_feedrate;

  // TODO Fix for this:
#if ENABLED(INCH_MODE_SUPPORT)
  static_assert(false, "Need to add inch mode support.");
#endif

  MovementMode move_mode_local = move_mode;
  if (move_mode == MovementMode::Modal)
  {
    move_mode_local = (axis_relative_modes[E_AXIS] || relative_mode) ? MovementMode::Relative : MovementMode::Absolute;
  }

  if (move_mode_local == MovementMode::Relative)
  {
    destination[E_AXIS] = E + current_position[E_AXIS];
  }
  else // Absolute
  {
    destination[E_AXIS] = E;
  }

  if constexpr (!feed_param)
  {
    max_feedrate = planner.max_feedrate_mm_s[E_AXIS];
  }

  if constexpr (feed_param)
  {
    if (FeedRate != none)
    {
      feedrate_mm_s = MMM_TO_MMS(FeedRate);
    }
  }
  else
  {
    feedrate_mm_s = max_feedrate;
  }

  if (!DEBUGGING(DRYRUN))
  {
    if constexpr (move_mode == MovementMode::Relative)
    {
      print_job_timer.incFilamentUsed(E);
    }
    else
    {
      print_job_timer.incFilamentUsed(destination[E_AXIS] - current_position[E_AXIS]);
    }
  }

  // perform move.
  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();

  if (!DEBUGGING(DRYRUN)) {
    if (Temperature::is_coldextrude()) {
      current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
      SERIAL_ECHO_START();
      SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
    }
#if ENABLED(PREVENT_LENGTHY_EXTRUDE)
    if (destination[E_AXIS] - current_position[E_AXIS] > EXTRUDE_MAXLENGTH) {
      current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
      SERIAL_ECHO_START();
      SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
    }
#endif
  }

  if (prepare_move_to_destination_cartesian())
    return;

  set_current_to_destination();
}
