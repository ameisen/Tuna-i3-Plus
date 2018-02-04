/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Conditionals_post.h
 * Defines that depend on configuration but are not editable.
 */

#ifndef CONDITIONALS_POST_H
#define CONDITIONALS_POST_H

  /**
   * Axis lengths and center
   */
  #define X_MAX_LENGTH (X_MAX_POS - (X_MIN_POS))
  #define Y_MAX_LENGTH (Y_MAX_POS - (Y_MIN_POS))
  #define Z_MAX_LENGTH (Z_MAX_POS - (Z_MIN_POS))
  #define X_CENTER float((X_MIN_POS + X_MAX_POS) * 0.5)
  #define Y_CENTER float((Y_MIN_POS + Y_MAX_POS) * 0.5)
  #define Z_CENTER float((Z_MIN_POS + Z_MAX_POS) * 0.5)

  /**
   * CoreXY, CoreXZ, and CoreYZ - and their reverse
   */
  #define CORE_IS_XY 0
  #define CORE_IS_XZ 0
  #define CORE_IS_YZ 0
  #define IS_CORE 0
  #define IS_SCARA 0
  #define IS_KINEMATIC 0
  #define IS_CARTESIAN 1

  /**
   * Set the home position based on settings or manual overrides
   */
  #define X_HOME_POS (X_HOME_DIR < 0 ? X_MIN_POS : X_MAX_POS)
  #define Y_HOME_POS (Y_HOME_DIR < 0 ? Y_MIN_POS : Y_MAX_POS)
  #define Z_HOME_POS (Z_HOME_DIR < 0 ? Z_MIN_POS : Z_MAX_POS)

  /**
   * Auto Bed Leveling and Z Probe Repeatability Test
   */
  #define HOMING_Z_WITH_PROBE 0

#define X_TILT_FULCRUM X_HOME_POS
#define Y_TILT_FULCRUM Y_HOME_POS

  /**
   * Host keep alive
   */
  #define DEFAULT_KEEPALIVE_INTERVAL 2

  /**
   * MAX_STEP_FREQUENCY differs for TOSHIBA
   */
  #define MAX_STEP_FREQUENCY 65535 // Max step frequency for Ultimaker (5000 pps / half step)

  // MS1 MS2 Stepper Driver Microstepping mode table
  #define MICROSTEP1 LOW,LOW
  #define MICROSTEP2 HIGH,LOW
  #define MICROSTEP4 LOW,HIGH
  #define MICROSTEP8 HIGH,HIGH
  #define MICROSTEP16 HIGH,HIGH

  /**
   * Set defaults for missing (newer) options
   */
  #ifndef DISABLE_INACTIVE_X
    #define DISABLE_INACTIVE_X DISABLE_X
  #endif
  #ifndef DISABLE_INACTIVE_Y
    #define DISABLE_INACTIVE_Y DISABLE_Y
  #endif
  #ifndef DISABLE_INACTIVE_Z
    #define DISABLE_INACTIVE_Z DISABLE_Z
  #endif
  #ifndef DISABLE_INACTIVE_E
    #define DISABLE_INACTIVE_E DISABLE_E
  #endif

  // Power Signal Control Definitions
  // By default use ATX definition
  #define HAS_POWER_SWITCH 0

  /**
   * Temp Sensor defines
   */
  #define THERMISTORHEATER_0 TEMP_SENSOR_0
  #define HEATER_0_USES_THERMISTOR

  #undef HEATER_1_MINTEMP
  #undef HEATER_1_MAXTEMP

  #undef HEATER_2_MINTEMP
  #undef HEATER_2_MAXTEMP

  #undef HEATER_3_MINTEMP
  #undef HEATER_3_MAXTEMP

  #undef HEATER_4_MINTEMP
  #undef HEATER_4_MAXTEMP

  #define THERMISTORBED TEMP_SENSOR_BED
  #define BED_USES_THERMISTOR

  /**
   * Flags for PID handling
   */
  #define HAS_PID_HEATING 1
  #define HAS_PID_FOR_BOTH 0

  /**
   * ARRAY_BY_EXTRUDERS based on EXTRUDERS
   */
  #define ARRAY_BY_EXTRUDERS(...) ARRAY_N(EXTRUDERS, __VA_ARGS__)
  #define ARRAY_BY_EXTRUDERS1(v1) ARRAY_BY_EXTRUDERS(v1, v1, v1, v1, v1, v1)

  /**
   * ARRAY_BY_HOTENDS based on HOTENDS
   */
  #define ARRAY_BY_HOTENDS(...) ARRAY_N(HOTENDS, __VA_ARGS__)
  #define ARRAY_BY_HOTENDS1(v1) ARRAY_BY_HOTENDS(v1, v1, v1, v1, v1, v1)

  // Is an endstop plug used for the Z2 endstop or the bed probe?
  #define IS_Z2_OR_PROBE(A,M) ( \
       (ENABLED(Z_DUAL_ENDSTOPS) && Z2_USE_ENDSTOP == _##A##M##_) \
    || (ENABLED(Z_MIN_PROBE_ENDSTOP) && Z_MIN_PROBE_PIN == A##_##M##_PIN ) )

  /**
   * Shorthand for pin tests, used wherever needed
   */

  // Steppers
  #define HAS_X_ENABLE      1
  #define HAS_X_DIR         1
  #define HAS_X_STEP        1
  #define HAS_X_MICROSTEPS  0

  #define HAS_X2_ENABLE     0
  #define HAS_X2_DIR        0
  #define HAS_X2_STEP       0
  #define HAS_Y_MICROSTEPS  0

  #define HAS_Y_ENABLE      1
  #define HAS_Y_DIR         1
  #define HAS_Y_STEP        1
  #define HAS_Z_MICROSTEPS  0

  #define HAS_Y2_ENABLE     0
  #define HAS_Y2_DIR        0
  #define HAS_Y2_STEP       0

  #define HAS_Z_ENABLE      1
  #define HAS_Z_DIR         1
  #define HAS_Z_STEP        1

  #define HAS_Z2_ENABLE     0
  #define HAS_Z2_DIR        0
  #define HAS_Z2_STEP       0

  // Extruder steppers and solenoids
  #define HAS_E0_ENABLE     1
  #define HAS_E0_DIR        1
  #define HAS_E0_STEP       1
  #define HAS_E0_MICROSTEPS 0
  #define HAS_SOLENOID_0    0

  #define HAS_E1_ENABLE     0
  #define HAS_E1_DIR        0
  #define HAS_E1_STEP       0
  #define HAS_E1_MICROSTEPS 0
  #define HAS_SOLENOID_1    0

  #define HAS_E2_ENABLE     0
  #define HAS_E2_DIR        0
  #define HAS_E2_STEP       0
  #define HAS_E2_MICROSTEPS 0
  #define HAS_SOLENOID_2    0

  #define HAS_E3_ENABLE     0
  #define HAS_E3_DIR        0
  #define HAS_E3_STEP       0
  #define HAS_E3_MICROSTEPS 0
  #define HAS_SOLENOID_3    0

  #define HAS_E4_ENABLE     0
  #define HAS_E4_DIR        0
  #define HAS_E4_STEP       0
  #define HAS_E4_MICROSTEPS 0
  #define HAS_SOLENOID_4    0

  // Endstops and bed probe
  #define HAS_X_MIN 1
  #define HAS_X_MAX 0
  #define HAS_Y_MIN 1
  #define HAS_Y_MAX 0
  #define HAS_Z_MIN 1
  #define HAS_Z_MAX 0
  #define HAS_Z2_MIN 0
  #define HAS_Z2_MAX 0
  #define HAS_Z_MIN_PROBE_PIN 0

  // Thermistors
  #define HAS_TEMP_0 1
  #define HAS_TEMP_1 0
  #define HAS_TEMP_2 0
  #define HAS_TEMP_3 0
  #define HAS_TEMP_4 0
  #define HAS_TEMP_HOTEND 1
  #define HAS_TEMP_BED 1

  // Heaters
  #define HAS_HEATER_0 1
  #define HAS_HEATER_1 0
  #define HAS_HEATER_2 0
  #define HAS_HEATER_3 0
  #define HAS_HEATER_4 0
  #define HAS_HEATER_BED 1

  // Thermal protection
  #define HAS_THERMALLY_PROTECTED_BED 1
  #define WATCH_HOTENDS 1
  #define WATCH_THE_BED 1

  // Auto fans
  #define HAS_AUTO_FAN_0 0
  #define HAS_AUTO_FAN_1 0
  #define HAS_AUTO_FAN_2 0
  #define HAS_AUTO_FAN_3 0
  #define HAS_AUTO_FAN_4 0
  #define HAS_AUTO_FAN 0
  #define AUTO_1_IS_0 1
  #define AUTO_2_IS_0 1
  #define AUTO_2_IS_1 1
  #define AUTO_3_IS_0 1
  #define AUTO_3_IS_1 1
  #define AUTO_3_IS_2 1
  #define AUTO_4_IS_0 1
  #define AUTO_4_IS_1 1
  #define AUTO_4_IS_2 1
  #define AUTO_4_IS_3 1

  // Other fans
  #define HAS_FAN0 1
  #define HAS_FAN1 0
  #define HAS_FAN2 0
  #define HAS_CONTROLLER_FAN 0

  // Servos
  #define HAS_SERVOS 0
  #define HAS_SERVO_0 0
  #define HAS_SERVO_1 0
  #define HAS_SERVO_2 0
  #define HAS_SERVO_3 0

  // Sensors
  #define HAS_FILAMENT_WIDTH_SENSOR 0
  #define HAS_FIL_RUNOUT 0

  // User Interface
  #define HAS_HOME 0
  #define HAS_KILL 0
  #define HAS_SUICIDE 0
  #define HAS_PHOTOGRAPH 0
  #define HAS_BUZZER 0
  #define HAS_CASE_LIGHT 0

  // Digital control
  #define HAS_MICROSTEPS 0
  #define HAS_STEPPER_RESET 0
  #define HAS_DIGIPOTSS 0
  #define HAS_MOTOR_CURRENT_PWM 0

  /**
   * Helper Macros for heaters and extruder fan
   */
  #define WRITE_HEATER_0P(v) WRITE(HEATER_0_PIN, v)
  #define WRITE_HEATER_0(v) WRITE_HEATER_0P(v)
  #define WRITE_HEATER_BED(v) WRITE(HEATER_BED_PIN, v)

  /**
   * Up to 3 PWM fans
   */
  #define FAN_COUNT 1

  #define WRITE_FAN(v) WRITE(FAN_PIN, v)
  #define WRITE_FAN0(v) WRITE_FAN(v)
  #define WRITE_FAN_N(n, v) WRITE_FAN##n(v)


  /**
   * Heater & Fan Pausing
   */
  #define QUIET_PROBING 0
  #define HEATER_IDLE_HANDLER 0

  #define PROBE_PIN_CONFIGURED 0

  #define HAS_BED_PROBE 0

  /**
   * Bed Probe dependencies
   */
	#undef X_PROBE_OFFSET_FROM_EXTRUDER
	#undef Y_PROBE_OFFSET_FROM_EXTRUDER
	#undef Z_PROBE_OFFSET_FROM_EXTRUDER
	#define X_PROBE_OFFSET_FROM_EXTRUDER 0
	#define Y_PROBE_OFFSET_FROM_EXTRUDER 0
	#define Z_PROBE_OFFSET_FROM_EXTRUDER 0

  /**
   * Set granular options based on the specific type of leveling
   */

  #define UBL_DELTA  0
  #define ABL_PLANAR 0
  #define ABL_GRID   0
  #define HAS_ABL    0
  #define HAS_LEVELING          0
  #define PLANNER_LEVELING      0
  #define HAS_PROBING_PROCEDURE 0

  /**
   * Buzzer/Speaker
   */
#ifndef LCD_FEEDBACK_FREQUENCY_HZ
    #define LCD_FEEDBACK_FREQUENCY_HZ 5000
#endif
#ifndef LCD_FEEDBACK_FREQUENCY_DURATION_MS
    #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2
#endif

  /**
   * Z_HOMING_HEIGHT / Z_CLEARANCE_BETWEEN_PROBES
   */
  #define Z_HOMING_HEIGHT Z_CLEARANCE_BETWEEN_PROBES
  #define MANUAL_PROBE_HEIGHT Z_HOMING_HEIGHT

    // Boundaries for probing based on set limits
    #define MIN_PROBE_X (max(X_MIN_POS, X_MIN_POS + X_PROBE_OFFSET_FROM_EXTRUDER))
    #define MAX_PROBE_X (min(X_MAX_POS, X_MAX_POS + X_PROBE_OFFSET_FROM_EXTRUDER))
    #define MIN_PROBE_Y (max(Y_MIN_POS, Y_MIN_POS + Y_PROBE_OFFSET_FROM_EXTRUDER))
    #define MAX_PROBE_Y (min(Y_MAX_POS, Y_MAX_POS + Y_PROBE_OFFSET_FROM_EXTRUDER))

  // Stepper pulse duration, in cycles
  #define STEP_PULSE_CYCLES ((MINIMUM_STEPPER_PULSE) * CYCLES_PER_MICROSECOND)

  // Updated G92 behavior shifts the workspace
  #define HAS_POSITION_SHIFT 1
  // The home offset also shifts the coordinate space
  #define HAS_HOME_OFFSET 1
  // Either offset yields extra calculations on all moves
  #define HAS_WORKSPACE_OFFSET 1
  // M206 doesn't apply to DELTA
  #define HAS_M206_COMMAND 1

  // LCD timeout to status screen default is 15s
  #define LCD_TIMEOUT_TO_STATUS 15000

  // Shorthand
  #define GRID_MAX_POINTS ((GRID_MAX_POINTS_X) * (GRID_MAX_POINTS_Y))

  // Add commands that need sub-codes to this list
  #define USE_GCODE_SUBCODES 0

#endif // CONDITIONALS_POST_H
