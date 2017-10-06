/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016, 2017 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
  * About Marlin
  *
  * This firmware is a mashup between Sprinter and grbl.
  *  - https://github.com/kliment/Sprinter
  *  - https://github.com/simen/grbl/tree
  */

  /**
   * -----------------
   * G-Codes in Marlin
   * -----------------
   *
   * Helpful G-code references:
   *  - http://linuxcnc.org/handbook/gcode/g-code.html
   *  - http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes
   *
   * Help to document Marlin's G-codes online:
   *  - http://reprap.org/wiki/G-code
   *  - https://github.com/MarlinFirmware/MarlinDocumentation
   *
   * -----------------
   *
   * "G" Codes
   *
   * G0   -> G1
   * G1   - Coordinated Movement X Y Z E
   * G2   - CW ARC
   * G3   - CCW ARC
   * G4   - Dwell S<seconds> or P<milliseconds>
   * G5   - Cubic B-spline with XYZE destination and IJPQ offsets
   * G10  - Retract filament according to settings of M207
   * G11  - Retract recover filament according to settings of M208
   * G12  - Clean tool
   * G17  - Select Plane XY (Requires CNC_WORKSPACE_PLANES)
   * G18  - Select Plane ZX (Requires CNC_WORKSPACE_PLANES)
   * G19  - Select Plane YZ (Requires CNC_WORKSPACE_PLANES)
   * G20  - Set input units to inches
   * G21  - Set input units to millimeters
   * G26  - Mesh Validation Pattern (Requires UBL_G26_MESH_VALIDATION)
   * G27  - Park Nozzle (Requires NOZZLE_PARK_FEATURE)
   * G28  - Home one or more axes
   * G29  - Detailed Z probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
   * G30  - Single Z probe, probes bed at X Y location (defaults to current XY location)
   * G31  - Dock sled (Z_PROBE_SLED only)
   * G32  - Undock sled (Z_PROBE_SLED only)
   * G33  - Delta Auto-Calibration (Requires DELTA_AUTO_CALIBRATION)
   * G38  - Probe target - similar to G28 except it uses the Z_MIN_PROBE for all three axes
   * G42  - Coordinated move to a mesh point (Requires AUTO_BED_LEVELING_UBL)
   * G90  - Use Absolute Coordinates
   * G91  - Use Relative Coordinates
   * G92  - Set current position to coordinates given
   *
   * "M" Codes
   *
   * M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
   * M1   -> M0
   * M3   - Turn laser/spindle on, set spindle/laser speed/power, set rotation to clockwise
   * M4   - Turn laser/spindle on, set spindle/laser speed/power, set rotation to counter-clockwise
   * M5   - Turn laser/spindle off
   * M17  - Enable/Power all stepper motors
   * M18  - Disable all stepper motors; same as M84
   * M20  - List SD card. (Requires SDSUPPORT)
   * M21  - Init SD card. (Requires SDSUPPORT)
   * M22  - Release SD card. (Requires SDSUPPORT)
   * M23  - Select SD file: "M23 /path/file.gco". (Requires SDSUPPORT)
   * M24  - Start/resume SD print. (Requires SDSUPPORT)
   * M25  - Pause SD print. (Requires SDSUPPORT)
   * M26  - Set SD position in bytes: "M26 S12345". (Requires SDSUPPORT)
   * M27  - Report SD print status. (Requires SDSUPPORT)
   * M28  - Start SD write: "M28 /path/file.gco". (Requires SDSUPPORT)
   * M29  - Stop SD write. (Requires SDSUPPORT)
   * M30  - Delete file from SD: "M30 /path/file.gco"
   * M31  - Report time since last M109 or SD card start to serial.
   * M32  - Select file and start SD print: "M32 [S<bytepos>] !/path/file.gco#". (Requires SDSUPPORT)
   *        Use P to run other files as sub-programs: "M32 P !filename#"
   *        The '#' is necessary when calling from within sd files, as it stops buffer prereading
   * M33  - Get the longname version of a path. (Requires LONG_FILENAME_HOST_SUPPORT)
   * M34  - Set SD Card sorting options. (Requires SDCARD_SORT_ALPHA)
   * M42  - Change pin status via gcode: M42 P<pin> S<value>. LED pin assumed if P is omitted.
   * M43  - Display pin status, watch pins for changes, watch endstops & toggle LED, Z servo probe test, toggle pins
   * M48  - Measure Z Probe repeatability: M48 P<points> X<pos> Y<pos> V<level> E<engage> L<legs>. (Requires Z_MIN_PROBE_REPEATABILITY_TEST)
   * M75  - Start the print job timer.
   * M76  - Pause the print job timer.
   * M77  - Stop the print job timer.
   * M78  - Show statistical information about the print jobs. (Requires PRINTCOUNTER)
   * M80  - Turn on Power Supply. (Requires POWER_SUPPLY > 0)
   * M81  - Turn off Power Supply. (Requires POWER_SUPPLY > 0)
   * M82  - Set E codes absolute (default).
   * M83  - Set E codes relative while in Absolute (G90) mode.
   * M84  - Disable steppers until next move, or use S<seconds> to specify an idle
   *        duration after which steppers should turn off. S0 disables the timeout.
   * M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
   * M92  - Set planner.axis_steps_per_mm for one or more axes.
   * M100 - Watch Free Memory (for debugging) (Requires M100_FREE_MEMORY_WATCHER)
   * M104 - Set extruder target temp.
   * M105 - Report current temperatures.
   * M106 - Fan on.
   * M107 - Fan off.
   * M108 - Break out of heating loops (M109, M190, M303). With no controller, breaks out of M0/M1. (Requires EMERGENCY_PARSER)
   * M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
   *        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
   *        If AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
   * M110 - Set the current line number. (Used by host printing)
   * M111 - Set debug flags: "M111 S<flagbits>". See flag bits defined in enum.h.
   * M112 - Emergency stop.
   * M113 - Get or set the timeout interval for Host Keepalive "busy" messages. (Requires HOST_KEEPALIVE_FEATURE)
   * M114 - Report current position.
   * M115 - Report capabilities. (Extended capabilities requires EXTENDED_CAPABILITIES_REPORT)
   * M117 - Display a message on the controller screen. (Requires an LCD)
   * M118 - Display a message in the host console.
   * M119 - Report endstops status.
   * M120 - Enable endstops detection.
   * M121 - Disable endstops detection.
   * M125 - Save current position and move to filament change position. (Requires PARK_HEAD_ON_PAUSE)
   * M126 - Solenoid Air Valve Open. (Requires BARICUDA)
   * M127 - Solenoid Air Valve Closed. (Requires BARICUDA)
   * M128 - EtoP Open. (Requires BARICUDA)
   * M129 - EtoP Closed. (Requires BARICUDA)
   * M140 - Set bed target temp. S<temp>
   * M145 - Set heatup values for materials on the LCD. H<hotend> B<bed> F<fan speed> for S<material> (0=PLA, 1=ABS)
   * M149 - Set temperature units. (Requires TEMPERATURE_UNITS_SUPPORT)
   * M150 - Set Status LED Color as R<red> U<green> B<blue>. Values 0-255. (Requires BLINKM, RGB_LED, RGBW_LED, or PCA9632)
   * M155 - Auto-report temperatures with interval of S<seconds>. (Requires AUTO_REPORT_TEMPERATURES)
   * M163 - Set a single proportion for a mixing extruder. (Requires MIXING_EXTRUDER)
   * M164 - Save the mix as a virtual extruder. (Requires MIXING_EXTRUDER and MIXING_VIRTUAL_TOOLS)
   * M165 - Set the proportions for a mixing extruder. Use parameters ABCDHI to set the mixing factors. (Requires MIXING_EXTRUDER)
   * M190 - Sxxx Wait for bed current temp to reach target temp. ** Waits only when heating! **
   *        Rxxx Wait for bed current temp to reach target temp. ** Waits for heating or cooling. **
   * M200 - Set filament diameter, D<diameter>, setting E axis units to cubic. (Use S0 to revert to linear units.)
   * M201 - Set max acceleration in units/s^2 for print moves: "M201 X<accel> Y<accel> Z<accel> E<accel>"
   * M202 - Set max acceleration in units/s^2 for travel moves: "M202 X<accel> Y<accel> Z<accel> E<accel>" ** UNUSED IN MARLIN! **
   * M203 - Set maximum feedrate: "M203 X<fr> Y<fr> Z<fr> E<fr>" in units/sec.
   * M204 - Set default acceleration in units/sec^2: P<printing> R<extruder_only> T<travel>
   * M205 - Set advanced settings. Current units apply:
			  S<print> T<travel> minimum speeds
			  B<minimum segment time>
			  X<max X jerk>, Y<max Y jerk>, Z<max Z jerk>, E<max E jerk>
   * M206 - Set additional homing offset. (Disabled by NO_WORKSPACE_OFFSETS or DELTA)
   * M207 - Set Retract Length: S<length>, Feedrate: F<units/min>, and Z lift: Z<distance>. (Requires FWRETRACT)
   * M208 - Set Recover (unretract) Additional (!) Length: S<length> and Feedrate: F<units/min>. (Requires FWRETRACT)
   * M209 - Turn Automatic Retract Detection on/off: S<0|1> (For slicers that don't support G10/11). (Requires FWRETRACT)
			Every normal extrude-only move will be classified as retract depending on the direction.
   * M211 - Enable, Disable, and/or Report software endstops: S<0|1> (Requires MIN_SOFTWARE_ENDSTOPS or MAX_SOFTWARE_ENDSTOPS)
   * M218 - Set a tool offset: "M218 T<index> X<offset> Y<offset>". (Requires 2 or more extruders)
   * M220 - Set Feedrate Percentage: "M220 S<percent>" (i.e., "FR" on the LCD)
   * M221 - Set Flow Percentage: "M221 S<percent>"
   * M226 - Wait until a pin is in a given state: "M226 P<pin> S<state>"
   * M240 - Trigger a camera to take a photograph. (Requires CHDK or PHOTOGRAPH_PIN)
   * M250 - Set LCD contrast: "M250 C<contrast>" (0-63). (Requires LCD support)
   * M260 - i2c Send Data (Requires EXPERIMENTAL_I2CBUS)
   * M261 - i2c Request Data (Requires EXPERIMENTAL_I2CBUS)
   * M280 - Set servo position absolute: "M280 P<index> S<angle|µs>". (Requires servos)
   * M300 - Play beep sound S<frequency Hz> P<duration ms>
   * M301 - Set PID parameters P I and D. (Requires PIDTEMP)
   * M302 - Allow cold extrudes, or set the minimum extrude S<temperature>. (Requires PREVENT_COLD_EXTRUSION)
   * M303 - PID relay autotune S<temperature> sets the target temperature. Default 150C. (Requires PIDTEMP)
   * M304 - Set bed PID parameters P I and D. (Requires PIDTEMPBED)
   * M350 - Set microstepping mode. (Requires digital microstepping pins.)
   * M351 - Toggle MS1 MS2 pins directly. (Requires digital microstepping pins.)
   * M355 - Set Case Light on/off and set brightness. (Requires CASE_LIGHT_PIN)
   * M380 - Activate solenoid on active extruder. (Requires EXT_SOLENOID)
   * M381 - Disable all solenoids. (Requires EXT_SOLENOID)
   * M400 - Finish all moves.
   * M401 - Lower Z probe. (Requires a probe)
   * M402 - Raise Z probe. (Requires a probe)
   * M404 - Display or set the Nominal Filament Width: "W<diameter>". (Requires FILAMENT_WIDTH_SENSOR)
   * M405 - Enable Filament Sensor flow control. "M405 D<delay_cm>". (Requires FILAMENT_WIDTH_SENSOR)
   * M406 - Disable Filament Sensor flow control. (Requires FILAMENT_WIDTH_SENSOR)
   * M407 - Display measured filament diameter in millimeters. (Requires FILAMENT_WIDTH_SENSOR)
   * M410 - Quickstop. Abort all planned moves.
   * M420 - Enable/Disable Leveling (with current values) S1=enable S0=disable (Requires MESH_BED_LEVELING or ABL)
   * M421 - Set a single Z coordinate in the Mesh Leveling grid. X<units> Y<units> Z<units> (Requires MESH_BED_LEVELING or AUTO_BED_LEVELING_UBL)
   * M428 - Set the home_offset based on the current_position. Nearest edge applies. (Disabled by NO_WORKSPACE_OFFSETS or DELTA)
   * M500 - Store parameters in EEPROM. (Requires EEPROM_SETTINGS)
   * M501 - Restore parameters from EEPROM. (Requires EEPROM_SETTINGS)
   * M502 - Revert to the default "factory settings". ** Does not write them to EEPROM! **
   * M503 - Print the current settings (in memory): "M503 S<verbose>". S0 specifies compact output.
   * M540 - Enable/disable SD card abort on endstop hit: "M540 S<state>". (Requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
   * M600 - Pause for filament change: "M600 X<pos> Y<pos> Z<raise> E<first_retract> L<later_retract>". (Requires ADVANCED_PAUSE_FEATURE)
   * M665 - Set delta configurations: "M665 L<diagonal rod> R<delta radius> S<segments/s> A<rod A trim mm> B<rod B trim mm> C<rod C trim mm> I<tower A trim angle> J<tower B trim angle> K<tower C trim angle>" (Requires DELTA)
   * M666 - Set delta endstop adjustment. (Requires DELTA)
   * M605 - Set dual x-carriage movement mode: "M605 S<mode> [X<x_offset>] [R<temp_offset>]". (Requires DUAL_X_CARRIAGE)
   * M851 - Set Z probe's Z offset in current units. (Negative = below the nozzle.)
   * M860 - Report the position of position encoder modules.
   * M861 - Report the status of position encoder modules.
   * M862 - Perform an axis continuity test for position encoder modules.
   * M863 - Perform steps-per-mm calibration for position encoder modules.
   * M864 - Change position encoder module I2C address.
   * M865 - Check position encoder module firmware version.
   * M866 - Report or reset position encoder module error count.
   * M867 - Enable/disable or toggle error correction for position encoder modules.
   * M868 - Report or set position encoder module error correction threshold.
   * M869 - Report position encoder module error.
   * M900 - Get and/or Set advance K factor and WH/D ratio. (Requires LIN_ADVANCE)
   * M906 - Set or get motor current in milliamps using axis codes X, Y, Z, E. Report values if no axis codes given. (Requires HAVE_TMC2130)
   * M907 - Set digital trimpot motor current using axis codes. (Requires a board with digital trimpots)
   * M908 - Control digital trimpot directly. (Requires DAC_STEPPER_CURRENT or DIGIPOTSS_PIN)
   * M909 - Print digipot/DAC current value. (Requires DAC_STEPPER_CURRENT)
   * M910 - Commit digipot/DAC value to external EEPROM via I2C. (Requires DAC_STEPPER_CURRENT)
   * M911 - Report stepper driver overtemperature pre-warn condition. (Requires HAVE_TMC2130)
   * M912 - Clear stepper driver overtemperature pre-warn condition flag. (Requires HAVE_TMC2130)
   * M913 - Set HYBRID_THRESHOLD speed. (Requires HYBRID_THRESHOLD)
   * M914 - Set SENSORLESS_HOMING sensitivity. (Requires SENSORLESS_HOMING)
   *
   * M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
   * M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
   * M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration)
   * M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
   * M364 - SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
   *
   * ************ Custom codes - This can change to suit future G-code regulations
   * M928 - Start SD logging: "M928 filename.gco". Stop with M29. (Requires SDSUPPORT)
   * M999 - Restart after being stopped by error
   *
   * "T" Codes
   *
   * T0-T3 - Select an extruder (tool) by index: "T<n> F<units/min>"
   *
   */

#include "Marlin.h"

#include "bi3_plus_lcd.h"
#include "planner.h"
#include "stepper.h"
#include "endstops.h"
#include "temperature.hpp"
#include "cardreader.h"
#include "configuration_store.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "duration_t.h"
#include "types.h"
#include "bi3_plus_lcd.h"
#include "gcode.h"

#include "planner_bezier.h"
#include "watchdog.h"

CardReader card;

bool Running = true;

uint8_t marlin_debug_flags = DEBUG_NONE;

/**
 * Cartesian Current Position
 *   Used to track the logical position as moves are queued.
 *   Used by 'line_to_current_position' to do a move after changing it.
 *   Used by 'SYNC_PLAN_POSITION_KINEMATIC' to update 'planner.position'.
 */
float current_position[XYZE] = { 0.0 };

/**
 * Cartesian Destination
 *   A temporary position, usually applied to 'current_position'.
 *   Set with 'gcode_get_destination' or 'set_destination_to_current'.
 *   'line_to_destination' sets 'current_position' to 'destination'.
 */
float destination[XYZE] = { 0.0 };

/**
 * axis_homed
 *   Flags that each linear axis was homed.
 *   XYZ on cartesian, ABC on delta, ABZ on SCARA.
 *
 * axis_known_position
 *   Flags that the position is known in each linear axis. Set when homed.
 *   Cleared whenever a stepper powers off, potentially losing its position.
 */
bool axis_homed[XYZ] = { false }, axis_known_position[XYZ] = { false };

/**
 * GCode line number handling. Hosts may opt to include line numbers when
 * sending commands to Marlin, and lines will be checked for sequentiality.
 * M110 N<int> sets the current line number.
 */
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

/**
 * GCode Command Queue
 * A simple ring buffer of BUFSIZE command strings.
 *
 * Commands are copied into this buffer by the command injectors
 * (immediate, serial, sd card) and they are processed sequentially by
 * the main loop. The process_next_command function parses the next
 * command and hands off execution to individual handler functions.
 */
uint8_t commands_in_queue = 0; // Count of commands in the queue
static uint8_t cmd_queue_index_r = 0, // Ring buffer read position
cmd_queue_index_w = 0; // Ring buffer write position
static char command_queue[BUFSIZE][MAX_CMD_SIZE];

/**
 * Next Injected Command pointer. nullptr if no commands are being injected.
 * Used by Marlin internally to ensure that commands initiated from within
 * are enqueued ahead of any pending serial or sd card commands.
 */
static const char *injected_commands_P = nullptr;

/**
 * Feed rates are often configured with mm/m
 * but the planner and stepper like mm/s units.
 */
static const float homing_feedrate_mm_s[] PROGMEM = {
  MMM_TO_MMS(HOMING_FEEDRATE_XY), MMM_TO_MMS(HOMING_FEEDRATE_XY),
  MMM_TO_MMS(HOMING_FEEDRATE_Z), 0
};
FORCE_INLINE float homing_feedrate(const AxisEnum a) { return pgm_read_float(&homing_feedrate_mm_s[a]); }

float feedrate_mm_s = MMM_TO_MMS(1500.0);
static float saved_feedrate_mm_s;
int16_t feedrate_percentage = 100, saved_feedrate_percentage,
flow_percentage[EXTRUDERS] = { 100 };

bool axis_relative_modes[] = AXIS_RELATIVE_MODES,
volumetric_enabled =
#if ENABLED(VOLUMETRIC_DEFAULT_ON)
true
#else
false
#endif
;
float filament_size[EXTRUDERS] = { DEFAULT_NOMINAL_FILAMENT_DIA },
volumetric_multiplier[EXTRUDERS] = { 1.0 };

// The distance that XYZ has been offset by G92. Reset by G28.
float position_shift[XYZ] = { 0 };
// This offset is added to the configured home position.
// Set by M206, M428, or menu item. Saved to EEPROM.
float home_offset[XYZ] = { 0 };
// The above two are combined to save on computes
float workspace_offset[XYZ] = { 0 };

// Software Endstops are based on the configured limits.
bool soft_endstops_enabled = true;
float soft_endstop_min[XYZ] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
soft_endstop_max[XYZ] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

int16_t fanSpeeds[FAN_COUNT] = { 0 };

// The active extruder (tool). Set with T<extruder> command.
uint8_t active_extruder = 0;

// Relative Mode. Enable with G91, disable with G90.
static bool relative_mode = false;

// For M109 and M190, this flag may be cleared (by M108) to exit the wait loop
volatile bool wait_for_heatup = true;

const char axis_codes[XYZE] = { 'X', 'Y', 'Z', 'E' };

// Number of characters read in the current line of serial input
static int serial_count = 0;

// Inactivity shutdown
millis_t previous_cmd_ms = 0;
static millis_t max_inactive_time = 0;
static millis_t stepper_inactive_time = (DEFAULT_STEPPER_DEACTIVE_TIME) * 1000UL;

// Print Job Timer
PrintCounter print_job_timer = PrintCounter();

// Buzzer - I2C on the LCD or a BEEPER_PIN
#define BUZZ(d,f) {}

static uint8_t target_extruder;

#define XY_PROBE_FEEDRATE_MM_S MMM_TO_MMS(XY_PROBE_SPEED)

float cartes[XYZ] = { 0 };

static bool send_ok[BUFSIZE];

MarlinBusyState busy_state = NOT_BUSY;
static millis_t next_busy_signal_ms = 0;
uint8_t host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;

FORCE_INLINE float pgm_read_any(const float *p) { return pgm_read_float_near(p); }
FORCE_INLINE signed char pgm_read_any(const signed char *p) { return pgm_read_byte_near(p); }

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const PROGMEM type array##_P[XYZ] = { X_##CONFIG, Y_##CONFIG, Z_##CONFIG }; \
  static inline type array(AxisEnum axis) { return pgm_read_any(&array##_P[axis]); } \
  typedef void __void_##CONFIG##__

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos, MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos, MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos, HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length, MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm, HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR);

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void stop();

void get_available_commands();
void process_next_command();
void prepare_move_to_destination();

void get_cartesian_from_steppers();
void set_current_from_steppers_for_axis(const AxisEnum axis);

void plan_arc(float target[XYZE], float* offset, uint8_t clockwise);

void plan_cubic_move(const float offset[4]);

void tool_change(const uint8_t tmp_extruder, const float fr_mm_s = 0.0, bool no_move = false);
void report_current_position();
void report_current_position_detail();

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void sync_plan_position() {
	planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
inline void sync_plan_position_e() { planner.set_e_position_mm(current_position[E_AXIS]); }

#define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()

#include "SdFatUtil.h"
int freeMemory() { return SdFatUtil::FreeRam(); }

/**
 * Inject the next "immediate" command, when possible, onto the front of the queue.
 * Return true if any immediate commands remain to inject.
 */
static bool drain_injected_commands_P() {
	if (injected_commands_P != nullptr) {
		size_t i = 0;
		char c, cmd[30];
		strncpy_P(cmd, injected_commands_P, sizeof(cmd) - 1);
		cmd[sizeof(cmd) - 1] = '\0';
		while ((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
		cmd[i] = '\0';
		if (enqueue_and_echo_command(cmd))     // success?
			injected_commands_P = c ? injected_commands_P + i + 1 : nullptr; // next command or done
	}
	return (injected_commands_P != nullptr);    // return whether any more remain
}

/**
 * Record one or many commands to run from program memory.
 * Aborts the current queue, if any.
 * Note: drain_injected_commands_P() must be called repeatedly to drain the commands afterwards
 */
void enqueue_and_echo_commands_P(const char * const pgcode) {
	injected_commands_P = pgcode;
	drain_injected_commands_P(); // first command executed asap (when possible)
}

/**
 * Clear the Marlin command queue
 */
void clear_command_queue() {
	cmd_queue_index_r = cmd_queue_index_w;
	commands_in_queue = 0;
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
inline void _commit_command(bool say_ok) {
	send_ok[cmd_queue_index_w] = say_ok;
	if (++cmd_queue_index_w >= BUFSIZE) cmd_queue_index_w = 0;
	commands_in_queue++;
}

/**
 * Copy a command from RAM into the main command buffer.
 * Return true if the command was successfully added.
 * Return false for a full buffer, or if the 'command' is a comment.
 */
inline bool _enqueuecommand(const char* cmd, bool say_ok = false) {
	if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;
	strcpy(command_queue[cmd_queue_index_w], cmd);
	_commit_command(say_ok);
	return true;
}

/**
 * Enqueue with Serial Echo
 */
bool enqueue_and_echo_command(const char* cmd, bool say_ok/*=false*/) {
	if (_enqueuecommand(cmd, say_ok)) {
		SERIAL_ECHO_START();
		SERIAL_ECHOPAIR(MSG_ENQUEUEING, cmd);
		SERIAL_CHAR('"');
		SERIAL_EOL();
		return true;
	}
	return false;
}

void setup_killpin() {
}

void setup_powerhold() {
}

void suicide() {
}

void servo_init() {
}

void gcode_line_error(const char* err, bool doFlush = true) {
	SERIAL_ERROR_START();
	serialprintPGM(err);
	SERIAL_ERRORLN(gcode_LastN);
	//Serial.println(gcode_N);
	if (doFlush) FlushSerialRequestResend();
	serial_count = 0;
}

/**
 * Get all commands waiting on the serial port and queue them.
 * Exit when the buffer is full or when no more characters are
 * left on the serial port.
 */
inline void get_serial_commands() {
	static char serial_line_buffer[MAX_CMD_SIZE];
	static bool serial_comment_mode = false;

	/**
	 * Loop while serial characters are incoming and the queue is not full
	 */
	while (commands_in_queue < BUFSIZE && MYSERIAL.available() > 0) {

		char serial_char = MYSERIAL.read();

		/**
		 * If the character ends the line
		 */
		if (serial_char == '\n' || serial_char == '\r') {

			serial_comment_mode = false; // end of line == end of comment

			if (!serial_count) continue; // skip empty lines

			serial_line_buffer[serial_count] = 0; // terminate string
			serial_count = 0; //reset buffer

			char* command = serial_line_buffer;

			while (*command == ' ') command++; // skip any leading spaces
			char *npos = (*command == 'N') ? command : nullptr, // Require the N parameter to start the line
				*apos = strchr(command, '*');

			if (npos) {

				bool M110 = strstr_P(command, PSTR("M110")) != nullptr;

				if (M110) {
					char* n2pos = strchr(command + 4, 'N');
					if (n2pos) npos = n2pos;
				}

				gcode_N = strtol(npos + 1, nullptr, 10);

				if (gcode_N != gcode_LastN + 1 && !M110) {
					gcode_line_error(PSTR(MSG_ERR_LINE_NO));
					return;
				}

				if (apos) {
					byte checksum = 0, count = 0;
					while (command[count] != '*') checksum ^= command[count++];

					if (strtol(apos + 1, nullptr, 10) != checksum) {
						gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH));
						return;
					}
					// if no errors, continue parsing
				}
				else {
					gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
					return;
				}

				gcode_LastN = gcode_N;
				// if no errors, continue parsing
			}
			else if (apos) { // No '*' without 'N'
				gcode_line_error(PSTR(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM), false);
				return;
			}

			// Movement commands alert when stopped
			if (IsStopped()) {
				char* gpos = strchr(command, 'G');
				if (gpos) {
					const int codenum = strtol(gpos + 1, nullptr, 10);
					switch (codenum) {
					case 0:
					case 1:
					case 2:
					case 3:
						SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
						LCD_MESSAGEPGM(MSG_STOPPED);
						break;
					}
				}
			}

#if DISABLED(EMERGENCY_PARSER)
			// If command was e-stop process now
			if (strcmp(command, "M108") == 0) {
				wait_for_heatup = false;
#if ENABLED(ULTIPANEL)
				wait_for_user = false;
#endif
			}
			if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));
			if (strcmp(command, "M410") == 0) { quickstop_stepper(); }
#endif

#if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
			last_command_time = ms;
#endif

			// Add the command to the queue
			_enqueuecommand(serial_line_buffer, true);
		}
		else if (serial_count >= MAX_CMD_SIZE - 1) {
			// Keep fetching, but ignore normal characters beyond the max length
			// The command will be injected when EOL is reached
		}
		else if (serial_char == '\\') {  // Handle escapes
			if (MYSERIAL.available() > 0) {
				// if we have one more character, copy it over
				serial_char = MYSERIAL.read();
				if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
			}
			// otherwise do nothing
		}
		else { // it's not a newline, carriage return or escape char
			if (serial_char == ';') serial_comment_mode = true;
			if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
		}

	} // queue has space, serial has data
}

/**
 * Get commands from the SD Card until the command buffer is full
 * or until the end of the file is reached. The special character '#'
 * can also interrupt buffering.
 */
inline void get_sdcard_commands() {
	static bool stop_buffering = false,
		sd_comment_mode = false;

	if (!card.sdprinting) return;

	/**
	 * '#' stops reading from SD to the buffer prematurely, so procedural
	 * macro calls are possible. If it occurs, stop_buffering is triggered
	 * and the buffer is run dry; this character _can_ occur in serial com
	 * due to checksums, however, no checksums are used in SD printing.
	 */

	if (commands_in_queue == 0) stop_buffering = false;

	uint16_t sd_count = 0;
	bool card_eof = card.eof();
	while (commands_in_queue < BUFSIZE && !card_eof && !stop_buffering) {
		const int16_t n = card.get();
		char sd_char = (char)n;
		card_eof = card.eof();
		if (card_eof || n == -1
			|| sd_char == '\n' || sd_char == '\r'
			|| ((sd_char == '#' || sd_char == ':') && !sd_comment_mode)
			) {
			if (card_eof) {
				SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
				card.printingHasFinished();
#if ENABLED(PRINTER_EVENT_LEDS)
				LCD_MESSAGEPGM(MSG_INFO_COMPLETED_PRINTS);
				set_led_color(0, 255, 0); // Green
#if HAS_RESUME_CONTINUE
				enqueue_and_echo_commands_P(PSTR("M0")); // end of the queue!
#else
				safe_delay(1000);
#endif
				set_led_color(0, 0, 0);   // OFF
#endif
				card.checkautostart(true);
			}
			else if (n == -1) {
				SERIAL_ERROR_START();
				SERIAL_ECHOLNPGM(MSG_SD_ERR_READ);
			}
			if (sd_char == '#') stop_buffering = true;

			sd_comment_mode = false; // for new command

			if (!sd_count) continue; // skip empty lines (and comment lines)

			command_queue[cmd_queue_index_w][sd_count] = '\0'; // terminate string
			sd_count = 0; // clear sd line buffer

			_commit_command(false);
		}
		else if (sd_count >= MAX_CMD_SIZE - 1) {
			/**
			 * Keep fetching, but ignore normal characters beyond the max length
			 * The command will be injected when EOL is reached
			 */
		}
		else {
			if (sd_char == ';') sd_comment_mode = true;
			if (!sd_comment_mode) command_queue[cmd_queue_index_w][sd_count++] = sd_char;
		}
	}
}

/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (injected_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void get_available_commands() {

	// if any immediate commands remain, don't get other commands yet
	if (drain_injected_commands_P()) return;

	get_serial_commands();

	get_sdcard_commands();
}

/**
 * Set target_extruder from the T parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
bool get_target_extruder_from_command(const uint16_t code) {
	if (parser.seenval('T')) {
		const int8_t e = parser.value_byte();
		if (e >= EXTRUDERS) {
			SERIAL_ECHO_START();
			SERIAL_CHAR('M');
			SERIAL_ECHO(code);
			SERIAL_ECHOLNPAIR(" " MSG_INVALID_EXTRUDER " ", e);
			return true;
		}
		target_extruder = e;
	}
	else
		target_extruder = active_extruder;

	return false;
}

/**
 * Software endstops can be used to monitor the open end of
 * an axis that has a hardware endstop on the other end. Or
 * they can prevent axes from moving past endstops and grinding.
 *
 * To keep doing their job as the coordinate system changes,
 * the software endstop positions must be refreshed to remain
 * at the same positions relative to the machine.
 */
void update_software_endstops(const AxisEnum axis) {
	const float offs = 0.0
		+ home_offset[axis]
		+ position_shift[axis]
		;

	workspace_offset[axis] = offs;

	soft_endstop_min[axis] = base_min_pos(axis) + offs;
	soft_endstop_max[axis] = base_max_pos(axis) + offs;
}


/**
 * Change the home offset for an axis, update the current
 * position and the software endstops to retain the same
 * relative distance to the new home.
 *
 * Since this changes the current_position, code should
 * call sync_plan_position soon after this.
 */
static void set_home_offset(const AxisEnum axis, const float v) {
	current_position[axis] += v - home_offset[axis];
	home_offset[axis] = v;
	update_software_endstops(axis);
}

/**
 * Set an axis' current position to its home position (after homing).
 *
 * For Core and Cartesian robots this applies one-to-one when an
 * individual axis has been homed.
 *
 * DELTA should wait until all homing is done before setting the XYZ
 * current_position to home, because homing is a single operation.
 * In the case where the axis positions are already known and previously
 * homed, DELTA could home to X or Y individually by moving either one
 * to the center. However, homing Z always homes XY and Z.
 *
 * SCARA should wait until all XY homing is done before setting the XY
 * current_position to home, because neither X nor Y is at home until
 * both are at home. Z can however be homed individually.
 *
 * Callers must sync the planner position after calling this!
 */
static void set_axis_is_at_home(const AxisEnum axis) {
	axis_known_position[axis] = axis_homed[axis] = true;

	position_shift[axis] = 0;
	update_software_endstops(axis);

	current_position[axis] = LOGICAL_POSITION(base_home_pos(axis), axis);
}

/**
 * Some planner shorthand inline functions
 */
inline float get_homing_bump_feedrate(const AxisEnum axis) {
	static const uint8_t homing_bump_divisor[] PROGMEM = HOMING_BUMP_DIVISOR;
	uint8_t hbd = pgm_read_byte(&homing_bump_divisor[axis]);
	if (hbd < 1) {
		hbd = 10;
		SERIAL_ECHO_START();
		SERIAL_ECHOLNPGM("Warning: Homing Bump Divisor < 1");
	}
	return homing_feedrate(axis) / hbd;
}

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
inline void line_to_current_position() {
	planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate_mm_s, active_extruder);
}

/**
 * Move the planner to the position stored in the destination array, which is
 * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
 */
inline void line_to_destination(const float fr_mm_s) {
	planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, active_extruder);
}
inline void line_to_destination() { line_to_destination(feedrate_mm_s); }

inline void set_current_to_destination() { COPY(current_position, destination); }
inline void set_destination_to_current() { COPY(destination, current_position); }

/**
 *  Plan a move to (X, Y, Z) and set the current_position
 *  The final current_position may not be the one that was requested
 */
void do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s/*=0.0*/) {
	const float old_feedrate_mm_s = feedrate_mm_s;

	// If Z needs to raise, do it before moving XY
	if (current_position[Z_AXIS] < lz) {
		feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate(Z_AXIS);
		current_position[Z_AXIS] = lz;
		line_to_current_position();
	}

	feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
	current_position[X_AXIS] = lx;
	current_position[Y_AXIS] = ly;
	line_to_current_position();

	// If Z needs to lower, do it after moving XY
	if (current_position[Z_AXIS] > lz) {
		feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate(Z_AXIS);
		current_position[Z_AXIS] = lz;
		line_to_current_position();
	}

	stepper.synchronize();

	feedrate_mm_s = old_feedrate_mm_s;
}
void do_blocking_move_to_x(const float &lx, const float &fr_mm_s/*=0.0*/) {
	do_blocking_move_to(lx, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
}
void do_blocking_move_to_z(const float &lz, const float &fr_mm_s/*=0.0*/) {
	do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], lz, fr_mm_s);
}
void do_blocking_move_to_xy(const float &lx, const float &ly, const float &fr_mm_s/*=0.0*/) {
	do_blocking_move_to(lx, ly, current_position[Z_AXIS], fr_mm_s);
}

//
// Prepare to do endstop or probe moves
// with custom feedrates.
//
//  - Save current feedrates
//  - Reset the rate multiplier
//  - Reset the command timeout
//  - Enable the endstops (for endstop moves)
//
static void setup_for_endstop_or_probe_move() {
	saved_feedrate_mm_s = feedrate_mm_s;
	saved_feedrate_percentage = feedrate_percentage;
	feedrate_percentage = 100;
	refresh_cmd_timeout();
}

static void clean_up_after_endstop_or_probe_move() {
	feedrate_mm_s = saved_feedrate_mm_s;
	feedrate_percentage = saved_feedrate_percentage;
	refresh_cmd_timeout();
}

/**
 * Home an individual linear axis
 */
static void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s = 0.0) {

	// Tell the planner we're at Z=0
	current_position[axis] = 0;

	sync_plan_position();
	current_position[axis] = distance;
	planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate(axis), active_extruder);

	stepper.synchronize();

	endstops.hit_on_purpose();
}

/**
 * Home an individual "raw axis" to its endstop.
 * This applies to XYZ on Cartesian and Core robots, and
 * to the individual ABC steppers on DELTA and SCARA.
 *
 * At the end of the procedure the axis is marked as
 * homed and the current position of that axis is updated.
 * Kinematic robots should wait till all axes are homed
 * before updating the current position.
 */

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

static void homeaxis(const AxisEnum axis) {

#define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
	if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;

	const int axis_home_dir =
		home_dir(axis);

	// Fast move towards endstop until triggered
	do_homing_move(axis, 1.5 * max_length(axis) * axis_home_dir);

	// When homing Z with probe respect probe clearance
	const float bump = axis_home_dir * (
		home_bump_mm(axis)
		);

	// If a second homing move is configured...
	if (bump) {
		// Move away from the endstop by the axis HOME_BUMP_MM
		do_homing_move(axis, -bump);

		// Slow move towards endstop until triggered
		do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
	}

	// For cartesian/core machines,
	// set the axis to its home position
	set_axis_is_at_home(axis);
	sync_plan_position();

	destination[axis] = current_position[axis];
} // homeaxis()

/**
 * ***************************************************************************
 * ***************************** G-CODE HANDLING *****************************
 * ***************************************************************************
 */

 /**
  * Set XYZE destination and feedrate from the current GCode command
  *
  *  - Set destination from included axis codes
  *  - Set to current for missing axis codes
  *  - Set the feedrate, if included
  */
void gcode_get_destination() {
	LOOP_XYZE(i) {
		if (parser.seen(axis_codes[i]))
			destination[i] = parser.value_axis_units((AxisEnum)i) + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
		else
			destination[i] = current_position[i];
	}

	if (parser.linearval('F') > 0.0)
		feedrate_mm_s = MMM_TO_MMS(parser.value_feedrate());

	if (!DEBUGGING(DRYRUN))
		print_job_timer.incFilamentUsed(destination[E_AXIS] - current_position[E_AXIS]);
}

/**
 * Output a "busy" message at regular intervals
 * while the machine is not accepting commands.
 */
void host_keepalive() {
	const millis_t ms = millis();
	if (host_keepalive_interval && busy_state != NOT_BUSY) {
		if (PENDING(ms, next_busy_signal_ms)) return;
		switch (busy_state) {
		case IN_HANDLER:
		case IN_PROCESS:
			SERIAL_ECHO_START();
			SERIAL_ECHOLNPGM(MSG_BUSY_PROCESSING);
			break;
		case PAUSED_FOR_USER:
			SERIAL_ECHO_START();
			SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_USER);
			break;
		case PAUSED_FOR_INPUT:
			SERIAL_ECHO_START();
			SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_INPUT);
			break;
		default:
			break;
		}
	}
	next_busy_signal_ms = ms + host_keepalive_interval * 1000UL;
}


/**************************************************
 ***************** GCode Handlers *****************
 **************************************************/

 /**
  * G0, G1: Coordinated movement of X Y Z E axes
  */
inline void gcode_G0_G1(
) {
	if (IsRunning()) {
		gcode_get_destination(); // For X Y Z E F

		prepare_move_to_destination();
	}
}

/**
 * G2: Clockwise Arc
 * G3: Counterclockwise Arc
 *
 * This command has two forms: IJ-form and R-form.
 *
 *  - I specifies an X offset. J specifies a Y offset.
 *    At least one of the IJ parameters is required.
 *    X and Y can be omitted to do a complete circle.
 *    The given XY is not error-checked. The arc ends
 *     based on the angle of the destination.
 *    Mixing I or J with R will throw an error.
 *
 *  - R specifies the radius. X or Y is required.
 *    Omitting both X and Y will throw an error.
 *    X or Y must differ from the current XY.
 *    Mixing R with I or J will throw an error.
 *
 *  - P specifies the number of full circles to do
 *    before the specified arc move.
 *
 *  Examples:
 *
 *    G2 I10           ; CW circle centered at X+10
 *    G3 X20 Y12 R14   ; CCW circle with r=14 ending at X20 Y12
 */

inline void gcode_G2_G3(bool clockwise) {
	if (IsRunning()) {
		gcode_get_destination();

		float arc_offset[2] = { 0.0, 0.0 };
		if (parser.seenval('R')) {
			const float r = parser.value_linear_units(),
				p1 = current_position[X_AXIS], q1 = current_position[Y_AXIS],
				p2 = destination[X_AXIS], q2 = destination[Y_AXIS];
			if (r && (p2 != p1 || q2 != q1)) {
				const float e = clockwise ^ (r < 0) ? -1 : 1,           // clockwise -1/1, counterclockwise 1/-1
					dx = p2 - p1, dy = q2 - q1,                 // X and Y differences
					d = HYPOT(dx, dy),                          // Linear distance between the points
					h = SQRT(sq(r) - sq(d * 0.5)),              // Distance to the arc pivot-point
					mx = (p1 + p2) * 0.5, my = (q1 + q2) * 0.5, // Point between the two points
					sx = -dy / d, sy = dx / d,                  // Slope of the perpendicular bisector
					cx = mx + e * h * sx, cy = my + e * h * sy; // Pivot-point of the arc
				arc_offset[0] = cx - p1;
				arc_offset[1] = cy - q1;
			}
		}
		else {
			if (parser.seenval('I')) arc_offset[0] = parser.value_linear_units();
			if (parser.seenval('J')) arc_offset[1] = parser.value_linear_units();
		}

		if (arc_offset[0] || arc_offset[1]) {

			// Send the arc to the planner
			plan_arc(destination, arc_offset, clockwise);
			refresh_cmd_timeout();
		}
		else {
			// Bad arguments
			SERIAL_ERROR_START();
			SERIAL_ERRORLNPGM(MSG_ERR_ARC_ARGS);
		}
	}
}

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
	millis_t dwell_ms = 0;

	if (parser.seenval('P')) dwell_ms = parser.value_millis(); // milliseconds to wait
	if (parser.seenval('S')) dwell_ms = parser.value_millis_from_seconds(); // seconds to wait

	stepper.synchronize();
	refresh_cmd_timeout();
	dwell_ms += previous_cmd_ms;  // keep track of when we started waiting

	if (!lcd::has_status()) LCD_MESSAGEPGM(MSG_DWELL);

	while (PENDING(millis(), dwell_ms)) idle();
}

/**
 * Parameters interpreted according to:
 * http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G5-Cubic-Spline
 * However I, J omission is not supported at this point; all
 * parameters can be omitted and default to zero.
 */

 /**
  * G5: Cubic B-spline
  */
inline void gcode_G5() {
	if (IsRunning()) {

		gcode_get_destination();

		const float offset[] = {
		  parser.linearval('I'),
		  parser.linearval('J'),
		  parser.linearval('P'),
		  parser.linearval('Q')
		};

		plan_cubic_move(offset);
	}
}

constexpr bool g29_in_progress = false;

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 */
inline void gcode_G28(const bool always_home_all) {
	// Wait for planner moves to finish!
	stepper.synchronize();

	setup_for_endstop_or_probe_move();
	endstops.enable(true); // Enable endstops for next homing move

	const bool homeX = always_home_all || parser.seen('X'),
		homeY = always_home_all || parser.seen('Y'),
		homeZ = always_home_all || parser.seen('Z'),
		home_all = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

	set_destination_to_current();

	if (home_all || homeX || homeY) {
		// Raise Z before homing any other axes and z is not already high enough (never lower z)
		destination[Z_AXIS] = LOGICAL_Z_POSITION(Z_HOMING_HEIGHT);
		if (destination[Z_AXIS] > current_position[Z_AXIS]) {

#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING))
				SERIAL_ECHOLNPAIR("Raise Z (before homing) to ", destination[Z_AXIS]);
#endif

			do_blocking_move_to_z(destination[Z_AXIS]);
		}
	}

	// Home X
	if (home_all || homeX) {
		HOMEAXIS(X);
	}

	// Home Y
	if (home_all || homeY) {
		HOMEAXIS(Y);
	}

	// Home Z last if homing towards the bed
	if (home_all || homeZ) {
		HOMEAXIS(Z);
	} // home_all || homeZ

	SYNC_PLAN_POSITION_KINEMATIC();

	endstops.not_homing();

	clean_up_after_endstop_or_probe_move();

	lcd::refresh();

	report_current_position();

} // G28

void home_all_axes() { gcode_G28(true); }

/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {
	bool didXYZ = false,
		didE = parser.seenval('E');

	if (!didE) stepper.synchronize();

	LOOP_XYZE(i) {
		if (parser.seenval(axis_codes[i])) {
			const float p = current_position[i];
			const float v = parser.value_axis_units((AxisEnum)i);

			current_position[i] = v;

			if (i != E_AXIS) {
				didXYZ = true;

				position_shift[i] += v - p; // Offset the coordinate space
				update_software_endstops((AxisEnum)i);
			}
		}
	}
	if (didXYZ)
		SYNC_PLAN_POSITION_KINEMATIC();
	else if (didE)
		sync_plan_position_e();

	report_current_position();
}

/**
 * M17: Enable power on all stepper motors
 */
inline void gcode_M17() {
	LCD_MESSAGEPGM(MSG_NO_MOVE);
	enable_all_steppers();
}

#define RUNPLAN(RATE_MM_S) line_to_destination(RATE_MM_S)

/**
 * M20: List SD card to serial output
 */
inline void gcode_M20() {
	SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
	card.ls();
	SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
}

/**
 * M21: Init SD Card
 */
inline void gcode_M21() { card.initsd(); }

/**
 * M22: Release SD Card
 */
inline void gcode_M22() { card.release(); }

/**
 * M23: Open a file
 */
inline void gcode_M23() { card.openFile(parser.string_arg, true); }

/**
 * M24: Start or Resume SD Print
 */
inline void gcode_M24() {
	card.startFileprint();
	print_job_timer.start();
}

/**
 * M25: Pause SD Print
 */
inline void gcode_M25() {
	card.pauseSDPrint();
	print_job_timer.pause();
}

/**
 * M26: Set SD Card file index
 */
inline void gcode_M26() {
	if (card.cardOK && parser.seenval('S'))
		card.setIndex(parser.value_long());
}

/**
 * M27: Get SD Card status
 */
inline void gcode_M27() { card.getStatus(); }

/**
 * M28: Start SD Write
 */
inline void gcode_M28() { card.openFile(parser.string_arg, false); }

/**
 * M29: Stop SD Write
 * Processed in write to file routine above
 */
inline void gcode_M29() {
	// card.saving = false;
}

/**
 * M30 <filename>: Delete SD Card file
 */
inline void gcode_M30() {
	if (card.cardOK) {
		card.closefile();
		card.removeFile(parser.string_arg);
	}
}

/**
 * M31: Get the time since the start of SD Print (or last M109)
 */
inline void gcode_M31() {
	char buffer[21];
	duration_t elapsed = print_job_timer.duration();
	elapsed.toString(buffer);
	lcd::set_status(buffer);

	SERIAL_ECHO_START();
	SERIAL_ECHOLNPAIR("Print time: ", buffer);
}

/**
 * M32: Select file and start SD Print
 */
inline void gcode_M32() {
	if (card.sdprinting)
		stepper.synchronize();

	char* namestartpos = parser.string_arg;
	const bool call_procedure = parser.boolval('P');

	if (card.cardOK) {
		card.openFile(namestartpos, true, call_procedure);

		if (parser.seenval('S'))
			card.setIndex(parser.value_long());

		card.startFileprint();

		// Procedure calls count as normal print time.
		if (!call_procedure) print_job_timer.start();
	}
}

/**
 * M33: Get the long full path of a file or folder
 *
 * Parameters:
 *   <dospath> Case-insensitive DOS-style path to a file or folder
 *
 * Example:
 *   M33 miscel~1/armchair/armcha~1.gco
 *
 * Output:
 *   /Miscellaneous/Armchair/Armchair.gcode
 */
inline void gcode_M33() {
	card.printLongPath(parser.string_arg);
}

/**
 * M928: Start SD Write
 */
inline void gcode_M928() {
	card.openLogFile(parser.string_arg);
}

/**
 * Sensitive pin test for M42, M226
 */
static bool pin_is_protected(const int8_t pin) {
	static const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS;
	for (uint8_t i = 0; i < COUNT(sensitive_pins); i++)
		if (pin == (int8_t)pgm_read_byte(&sensitive_pins[i])) return true;
	return false;
}

/**
 * M42: Change pin status via GCode
 *
 *  P<pin>  Pin number (LED if omitted)
 *  S<byte> Pin status from 0 - 255
 */
inline void gcode_M42() {
	if (!parser.seenval('S')) return;
	const byte pin_status = parser.value_byte();

	const int pin_number = parser.intval('P', LED_PIN);
	if (pin_number < 0) return;

	if (pin_is_protected(pin_number)) {
		SERIAL_ERROR_START();
		SERIAL_ERRORLNPGM(MSG_ERR_PROTECTED_PIN);
		return;
	}

	pinMode(pin_number, OUTPUT);
	digitalWrite(pin_number, pin_status);
	analogWrite(pin_number, pin_status);

	switch (pin_number) {
	case FAN_PIN: fanSpeeds[0] = pin_status; break;
	}
}

/**
 * M75: Start print timer
 */
inline void gcode_M75() { print_job_timer.start(); }

/**
 * M76: Pause print timer
 */
inline void gcode_M76() { print_job_timer.pause(); }

/**
 * M77: Stop print timer
 */
inline void gcode_M77() { print_job_timer.stop(); }

/**
 * M78: Show print statistics
 */
inline void gcode_M78() {
	// "M78 S78" will reset the statistics
	if (parser.intval('S') == 78)
		print_job_timer.initStats();
	else
		print_job_timer.showStats();
}

/**
 * M104: Set hot end temperature
 */
inline void gcode_M104() {
	if (get_target_extruder_from_command(104)) return;
	if (DEBUGGING(DRYRUN)) return;

	if (parser.seenval('S')) {
		const int16_t temp = parser.value_celsius();
		thermalManager.setTargetHotend(temp);

		/**
		 * Stop the timer at the end of print. Start is managed by 'heat and wait' M109.
		 * We use half EXTRUDE_MINTEMP here to allow nozzles to be put into hot
		 * standby mode, for instance in a dual extruder setup, without affecting
		 * the running print timer.
		 */
		if (parser.value_celsius() <= (EXTRUDE_MINTEMP) / 2) {
			print_job_timer.stop();
			LCD_MESSAGEPGM(WELCOME_MSG);
		}

		if (parser.value_celsius() > thermalManager.degHotend())
			lcd::statusf(0, PSTR("E%i %s"), target_extruder + 1, MSG_HEATING);
	}

	planner.autotemp_M104_M109();
}

void print_heater_state(const float &c, const float &t,
#if ENABLED(SHOW_TEMP_ADC_VALUES)
	const float r,
#endif
	const int8_t e = -2
) {
	SERIAL_PROTOCOLCHAR(' ');
	SERIAL_PROTOCOLCHAR(
		e == -1 ? 'B' : 'T'
	);
	SERIAL_PROTOCOLCHAR(':');
	SERIAL_PROTOCOL(c);
	SERIAL_PROTOCOLPAIR(" /", t);
}

void print_heaterstates() {
	print_heater_state(thermalManager.degHotend(), thermalManager.degTargetHotend()
	);
	print_heater_state(thermalManager.degBed(), thermalManager.degTargetBed(),
		-1 // BED
	);
	SERIAL_PROTOCOLPGM(" @:");
	SERIAL_PROTOCOL(thermalManager.getHeaterPower<Temperature::Manager::Hotend>());
	SERIAL_PROTOCOLPGM(" B@:");
	SERIAL_PROTOCOL(thermalManager.getHeaterPower<Temperature::Manager::Bed>());
}

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105() {
	if (get_target_extruder_from_command(105)) return;

	SERIAL_PROTOCOLPGM(MSG_OK);
	print_heaterstates();

	SERIAL_EOL();
}

static uint8_t auto_report_temp_interval;
static millis_t next_temp_report_ms;

/**
 * M155: Set temperature auto-report interval. M155 S<seconds>
 */
inline void gcode_M155() {
	if (parser.seenval('S')) {
		auto_report_temp_interval = parser.value_byte();
		NOMORE(auto_report_temp_interval, 60);
		next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
	}
}

inline void auto_report_temperatures() {
	if (auto_report_temp_interval && ELAPSED(millis(), next_temp_report_ms)) {
		next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
		print_heaterstates();
		SERIAL_EOL();
	}
}

/**
 * M106: Set Fan Speed
 *
 *  S<int>   Speed between 0-255
 *  P<index> Fan index, if more than one fan
 */
inline void gcode_M106() {
	uint16_t s = parser.ushortval('S', 255);
	NOMORE(s, 255);
	const uint8_t p = parser.byteval('P', 0);
	if (p < FAN_COUNT) fanSpeeds[p] = s;
}

/**
 * M107: Fan Off
 */
inline void gcode_M107() {
	const uint16_t p = parser.ushortval('P');
	if (p < FAN_COUNT) fanSpeeds[p] = 0;
}

/**
 * M108: Stop the waiting for heaters in M109, M190, M303. Does not affect the target temperature.
 */
inline void gcode_M108() { wait_for_heatup = false; }


/**
 * M112: Emergency Stop
 */
inline void gcode_M112() { kill(PSTR(MSG_KILLED)); }


/**
 * M410: Quickstop - Abort all planned moves
 *
 * This will stop the carriages mid-move, so most likely they
 * will be out of sync with the stepper position after this.
 */
inline void gcode_M410() { quickstop_stepper(); }

/**
 * M109: Sxxx Wait for extruder(s) to reach temperature. Waits only when heating.
 *       Rxxx Wait for extruder(s) to reach temperature. Waits when heating and cooling.
 */

#define MIN_COOLING_SLOPE_DEG 1.50
#define MIN_COOLING_SLOPE_TIME 60

inline void gcode_M109() {

	if (get_target_extruder_from_command(109)) return;
	if (DEBUGGING(DRYRUN)) return;

	const bool no_wait_for_cooling = parser.seenval('S');
	if (no_wait_for_cooling || parser.seenval('R')) {
		const int16_t temp = parser.value_celsius();
		thermalManager.setTargetHotend(temp);

		/**
		 * Use half EXTRUDE_MINTEMP to allow nozzles to be put into hot
		 * standby mode, (e.g., in a dual extruder setup) without affecting
		 * the running print timer.
		 */
		if (parser.value_celsius() <= (EXTRUDE_MINTEMP) / 2) {
			print_job_timer.stop();
			LCD_MESSAGEPGM(WELCOME_MSG);
		}
		else
			print_job_timer.start();

		if (thermalManager.isHeatingHotend()) lcd::statusf(0, PSTR("E%i %s"), target_extruder + 1, MSG_HEATING);
	}
	else return;

	planner.autotemp_M104_M109();

	millis_t residency_start_ms = 0;
	// Loop until the temperature has stabilized
#define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))

	float target_temp = -1.0, old_temp = 9999.0;
	bool wants_to_cool = false;
	wait_for_heatup = true;
	millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

	KEEPALIVE_STATE(NOT_BUSY);

	do {
		// Target temperature might be changed during the loop
		if (target_temp != thermalManager.degTargetHotend()) {
			wants_to_cool = thermalManager.isCoolingHotend();
			target_temp = thermalManager.degTargetHotend();

			// Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
			if (no_wait_for_cooling && wants_to_cool) break;
		}

		now = millis();
		if (ELAPSED(now, next_temp_ms)) { //Print temp & remaining time every 1s while waiting
			next_temp_ms = now + 1000UL;
			print_heaterstates();
			SERIAL_PROTOCOLPGM(" W:");
			if (residency_start_ms)
				SERIAL_PROTOCOL(long((((TEMP_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL));
			else
				SERIAL_PROTOCOLCHAR('?');
			SERIAL_EOL();
		}

		idle();
		refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

		const float temp = thermalManager.degHotend();

		const float temp_diff = FABS(target_temp - temp);

		if (!residency_start_ms) {
			// Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
			if (temp_diff < TEMP_WINDOW) residency_start_ms = now;
		}
		else if (temp_diff > TEMP_HYSTERESIS) {
			// Restart the timer whenever the temperature falls outside the hysteresis.
			residency_start_ms = now;
		}

		// Prevent a wait-forever situation if R is misused i.e. M109 R0
		if (wants_to_cool) {
			// break after MIN_COOLING_SLOPE_TIME seconds
			// if the temperature did not drop at least MIN_COOLING_SLOPE_DEG
			if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
				if (old_temp - temp < MIN_COOLING_SLOPE_DEG) break;
				next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME;
				old_temp = temp;
			}
		}

	} while (wait_for_heatup && TEMP_CONDITIONS);

	if (wait_for_heatup) {
		LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
	}

	KEEPALIVE_STATE(IN_HANDLER);
}

#define MIN_COOLING_SLOPE_DEG_BED 1.50
#define MIN_COOLING_SLOPE_TIME_BED 60

/**
 * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating
 *       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
 */
inline void gcode_M190() {
	if (DEBUGGING(DRYRUN)) return;

	LCD_MESSAGEPGM(MSG_BED_HEATING);
	const bool no_wait_for_cooling = parser.seenval('S');
	if (no_wait_for_cooling || parser.seenval('R')) {
		thermalManager.setTargetBed(parser.value_celsius());
		if (parser.value_celsius() > BED_MINTEMP)
			print_job_timer.start();
	}
	else return;

	millis_t residency_start_ms = 0;
	// Loop until the temperature has stabilized
#define TEMP_BED_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_BED_RESIDENCY_TIME) * 1000UL))

	float target_temp = -1.0, old_temp = 9999.0;
	bool wants_to_cool = false;
	wait_for_heatup = true;
	millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

	KEEPALIVE_STATE(NOT_BUSY);

	target_extruder = active_extruder; // for print_heaterstates

	do {
		// Target temperature might be changed during the loop
		if (target_temp != thermalManager.degTargetBed()) {
			wants_to_cool = thermalManager.isCoolingBed();
			target_temp = thermalManager.degTargetBed();

			// Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
			if (no_wait_for_cooling && wants_to_cool) break;
		}

		now = millis();
		if (ELAPSED(now, next_temp_ms)) { //Print Temp Reading every 1 second while heating up.
			next_temp_ms = now + 1000UL;
			print_heaterstates();
			SERIAL_PROTOCOLPGM(" W:");
			if (residency_start_ms)
				SERIAL_PROTOCOL(long((((TEMP_BED_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL));
			else
				SERIAL_PROTOCOLCHAR('?');
			SERIAL_EOL();
		}

		idle();
		refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

		const float temp = thermalManager.degBed();

		const float temp_diff = FABS(target_temp - temp);

		if (!residency_start_ms) {
			// Start the TEMP_BED_RESIDENCY_TIME timer when we reach target temp for the first time.
			if (temp_diff < TEMP_BED_WINDOW) residency_start_ms = now;
		}
		else if (temp_diff > TEMP_BED_HYSTERESIS) {
			// Restart the timer whenever the temperature falls outside the hysteresis.
			residency_start_ms = now;
		}

		// Prevent a wait-forever situation if R is misused i.e. M190 R0
		if (wants_to_cool) {
			// Break after MIN_COOLING_SLOPE_TIME_BED seconds
			// if the temperature did not drop at least MIN_COOLING_SLOPE_DEG_BED
			if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
				if (old_temp - temp < MIN_COOLING_SLOPE_DEG_BED) break;
				next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME_BED;
				old_temp = temp;
			}
		}

	} while (wait_for_heatup && TEMP_BED_CONDITIONS);

	if (wait_for_heatup) LCD_MESSAGEPGM(MSG_BED_DONE);
	KEEPALIVE_STATE(IN_HANDLER);
}

/**
 * M110: Set Current Line Number
 */
inline void gcode_M110() {
	if (parser.seenval('N')) gcode_LastN = parser.value_long();
}

/**
 * M111: Set the debug level
 */
inline void gcode_M111() {
	marlin_debug_flags = parser.byteval('S', (uint8_t)DEBUG_NONE);

	const static char str_debug_1[] PROGMEM = MSG_DEBUG_ECHO;
	const static char str_debug_2[] PROGMEM = MSG_DEBUG_INFO;
	const static char str_debug_4[] PROGMEM = MSG_DEBUG_ERRORS;
	const static char str_debug_8[] PROGMEM = MSG_DEBUG_DRYRUN;
	const static char str_debug_16[] PROGMEM = MSG_DEBUG_COMMUNICATION;

	const static char* const debug_strings[] PROGMEM = {
	  str_debug_1, str_debug_2, str_debug_4, str_debug_8, str_debug_16
	};

	SERIAL_ECHO_START();
	SERIAL_ECHOPGM(MSG_DEBUG_PREFIX);
	if (marlin_debug_flags) {
		uint8_t comma = 0;
		for (uint8_t i = 0; i < COUNT(debug_strings); i++) {
			if (TEST(marlin_debug_flags, i)) {
				if (comma++) SERIAL_CHAR(',');
				serialprintPGM((char*)pgm_read_word(&debug_strings[i]));
			}
		}
	}
	else {
		SERIAL_ECHOPGM(MSG_DEBUG_OFF);
	}
	SERIAL_EOL();
}

/**
 * M113: Get or set Host Keepalive interval (0 to disable)
 *
 *   S<seconds> Optional. Set the keepalive interval.
 */
inline void gcode_M113() {
	if (parser.seenval('S')) {
		host_keepalive_interval = parser.value_byte();
		NOMORE(host_keepalive_interval, 60);
	}
	else {
		SERIAL_ECHO_START();
		SERIAL_ECHOLNPAIR("M113 S", (unsigned long)host_keepalive_interval);
	}
}

/**
 * M140: Set bed temperature
 */
inline void gcode_M140() {
	if (DEBUGGING(DRYRUN)) return;
	if (parser.seenval('S')) thermalManager.setTargetBed(parser.value_celsius());
}

/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *      This code should ALWAYS be available for EMERGENCY SHUTDOWN!
 */
inline void gcode_M81() {
	thermalManager.disable_all_heaters();
	stepper.finish_and_disable();

	for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;

	safe_delay(1000); // Wait 1 second before switching off
}

/**
 * M82: Set E codes absolute (default)
 */
inline void gcode_M82() { axis_relative_modes[E_AXIS] = false; }

/**
 * M83: Set E codes relative while in Absolute Coordinates (G90) mode
 */
inline void gcode_M83() { axis_relative_modes[E_AXIS] = true; }

/**
 * M18, M84: Disable stepper motors
 */
inline void gcode_M18_M84() {
	if (parser.seenval('S')) {
		stepper_inactive_time = parser.value_millis_from_seconds();
	}
	else {
		bool all_axis = !((parser.seen('X')) || (parser.seen('Y')) || (parser.seen('Z')) || (parser.seen('E')));
		if (all_axis) {
			stepper.finish_and_disable();
		}
		else {
			stepper.synchronize();
			if (parser.seen('X')) disable_X();
			if (parser.seen('Y')) disable_Y();
			if (parser.seen('Z')) disable_Z();
			if (parser.seen('E')) disable_e_steppers();
		}
	}
}

/**
 * M85: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M85() {
	if (parser.seen('S')) max_inactive_time = parser.value_millis_from_seconds();
}

/**
 * Multi-stepper support for M92, M201, M203
 */
#define GET_TARGET_EXTRUDER(CMD) NOOP
#define TARGET_EXTRUDER 0

 /**
  * M92: Set axis steps-per-unit for one or more axes, X, Y, Z, and E.
  *      (Follows the same syntax as G92)
  *
  *      With multiple extruders use T to specify which one.
  */
inline void gcode_M92() {

	GET_TARGET_EXTRUDER(92);

	LOOP_XYZE(i) {
		if (parser.seen(axis_codes[i])) {
			if (i == E_AXIS) {
				const float value = parser.value_per_axis_unit((AxisEnum)(E_AXIS + TARGET_EXTRUDER));
				if (value < 20.0) {
					float factor = planner.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] / value; // increase e constants if M92 E14 is given for netfab.
					planner.max_jerk[E_AXIS] *= factor;
					planner.max_feedrate_mm_s[E_AXIS + TARGET_EXTRUDER] *= factor;
					planner.max_acceleration_steps_per_s2[E_AXIS + TARGET_EXTRUDER] *= factor;
				}
				planner.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] = value;
			}
			else {
				planner.axis_steps_per_mm[i] = parser.value_per_axis_unit((AxisEnum)i);
			}
		}
	}
	planner.refresh_positioning();
}

/**
 * Output the current position to serial
 */
void report_current_position() {
	SERIAL_PROTOCOLPGM("X:");
	SERIAL_PROTOCOL(current_position[X_AXIS]);
	SERIAL_PROTOCOLPGM(" Y:");
	SERIAL_PROTOCOL(current_position[Y_AXIS]);
	SERIAL_PROTOCOLPGM(" Z:");
	SERIAL_PROTOCOL(current_position[Z_AXIS]);
	SERIAL_PROTOCOLPGM(" E:");
	SERIAL_PROTOCOL(current_position[E_AXIS]);

	stepper.report_positions();
}

/**
 * M114: Report current position to host
 */
inline void gcode_M114() {
	stepper.synchronize();
	report_current_position();
}

/**
 * M115: Capabilities string
 */
inline void gcode_M115() {
	SERIAL_PROTOCOLLNPGM(MSG_M115_REPORT);

	// EEPROM (M500, M501)
	SERIAL_PROTOCOLLNPGM("Cap:EEPROM:1");

	// AUTOREPORT_TEMP (M155)
	SERIAL_PROTOCOLLNPGM("Cap:AUTOREPORT_TEMP:1");

	// PROGRESS (M530 S L, M531 <file>, M532 X L)
	SERIAL_PROTOCOLLNPGM("Cap:PROGRESS:0");

	// Print Job timer M75, M76, M77
	SERIAL_PROTOCOLLNPGM("Cap:PRINT_JOB:1");

	// AUTOLEVEL (G29)
	SERIAL_PROTOCOLLNPGM("Cap:AUTOLEVEL:0");

	// Z_PROBE (G30)
	SERIAL_PROTOCOLLNPGM("Cap:Z_PROBE:0");

	// MESH_REPORT (M420 V)
	SERIAL_PROTOCOLLNPGM("Cap:LEVELING_DATA:0");

	// SOFTWARE_POWER (M80, M81)
	SERIAL_PROTOCOLLNPGM("Cap:SOFTWARE_POWER:0");

	// CASE LIGHTS (M355)
	SERIAL_PROTOCOLLNPGM("Cap:TOGGLE_LIGHTS:0");
	SERIAL_PROTOCOLLNPGM("Cap:CASE_LIGHT_BRIGHTNESS:0");

	// EMERGENCY_PARSER (M108, M112, M410)
	SERIAL_PROTOCOLLNPGM("Cap:EMERGENCY_PARSER:0");
}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() { lcd::set_status(parser.string_arg); }

/**
 * M118: Display a message in the host console.
 *
 *  A  Append '// ' for an action command, as in OctoPrint
 *  E  Have the host 'echo:' the text
 */
inline void gcode_M118() {
	if (parser.boolval('E')) SERIAL_ECHO_START();
	if (parser.boolval('A')) SERIAL_ECHOPGM("// ");
	SERIAL_ECHOLN(parser.string_arg);
}

/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119() { endstops.M119(); }

/**
 * M120: Enable endstops and set non-homing endstop state to "enabled"
 */
inline void gcode_M120() { endstops.enable_globally(true); }

/**
 * M121: Disable endstops and set non-homing endstop state to "disabled"
 */
inline void gcode_M121() { endstops.enable_globally(false); }

/**
 * M200: Set filament diameter and set E axis units to cubic units
 *
 *    T<extruder> - Optional extruder number. Current extruder if omitted.
 *    D<linear> - Diameter of the filament. Use "D0" to switch back to linear units on the E axis.
 */
inline void gcode_M200() {

	if (get_target_extruder_from_command(200)) return;

	if (parser.seen('D')) {
		// setting any extruder filament size disables volumetric on the assumption that
		// slicers either generate in extruder values as cubic mm or as as filament feeds
		// for all extruders
		volumetric_enabled = (parser.value_linear_units() != 0.0);
		if (volumetric_enabled) {
			filament_size[target_extruder] = parser.value_linear_units();
			// make sure all extruders have some sane value for the filament size
			for (uint8_t i = 0; i < COUNT(filament_size); i++)
				if (!filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
		}
	}
	calculate_volumetric_multipliers();
}

/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M201() {

	GET_TARGET_EXTRUDER(201);

	LOOP_XYZE(i) {
		if (parser.seen(axis_codes[i])) {
			const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
			planner.max_acceleration_mm_per_s2[a] = parser.value_axis_units((AxisEnum)a);
		}
	}
	// steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
	planner.reset_acceleration_rates();
}


/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in units/sec
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M203() {

	GET_TARGET_EXTRUDER(203);

	LOOP_XYZE(i)
		if (parser.seen(axis_codes[i])) {
			const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
			planner.max_feedrate_mm_s[a] = parser.value_axis_units((AxisEnum)a);
		}
}

/**
 * M204: Set Accelerations in units/sec^2 (M204 P1200 R3000 T3000)
 *
 *    P = Printing moves
 *    R = Retract only (no X, Y, Z) moves
 *    T = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
 */
inline void gcode_M204() {
	if (parser.seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
		planner.travel_acceleration = planner.acceleration = parser.value_linear_units();
		SERIAL_ECHOLNPAIR("Setting Print and Travel Acceleration: ", planner.acceleration);
	}
	if (parser.seen('P')) {
		planner.acceleration = parser.value_linear_units();
		SERIAL_ECHOLNPAIR("Setting Print Acceleration: ", planner.acceleration);
	}
	if (parser.seen('R')) {
		planner.retract_acceleration = parser.value_linear_units();
		SERIAL_ECHOLNPAIR("Setting Retract Acceleration: ", planner.retract_acceleration);
	}
	if (parser.seen('T')) {
		planner.travel_acceleration = parser.value_linear_units();
		SERIAL_ECHOLNPAIR("Setting Travel Acceleration: ", planner.travel_acceleration);
	}
}

/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (units/s)
 *    T = Min Travel Feed Rate (units/s)
 *    B = Min Segment Time (µs)
 *    X = Max X Jerk (units/sec^2)
 *    Y = Max Y Jerk (units/sec^2)
 *    Z = Max Z Jerk (units/sec^2)
 *    E = Max E Jerk (units/sec^2)
 */
inline void gcode_M205() {
	if (parser.seen('S')) planner.min_feedrate_mm_s = parser.value_linear_units();
	if (parser.seen('T')) planner.min_travel_feedrate_mm_s = parser.value_linear_units();
	if (parser.seen('B')) planner.min_segment_time = parser.value_millis();
	if (parser.seen('X')) planner.max_jerk[X_AXIS] = parser.value_linear_units();
	if (parser.seen('Y')) planner.max_jerk[Y_AXIS] = parser.value_linear_units();
	if (parser.seen('Z')) planner.max_jerk[Z_AXIS] = parser.value_linear_units();
	if (parser.seen('E')) planner.max_jerk[E_AXIS] = parser.value_linear_units();
}

/**
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 *
 * *** @thinkyhead: I recommend deprecating M206 for SCARA in favor of M665.
 * ***              M206 for SCARA will remain enabled in 1.1.x for compatibility.
 * ***              In the next 1.2 release, it will simply be disabled by default.
 */
inline void gcode_M206() {
	LOOP_XYZ(i)
		if (parser.seen(axis_codes[i]))
			set_home_offset((AxisEnum)i, parser.value_linear_units());

	SYNC_PLAN_POSITION_KINEMATIC();
	report_current_position();
}

/**
 * M211: Enable, Disable, and/or Report software endstops
 *
 * Usage: M211 S1 to enable, M211 S0 to disable, M211 alone for report
 */
inline void gcode_M211() {
	SERIAL_ECHO_START();
	if (parser.seen('S')) soft_endstops_enabled = parser.value_bool();
	SERIAL_ECHOPGM(MSG_SOFT_ENDSTOPS);
	serialprintPGM(soft_endstops_enabled ? PSTR(MSG_ON) : PSTR(MSG_OFF));
	SERIAL_ECHOPGM(MSG_SOFT_MIN);
	SERIAL_ECHOPAIR(MSG_X, soft_endstop_min[X_AXIS]);
	SERIAL_ECHOPAIR(" " MSG_Y, soft_endstop_min[Y_AXIS]);
	SERIAL_ECHOPAIR(" " MSG_Z, soft_endstop_min[Z_AXIS]);
	SERIAL_ECHOPGM(MSG_SOFT_MAX);
	SERIAL_ECHOPAIR(MSG_X, soft_endstop_max[X_AXIS]);
	SERIAL_ECHOPAIR(" " MSG_Y, soft_endstop_max[Y_AXIS]);
	SERIAL_ECHOLNPAIR(" " MSG_Z, soft_endstop_max[Z_AXIS]);
}

/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
inline void gcode_M220() {
	if (parser.seenval('S')) feedrate_percentage = parser.value_int();
}

/**
 * M221: Set extrusion percentage (M221 T0 S95)
 */
inline void gcode_M221() {
	if (get_target_extruder_from_command(221)) return;
	if (parser.seenval('S'))
		flow_percentage[target_extruder] = parser.value_int();
}

/**
 * M226: Wait until the specified pin reaches the state required (M226 P<pin> S<state>)
 */
inline void gcode_M226() {
	if (parser.seen('P')) {
		const int pin_number = parser.value_int(),
			pin_state = parser.intval('S', -1); // required pin state - default is inverted

		if (WITHIN(pin_state, -1, 1) && pin_number > -1 && !pin_is_protected(pin_number)) {

			int target = LOW;

			stepper.synchronize();

			pinMode(pin_number, INPUT);
			switch (pin_state) {
			case 1:
				target = HIGH;
				break;
			case 0:
				target = LOW;
				break;
			case -1:
				target = !digitalRead(pin_number);
				break;
			}

			while (digitalRead(pin_number) != target) idle();

		} // pin_state -1 0 1 && pin_number > -1
	} // parser.seen('P')
}

/**
 * M301: Set PID parameters P I D (and optionally C, L)
 *
 *   P[float] Kp term
 *   I[float] Ki term (unscaled)
 *   D[float] Kd term (unscaled)
 *
 * With PID_EXTRUSION_SCALING:
 *
 *   C[float] Kc term
 *   L[float] LPQ length
 */
inline void gcode_M301() {

	// multi-extruder PID patch: M301 updates or prints a single extruder's PID values
	// default behaviour (omitting E parameter) is to update for extruder 0 only
	const uint8_t e = parser.byteval('E'); // extruder being updated

	if (e < HOTENDS) { // catch bad input value
		if (parser.seen('P')) PID_PARAM(Kp) = parser.value_float();
		if (parser.seen('I')) PID_PARAM(Ki) = scalePID_i(parser.value_float());
		if (parser.seen('D')) PID_PARAM(Kd) = scalePID_d(parser.value_float());

		thermalManager.updatePID();
		SERIAL_ECHO_START();
		SERIAL_ECHOPAIR(" p:", PID_PARAM(Kp));
		SERIAL_ECHOPAIR(" i:", unscalePID_i(PID_PARAM(Ki)));
		SERIAL_ECHOPAIR(" d:", unscalePID_d(PID_PARAM(Kd)));
		SERIAL_EOL();
	}
	else {
		SERIAL_ERROR_START();
		SERIAL_ERRORLN(MSG_INVALID_EXTRUDER);
	}
}

/**
 * M302: Allow cold extrudes, or set the minimum extrude temperature
 *
 *       S<temperature> sets the minimum extrude temperature
 *       P<bool> enables (1) or disables (0) cold extrusion
 *
 *  Examples:
 *
 *       M302         ; report current cold extrusion state
 *       M302 P0      ; enable cold extrusion checking
 *       M302 P1      ; disables cold extrusion checking
 *       M302 S0      ; always allow extrusion (disables checking)
 *       M302 S170    ; only allow extrusion above 170
 *       M302 S170 P1 ; set min extrude temp to 170 but leave disabled
 */
inline void gcode_M302() {
	const bool seen_S = parser.seen('S');
	if (seen_S) {
		thermalManager.extrude_min_temp = parser.value_celsius();
		thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0);
	}

	if (parser.seen('P'))
		thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0) || parser.value_bool();
	else if (!seen_S) {
		// Report current state
		SERIAL_ECHO_START();
		SERIAL_ECHOPAIR("Cold extrudes are ", (thermalManager.allow_cold_extrude ? "en" : "dis"));
		SERIAL_ECHOPAIR("abled (min temp ", thermalManager.extrude_min_temp);
		SERIAL_ECHOLNPGM("C)");
	}
}

/**
 * M303: PID relay autotune
 *
 *       S<temperature> sets the target temperature. (default 150C)
 *       E<extruder> (-1 for the bed) (default 0)
 *       C<cycles>
 *       U<bool> with a non-zero value will apply the result to current settings
 */
inline void gcode_M303() {
	const int e = parser.intval('E'), c = parser.intval('C', 5);
	const bool u = parser.boolval('U');

	int16_t temp = parser.celsiusval('S', e < 0 ? 70 : 150);

	if (WITHIN(e, 0, HOTENDS - 1))
		target_extruder = e;

	KEEPALIVE_STATE(NOT_BUSY); // don't send "busy: processing" messages during autotune output

	thermalManager.PID_autotune(temp, c, u);

	KEEPALIVE_STATE(IN_HANDLER);
}

/**
 * M400: Finish all moves
 */
inline void gcode_M400() { stepper.synchronize(); }

void quickstop_stepper() {
	stepper.quick_stop();
	stepper.synchronize();
	set_current_from_steppers_for_axis(ALL_AXES);
	SYNC_PLAN_POSITION_KINEMATIC();
}

/**
 * M428: Set home_offset based on the distance between the
 *       current_position and the nearest "reference point."
 *       If an axis is past center its endstop position
 *       is the reference-point. Otherwise it uses 0. This allows
 *       the Z offset to be set near the bed when using a max endstop.
 *
 *       M428 can't be used more than 2cm away from 0 or an endstop.
 *
 *       Use M206 to set these values directly.
 */
inline void gcode_M428() {
	bool err = false;
	LOOP_XYZ(i) {
		if (axis_homed[i]) {
			const float base = (current_position[i] > (soft_endstop_min[i] + soft_endstop_max[i]) * 0.5) ? base_home_pos((AxisEnum)i) : 0,
				diff = base - RAW_POSITION(current_position[i], i);
			if (WITHIN(diff, -20, 20)) {
				set_home_offset((AxisEnum)i, diff);
			}
			else {
				SERIAL_ERROR_START();
				SERIAL_ERRORLNPGM(MSG_ERR_M428_TOO_FAR);
				LCD_ALERTMESSAGEPGM("Err: Too far!");
				BUZZ(200, 40);
				err = true;
				break;
			}
		}
	}

	if (!err) {
		SYNC_PLAN_POSITION_KINEMATIC();
		report_current_position();
		LCD_MESSAGEPGM(MSG_HOME_OFFSETS_APPLIED);
		BUZZ(100, 659);
		BUZZ(100, 698);
	}
}

/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500() {
	(void)settings.save();
}

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501() {
	(void)settings.load();
}

/**
 * M502: Revert to default settings
 */
inline void gcode_M502() {
	(void)settings.reset();
}

/**
 * M503: print settings currently in memory
 */
inline void gcode_M503() {
	(void)settings.report(!parser.boolval('S', true));
}

/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {
}

/**
 * M355: Turn case light on/off and set brightness
 *
 *   P<byte>  Set case light brightness (PWM pin required - ignored otherwise)
 *
 *   S<bool>  Set case light on/off
 *
 *   When S turns on the light on a PWM pin then the current brightness level is used/restored
 *
 *   M355 P200 S0 turns off the light & sets the brightness level
 *   M355 S1 turns on the light with a brightness of 200 (assuming a PWM pin)
 */
inline void gcode_M355() {
	SERIAL_ERROR_START();
	SERIAL_ERRORLNPGM(MSG_ERR_M355_NONE);
}

/**
 * M999: Restart after being stopped
 *
 * Default behaviour is to flush the serial buffer and request
 * a resend to the host starting on the last N line received.
 *
 * Sending "M999 S1" will resume printing without flushing the
 * existing command buffer.
 *
 */
inline void gcode_M999() {
	Running = true;
	lcd::reset_alert_level();

	if (parser.boolval('S')) return;

	// gcode_LastN = Stopped_gcode_LastN;
	FlushSerialRequestResend();
}

inline void invalid_extruder_error(const uint8_t e) {
	SERIAL_ECHO_START();
	SERIAL_CHAR('T');
	SERIAL_ECHO_F(e, DEC);
	SERIAL_CHAR(' ');
	SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
}

/**
 * Perform a tool-change, which may result in moving the
 * previous tool out of the way and the new tool into place.
 */
void tool_change(const uint8_t tmp_extruder, const float fr_mm_s/*=0.0*/, bool no_move/*=false*/) {
	if (tmp_extruder >= EXTRUDERS)
		return invalid_extruder_error(tmp_extruder);

	UNUSED(fr_mm_s);
	UNUSED(no_move);

	active_extruder = tmp_extruder;

	SERIAL_ECHO_START();
	SERIAL_ECHOLNPAIR(MSG_ACTIVE_EXTRUDER, (int)active_extruder);
}

/**
 * T0-T3: Switch tool, usually switching extruders
 *
 *   F[units/min] Set the movement feedrate
 *   S1           Don't move the tool in XY after change
 */
inline void gcode_T(uint8_t tmp_extruder) {

	tool_change(tmp_extruder);
}

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void process_next_command() {
	char * const current_command = command_queue[cmd_queue_index_r];

	if (DEBUGGING(ECHO)) {
		SERIAL_ECHO_START();
		SERIAL_ECHOLN(current_command);
	}

	KEEPALIVE_STATE(IN_HANDLER);

	// Parse the next command in the queue
	parser.parse(current_command);

	// Handle a known G, M, or T
	switch (parser.command_letter) {
	case 'G': switch (parser.codenum) {

		// G0, G1
	case 0:
	case 1:
		gcode_G0_G1();
		break;

		// G2, G3
	case 2: // G2  - CW ARC
	case 3: // G3  - CCW ARC
		gcode_G2_G3(parser.codenum == 2);
		break;

		// G4 Dwell
	case 4:
		gcode_G4();
		break;

		// G5
	case 5: // G5  - Cubic B_spline
		gcode_G5();
		break;

	case 28: // G28: Home all axes, one at a time
		gcode_G28(false);
		break;

	case 90: // G90
		relative_mode = false;
		break;
	case 91: // G91
		relative_mode = true;
		break;

	case 92: // G92
		gcode_G92();
		break;
	}
			  break;

	case 'M': switch (parser.codenum) {
	case 17: // M17: Enable all stepper motors
		gcode_M17();
		break;

	case 20: // M20: list SD card
		gcode_M20(); break;
	case 21: // M21: init SD card
		gcode_M21(); break;
	case 22: // M22: release SD card
		gcode_M22(); break;
	case 23: // M23: Select file
		gcode_M23(); break;
	case 24: // M24: Start SD print
		gcode_M24(); break;
	case 25: // M25: Pause SD print
		gcode_M25(); break;
	case 26: // M26: Set SD index
		gcode_M26(); break;
	case 27: // M27: Get SD status
		gcode_M27(); break;
	case 28: // M28: Start SD write
		gcode_M28(); break;
	case 29: // M29: Stop SD write
		gcode_M29(); break;
	case 30: // M30 <filename> Delete File
		gcode_M30(); break;
	case 32: // M32: Select file and start SD print
		gcode_M32(); break;
	case 33: // M33: Get the long full path to a file or folder
		gcode_M33(); break;
	case 928: // M928: Start SD write
		gcode_M928(); break;

	case 31: // M31: Report time since the start of SD print or last M109
		gcode_M31(); break;

	case 42: // M42: Change pin state
		gcode_M42(); break;

	case 75: // M75: Start print timer
		gcode_M75(); break;
	case 76: // M76: Pause print timer
		gcode_M76(); break;
	case 77: // M77: Stop print timer
		gcode_M77(); break;

	case 78: // M78: Show print statistics
		gcode_M78(); break;

	case 104: // M104: Set hot end temperature
		gcode_M104();
		break;

	case 110: // M110: Set Current Line Number
		gcode_M110();
		break;

	case 111: // M111: Set debug level
		gcode_M111();
		break;

	case 108: // M108: Cancel Waiting
		gcode_M108();
		break;

	case 112: // M112: Emergency Stop
		gcode_M112();
		break;

	case 410: // M410 quickstop - Abort all the planned moves.
		gcode_M410();
		break;

	case 113: // M113: Set Host Keepalive interval
		gcode_M113();
		break;

	case 140: // M140: Set bed temperature
		gcode_M140();
		break;

	case 105: // M105: Report current temperature
		gcode_M105();
		KEEPALIVE_STATE(NOT_BUSY);
		return; // "ok" already printed

	case 155: // M155: Set temperature auto-report interval
		gcode_M155();
		break;

	case 109: // M109: Wait for hotend temperature to reach target
		gcode_M109();
		break;

	case 190: // M190: Wait for bed temperature to reach target
		gcode_M190();
		break;

	case 106: // M106: Fan On
		gcode_M106();
		break;
	case 107: // M107: Fan Off
		gcode_M107();
		break;

	case 81: // M81: Turn off Power, including Power Supply, if possible
		gcode_M81();
		break;

	case 82: // M82: Set E axis normal mode (same as other axes)
		gcode_M82();
		break;
	case 83: // M83: Set E axis relative mode
		gcode_M83();
		break;
	case 18: // M18 => M84
	case 84: // M84: Disable all steppers or set timeout
		gcode_M18_M84();
		break;
	case 85: // M85: Set inactivity stepper shutdown timeout
		gcode_M85();
		break;
	case 92: // M92: Set the steps-per-unit for one or more axes
		gcode_M92();
		break;
	case 114: // M114: Report current position
		gcode_M114();
		break;
	case 115: // M115: Report capabilities
		gcode_M115();
		break;
	case 117: // M117: Set LCD message text, if possible
		gcode_M117();
		break;
	case 118: // M118: Display a message in the host console
		gcode_M118();
		break;
	case 119: // M119: Report endstop states
		gcode_M119();
		break;
	case 120: // M120: Enable endstops
		gcode_M120();
		break;
	case 121: // M121: Disable endstops
		gcode_M121();
		break;

	case 200: // M200: Set filament diameter, E to cubic units
		gcode_M200();
		break;
	case 201: // M201: Set max acceleration for print moves (units/s^2)
		gcode_M201();
		break;
#if 0 // Not used for Sprinter/grbl gen6
	case 202: // M202
		gcode_M202();
		break;
#endif
	case 203: // M203: Set max feedrate (units/sec)
		gcode_M203();
		break;
	case 204: // M204: Set acceleration
		gcode_M204();
		break;
	case 205: //M205: Set advanced settings
		gcode_M205();
		break;

	case 206: // M206: Set home offsets
		gcode_M206();
		break;



	case 211: // M211: Enable, Disable, and/or Report software endstops
		gcode_M211();
		break;

	case 220: // M220: Set Feedrate Percentage: S<percent> ("FR" on your LCD)
		gcode_M220();
		break;

	case 221: // M221: Set Flow Percentage
		gcode_M221();
		break;

	case 226: // M226: Wait until a pin reaches a state
		gcode_M226();
		break;

	case 301: // M301: Set hotend PID parameters
		gcode_M301();
		break;

	case 302: // M302: Allow cold extrudes (set the minimum extrude temperature)
		gcode_M302();
		break;

	case 303: // M303: PID autotune
		gcode_M303();
		break;

	case 400: // M400: Finish all moves
		gcode_M400();
		break;

	case 428: // M428: Apply current_position to home_offset
		gcode_M428();
		break;

	case 500: // M500: Store settings in EEPROM
		gcode_M500();
		break;
	case 501: // M501: Read settings from EEPROM
		gcode_M501();
		break;
	case 502: // M502: Revert to default settings
		gcode_M502();
		break;

	case 503: // M503: print settings currently in memory
		gcode_M503();
		break;

	case 907: // M907: Set digital trimpot motor current using axis codes.
		gcode_M907();
		break;

	case 355: // M355 set case light brightness
		gcode_M355();
		break;

	case 999: // M999: Restart after being Stopped
		gcode_M999();
		break;
	}
			  break;

	case 'T':
		gcode_T(parser.codenum);
		break;

	default: parser.unknown_command_error();
	}

	KEEPALIVE_STATE(NOT_BUSY);

	ok_to_send();
}

/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void FlushSerialRequestResend() {
	//char command_queue[cmd_queue_index_r][100]="Resend:";
	MYSERIAL.flush();
	SERIAL_PROTOCOLPGM(MSG_RESEND);
	SERIAL_PROTOCOLLN(gcode_LastN + 1);
	ok_to_send();
}

/**
 * Send an "ok" message to the host, indicating
 * that a command was successfully processed.
 *
 * If ADVANCED_OK is enabled also include:
 *   N<int>  Line number of the command, if any
 *   P<int>  Planner space remaining
 *   B<int>  Block queue space remaining
 */
void ok_to_send() {
	refresh_cmd_timeout();
	if (!send_ok[cmd_queue_index_r]) return;
	SERIAL_PROTOCOLPGM(MSG_OK);
#if ENABLED(ADVANCED_OK)
	char* p = command_queue[cmd_queue_index_r];
	if (*p == 'N') {
		SERIAL_PROTOCOL(' ');
		SERIAL_ECHO(*p++);
		while (NUMERIC_SIGNED(*p))
			SERIAL_ECHO(*p++);
	}
	SERIAL_PROTOCOLPGM(" P"); SERIAL_PROTOCOL(int(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
	SERIAL_PROTOCOLPGM(" B"); SERIAL_PROTOCOL(BUFSIZE - commands_in_queue);
#endif
	SERIAL_EOL();
}

/**
 * Constrain the given coordinates to the software endstops.
 */

 // NOTE: This makes no sense for delta beds other than Z-axis.
 //       For delta the X/Y would need to be clamped at
 //       DELTA_PRINTABLE_RADIUS from center of bed, but delta
 //       now enforces is_position_reachable for X/Y regardless
 //       of HAS_SOFTWARE_ENDSTOPS, so that enforcement would be
 //       redundant here.  Probably should #ifdef out the X/Y
 //       axis clamps here for delta and just leave the Z clamp.

void clamp_to_software_endstops(float target[XYZ]) {
	if (!soft_endstops_enabled) return;
	NOLESS(target[X_AXIS], soft_endstop_min[X_AXIS]);
	NOLESS(target[Y_AXIS], soft_endstop_min[Y_AXIS]);
	NOLESS(target[Z_AXIS], soft_endstop_min[Z_AXIS]);
	NOMORE(target[X_AXIS], soft_endstop_max[X_AXIS]);
	NOMORE(target[Y_AXIS], soft_endstop_max[Y_AXIS]);
	NOMORE(target[Z_AXIS], soft_endstop_max[Z_AXIS]);
}

/**
 * Get the stepper positions in the cartes[] array.
 * Forward kinematics are applied for DELTA and SCARA.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain the "ideal" coordinates
 * suitable for current_position, etc.
 */
void get_cartesian_from_steppers() {
	cartes[X_AXIS] = stepper.get_axis_position_mm(X_AXIS);
	cartes[Y_AXIS] = stepper.get_axis_position_mm(Y_AXIS);
	cartes[Z_AXIS] = stepper.get_axis_position_mm(Z_AXIS);
}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 */
void set_current_from_steppers_for_axis(const AxisEnum axis) {
	get_cartesian_from_steppers();
	if (axis == ALL_AXES)
		COPY(current_position, cartes);
	else
		current_position[axis] = cartes[axis];
}

/**
 * Prepare a linear move in a Cartesian setup.
 * If Mesh Bed Leveling is enabled, perform a mesh move.
 *
 * Returns true if the caller didn't update current_position.
 */
inline bool prepare_move_to_destination_cartesian() {
	// Do not use feedrate_percentage for E or Z only moves
	if (current_position[X_AXIS] == destination[X_AXIS] && current_position[Y_AXIS] == destination[Y_AXIS])
		line_to_destination();
	else {
		const float fr_scaled = MMS_SCALED(feedrate_mm_s);
		line_to_destination(fr_scaled);
	}
	return false;
}

/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 */
void prepare_move_to_destination() {
	clamp_to_software_endstops(destination);
	refresh_cmd_timeout();

	if (!DEBUGGING(DRYRUN)) {
		if (destination[E_AXIS] != current_position[E_AXIS]) {
			if (thermalManager.tooColdToExtrude()) {
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
	}

	if (
		prepare_move_to_destination_cartesian()
		) return;

	set_current_to_destination();
}

/**
 * Plan an arc in 2 dimensions
 *
 * The arc is approximated by generating many small linear segments.
 * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
 * Arcs should only be made relatively large (over 5mm), as larger arcs with
 * larger segments will tend to be more efficient. Your slicer should have
 * options for G2/G3 arc generation. In future these options may be GCode tunable.
 */
void plan_arc(
	float logical[XYZE], // Destination position
	float *offset,       // Center of rotation relative to current_position
	uint8_t clockwise    // Clockwise?
) {
	constexpr AxisEnum p_axis = X_AXIS, q_axis = Y_AXIS, l_axis = Z_AXIS;

	// Radius vector from center to current location
	float r_P = -offset[0], r_Q = -offset[1];

	const float radius = HYPOT(r_P, r_Q),
		center_P = current_position[p_axis] - r_P,
		center_Q = current_position[q_axis] - r_Q,
		rt_X = logical[p_axis] - center_P,
		rt_Y = logical[q_axis] - center_Q,
		linear_travel = logical[l_axis] - current_position[l_axis],
		extruder_travel = logical[E_AXIS] - current_position[E_AXIS];

	// CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
	float angular_travel = ATAN2(r_P * rt_Y - r_Q * rt_X, r_P * rt_X + r_Q * rt_Y);
	if (angular_travel < 0) angular_travel += RADIANS(360);
	if (clockwise) angular_travel -= RADIANS(360);

	// Make a circle if the angular rotation is 0 and the target is current position
	if (angular_travel == 0 && current_position[p_axis] == logical[p_axis] && current_position[q_axis] == logical[q_axis])
		angular_travel = RADIANS(360);

	const float mm_of_travel = HYPOT(angular_travel * radius, FABS(linear_travel));
	if (mm_of_travel < 0.001) return;

	uint16_t segments = FLOOR(mm_of_travel / (MM_PER_ARC_SEGMENT));
	if (segments == 0) segments = 1;

	/**
	 * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
	 * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
	 *     r_T = [cos(phi) -sin(phi);
	 *            sin(phi)  cos(phi)] * r ;
	 *
	 * For arc generation, the center of the circle is the axis of rotation and the radius vector is
	 * defined from the circle center to the initial position. Each line segment is formed by successive
	 * vector rotations. This requires only two cos() and sin() computations to form the rotation
	 * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
	 * all double numbers are single precision on the Arduino. (True double precision will not have
	 * round off issues for CNC applications.) Single precision error can accumulate to be greater than
	 * tool precision in some cases. Therefore, arc path correction is implemented.
	 *
	 * Small angle approximation may be used to reduce computation overhead further. This approximation
	 * holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
	 * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
	 * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
	 * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
	 * issue for CNC machines with the single precision Arduino calculations.
	 *
	 * This approximation also allows plan_arc to immediately insert a line segment into the planner
	 * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
	 * a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
	 * This is important when there are successive arc motions.
	 */
	 // Vector rotation matrix values
	float arc_target[XYZE];
	const float theta_per_segment = angular_travel / segments,
		linear_per_segment = linear_travel / segments,
		extruder_per_segment = extruder_travel / segments,
		sin_T = theta_per_segment,
		cos_T = 1 - 0.5 * sq(theta_per_segment); // Small angle approximation

// Initialize the linear axis
	arc_target[l_axis] = current_position[l_axis];

	// Initialize the extruder axis
	arc_target[E_AXIS] = current_position[E_AXIS];

	const float fr_mm_s = MMS_SCALED(feedrate_mm_s);

	millis_t next_idle_ms = millis() + 200UL;

	int8_t count = N_ARC_CORRECTION;

	for (uint16_t i = 1; i < segments; i++) { // Iterate (segments-1) times

		thermalManager.manage_heater();
		if (ELAPSED(millis(), next_idle_ms)) {
			next_idle_ms = millis() + 200UL;
			idle();
		}

		if (--count) {
			// Apply vector rotation matrix to previous r_P / 1
			const float r_new_Y = r_P * sin_T + r_Q * cos_T;
			r_P = r_P * cos_T - r_Q * sin_T;
			r_Q = r_new_Y;
		}
		else
		{
			count = N_ARC_CORRECTION;

			// Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
			// Compute exact location by applying transformation matrix from initial radius vector(=-offset).
			// To reduce stuttering, the sin and cos could be computed at different times.
			// For now, compute both at the same time.
			const float cos_Ti = cos(i * theta_per_segment), sin_Ti = sin(i * theta_per_segment);
			r_P = -offset[0] * cos_Ti + offset[1] * sin_Ti;
			r_Q = -offset[0] * sin_Ti - offset[1] * cos_Ti;
		}

		// Update arc_target location
		arc_target[p_axis] = center_P + r_P;
		arc_target[q_axis] = center_Q + r_Q;
		arc_target[l_axis] += linear_per_segment;
		arc_target[E_AXIS] += extruder_per_segment;

		clamp_to_software_endstops(arc_target);

		planner.buffer_line_kinematic(arc_target, fr_mm_s, active_extruder);
	}

	// Ensure last segment arrives at target location.
	planner.buffer_line_kinematic(logical, fr_mm_s, active_extruder);

	// As far as the parser is concerned, the position is now == target. In reality the
	// motion control system might still be processing the action and the real tool position
	// in any intermediate location.
	set_current_to_destination();
}

void plan_cubic_move(const float offset[4]) {
	cubic_b_spline(current_position, destination, offset, MMS_SCALED(feedrate_mm_s), active_extruder);

	// As far as the parser is concerned, the position is now == destination. In reality the
	// motion control system might still be processing the action and the real tool position
	// in any intermediate location.
	set_current_to_destination();
}

float calculate_volumetric_multiplier(float diameter) {
	if (!volumetric_enabled || diameter == 0) return 1.0;
	return 1.0 / (M_PI * sq(diameter * 0.5));
}

void calculate_volumetric_multipliers() {
	for (uint8_t i = 0; i < COUNT(filament_size); i++)
		volumetric_multiplier[i] = calculate_volumetric_multiplier(filament_size[i]);
}

void enable_all_steppers() {
	enable_X();
	enable_Y();
	enable_Z();
	enable_E0();
	enable_E1();
	enable_E2();
	enable_E3();
	enable_E4();
}

void disable_e_steppers() {
	disable_E0();
	disable_E1();
	disable_E2();
	disable_E3();
	disable_E4();
}

void disable_all_steppers() {
	disable_X();
	disable_Y();
	disable_Z();
	disable_e_steppers();
}

/**
 * Manage several activities:
 *  - Check for Filament Runout
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if pin CHDK needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {

	if (commands_in_queue < BUFSIZE) get_available_commands();

	const millis_t ms = millis();

	if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)) {
		SERIAL_ERROR_START();
		SERIAL_ECHOLNPAIR(MSG_KILL_INACTIVE_TIME, parser.command_ptr);
		kill(PSTR(MSG_KILLED));
	}

	// Prevent steppers timing-out in the middle of M600
#define MOVE_AWAY_TEST true

	if (MOVE_AWAY_TEST && stepper_inactive_time && ELAPSED(ms, previous_cmd_ms + stepper_inactive_time)
		&& !ignore_stepper_queue && !planner.blocks_queued()) {
		disable_X();
		disable_Y();
		disable_Z();
		disable_e_steppers();
	}

	planner.check_axes_activity();
}

/**
 * Standard idle routine keeps the machine alive
 */
void idle(
) {
	lcd::update();

	host_keepalive();

	auto_report_temperatures();

	manage_inactivity(
	);

	thermalManager.manage_heater();

	print_job_timer.tick();

}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill(const char* lcd_msg) {
	SERIAL_ERROR_START();
	SERIAL_ERRORLNPGM(MSG_ERR_KILLED);

	thermalManager.disable_all_heaters();
	disable_all_steppers();

	UNUSED(lcd_msg);

	_delay_ms(600); // Wait a short time (allows messages to get out before shutting down.
	cli(); // Stop interrupts

	_delay_ms(250); //Wait to ensure all interrupts routines stopped
	thermalManager.disable_all_heaters(); //turn off heaters again

	suicide();
	while (1) {
		watchdog_reset();
	} // Wait for reset
}

/**
 * Turn off heaters and stop the print in progress
 * After a stop the machine may be resumed with M999
 */
void stop() {
	thermalManager.disable_all_heaters(); // 'unpause' taken care of in here

	if (IsRunning()) {
		Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
		SERIAL_ERROR_START();
		SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
		LCD_MESSAGEPGM(MSG_STOPPED);
		safe_delay(350);       // allow enough time for messages to get out before stopping
		Running = false;
	}
}

/**
 * Marlin entry-point: Set up before the program loop
 *  - Set up the kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
 *    • planner
 *    • watchdog
 *    • stepper
 *    • photo pin
 *    • servos
 *    • LCD controller
 *    • Digipot I2C
 *    • Z probe sled
 *    • status LEDs
 */
void setup() {
	setup_killpin();

	setup_powerhold();

	MYSERIAL.begin(BAUDRATE);
	SERIAL_PROTOCOLLNPGM("start");
	SERIAL_ECHO_START();

	// Check startup - does nothing if bootloader sets MCUSR to 0
	byte mcu = MCUSR;
	if (mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
	if (mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
	if (mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
	if (mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
	if (mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
	MCUSR = 0;

	SERIAL_ECHOPGM(MSG_MARLIN);
	SERIAL_CHAR(' ');
	SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
	SERIAL_EOL();

	SERIAL_ECHO_START();
	SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
	SERIAL_ECHOPGM(STRING_DISTRIBUTION_DATE);
	SERIAL_ECHOLNPGM(MSG_AUTHOR STRING_CONFIG_H_AUTHOR);
	SERIAL_ECHOLNPGM("Compiled: " __DATE__);

	SERIAL_ECHO_START();
	SERIAL_ECHOPAIR(MSG_FREE_MEMORY, freeMemory());
	SERIAL_ECHOLNPAIR(MSG_PLANNER_BUFFER_BYTES, (int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

	// Send "ok" after commands by default
	for (int8_t i = 0; i < BUFSIZE; i++) send_ok[i] = true;

	// Load data from EEPROM if available (or use defaults)
	// This also updates variables in the planner, elsewhere
	(void)settings.load();

	// Initialize current position based on home_offset
	COPY(current_position, home_offset);

	// Vital to init stepper/planner equivalent for current_position
	SYNC_PLAN_POSITION_KINEMATIC();

	thermalManager.init();    // Initialize temperature loop

	watchdog_init();

	stepper.init();    // Initialize stepper, this enables interrupts!
	servo_init();

#define CUSTOM_BOOTSCREEN_TIMEOUT 2500

	lcd::initialize();
}

/**
 * The main Marlin program loop
 *
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call heater manager
 *  - Call inactivity manager
 *  - Call endstop manager
 *  - Call LCD update
 */
void loop() {
	if (commands_in_queue < BUFSIZE) get_available_commands();

	card.checkautostart(false);

	if (commands_in_queue) {
		if (card.saving) {
			char* command = command_queue[cmd_queue_index_r];
			if (strstr_P(command, PSTR("M29"))) {
				// M29 closes the file
				card.closefile();
				SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
				ok_to_send();
			}
			else {
				// Write the string from the read buffer to SD
				card.write_command(command);
				if (card.logging)
					process_next_command(); // The card is saving because it's logging
				else
					ok_to_send();
			}
		}
		else
			process_next_command();

		// The queue may be reset by a command handler or by code invoked by idle() within a handler
		if (commands_in_queue) {
			--commands_in_queue;
			if (++cmd_queue_index_r >= BUFSIZE) cmd_queue_index_r = 0;
		}
}
	endstops.report_state();
	idle();
}

