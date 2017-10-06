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
 * Conditionals_LCD.h
 * Conditionals that need to be set before Configuration_adv.h or pins.h
 */

#ifndef CONDITIONALS_LCD_H // Get the LCD defines which are needed first
#define CONDITIONALS_LCD_H

/* Custom characters defined in the first 8 characters of the LCD */
#define LCD_BEDTEMP_CHAR     0x00  // Print only as a char. This will have 'unexpected' results when used in a string!
#define LCD_DEGREE_CHAR      0x01
#define LCD_STR_THERMOMETER "\x02" // Still used with string concatenation
#define LCD_UPLEVEL_CHAR     0x03
#define LCD_STR_REFRESH     "\x04"
#define LCD_STR_FOLDER      "\x05"
#define LCD_FEEDRATE_CHAR    0x06
#define LCD_CLOCK_CHAR       0x07
#define LCD_STR_ARROW_RIGHT ">"  /* from the default character set */

  #ifndef BOOTSCREEN_TIMEOUT
    #define BOOTSCREEN_TIMEOUT 2500
  #endif

  #define HAS_DEBUG_MENU ENABLED(LCD_PROGRESS_BAR_TEST)

  /**
   * Extruders have some combination of stepper motors and hotends
   * so we separate these concepts into the defines:
   *
   *  EXTRUDERS    - Number of Selectable Tools
   *  HOTENDS      - Number of hotends, whether connected or separate
   *  E_STEPPERS   - Number of actual E stepper motors
   *  E_MANUAL     - Number of E steppers for LCD move options
   *  TOOL_E_INDEX - Index to use when getting/setting the tool state
   *
   */

    #define HOTENDS       EXTRUDERS

    #define E_STEPPERS    EXTRUDERS
    #define E_MANUAL      EXTRUDERS
    #define TOOL_E_INDEX  current_block->active_extruder

  /**
   * DISTINCT_E_FACTORS affects how some E factors are accessed
   */
#undef DISTINCT_E_FACTORS
#define XYZE_N XYZE
#define E_AXIS_N E_AXIS

  /**
   * Set a flag for a servo probe
   */
  #define HAS_Z_SERVO_ENDSTOP 0

  /**
   * Set a flag for any enabled probe
   */
  #define PROBE_SELECTED 0

  /**
   * Clear probe pin settings when no probe is selected
   */
  #undef Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
  #undef Z_MIN_PROBE_ENDSTOP

  #define HAS_SOFTWARE_ENDSTOPS 1
  #define HAS_RESUME_CONTINUE 0
  #define HAS_COLOR_LEDS 0

#endif // CONDITIONALS_LCD_H
