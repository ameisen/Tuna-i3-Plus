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
 * gcode.h - Parser for a GCode line, providing a parameter interface.
 *           Codes like M149 control the way the GCode parser behaves,
 *           so settings for these codes are located in this class.
 */

#ifndef GCODE_H
#define GCODE_H

#include "enum.h"
#include "types.h"
#include "MarlinConfig.h"

//#define DEBUG_GCODE_PARSER

#if ENABLED(DEBUG_GCODE_PARSER)
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    extern char* hex_address(const void * const w);
  #else
    #include "hex_print_routines.h"
  #endif
  #include "serial.h"
#endif

#if ENABLED(INCH_MODE_SUPPORT)
  extern bool volumetric_enabled;
#endif

/**
 * GCode parser
 *
 *  - Parse a single gcode line for its letter, code, subcode, and parameters
 *  - FASTER_GCODE_PARSER:
 *    - Flags existing params (1 bit each)
 *    - Stores value offsets (1 byte each)
 *  - Provide accessors for parameters:
 *    - Parameter exists
 *    - Parameter has value
 *    - Parameter value in different units and types
 */
class GCodeParser final {

private:
  static char *value_ptr;           // Set by seen, used to fetch the value

  #if ENABLED(FASTER_GCODE_PARSER)
    static byte codebits[4];        // Parameters pre-scanned
    static uint8_t param[26];       // For A-Z, offsets into command args
  #else
    static char *command_args;      // Args start here, for slow scan
  #endif

public:

  // Global states for GCode-level units features

  #if ENABLED(INCH_MODE_SUPPORT)
    static float linear_unit_factor, volumetric_unit_factor;
  #endif

  #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
    static TempUnit input_temp_units;
  #endif

  // Command line state
  static char *command_ptr,               // The command, so it can be echoed
              *string_arg;                // string of command line

  static char command_letter;             // G, M, or T
  static int codenum;                     // 123
  #if USE_GCODE_SUBCODES
    static int subcode;                   // .1
  #endif

  #if ENABLED(DEBUG_GCODE_PARSER)
    void debug();
  #endif

  // Reset is done before parsing
  static void __forceinline __flatten reset();

  // Index so that 'X' falls on index 24
  #define PARAM_IND(N)  ((N) >> 3)
  #define PARAM_BIT(N)  ((N) & 0x7)
  #define LETTER_OFF(N) ((N) - 'A')
  #define LETTER_IND(N) PARAM_IND(LETTER_OFF(N))
  #define LETTER_BIT(N) PARAM_BIT(LETTER_OFF(N))

  #if ENABLED(FASTER_GCODE_PARSER)

    // Set the flag and pointer for a parameter
    static void __forceinline __flatten set(const char c, char * const ptr
      #if ENABLED(DEBUG_GCODE_PARSER)
        , const bool debug=false
      #endif
    ) {
      const uint8_t ind = LETTER_OFF(c);
      if (ind >= COUNT(param)) return;           // Only A-Z
      SBI(codebits[PARAM_IND(ind)], PARAM_BIT(ind));        // parameter exists
      param[ind] = ptr ? ptr - command_ptr : 0;  // parameter offset or 0
      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) {
          SERIAL_ECHOPAIR("Set bit ", (int)PARAM_BIT(ind));
          SERIAL_ECHOPAIR(" of index ", (int)PARAM_IND(ind));
          SERIAL_ECHOLNPAIR(" | param = ", (int)param[ind]);
        }
      #endif
    }

    // Code seen bit was set. If not found, value_ptr is unchanged.
    // This allows "if (seen('A')||seen('B'))" to use the last-found value.
    // This is volatile because its side-effects are important
    static volatile bool __forceinline __flatten seen(const char c) {
      const uint8_t ind = LETTER_OFF(c);
      if (ind >= COUNT(param)) return false; // Only A-Z
      const bool b = TEST(codebits[PARAM_IND(ind)], PARAM_BIT(ind));
      if (b) value_ptr = param[ind] ? command_ptr + param[ind] : (char*)nullptr;
      return b;
    }

    static bool __forceinline __flatten seen_any() { return codebits[3] || codebits[2] || codebits[1] || codebits[0]; }

    #define SEEN_TEST(L) TEST(codebits[LETTER_IND(L)], LETTER_BIT(L))

  #else // !FASTER_GCODE_PARSER

    // Code is found in the string. If not found, value_ptr is unchanged.
    // This allows "if (seen('A')||seen('B'))" to use the last-found value.
    // This is volatile because its side-effects are important
    static volatile bool seen(const char c) {
      const char *p = strchr(command_args, c);
      const bool b = !!p;
      if (b) value_ptr = DECIMAL_SIGNED(p[1]) ? &p[1] : (char*)nullptr;
      return b;
    }

    static bool seen_any() { return *command_args == '\0'; }

    #define SEEN_TEST(L) !!strchr(command_args, L)

  #endif // !FASTER_GCODE_PARSER

  // Seen any axis parameter
  static bool __forceinline __flatten seen_axis() {
    return SEEN_TEST('X') || SEEN_TEST('Y') || SEEN_TEST('Z') || SEEN_TEST('E');
  }

  // Populate all fields by parsing a single line of GCode
  // This uses 54 bytes of SRAM to speed up seen/value
  static void __forceinline __flatten parse(char * p);

  // The code value pointer was set
  static bool __forceinline __flatten has_value() { return value_ptr != nullptr; }

  // Seen a parameter with a value
  inline static bool __forceinline __flatten seenval(const char c) { return seen(c) && has_value(); }

  // Float removes 'E' to prevent scientific notation interpretation
  inline static float __forceinline __flatten value_float() {
    if (value_ptr) {
      char *e = value_ptr;
      for (;;) {
        const char c = *e;
        if (c == '\0' || c == ' ') break;
        if (c == 'E' || c == 'e') {
          *e = '\0';
          const float ret = strtod(value_ptr, nullptr);
          *e = c;
          return ret;
        }
        ++e;
      }
      return strtod(value_ptr, nullptr);
    }
    return 0.0;
  }

  // Code value as a long or ulong
  inline static int32 value_long() { return value_ptr ? strtol(value_ptr, nullptr, 10) : 0L; }
  inline static uint32 value_ulong() { return value_ptr ? strtoul(value_ptr, nullptr, 10) : 0UL; }

  inline static int24 value_i24() { return value_ptr ? int24(strtol(value_ptr, nullptr, 10)) : 0_i24; }
  inline static uint24 value_u24() { return value_ptr ? uint24(strtoul(value_ptr, nullptr, 10)) : 0_u24; }

  // Code value for use as time
  static millis_t __forceinline value_millis() { return value_ulong(); }
  static millis_t __forceinline value_millis_from_seconds() { return value_float() * 1000UL; }

  // Reduce to fewer bits
  static int16_t __forceinline value_int() { return (int16_t)value_long(); }
  static uint16_t __forceinline value_ushort() { return (uint16_t)value_long(); }
  inline static uint8_t value_byte() { return (uint8_t)clamp(value_long(), 0_i32, 255_i32); }

  // Bool is true with no value or non-zero
  inline static bool value_bool() { return !has_value() || value_byte(); }

  // Units modes: Inches, Fahrenheit, Kelvin

  #if ENABLED(INCH_MODE_SUPPORT)

    inline static void set_input_linear_units(const LinearUnit units) {
      switch (units) {
        case LINEARUNIT_INCH:
          linear_unit_factor = 25.4;
          break;
        case LINEARUNIT_MM:
        default:
          linear_unit_factor = 1.0;
          break;
      }
      volumetric_unit_factor = POW(linear_unit_factor, 3.0);
    }

    inline static float axis_unit_factor(const AxisEnum axis) {
      return (axis >= E_AXIS && volumetric_enabled ? volumetric_unit_factor : linear_unit_factor);
    }

    inline static float value_linear_units()                     { return value_float() * linear_unit_factor; }
    inline static float value_axis_units(const AxisEnum axis)    { return value_float() * axis_unit_factor(axis); }
    inline static float value_per_axis_unit(const AxisEnum axis) { return value_float() / axis_unit_factor(axis); }

  #else

    static float __forceinline value_linear_units()                  {            return value_float(); }
    static float __forceinline value_axis_units(const AxisEnum a)    { UNUSED(a); return value_float(); }
    static float __forceinline value_per_axis_unit(const AxisEnum a) { UNUSED(a); return value_float(); }

  #endif

  #if ENABLED(TEMPERATURE_UNITS_SUPPORT)

    inline static void set_input_temp_units(TempUnit units) { input_temp_units = units; }

    #if ENABLED(ULTIPANEL) && DISABLED(DISABLE_M503)

      static char __forceinline temp_units_code() {
        return input_temp_units == TEMPUNIT_K ? 'K' : input_temp_units == TEMPUNIT_F ? 'F' : 'C';
      }
      static char* __forceinline temp_units_name() {
        return input_temp_units == TEMPUNIT_K ? PSTR("Kelvin") : input_temp_units == TEMPUNIT_F ? PSTR("Fahrenheit") : PSTR("Celsius");
      }
      inline static float to_temp_units(const float &f) {
        switch (input_temp_units) {
          case TEMPUNIT_F:
            return f * 0.5555555556 + 32.0;
          case TEMPUNIT_K:
            return f + 273.15;
          case TEMPUNIT_C:
          default:
            return f;
        }
      }

    #endif // ULTIPANEL && !DISABLE_M503

    inline static float value_celsius() {
      const float f = value_float();
      switch (input_temp_units) {
        case TEMPUNIT_F:
          return (f - 32.0) * 0.5555555556;
        case TEMPUNIT_K:
          return f - 273.15;
        case TEMPUNIT_C:
        default:
          return f;
      }
    }

    inline static float value_celsius_diff() {
      switch (input_temp_units) {
        case TEMPUNIT_F:
          return value_float() * 0.5555555556;
        case TEMPUNIT_C:
        case TEMPUNIT_K:
        default:
          return value_float();
      }
    }

  #else // !TEMPERATURE_UNITS_SUPPORT

    static float __forceinline value_celsius()      { return value_float(); }
    static float __forceinline value_celsius_diff() { return value_float(); }

  #endif // !TEMPERATURE_UNITS_SUPPORT

  static float __forceinline value_feedrate() { return value_linear_units(); }

  void unknown_command_error();

  // Provide simple value accessors with default option
  static float    __forceinline __flatten floatval(const char c, const float dval=0.0)   { return seenval(c) ? value_float()        : dval; }
  static bool     __forceinline boolval(const char c, const bool dval=false)   { return seen(c)    ? value_bool()         : dval; }
  static uint8_t  __forceinline byteval(const char c, const uint8_t dval=0)    { return seenval(c) ? value_byte()         : dval; }
  static int16_t  __forceinline intval(const char c, const int16_t dval=0)     { return seenval(c) ? value_int()          : dval; }
  static uint16_t __forceinline ushortval(const char c, const uint16_t dval=0) { return seenval(c) ? value_ushort()       : dval; }
  static int24    __forceinline i24val(const char c, const int24 dval = 0)     { return seenval(c) ? value_i24()          : dval; }
  static uint24   __forceinline u24val(const char c, const uint24 dval = 0)    { return seenval(c) ? value_u24()          : dval; }
  static int32  __forceinline longval(const char c, const int32 dval=0)    { return seenval(c) ? value_long()         : dval; }
  static uint32 __forceinline ulongval(const char c, const uint32 dval=0)  { return seenval(c) ? value_ulong()        : dval; }
  static float    __forceinline linearval(const char c, const float dval=0.0)  { return seenval(c) ? value_linear_units() : dval; }
  static float    __forceinline celsiusval(const char c, const float dval=0.0) { return seenval(c) ? value_celsius()      : dval; }

};

extern GCodeParser parser;

#endif // GCODE_H
