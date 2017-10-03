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
 * temperature.h - temperature controller
 */

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "thermistortables.h"

#include "MarlinConfig.h"

#define HOTEND_LOOP() for (int8_t e = 0; e < HOTENDS; e++)

#define HOTEND_INDEX  0
#define EXTRUDER_IDX  0

/**
 * States for ADC reading in the ISR
 */
enum ADCSensorState {
  PrepareTemp_0,
  MeasureTemp_0,
  PrepareTemp_BED,
  MeasureTemp_BED,
  SensorsReady, // Temperatures ready. Delay the next round of readings to let ADC pins settle.
  StartupDelay  // Startup, delay initial temp reading a tiny bit so the hardware can settle
};

// Minimum number of Temperature::ISR loops between sensor readings.
// Multiplied by 16 (OVERSAMPLENR) to obtain the total time to
// get all oversampled sensor readings
#define MIN_ADC_ISR_LOOPS 10

constexpr int ACTUAL_ADC_SAMPLES = max(int(MIN_ADC_ISR_LOOPS), int(SensorsReady));

class Temperature {

  public:

    static float current_temperature[HOTENDS],
                 current_temperature_bed;
    static int16_t current_temperature_raw[HOTENDS],
                   target_temperature[HOTENDS],
                   current_temperature_bed_raw;

    static int16_t target_temperature_bed;

    static volatile bool in_temp_isr;

    static uint8_t soft_pwm_amount[HOTENDS],
                   soft_pwm_amount_bed;

      #define PID_dT ((OVERSAMPLENR * float(ACTUAL_ADC_SAMPLES)) / (F_CPU / 64.0f / 256.0f))

        static float Kp, Ki, Kd;
        #define PID_PARAM(param, h) Temperature::param

      // Apply the scale factors to the PID values
      #define scalePID_i(i)   ( (i) * PID_dT )
      #define unscalePID_i(i) ( (i) / PID_dT )
      #define scalePID_d(d)   ( (d) / PID_dT )
      #define unscalePID_d(d) ( (d) * PID_dT )

      static uint16_t watch_target_temp[HOTENDS];
      static millis_t watch_heater_next_ms[HOTENDS];

      static uint16_t watch_target_bed_temp;
      static millis_t watch_bed_next_ms;

      static bool allow_cold_extrude;
      static int16_t extrude_min_temp;
      static bool tooColdToExtrude(uint8_t e) {
          UNUSED(e);
        return allow_cold_extrude ? false : degHotend(HOTEND_INDEX) < extrude_min_temp;
      }

  private:

    static volatile bool temp_meas_ready;

      static float temp_iState[HOTENDS],
                   temp_dState[HOTENDS],
                   pTerm[HOTENDS],
                   iTerm[HOTENDS],
                   dTerm[HOTENDS];

      static float pid_error[HOTENDS];
      static bool pid_reset[HOTENDS];

      static millis_t next_bed_check_ms;

    static uint16_t raw_temp_value[MAX_EXTRUDERS],
                    raw_temp_bed_value;

    // Init min and max temp with extreme values to prevent false errors during startup
    static int16_t minttemp_raw[HOTENDS],
                   maxttemp_raw[HOTENDS],
                   minttemp[HOTENDS],
                   maxttemp[HOTENDS];

      static int16_t bed_minttemp_raw;

      static int16_t bed_maxttemp_raw;

  public:
    /**
     * Instance Methods
     */

    Temperature();

    void init();

    /**
     * Static (class) methods
     */
    static float analog2temp(int raw, uint8_t e);
    static float analog2tempBed(int raw);

    /**
     * Called from the Temperature ISR
     */
    static void isr();

    /**
     * Call periodically to manage heaters
     */
    static void manage_heater() _O2; // Added _O2 to work around a compiler error

    /**
     * Preheating hotends
     */
      #define is_preheating(n) (false)

    //high level conversion routines, for use outside of temperature.cpp
    //inline so that there is no performance decrease.
    //deg=degreeCelsius

    static float degHotend(uint8_t e) {
        UNUSED(e);
      return current_temperature[HOTEND_INDEX];
    }
    static float degBed() { return current_temperature_bed; }

    static int16_t degTargetHotend(uint8_t e) {
        UNUSED(e);
      return target_temperature[HOTEND_INDEX];
    }

    static int16_t degTargetBed() { return target_temperature_bed; }

      static void start_watching_heater(uint8_t e = 0);

      static void start_watching_bed();

    static void setTargetHotend(const int16_t celsius, uint8_t e) {
        UNUSED(e);
      target_temperature[HOTEND_INDEX] = celsius;
        start_watching_heater(HOTEND_INDEX);
    }

    static void setTargetBed(const int16_t celsius) {
        target_temperature_bed =
            min(celsius, BED_MAXTEMP)
        ;
          start_watching_bed();
    }

    static bool isHeatingHotend(uint8_t e) {
        UNUSED(e);
      return target_temperature[HOTEND_INDEX] > current_temperature[HOTEND_INDEX];
    }
    static bool isHeatingBed() { return target_temperature_bed > current_temperature_bed; }

    static bool isCoolingHotend(uint8_t e) {
        UNUSED(e);
      return target_temperature[HOTEND_INDEX] < current_temperature[HOTEND_INDEX];
    }
    static bool isCoolingBed() { return target_temperature_bed < current_temperature_bed; }

    /**
     * The software PWM power for a heater
     */
    static int getHeaterPower(int heater);

    /**
     * Switch off all heaters, set all target temperatures to 0
     */
    static void disable_all_heaters();

    /**
     * Perform auto-tuning for hotend or bed in response to M303
     */
      static void PID_autotune(float temp, int hotend, int ncycles, bool set_result=false);

    /**
     * Update the temp manager when PID values change
     */
    static void updatePID();

  private:

    static void set_current_temp_raw();

    static void updateTemperaturesFromRawValues();

    static void checkExtruderAutoFans();

    static float get_pid_output(const int8_t e);

    static void _temp_error(const int8_t e, const char * const serial_msg, const char * const lcd_msg);
    static void min_temp_error(const int8_t e);
    static void max_temp_error(const int8_t e);

      typedef enum TRState { TRInactive, TRFirstHeating, TRStable, TRRunaway } TRstate;

      static void thermal_runaway_protection(TRState* state, millis_t* timer, float temperature, float target_temperature, int heater_id, int period_seconds, int hysteresis_degc);

        static TRState thermal_runaway_state_machine[HOTENDS];
        static millis_t thermal_runaway_timer[HOTENDS];

        static TRState thermal_runaway_bed_state_machine;
        static millis_t thermal_runaway_bed_timer;

};

extern Temperature thermalManager;

#endif // TEMPERATURE_H
