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
 * temperature.cpp - temperature control
 */

#include "Marlin.h"
#include "temperature.h"
#include "thermistortables.h"
#include "bi3_plus_lcd.h"
#include "planner.h"
#include "language.h"
#include "configuration_store.h"
#include "watchdog.h"

#define K2 (1.0-K1)

static void* heater_ttbl_map[HOTENDS] = ARRAY_BY_HOTENDS((void*)HEATER_0_TEMPTABLE, (void*)HEATER_1_TEMPTABLE, (void*)HEATER_2_TEMPTABLE, (void*)HEATER_3_TEMPTABLE, (void*)HEATER_4_TEMPTABLE);
static uint8_t heater_ttbllen_map[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN, HEATER_3_TEMPTABLE_LEN, HEATER_4_TEMPTABLE_LEN);

Temperature thermalManager;

// public:

float Temperature::current_temperature[HOTENDS] = { 0.0 },
      Temperature::current_temperature_bed = 0.0;
int16_t Temperature::current_temperature_raw[HOTENDS] = { 0 },
        Temperature::target_temperature[HOTENDS] = { 0 },
        Temperature::current_temperature_bed_raw = 0;

int16_t Temperature::target_temperature_bed = 0;

    float Temperature::Kp = DEFAULT_Kp,
          Temperature::Ki = (DEFAULT_Ki) * (PID_dT),
          Temperature::Kd = (DEFAULT_Kd) / (PID_dT);

  uint16_t Temperature::watch_target_temp[HOTENDS] = { 0 };
  millis_t Temperature::watch_heater_next_ms[HOTENDS] = { 0 };

  uint16_t Temperature::watch_target_bed_temp = 0;
  millis_t Temperature::watch_bed_next_ms = 0;

  bool Temperature::allow_cold_extrude = false;
  int16_t Temperature::extrude_min_temp = EXTRUDE_MINTEMP;

// private:

volatile bool Temperature::temp_meas_ready = false;

  float Temperature::temp_iState[HOTENDS] = { 0 },
        Temperature::temp_dState[HOTENDS] = { 0 },
        Temperature::pTerm[HOTENDS],
        Temperature::iTerm[HOTENDS],
        Temperature::dTerm[HOTENDS];

  float Temperature::pid_error[HOTENDS];
  bool Temperature::pid_reset[HOTENDS];

  millis_t Temperature::next_bed_check_ms;

uint16_t Temperature::raw_temp_value[MAX_EXTRUDERS] = { 0 },
         Temperature::raw_temp_bed_value = 0;

// Init min and max temp with extreme values to prevent false errors during startup
int16_t Temperature::minttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP, HEATER_3_RAW_LO_TEMP, HEATER_4_RAW_LO_TEMP),
        Temperature::maxttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP, HEATER_3_RAW_HI_TEMP, HEATER_4_RAW_HI_TEMP),
        Temperature::minttemp[HOTENDS] = { 0 },
        Temperature::maxttemp[HOTENDS] = ARRAY_BY_HOTENDS1(16383);

  int16_t Temperature::bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP;
  int16_t Temperature::bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;

uint8_t Temperature::soft_pwm_amount[HOTENDS],
        Temperature::soft_pwm_amount_bed;

  void Temperature::PID_autotune(float temp, int hotend, int ncycles, bool set_result/*=false*/) {
    float input = 0.0;
    int cycles = 0;
    bool heating = true;

    millis_t temp_ms = millis(), t1 = temp_ms, t2 = temp_ms;
    long t_high = 0, t_low = 0;

    long bias, d;
    float Ku, Tu;
    float workKp = 0, workKi = 0, workKd = 0;
    float max = 0, min = 10000;

    if (hotend >=
          HOTENDS
      || hotend <
          0
    ) {
      SERIAL_ECHOLN(MSG_PID_BAD_EXTRUDER_NUM);
      return;
    }

    SERIAL_ECHOLN(MSG_PID_AUTOTUNE_START);

    disable_all_heaters(); // switch off all heaters.

    soft_pwm_amount[hotend] = bias = d = (PID_MAX) >> 1;

    wait_for_heatup = true;

    // PID Tuning loop
    while (wait_for_heatup) {

      millis_t ms = millis();

      if (temp_meas_ready) { // temp sample ready
        updateTemperaturesFromRawValues();

        input =
            current_temperature[hotend]
        ;

        NOLESS(max, input);
        NOMORE(min, input);

        if (heating && input > temp) {
          if (ELAPSED(ms, t2 + 5000UL)) {
            heating = false;
            soft_pwm_amount[hotend] = (bias - d) >> 1;
            t1 = ms;
            t_high = t1 - t2;
            max = temp;
          }
        }

        if (!heating && input < temp) {
          if (ELAPSED(ms, t1 + 5000UL)) {
            heating = true;
            t2 = ms;
            t_low = t2 - t1;
            if (cycles > 0) {
              long max_pow = PID_MAX;
              bias += (d * (t_high - t_low)) / (t_low + t_high);
              bias = constrain(bias, 20, max_pow - 20);
              d = (bias > max_pow / 2) ? max_pow - 1 - bias : bias;

              SERIAL_PROTOCOLPAIR(MSG_BIAS, bias);
              SERIAL_PROTOCOLPAIR(MSG_D, d);
              SERIAL_PROTOCOLPAIR(MSG_T_MIN, min);
              SERIAL_PROTOCOLPAIR(MSG_T_MAX, max);
              if (cycles > 2) {
                Ku = (4.0 * d) / (M_PI * (max - min) * 0.5);
                Tu = ((float)(t_low + t_high) * 0.001);
                SERIAL_PROTOCOLPAIR(MSG_KU, Ku);
                SERIAL_PROTOCOLPAIR(MSG_TU, Tu);
                workKp = 0.6 * Ku;
                workKi = 2 * workKp / Tu;
                workKd = workKp * Tu * 0.125;
                SERIAL_PROTOCOLLNPGM("\n" MSG_CLASSIC_PID);
                SERIAL_PROTOCOLPAIR(MSG_KP, workKp);
                SERIAL_PROTOCOLPAIR(MSG_KI, workKi);
                SERIAL_PROTOCOLLNPAIR(MSG_KD, workKd);
                /**
                workKp = 0.33*Ku;
                workKi = workKp/Tu;
                workKd = workKp*Tu/3;
                SERIAL_PROTOCOLLNPGM(" Some overshoot");
                SERIAL_PROTOCOLPAIR(" Kp: ", workKp);
                SERIAL_PROTOCOLPAIR(" Ki: ", workKi);
                SERIAL_PROTOCOLPAIR(" Kd: ", workKd);
                workKp = 0.2*Ku;
                workKi = 2*workKp/Tu;
                workKd = workKp*Tu/3;
                SERIAL_PROTOCOLLNPGM(" No overshoot");
                SERIAL_PROTOCOLPAIR(" Kp: ", workKp);
                SERIAL_PROTOCOLPAIR(" Ki: ", workKi);
                SERIAL_PROTOCOLPAIR(" Kd: ", workKd);
                */
              }
            }
            soft_pwm_amount[hotend] = (bias + d) >> 1;
            cycles++;
            min = temp;
          }
        }
      }
      #define MAX_OVERSHOOT_PID_AUTOTUNE 20
      if (input > temp + MAX_OVERSHOOT_PID_AUTOTUNE) {
        SERIAL_PROTOCOLLNPGM(MSG_PID_TEMP_TOO_HIGH);
        return;
      }
      // Every 2 seconds...
      if (ELAPSED(ms, temp_ms + 2000UL)) {
          print_heaterstates();
          lcd::update_graph();
          SERIAL_EOL();

        temp_ms = ms;
      } // every 2 seconds
      // Over 2 minutes?
      if (((ms - t1) + (ms - t2)) > (10L * 60L * 1000L * 2L)) {
        SERIAL_PROTOCOLLNPGM(MSG_PID_TIMEOUT);
        return;
      }
      if (cycles > ncycles) {
        SERIAL_PROTOCOLLNPGM(MSG_PID_AUTOTUNE_FINISHED);

        SERIAL_PROTOCOLPAIR("#define  DEFAULT_Kp ", workKp); SERIAL_EOL();
        SERIAL_PROTOCOLPAIR("#define  DEFAULT_Ki ", workKi); SERIAL_EOL();
        SERIAL_PROTOCOLPAIR("#define  DEFAULT_Kd ", workKd); SERIAL_EOL();

        #define _SET_EXTRUDER_PID() do { \
          PID_PARAM(Kp, hotend) = workKp; \
          PID_PARAM(Ki, hotend) = scalePID_i(workKi); \
          PID_PARAM(Kd, hotend) = scalePID_d(workKd); \
          updatePID(); }while(0)

        // Use the result? (As with "M303 U1")
        if (set_result) {
            _SET_EXTRUDER_PID();
        }
        lcd::show_page(66);
        enqueue_and_echo_command("M106 S0");
        settings.save();
        return;
      }
	  lcd::update();
    }
    if (!wait_for_heatup) disable_all_heaters();
  }

/**
 * Class and Instance Methods
 */

Temperature::Temperature() { }

void Temperature::updatePID() {}

int Temperature::getHeaterPower(int heater) {
  return heater < 0 ? soft_pwm_amount_bed : soft_pwm_amount[heater];
}

//
// Temperature Error Handlers
//
void Temperature::_temp_error(const int8_t e, const char * const serial_msg, const char * const lcd_msg) {
  static bool killed = false;
  lcd::show_page(68);
  if (IsRunning()) {
    SERIAL_ERROR_START();
    serialprintPGM(serial_msg);
    SERIAL_ERRORPGM(MSG_STOPPED_HEATER);
    if (e >= 0) SERIAL_ERRORLN((int)e); else SERIAL_ERRORLNPGM(MSG_HEATER_BED);
  }
  #if DISABLED(BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE)
    if (!killed) {
      Running = false;
      killed = true;
      kill(lcd_msg);
    }
    else
      disable_all_heaters(); // paranoia
  #endif
}

void Temperature::max_temp_error(const int8_t e) {
  _temp_error(e, PSTR(MSG_T_MAXTEMP), e >= 0 ? PSTR(MSG_ERR_MAXTEMP) : PSTR(MSG_ERR_MAXTEMP_BED));
}
void Temperature::min_temp_error(const int8_t e) {
  _temp_error(e, PSTR(MSG_T_MINTEMP), e >= 0 ? PSTR(MSG_ERR_MINTEMP) : PSTR(MSG_ERR_MINTEMP_BED));
}

float Temperature::get_pid_output(const int8_t e) {
    UNUSED(e);
    #define _HOTEND_TEST     true
  float pid_output;
    pid_error[HOTEND_INDEX] = target_temperature[HOTEND_INDEX] - current_temperature[HOTEND_INDEX];
    dTerm[HOTEND_INDEX] = K2 * PID_PARAM(Kd, HOTEND_INDEX) * (current_temperature[HOTEND_INDEX] - temp_dState[HOTEND_INDEX]) + K1 * dTerm[HOTEND_INDEX];
    temp_dState[HOTEND_INDEX] = current_temperature[HOTEND_INDEX];
    #if HEATER_IDLE_HANDLER
    if (heater_idle_timeout_exceeded[HOTEND_INDEX]) {
        pid_output = 0;
        pid_reset[HOTEND_INDEX] = true;
    }
    else
    #endif
    if (pid_error[HOTEND_INDEX] > PID_FUNCTIONAL_RANGE) {
    pid_output = BANG_MAX;
    pid_reset[HOTEND_INDEX] = true;
    }
    else if (pid_error[HOTEND_INDEX] < -(PID_FUNCTIONAL_RANGE) || target_temperature[HOTEND_INDEX] == 0
    #if HEATER_IDLE_HANDLER
        || heater_idle_timeout_exceeded[HOTEND_INDEX]
    #endif
    ) {
    pid_output = 0;
    pid_reset[HOTEND_INDEX] = true;
    }
    else {
    if (pid_reset[HOTEND_INDEX]) {
        temp_iState[HOTEND_INDEX] = 0.0;
        pid_reset[HOTEND_INDEX] = false;
    }
    pTerm[HOTEND_INDEX] = PID_PARAM(Kp, HOTEND_INDEX) * pid_error[HOTEND_INDEX];
    temp_iState[HOTEND_INDEX] += pid_error[HOTEND_INDEX];
    iTerm[HOTEND_INDEX] = PID_PARAM(Ki, HOTEND_INDEX) * temp_iState[HOTEND_INDEX];

    pid_output = pTerm[HOTEND_INDEX] + iTerm[HOTEND_INDEX] - dTerm[HOTEND_INDEX];

    #if ENABLED(PID_EXTRUSION_SCALING)
        cTerm[HOTEND_INDEX] = 0;
        if (_HOTEND_TEST) {
        long e_position = stepper.position(E_AXIS);
        if (e_position > last_e_position) {
            lpq[lpq_ptr] = e_position - last_e_position;
            last_e_position = e_position;
        }
        else {
            lpq[lpq_ptr] = 0;
        }
        if (++lpq_ptr >= lpq_len) lpq_ptr = 0;
        cTerm[HOTEND_INDEX] = (lpq[lpq_ptr] * planner.steps_to_mm[E_AXIS]) * PID_PARAM(Kc, HOTEND_INDEX);
        pid_output += cTerm[HOTEND_INDEX];
        }
    #endif // PID_EXTRUSION_SCALING

    if (pid_output > PID_MAX) {
        if (pid_error[HOTEND_INDEX] > 0) temp_iState[HOTEND_INDEX] -= pid_error[HOTEND_INDEX]; // conditional un-integration
        pid_output = PID_MAX;
    }
    else if (pid_output < 0) {
        if (pid_error[HOTEND_INDEX] < 0) temp_iState[HOTEND_INDEX] -= pid_error[HOTEND_INDEX]; // conditional un-integration
        pid_output = 0;
    }
    }

  return pid_output;
}

/**
 * Manage heating activities for extruder hot-ends and a heated bed
 *  - Acquire updated temperature readings
 *    - Also resets the watchdog timer
 *  - Invoke thermal runaway protection
 *  - Manage extruder auto-fan
 *  - Apply filament width to the extrusion rate (may move)
 *  - Update the heated bed PID output value
 */

/**
 * The following line SOMETIMES results in the dreaded "unable to find a register to spill in class 'POINTER_REGS'"
 * compile error.
 *    thermal_runaway_protection(&thermal_runaway_state_machine[e], &thermal_runaway_timer[e], current_temperature[e], target_temperature[e], e, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);
 *
 * This is due to a bug in the C++ compiler used by the Arduino IDE from 1.6.10 to at least 1.8.1.
 *
 * The work around is to add the compiler flag "__attribute__((__optimize__("O2")))" to the declaration for manage_heater()
 */
//void Temperature::manage_heater()  __attribute__((__optimize__("O2")));
void Temperature::manage_heater() {

  if (!temp_meas_ready) return;

  updateTemperaturesFromRawValues(); // also resets the watchdog

    millis_t ms = millis();

  HOTEND_LOOP() {

    // Check for thermal runaway
    thermal_runaway_protection(&thermal_runaway_state_machine[e], &thermal_runaway_timer[e], current_temperature[e], target_temperature[e], e, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);

    soft_pwm_amount[e] = (current_temperature[e] > minttemp[e] || is_preheating(e)) && current_temperature[e] < maxttemp[e] ? (int)get_pid_output(e) >> 1 : 0;

    // Make sure temperature is increasing
    if (watch_heater_next_ms[e] && ELAPSED(ms, watch_heater_next_ms[e])) { // Time to check this extruder?
    if (degHotend(e) < watch_target_temp[e])                             // Failed to increase enough?
        _temp_error(e, PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
    else                                                                 // Start again if the target is still far off
        start_watching_heater(e);
    }
  } // HOTEND_LOOP

    // Make sure temperature is increasing
    if (watch_bed_next_ms && ELAPSED(ms, watch_bed_next_ms)) {        // Time to check the bed?
      if (degBed() < watch_target_bed_temp)                           // Failed to increase enough?
        _temp_error(-1, PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
      else                                                            // Start again if the target is still far off
        start_watching_bed();
    }

    if (PENDING(ms, next_bed_check_ms)) return;
    next_bed_check_ms = ms + BED_CHECK_INTERVAL;

    thermal_runaway_protection(&thermal_runaway_bed_state_machine, &thermal_runaway_bed_timer, current_temperature_bed, target_temperature_bed, -1, THERMAL_PROTECTION_BED_PERIOD, THERMAL_PROTECTION_BED_HYSTERESIS);

    {
        // Check if temperature is within the correct range
        if (WITHIN(current_temperature_bed, BED_MINTEMP, BED_MAXTEMP)) {
          soft_pwm_amount_bed = current_temperature_bed < target_temperature_bed ? MAX_BED_POWER >> 1 : 0;
        }
        else {
          soft_pwm_amount_bed = 0;
          WRITE_HEATER_BED(LOW);
        }
    }
}

#define PGM_RD_W(x)   (short)pgm_read_word(&x)

// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
float Temperature::analog2temp(int raw, uint8_t e) {
    if (e >= HOTENDS)
    {
      SERIAL_ERROR_START();
      SERIAL_ERROR((int)e);
      SERIAL_ERRORLNPGM(MSG_INVALID_EXTRUDER_NUM);
      kill(PSTR(MSG_KILLED));
      return 0.0;
    }

  if (heater_ttbl_map[e] != NULL) {
    float celsius = 0;
    uint8_t i;
    short(*tt)[][2] = (short(*)[][2])(heater_ttbl_map[e]);

    for (i = 1; i < heater_ttbllen_map[e]; i++) {
      if (PGM_RD_W((*tt)[i][0]) > raw) {
        celsius = PGM_RD_W((*tt)[i - 1][1]) +
                  (raw - PGM_RD_W((*tt)[i - 1][0])) *
                  (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i - 1][1])) /
                  (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i - 1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) celsius = PGM_RD_W((*tt)[i - 1][1]);

    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * (TEMP_SENSOR_AD595_GAIN)) + TEMP_SENSOR_AD595_OFFSET;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
float Temperature::analog2tempBed(const int raw) {
    float celsius = 0;
    byte i;

    for (i = 1; i < BEDTEMPTABLE_LEN; i++) {
      if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw) {
        celsius  = PGM_RD_W(BEDTEMPTABLE[i - 1][1]) +
                   (raw - PGM_RD_W(BEDTEMPTABLE[i - 1][0])) *
                   (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i - 1][1])) /
                   (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i - 1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = PGM_RD_W(BEDTEMPTABLE[i - 1][1]);

    return celsius;
}

/**
 * Get the raw values into the actual temperatures.
 * The raw values are created in interrupt context,
 * and this function is called from normal context
 * as it would block the stepper routine.
 */
void Temperature::updateTemperaturesFromRawValues() {
  HOTEND_LOOP()
    current_temperature[e] = Temperature::analog2temp(current_temperature_raw[e], e);
  current_temperature_bed = Temperature::analog2tempBed(current_temperature_bed_raw);

    // Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();

  CRITICAL_SECTION_START;
  temp_meas_ready = false;
  CRITICAL_SECTION_END;
}

/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_heater()
 */
void Temperature::init()
{
  // Finish init of mult hotend arrays
  HOTEND_LOOP() maxttemp[e] = maxttemp[0];

  SET_OUTPUT(HEATER_0_PIN);
  SET_OUTPUT(HEATER_BED_PIN);

  SET_OUTPUT(FAN_PIN);

    #define ANALOG_SELECT(pin) do{ SBI(DIDR0, pin); }while(0)

  // Set analog inputs
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIF) | 0x07;
  DIDR0 = 0;
  ANALOG_SELECT(TEMP_0_PIN);
  ANALOG_SELECT(TEMP_BED_PIN);

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
  SBI(TIMSK0, OCIE0B);

  // Wait for temperature measurement to settle
  delay(250);

  #define TEMP_MIN_ROUTINE(NR) \
    minttemp[NR] = HEATER_ ##NR## _MINTEMP; \
    while (analog2temp(minttemp_raw[NR], NR) < HEATER_ ##NR## _MINTEMP) { \
      if (HEATER_ ##NR## _RAW_LO_TEMP < HEATER_ ##NR## _RAW_HI_TEMP) \
        minttemp_raw[NR] += OVERSAMPLENR; \
      else \
        minttemp_raw[NR] -= OVERSAMPLENR; \
    }
  #define TEMP_MAX_ROUTINE(NR) \
    maxttemp[NR] = HEATER_ ##NR## _MAXTEMP; \
    while (analog2temp(maxttemp_raw[NR], NR) > HEATER_ ##NR## _MAXTEMP) { \
      if (HEATER_ ##NR## _RAW_LO_TEMP < HEATER_ ##NR## _RAW_HI_TEMP) \
        maxttemp_raw[NR] -= OVERSAMPLENR; \
      else \
        maxttemp_raw[NR] += OVERSAMPLENR; \
    }

  TEMP_MIN_ROUTINE(0);
  TEMP_MAX_ROUTINE(0);

	while (analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
		#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
		bed_minttemp_raw += OVERSAMPLENR;
		#else
		bed_minttemp_raw -= OVERSAMPLENR;
		#endif
	}
    while (analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_maxttemp_raw -= OVERSAMPLENR;
      #else
        bed_maxttemp_raw += OVERSAMPLENR;
      #endif
    }
}

  /**
   * Start Heating Sanity Check for hotends that are below
   * their target temperature by a configurable margin.
   * This is called when the temperature is set. (M104, M109)
   */
  void Temperature::start_watching_heater(uint8_t e) {
    UNUSED(e);
    if (degHotend(HOTEND_INDEX) < degTargetHotend(HOTEND_INDEX) - (WATCH_TEMP_INCREASE + TEMP_HYSTERESIS + 1)) {
      watch_target_temp[HOTEND_INDEX] = degHotend(HOTEND_INDEX) + WATCH_TEMP_INCREASE;
      watch_heater_next_ms[HOTEND_INDEX] = millis() + (WATCH_TEMP_PERIOD) * 1000UL;
    }
    else
      watch_heater_next_ms[HOTEND_INDEX] = 0;
  }

  /**
   * Start Heating Sanity Check for hotends that are below
   * their target temperature by a configurable margin.
   * This is called when the temperature is set. (M140, M190)
   */
  void Temperature::start_watching_bed() {
    if (degBed() < degTargetBed() - (WATCH_BED_TEMP_INCREASE + TEMP_BED_HYSTERESIS + 1)) {
      watch_target_bed_temp = degBed() + WATCH_BED_TEMP_INCREASE;
      watch_bed_next_ms = millis() + (WATCH_BED_TEMP_PERIOD) * 1000UL;
    }
    else
      watch_bed_next_ms = 0;
  }

    Temperature::TRState Temperature::thermal_runaway_state_machine[HOTENDS] = { TRInactive };
    millis_t Temperature::thermal_runaway_timer[HOTENDS] = { 0 };

    Temperature::TRState Temperature::thermal_runaway_bed_state_machine = TRInactive;
    millis_t Temperature::thermal_runaway_bed_timer;

  void Temperature::thermal_runaway_protection(Temperature::TRState* state, millis_t* timer, float current, float target, int heater_id, int period_seconds, int hysteresis_degc) {

    static float tr_target_temperature[HOTENDS + 1] = { 0.0 };

    /**
        SERIAL_ECHO_START();
        SERIAL_ECHOPGM("Thermal Thermal Runaway Running. Heater ID: ");
        if (heater_id < 0) SERIAL_ECHOPGM("bed"); else SERIAL_ECHO(heater_id);
        SERIAL_ECHOPAIR(" ;  State:", *state);
        SERIAL_ECHOPAIR(" ;  Timer:", *timer);
        SERIAL_ECHOPAIR(" ;  Temperature:", current);
        SERIAL_ECHOPAIR(" ;  Target Temp:", target);
        if (heater_id >= 0)
          SERIAL_ECHOPAIR(" ;  Idle Timeout:", heater_idle_timeout_exceeded[heater_id]);
        else
          SERIAL_ECHOPAIR(" ;  Idle Timeout:", bed_idle_timeout_exceeded);
        SERIAL_EOL();
    */

    const int heater_index = heater_id >= 0 ? heater_id : HOTENDS;

    // If the target temperature changes, restart
    if (tr_target_temperature[heater_index] != target) {
      tr_target_temperature[heater_index] = target;
      *state = target > 0 ? TRFirstHeating : TRInactive;
    }

    switch (*state) {
      // Inactive state waits for a target temperature to be set
      case TRInactive: break;
      // When first heating, wait for the temperature to be reached then go to Stable state
      case TRFirstHeating:
        if (current < tr_target_temperature[heater_index]) break;
        *state = TRStable;
      // While the temperature is stable watch for a bad temperature
      case TRStable:
        if (current >= tr_target_temperature[heater_index] - hysteresis_degc) {
          *timer = millis() + period_seconds * 1000UL;
          break;
        }
        else if (PENDING(millis(), *timer)) break;
        *state = TRRunaway;
      case TRRunaway:
        _temp_error(heater_id, PSTR(MSG_T_THERMAL_RUNAWAY), PSTR(MSG_THERMAL_RUNAWAY));
    }
  }

void Temperature::disable_all_heaters() {

    planner.autotemp_enabled = false;

  HOTEND_LOOP() setTargetHotend(0, e);
  setTargetBed(0);

  // If all heaters go down then for sure our print job has stopped
  print_job_timer.stop();

  #define DISABLE_HEATER(NR) { \
    setTargetHotend(0, NR); \
    soft_pwm_amount[NR] = 0; \
    WRITE_HEATER_ ##NR (LOW); \
  }

    DISABLE_HEATER(0);

    target_temperature_bed = 0;
    soft_pwm_amount_bed = 0;
      WRITE_HEATER_BED(LOW);
}

/**
 * Get raw temperatures
 */
void Temperature::set_current_temp_raw() {
    current_temperature_raw[0] = raw_temp_value[0];
  current_temperature_bed_raw = raw_temp_bed_value;
  temp_meas_ready = true;
}

/**
 * Timer 0 is shared with millies so don't change the prescaler.
 *
 * This ISR uses the compare method so it runs at the base
 * frequency (16 MHz / 64 / 256 = 976.5625 Hz), but at the TCNT0 set
 * in OCR0B above (128 or halfway between OVFs).
 *
 *  - Manage PWM to all the heaters and fan
 *  - Prepare or Measure one of the raw ADC sensor values
 *  - Check new temperature values for MIN/MAX errors (kill on error)
 *  - Step the babysteps value for each axis towards 0
 *  - For PINS_DEBUGGING, monitor and report endstop pins
 *  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
 */
ISR(TIMER0_COMPB_vect) { Temperature::isr(); }

volatile bool Temperature::in_temp_isr = false;

void Temperature::isr() {
  // The stepper ISR can interrupt this ISR. When it does it re-enables this ISR
  // at the end of its run, potentially causing re-entry. This flag prevents it.
  if (in_temp_isr) return;
  in_temp_isr = true;

  // Allow UART and stepper ISRs
  CBI(TIMSK0, OCIE0B); //Disable Temperature ISR
  sei();

  static int8_t temp_count = -1;
  static ADCSensorState adc_sensor_state = StartupDelay;
  static uint8_t pwm_count = _BV(SOFT_PWM_SCALE);
  // avoid multiple loads of pwm_count
  uint8_t pwm_count_tmp = pwm_count;

  // Static members for each heater
  #define ISR_STATICS(n) static uint8_t soft_pwm_count_ ## n = 0

  // Statics per heater
  ISR_STATICS(0);
  ISR_STATICS(BED);

    constexpr uint8_t pwm_mask =
        0
    ;

    /**
     * Standard PWM modulation
     */
    if (pwm_count_tmp >= 127) {
      pwm_count_tmp -= 127;
      soft_pwm_count_0 = (soft_pwm_count_0 & pwm_mask) + soft_pwm_amount[0];
      WRITE_HEATER_0(soft_pwm_count_0 > pwm_mask ? HIGH : LOW);

		soft_pwm_count_BED = (soft_pwm_count_BED & pwm_mask) + soft_pwm_amount_bed;
		WRITE_HEATER_BED(soft_pwm_count_BED > pwm_mask ? HIGH : LOW);
    }
    else {
      if (soft_pwm_count_0 <= pwm_count_tmp) WRITE_HEATER_0(0);

        if (soft_pwm_count_BED <= pwm_count_tmp) WRITE_HEATER_BED(0);
    }

    // SOFT_PWM_SCALE to frequency:
    //
    // 0: 16000000/64/256/128 =   7.6294 Hz
    // 1:                / 64 =  15.2588 Hz
    // 2:                / 32 =  30.5176 Hz
    // 3:                / 16 =  61.0352 Hz
    // 4:                /  8 = 122.0703 Hz
    // 5:                /  4 = 244.1406 Hz
    pwm_count = pwm_count_tmp + _BV(SOFT_PWM_SCALE);

  //
  // Update lcd buttons 488 times per second
  //
  static bool do_buttons;
  if ((do_buttons ^= true)) lcd::update_buttons();

  /**
   * One sensor is sampled on every other call of the ISR.
   * Each sensor is read 16 (OVERSAMPLENR) times, taking the average.
   *
   * On each Prepare pass, ADC is started for a sensor pin.
   * On the next pass, the ADC value is read and accumulated.
   *
   * This gives each ADC 0.9765ms to charge up.
   */

  #define SET_ADMUX_ADCSRA(pin) ADMUX = _BV(REFS0) | (pin & 0x07); SBI(ADCSRA, ADSC)
  #define START_ADC(pin) if (pin > 7) ADCSRB = _BV(MUX5); else ADCSRB = 0; SET_ADMUX_ADCSRA(pin)

  switch (adc_sensor_state) {

    case SensorsReady: {
      // All sensors have been read. Stay in this state for a few
      // ISRs to save on calls to temp update/checking code below.
      constexpr int8_t extra_loops = MIN_ADC_ISR_LOOPS - (int8_t)SensorsReady;
      static uint8_t delay_count = 0;
      if (extra_loops > 0) {
        if (delay_count == 0) delay_count = extra_loops;   // Init this delay
        if (--delay_count)                                 // While delaying...
          adc_sensor_state = (ADCSensorState)(int(SensorsReady) - 1); // retain this state (else, next state will be 0)
        break;
      }
      else
        adc_sensor_state = (ADCSensorState)0; // Fall-through to start first sensor now
    }

    case PrepareTemp_0:
    START_ADC(TEMP_0_PIN);
    break;
    case MeasureTemp_0:
    raw_temp_value[0] += ADC;
    break;

    case PrepareTemp_BED:
    START_ADC(TEMP_BED_PIN);
    break;
    case MeasureTemp_BED:
    raw_temp_bed_value += ADC;
    break;

    case StartupDelay: break;

  } // switch(adc_sensor_state)

  if (!adc_sensor_state && ++temp_count >= OVERSAMPLENR) { // 10 * 16 * 1/(16000000/64/256)  = 164ms.

    temp_count = 0;

    // Update the raw values if they've been read. Else we could be updating them during reading.
    if (!temp_meas_ready) set_current_temp_raw();

    ZERO(raw_temp_value);
    raw_temp_bed_value = 0;

    #define TEMPDIR(N) ((HEATER_##N##_RAW_LO_TEMP) > (HEATER_##N##_RAW_HI_TEMP) ? -1 : 1)

    int constexpr temp_dir[] = {
        TEMPDIR(0)
    };

    for (uint8_t e = 0; e < COUNT(temp_dir); e++) {
      const int16_t tdir = temp_dir[e], rawtemp = current_temperature_raw[e] * tdir;
      if (rawtemp > maxttemp_raw[e] * tdir && target_temperature[e] > 0) max_temp_error(e);
      if (rawtemp < minttemp_raw[e] * tdir && !is_preheating(e) && target_temperature[e] > 0) {
            min_temp_error(e);
      }
    }

        #define GEBED <=
      if (current_temperature_bed_raw GEBED bed_maxttemp_raw && target_temperature_bed > 0) max_temp_error(-1);
      if (bed_minttemp_raw GEBED current_temperature_bed_raw && target_temperature_bed > 0) min_temp_error(-1);

  } // temp_count >= OVERSAMPLENR

  // Go to the next state, up to SensorsReady
  adc_sensor_state = (ADCSensorState)((int(adc_sensor_state) + 1) % int(StartupDelay));

  cli();
  in_temp_isr = false;
  SBI(TIMSK0, OCIE0B); //re-enable Temperature ISR
}
