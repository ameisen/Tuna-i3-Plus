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
#include "temperature.hpp"
#include "bi3_plus_lcd.h"
#include "planner.h"
#include "language.h"
#include "configuration_store.h"
#include "watchdog.h"
#include "tunalib/initializer_list.h"

#define K2 (1.0-K1)

Temperature thermalManager;

// public:

float Temperature::current_temperature = 0.0f,
Temperature::current_temperature_bed = 0.0f;
volatile int16_t Temperature::current_temperature_raw = 0_u16;
int16_t Temperature::target_temperature = 0_u16;
volatile int16_t Temperature::current_temperature_bed_raw = 0;

int16_t Temperature::target_temperature_bed = 0;

float Temperature::Kp = DEFAULT_Kp,
Temperature::Ki = (DEFAULT_Ki) * (PID_dT),
Temperature::Kd = (DEFAULT_Kd) / (PID_dT);

uint16_t Temperature::watch_target_temp = 0_u16;
millis_t Temperature::watch_heater_next_ms = 0_u16;

uint16_t Temperature::watch_target_bed_temp = 0;
millis_t Temperature::watch_bed_next_ms = 0;

bool Temperature::allow_cold_extrude = false;
int16_t Temperature::extrude_min_temp = EXTRUDE_MINTEMP;

// private:

volatile bool Temperature::temp_meas_ready = false;

float Temperature::temp_iState = 0.0f,
Temperature::temp_dState = 0.0f,
Temperature::pTerm,
Temperature::iTerm,
Temperature::dTerm;

float Temperature::pid_error;
bool Temperature::pid_reset;

millis_t Temperature::next_bed_check_ms;

uint16_t Temperature::raw_temp_value = 0_u16,
Temperature::raw_temp_bed_value = 0;

// Init min and max temp with extreme values to prevent false errors during startup
int16_t Temperature::minttemp_raw = HEATER_0_RAW_LO_TEMP,
Temperature::maxttemp_raw = HEATER_0_RAW_HI_TEMP,
Temperature::minttemp = 0,
Temperature::maxttemp = 16383;

int16_t Temperature::bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP;
int16_t Temperature::bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;

uint8_t Temperature::soft_pwm_amount,
Temperature::soft_pwm_amount_bed;

namespace Thermal
{
	namespace
	{
		namespace adc
		{
			constexpr const uint8 delay_period = 10_u8;

			enum class sensor : uint8
			{
				Extruder = 0,
				Bed,
				MAX,
				FINAL = MAX - 1
			};

			sensor & __restrict operator ++ (sensor &enumerator)
			{
				uint8 value = (uint8 & __restrict)enumerator;
				value = (value + 1_u8) % uint8(sensor::MAX);
				(uint8 &)enumerator = value;
				return enumerator;
			}

			enum class state : uint8
			{
				Charging = 0,
				Delay,
				Delay_End = Delay + (delay_period - 1_u8),
				MAX,
				FINAL = MAX - 1
			};

			state & __restrict operator ++ (state &enumerator)
			{
				uint8 value = (uint8 & __restrict)enumerator;
				value = (value + 1_u8) % uint8(state::MAX);
				(uint8 &)enumerator = value;
				return enumerator;
			}

			sensor current_sensor = sensor::Extruder;
			state current_state = state::Delay;
			uint16 raw_temperatures[uint8(sensor::MAX)] {};
		}
	}
}

void Temperature::PID_autotune(float temp, int ncycles, bool set_result/*=false*/) {
	float input = 0.0;
	int cycles = 0;
	bool heating = true;

	millis_t temp_ms = millis(), t1 = temp_ms, t2 = temp_ms;
	long t_high = 0, t_low = 0;

	long bias, d;
	float Ku, Tu;
	float workKp = 0, workKi = 0, workKd = 0;
	float max = 0, min = 10000;

	SERIAL_ECHOLN(MSG_PID_AUTOTUNE_START);

	disable_all_heaters(); // switch off all heaters.

	soft_pwm_amount = bias = d = (PID_MAX) >> 1;

	wait_for_heatup = true;

	// PID Tuning loop
	while (wait_for_heatup) {

		millis_t ms = millis();

		if (temp_meas_ready) { // temp sample ready
			updateTemperaturesFromRawValues();

			input = current_temperature;

			NOLESS(max, input);
			NOMORE(min, input);

			if (heating && input > temp) {
				if (ELAPSED(ms, t2 + 5000UL)) {
					heating = false;
					soft_pwm_amount = (bias - d) >> 1;
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
					soft_pwm_amount = (bias + d) >> 1;
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
          PID_PARAM(Kp) = workKp; \
          PID_PARAM(Ki) = scalePID_i(workKi); \
          PID_PARAM(Kd) = scalePID_d(workKd); \
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

template <Temperature::Manager manager_type>
int Temperature::getHeaterPower() {
	if constexpr (manager_type == Manager::Hotend)
	{
		return soft_pwm_amount;
	}
	else
	{
		return soft_pwm_amount_bed;
	}
}
template int Temperature::getHeaterPower<Temperature::Manager::Hotend>();
template int Temperature::getHeaterPower<Temperature::Manager::Bed>();

//
// Temperature Error Handlers
//
template <Temperature::Manager manager_type>
void Temperature::_temp_error(const char * const serial_msg, const char * const lcd_msg) {
	static bool killed = false;
	lcd::show_page(68);
	if (IsRunning()) {
		SERIAL_ERROR_START();
		serialprintPGM(serial_msg);
		SERIAL_ERRORPGM(MSG_STOPPED_HEATER);
		if constexpr (manager_type == Temperature::Manager::Hotend)
		{
			SERIAL_ERRORLN(0);
		}
		else
		{
			SERIAL_ERRORLNPGM(MSG_HEATER_BED);
		}
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

template <Temperature::Manager manager_type>
void Temperature::max_temp_error() {
	_temp_error<manager_type>(PSTR(MSG_T_MAXTEMP), (manager_type == Manager::Hotend) ? PSTR(MSG_ERR_MAXTEMP) : PSTR(MSG_ERR_MAXTEMP_BED));
}

template <Temperature::Manager manager_type>
void Temperature::min_temp_error() {
	_temp_error<manager_type>(PSTR(MSG_T_MINTEMP), (manager_type == Manager::Hotend) ? PSTR(MSG_ERR_MINTEMP) : PSTR(MSG_ERR_MINTEMP_BED));
}

float Temperature::get_pid_output() {
#define _HOTEND_TEST     true
	float pid_output;
	pid_error = target_temperature - current_temperature;
	dTerm = K2 * PID_PARAM(Kd) * (current_temperature - temp_dState) + K1 * dTerm;
	temp_dState = current_temperature;
	if (pid_error > PID_FUNCTIONAL_RANGE) {
		pid_output = BANG_MAX;
		pid_reset = true;
	}
	else if (pid_error < -(PID_FUNCTIONAL_RANGE) || target_temperature == 0
		) {
		pid_output = 0;
		pid_reset = true;
	}
	else {
		if (pid_reset) {
			temp_iState = 0.0f;
			pid_reset = false;
		}
		pTerm = PID_PARAM(Kp) * pid_error;
		temp_iState += pid_error;
		iTerm = PID_PARAM(Ki) * temp_iState;

		pid_output = pTerm + iTerm - dTerm;

		if (pid_output > PID_MAX) {
			if (pid_error > 0) temp_iState -= pid_error; // conditional un-integration
			pid_output = PID_MAX;
		}
		else if (pid_output < 0) {
			if (pid_error < 0) temp_iState -= pid_error; // conditional un-integration
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
void Temperature::manage_heater() {

	if (!temp_meas_ready) return;

	updateTemperaturesFromRawValues(); // also resets the watchdog

	millis_t ms = millis();

	// Check for thermal runaway
	thermal_runaway_protection<Manager::Hotend>(&thermal_runaway_state_machine, &thermal_runaway_timer, current_temperature, target_temperature, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);

	soft_pwm_amount = (current_temperature > minttemp || is_preheating()) && current_temperature < maxttemp ? (int)get_pid_output() >> 1 : 0;

	// Make sure temperature is increasing
	if (watch_heater_next_ms && ELAPSED(ms, watch_heater_next_ms)) { // Time to check this extruder?
		if (degHotend() < watch_target_temp)                             // Failed to increase enough?
			_temp_error<Manager::Hotend>(PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
		else                                                                 // Start again if the target is still far off
			start_watching_heater();
	}

	// Make sure temperature is increasing
	if (watch_bed_next_ms && ELAPSED(ms, watch_bed_next_ms)) {        // Time to check the bed?
		if (degBed() < watch_target_bed_temp)                           // Failed to increase enough?
			_temp_error<Manager::Bed>(PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
		else                                                            // Start again if the target is still far off
			start_watching_bed();
	}

	if (PENDING(ms, next_bed_check_ms)) return;
	next_bed_check_ms = ms + BED_CHECK_INTERVAL;

	thermal_runaway_protection<Manager::Bed>(&thermal_runaway_bed_state_machine, &thermal_runaway_bed_timer, current_temperature_bed, target_temperature_bed, THERMAL_PROTECTION_BED_PERIOD, THERMAL_PROTECTION_BED_HYSTERESIS);

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
float Temperature::analog2temp(const int raw)
{
	for (uint8 i = 1; i < HEATER_0_TEMPTABLE_LEN; ++i) {
		if (PGM_RD_W(HEATER_0_TEMPTABLE[i][0]) > raw) {
			return PGM_RD_W(HEATER_0_TEMPTABLE[i - 1][1]) +
				(raw - PGM_RD_W(HEATER_0_TEMPTABLE[i - 1][0])) *
				(float)(PGM_RD_W(HEATER_0_TEMPTABLE[i][1]) - PGM_RD_W(HEATER_0_TEMPTABLE[i - 1][1])) /
				(float)(PGM_RD_W(HEATER_0_TEMPTABLE[i][0]) - PGM_RD_W(HEATER_0_TEMPTABLE[i - 1][0]));
		}
	}

	// Overflow: Set to last value in the table
	return PGM_RD_W(HEATER_0_TEMPTABLE[HEATER_0_TEMPTABLE_LEN - 1][1]);
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
float Temperature::analog2tempBed(const int raw)
{
	for (uint8 i = 1; i < BEDTEMPTABLE_LEN; i++) {
		if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw) {
			return PGM_RD_W(BEDTEMPTABLE[i - 1][1]) +
				(raw - PGM_RD_W(BEDTEMPTABLE[i - 1][0])) *
				(float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i - 1][1])) /
				(float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i - 1][0]));
		}
	}

	// Overflow: Set to last value in the table
	return PGM_RD_W(BEDTEMPTABLE[BEDTEMPTABLE_LEN - 1][1]);
}

/**
 * Get the raw values into the actual temperatures.
 * The raw values are created in interrupt context,
 * and this function is called from normal context
 * as it would block the stepper routine.
 */
void Temperature::updateTemperaturesFromRawValues() {

	int16 temperature_raw;
	int16 temperature_bed_raw;
	if (temp_meas_ready)
	{
		marlin::utils::critical_section _critsec;
		temperature_raw = current_temperature_raw;
		temperature_bed_raw = current_temperature_bed_raw;
		temp_meas_ready = false;

		if constexpr (HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP)
		{
			if (temperature_raw >= maxttemp_raw && target_temperature > 0) { max_temp_error<Manager::Hotend>(); }
			if (minttemp_raw >= temperature_raw && !is_preheating() && target_temperature > 0) { min_temp_error<Manager::Hotend>(); }
		}
		else
		{
			if (temperature_raw <= maxttemp_raw && target_temperature > 0) { max_temp_error<Manager::Hotend>(); }
			if (minttemp_raw <= temperature_raw && !is_preheating() && target_temperature > 0) { min_temp_error<Manager::Hotend>(); }
		}

		if constexpr (HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP)
		{
			if (temperature_bed_raw >= bed_maxttemp_raw && target_temperature_bed > 0) { max_temp_error<Manager::Bed>(); }
			if (bed_minttemp_raw >= temperature_bed_raw && target_temperature_bed > 0) { min_temp_error<Manager::Bed>(); }
		}
		else
		{
			if (temperature_bed_raw <= bed_maxttemp_raw && target_temperature_bed > 0) { max_temp_error<Manager::Bed>(); }
			if (bed_minttemp_raw <= temperature_bed_raw && target_temperature_bed > 0) { min_temp_error<Manager::Bed>(); }
		}

		current_temperature = Temperature::analog2temp(temperature_raw);
		current_temperature_bed = Temperature::analog2tempBed(temperature_bed_raw);
	}

	// Reset the watchdog after we know we have a temperature measurement.
	watchdog_reset();
}

/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_heater()
 */
void Temperature::init()
{
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

	minttemp = HEATER_0_MINTEMP;
	while (analog2temp(minttemp_raw) < HEATER_0_MINTEMP)
	{
		if constexpr (HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP)
			minttemp_raw += OVERSAMPLENR;
		else
			minttemp_raw -= OVERSAMPLENR;
	}

	maxttemp = HEATER_0_MAXTEMP;
	while (analog2temp(maxttemp_raw) > HEATER_0_MAXTEMP)
	{
		if constexpr (HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP)
			maxttemp_raw += OVERSAMPLENR;
		else
			maxttemp_raw -= OVERSAMPLENR;
	}

	while (analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
		if constexpr (HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP)
		{
			bed_minttemp_raw += OVERSAMPLENR;
		}
		else
		{
			bed_minttemp_raw -= OVERSAMPLENR;
		}
	}
	while (analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
		if constexpr (HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP)
		{
			bed_maxttemp_raw -= OVERSAMPLENR;
		}
		else
		{
			bed_maxttemp_raw += OVERSAMPLENR;
		}
	}
}

/**
 * Start Heating Sanity Check for hotends that are below
 * their target temperature by a configurable margin.
 * This is called when the temperature is set. (M104, M109)
 */
void Temperature::start_watching_heater() {
	if (degHotend() < degTargetHotend() - (WATCH_TEMP_INCREASE + TEMP_HYSTERESIS + 1)) {
		watch_target_temp = degHotend() + WATCH_TEMP_INCREASE;
		watch_heater_next_ms = millis() + (WATCH_TEMP_PERIOD) * 1000UL;
	}
	else
		watch_heater_next_ms = 0;
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

Temperature::TRState Temperature::thermal_runaway_state_machine = TRInactive;
millis_t Temperature::thermal_runaway_timer = 0;

Temperature::TRState Temperature::thermal_runaway_bed_state_machine = TRInactive;
millis_t Temperature::thermal_runaway_bed_timer;

template <Temperature::Manager manager_type> static float tr_target_temperature = 0.0f;

template <Temperature::Manager manager_type>
void Temperature::thermal_runaway_protection(Temperature::TRState* state, millis_t* timer, float current, float target, int period_seconds, int hysteresis_degc) {
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

	// If the target temperature changes, restart
	if (tr_target_temperature<manager_type> != target) {
		tr_target_temperature<manager_type> = target;
		*state = target > 0 ? TRFirstHeating : TRInactive;
	}

	switch (*state) {
		// Inactive state waits for a target temperature to be set
	case TRInactive: break;
		// When first heating, wait for the temperature to be reached then go to Stable state
	case TRFirstHeating:
		if (current < tr_target_temperature<manager_type>) break;
		*state = TRStable;
		// While the temperature is stable watch for a bad temperature
	case TRStable:
		if (current >= tr_target_temperature<manager_type> -hysteresis_degc) {
			*timer = millis() + period_seconds * 1000UL;
			break;
		}
		else if (PENDING(millis(), *timer)) break;
		*state = TRRunaway;
	case TRRunaway:
		_temp_error<manager_type>(PSTR(MSG_T_THERMAL_RUNAWAY), PSTR(MSG_THERMAL_RUNAWAY));
	}
}

template void Temperature::thermal_runaway_protection<Temperature::Manager::Hotend>(Temperature::TRState* state, millis_t* timer, float current, float target, int period_seconds, int hysteresis_degc);
template void Temperature::thermal_runaway_protection<Temperature::Manager::Bed>(Temperature::TRState* state, millis_t* timer, float current, float target, int period_seconds, int hysteresis_degc);

void Temperature::disable_all_heaters() {

	planner.autotemp_enabled = false;

	setTargetHotend(0);
	setTargetBed(0);

	// If all heaters go down then for sure our print job has stopped
	print_job_timer.stop();

	setTargetHotend(0);
	soft_pwm_amount = 0;
	WRITE_HEATER_0(LOW);

	target_temperature_bed = 0;
	soft_pwm_amount_bed = 0;
	WRITE_HEATER_BED(LOW);
}

/**
 * Get raw temperatures
 */
void Temperature::set_current_temp_raw() {
	{
		marlin::utils::critical_section _critsec;
		current_temperature_raw = raw_temp_value;
		current_temperature_bed_raw = raw_temp_bed_value;
		temp_meas_ready = true;
	}
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

/**
* States for ADC reading in the ISR
*/
enum ADCSensorState {
	PrepareTemp_0 = 0,
	MeasureTemp_0,
	PrepareTemp_BED,
	MeasureTemp_BED,
	SensorsReady, // Temperatures ready. Delay the next round of readings to let ADC pins settle.
	StartupDelay  // Startup, delay initial temp reading a tiny bit so the hardware can settle
};

static_assert(MIN_ADC_ISR_LOOPS >= uint(SensorsReady), "bad");

void Temperature::isr() {
	// The stepper ISR can interrupt this ISR. When it does it re-enables this ISR
	// at the end of its run, potentially causing re-entry. This flag prevents it.
	if (in_temp_isr) return;
	in_temp_isr = true;

	// Allow UART and stepper ISRs
	CBI(TIMSK0, OCIE0B); //Disable Temperature ISR
	sei();

	static int8_t oversample_count = 0;
	static ADCSensorState adc_sensor_state = StartupDelay;
	static uint8_t pwm_count = _BV(SOFT_PWM_SCALE);
	// avoid multiple loads of pwm_count
	uint8_t pwm_count_tmp = pwm_count;

	// Static members for each heater
#define ISR_STATICS(n) static uint8_t soft_pwm_count_ ## n = 0

// Statics per heater
	ISR_STATICS(0);
	ISR_STATICS(BED);

	constexpr uint8_t pwm_mask = 0;

	/**
	 * Standard PWM modulation
	 */
	if (pwm_count_tmp >= 127) {
		pwm_count_tmp -= 127;
		soft_pwm_count_0 = (soft_pwm_count_0 & pwm_mask) + soft_pwm_amount;
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

	/**
	 * One sensor is sampled on every other call of the ISR.
	 * Each sensor is read 16 (OVERSAMPLENR) times, taking the average.
	 *
	 * On each Prepare pass, ADC is started for a sensor pin.
	 * On the next pass, the ADC value is read and accumulated.
	 *
	 * This gives each ADC 0.9765ms to charge up.
	 */

	const auto set_admux_adcsra = [](uint8 pin)
	{
		ADMUX = _BV(REFS0) | (pin & 0x07);
		SBI(ADCSRA, ADSC);
	};

	const auto start_adc = [&set_admux_adcsra](uint8 pin)
	{
		if (pin > 7)
			ADCSRB = _BV(MUX5);
		else
			ADCSRB = 0_u8;
		set_admux_adcsra(pin);
	};

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
		start_adc(TEMP_0_PIN);
		break;
	case MeasureTemp_0:
		raw_temp_value += ADC;
		break;

	case PrepareTemp_BED:
		start_adc(TEMP_BED_PIN);
		break;
	case MeasureTemp_BED:
		raw_temp_bed_value += ADC;
		break;

	case StartupDelay: break;

	} // switch(adc_sensor_state)

	if (!adc_sensor_state && ++oversample_count >= OVERSAMPLENR) { // 10 * 16 * 1/(16000000/64/256)  = 164ms.

		oversample_count = 0;

		// Update the raw values.
		set_current_temp_raw();

		raw_temp_value = 0;
		raw_temp_bed_value = 0;

	} // temp_count >= OVERSAMPLENR

	// Go to the next state, up to SensorsReady
	adc_sensor_state = (ADCSensorState)((int(adc_sensor_state) + 1) % int(StartupDelay));

	cli();
	in_temp_isr = false;
	SBI(TIMSK0, OCIE0B); //re-enable Temperature ISR
}
