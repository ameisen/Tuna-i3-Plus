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
  * temperature.hpp - temperature controller
  */

#pragma once

#include "thermistors/thermistortables.h"

#include "MarlinConfig.h"

#include "tuna.h"

// Minimum number of Temperature::ISR loops between sensor readings.
// Multiplied by 16 (OVERSAMPLENR) to obtain the total time to
// get all oversampled sensor readings
#define MIN_ADC_ISR_LOOPS 10

constexpr int ACTUAL_ADC_SAMPLES = int(MIN_ADC_ISR_LOOPS);

class Temperature
{
public:

	enum class Manager : uint8
	{
		Hotend = 0,
		Bed = 1
	};

	static uint16 current_temperature,
		current_temperature_bed;
	static volatile uint16_t current_temperature_raw;
	static uint16_t target_temperature;
	static volatile uint16_t current_temperature_bed_raw;
	static uint16_t target_temperature_bed;

	static volatile bool in_temp_isr;

	static uint8_t soft_pwm_amount,
		soft_pwm_amount_bed;

#define PID_dT ((OVERSAMPLENR * float(ACTUAL_ADC_SAMPLES)) / (F_CPU / 64.0f / 256.0f))

	static float Kp, Ki, Kd;
#define PID_PARAM(param) Temperature::param

	// Apply the scale factors to the PID values
#define scalePID_i(i)   ( (i) * PID_dT )
#define unscalePID_i(i) ( (i) / PID_dT )
#define scalePID_d(d)   ( (d) / PID_dT )
#define unscalePID_d(d) ( (d) * PID_dT )

	static uint16_t watch_target_temp;
	static millis_t watch_heater_next_ms;

	static uint16_t watch_target_bed_temp;
	static millis_t watch_bed_next_ms;

	static bool allow_cold_extrude;
	static int16_t extrude_min_temp;
	static bool tooColdToExtrude() {
		return allow_cold_extrude ? false : degHotend() < extrude_min_temp;
	}

private:

	static volatile bool temp_meas_ready;
	static_assert(sizeof(Temperature::temp_meas_ready) == 1, "atomic boolean must be one byte");

	static float temp_iState,
		temp_dState,
		pTerm,
		iTerm,
		dTerm;

	static float pid_error;
	static bool pid_reset;

	static millis_t next_bed_check_ms;

	static uint16_t raw_temp_value,
		raw_temp_bed_value;

	// Init min and max temp with extreme values to prevent false errors during startup
	static int16_t minttemp_raw,
		maxttemp_raw,
		minttemp,
		maxttemp;

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
	static float analog2temp(uint16 raw);
	static float analog2tempBed(uint16 raw);

	/**
	 * Called from the Temperature ISR
	 */
	static void isr();

	/**
	 * Call periodically to manage heaters
	 */
	static void manage_heater();

	/**
	 * Preheating hotends
	 */
	static constexpr bool is_preheating() { return false; }

	 //high level conversion routines, for use outside of temperature.cpp
	 //inline so that there is no performance decrease.
	 //deg=degreeCelsius

	static float degHotend() { return current_temperature; }
	static float degBed() { return current_temperature_bed; }

	static int16_t degTargetHotend() { return target_temperature; }

	static int16_t degTargetBed() { return target_temperature_bed; }

	static void start_watching_heater();

	static void start_watching_bed();

	static void setTargetHotend(const int16_t celsius) {
		target_temperature = celsius;
		start_watching_heater();
	}

	static void setTargetBed(const int16_t celsius) {
		target_temperature_bed =
			min(celsius, BED_MAXTEMP)
			;
		start_watching_bed();
	}

	static bool isHeatingHotend() {
		return target_temperature > current_temperature;
	}
	static bool isHeatingBed() { return target_temperature_bed > current_temperature_bed; }

	static bool isCoolingHotend() {
		return target_temperature < current_temperature;
	}
	static bool isCoolingBed() { return target_temperature_bed < current_temperature_bed; }

	/**
	 * The software PWM power for a heater
	 */
	template <Manager manager_type>
	static int getHeaterPower();

	/**
	 * Switch off all heaters, set all target temperatures to 0
	 */
	static void disable_all_heaters();

	/**
	 * Perform auto-tuning for hotend or bed in response to M303
	 */
	static void PID_autotune(float temp, int ncycles, bool set_result = false);

	/**
	 * Update the temp manager when PID values change
	 */
	static void updatePID();

private:

	static void set_current_temp_raw();

	static void updateTemperaturesFromRawValues();

	static void checkExtruderAutoFans();

	static float get_pid_output();

	template <Manager manager_type>
	static void _temp_error(const char * const serial_msg, const char * const lcd_msg);
	template <Manager manager_type>
	static void min_temp_error();
	template <Manager manager_type>
	static void max_temp_error();

	typedef enum TRState { TRInactive, TRFirstHeating, TRStable, TRRunaway } TRstate;

	template <Manager manager_type>
	static void thermal_runaway_protection(TRState* state, millis_t* timer, float temperature, float target_temperature, int period_seconds, int hysteresis_degc);

	static TRState thermal_runaway_state_machine;
	static millis_t thermal_runaway_timer;

	static TRState thermal_runaway_bed_state_machine;
	static millis_t thermal_runaway_bed_timer;

};

extern Temperature thermalManager;

