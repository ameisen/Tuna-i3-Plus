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
#include "thermal/thermal.hpp"
#include "bi3_plus_lcd.h"
#include "planner.h"
#include "configuration_store.h"
#include "watchdog.h"

#define ENABLE_ERROR_1 0
#define ENABLE_ERROR_2 0
#define ENABLE_ERROR_3 1
#define ENABLE_ERROR_4 1
#define ENABLE_ERROR_5 1

#include "managers/simple.hpp"

using HeaterManager = Tuna::Thermal::Manager::Simple;

temp_t Temperature::min_extrude_temp = (typename temp_t::type)EXTRUDE_MINTEMP;

temp_t Temperature::current_temperature = 0_C,
Temperature::current_temperature_bed = 0;
volatile uint16_t Temperature::current_temperature_raw = 0_u16;
temp_t Temperature::target_temperature = 0_C;
volatile uint16_t Temperature::current_temperature_bed_raw = 0_u16;
temp_t Temperature::target_temperature_bed = 0_C;

temp_t Temperature::watch_target_temp = 0_C;
millis_t Temperature::watch_heater_next_ms = 0_u16;

temp_t Temperature::watch_target_bed_temp = 0_C;
millis_t Temperature::watch_bed_next_ms = 0;

bool Temperature::allow_cold_extrude = false;

volatile bool Temperature::temp_meas_ready = false;

millis_t Temperature::next_bed_check_ms;

uint16_t Temperature::raw_temp_value = 0_u16,
Temperature::raw_temp_bed_value = 0;

volatile uint8_t Temperature::soft_pwm_amount = 0;
volatile uint8_t Temperature::soft_pwm_amount_bed = 0;

namespace
{
  class temp_trend final
  {
    static constexpr const auto MeanCount = make_uintsz<8>;
    static constexpr const auto PrinterMaxTempT = temp_t{ printer_max_temperature }.raw();
    using sum_t = uintsz<PrinterMaxTempT * MeanCount>;
    using tempt_t = typename temp_t::type;

    // positive hack because presently fixed-type doesn't support signed.
    bool m_Positive = true;
    sum_t m_MeanSum = 0;
  public:

    bool is_positive () const __restrict
    {
      return m_Positive;
    }

    temp_t getMean() const __restrict
    {
      return temp_t::from(m_MeanSum / MeanCount);
    }

    void appendValue(arg_type<temp_t> value, bool positive) __restrict
    {
      const auto raw_value = value.raw();
      m_MeanSum -= tempt_t(m_MeanSum / MeanCount);
      if (positive == m_Positive)
      {
        m_MeanSum += raw_value;
      }
      else
      {
        if (raw_value > m_MeanSum)
        {
          m_MeanSum = raw_value - m_MeanSum;
          m_Positive = !m_Positive;
        }
        else
        {
          m_MeanSum -= raw_value;
        }
      }
    }
  };
  temp_trend temperatureTrendCalculator;
}

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

pair<temp_t, bool> Temperature::get_temperature_trend()
{
  return { temperatureTrendCalculator.getMean(), temperatureTrendCalculator.is_positive() };
}

void Temperature::PID_autotune(arg_type<temp_t> temp, arg_type<int> ncycles, bool set_result/*=false*/) {
  HeaterManager::calibrate(temp);
}

void Temperature::updatePID() {}

template <Temperature::Manager manager_type>
uint8 Temperature::getHeaterPower() {
	if constexpr (manager_type == Manager::Hotend)
	{
		return soft_pwm_amount;
	}
	else
	{
		return soft_pwm_amount_bed;
	}
}
template uint8 Temperature::getHeaterPower<Temperature::Manager::Hotend>();
template uint8 Temperature::getHeaterPower<Temperature::Manager::Bed>();

//
// Temperature Error Handlers
//
template <Temperature::Manager manager_type>
void Temperature::_temp_error(const char * __restrict const serial_msg, const char * __restrict const lcd_msg) {
	static bool killed = false;
	lcd::show_page(lcd::Page::Thermal_Runaway);
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
		kill("ohno i broke");
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

/**
 * Manage heating activities for extruder hot-ends and a heated bed
 *  - Acquire updated temperature readings
 *    - Also resets the watchdog timer
 *  - Invoke thermal runaway protection
 *  - Manage extruder auto-fan
 *  - Apply filament width to the extrusion rate (may move)
 *  - Update the heated bed PID output value
 */
bool Temperature::manage_heater() {

  if (!updateTemperaturesFromRawValues())
  {
    return false;
  }

	millis_t ms = millis();

	// Check for thermal runaway
#if ENABLE_ERROR_2
	thermal_runaway_protection<Manager::Hotend>(thermal_runaway_state_machine, thermal_runaway_timer, current_temperature, target_temperature, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);
#endif

	// Make sure temperature is increasing
	if (watch_heater_next_ms && ELAPSED(ms, watch_heater_next_ms)) { // Time to check this extruder?
#if ENABLE_ERROR_1
		if (degHotend() < watch_target_temp)                             // Failed to increase enough?
			_temp_error<Manager::Hotend>(PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
		else                                                                 // Start again if the target is still far off
#endif
			start_watching_heater();
	}

	// Make sure temperature is increasing
	if (watch_bed_next_ms && ELAPSED(ms, watch_bed_next_ms)) {        // Time to check the bed?
#if ENABLE_ERROR_1
		if (degBed() < watch_target_bed_temp)                           // Failed to increase enough?
			_temp_error<Manager::Bed>(PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
		else                                                            // Start again if the target is still far off
#endif
			start_watching_bed();
	}

	//if (PENDING(ms, next_bed_check_ms)) return true;
	//next_bed_check_ms = ms + BED_CHECK_INTERVAL;

#if ENABLE_ERROR_2
	thermal_runaway_protection<Manager::Bed>(thermal_runaway_bed_state_machine, thermal_runaway_bed_timer, current_temperature_bed, target_temperature_bed, THERMAL_PROTECTION_BED_PERIOD, THERMAL_PROTECTION_BED_HYSTERESIS);
#endif

  // Failsafe to make sure fubar'd PID settings don't force the heater always on.
  if (target_temperature == 0_C)
  {
    soft_pwm_amount = 0;
    WRITE_HEATER_0(LOW);
  }
  else if ((current_temperature <= Hotend::min_temperature::Temperature || is_preheating()) || current_temperature >= Hotend::max_temperature::Temperature)
  {
    soft_pwm_amount = 0;
  }
  else
  {
    soft_pwm_amount = HeaterManager::get_power(current_temperature, target_temperature);
  }

  // Failsafe to make sure fubar'd PID settings don't force the heater always on.
  if (target_temperature_bed == 0_C)
  {
    soft_pwm_amount_bed = 0;
    WRITE_HEATER_BED(LOW);
  }

	// Check if temperature is within the correct range
	if (WITHIN(current_temperature_bed, temp_t(Bed::min_temperature::Temperature), temp_t(Bed::max_temperature::Temperature))) {
		soft_pwm_amount_bed = current_temperature_bed < target_temperature_bed ? MAX_BED_POWER >> 1 : 0;
	}
	else {
		soft_pwm_amount_bed = 0;
		WRITE_HEATER_BED(LOW);
	}

  return true;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
temp_t Temperature::analog2temp(arg_type<uint16> raw)
{
	return Thermistor::adc_to_temperature(raw);
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
temp_t Temperature::analog2tempBed(arg_type<uint16> raw)
{
	return Thermistor::adc_to_temperature(raw);
}

/**
 * Get the raw values into the actual temperatures.
 * The raw values are created in interrupt context,
 * and this function is called from normal context
 * as it would block the stepper routine.
 */
bool Temperature::updateTemperaturesFromRawValues() {

	if (temp_meas_ready) // TODO replace with CAS.
	{
    HeaterManager::debug_dump();

		uint16 temperature_raw;
		uint16 temperature_bed_raw;
		{
			Tuna::utils::critical_section_not_isr _critsec;
			temperature_raw = current_temperature_raw;
			temperature_bed_raw = current_temperature_bed_raw;
			temp_meas_ready = false;
		}

    //Serial.print("temperature raw    : "); Serial.println(temperature_raw);
    //Serial.print("max temperature raw: "); Serial.println(Hotend::max_temperature::Adc);
    //Serial.print("min temperature raw: "); Serial.println(Hotend::min_temperature::Adc);

#if ENABLE_ERROR_3
		if constexpr (HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP)
		{
			if ((temperature_raw >= Hotend::max_temperature::Adc) & (target_temperature > 0_C)) { max_temp_error<Manager::Hotend>(); }
			if ((Hotend::min_temperature::Adc >= temperature_raw) & (!is_preheating()) & (target_temperature > 0_C)) { min_temp_error<Manager::Hotend>(); }
		}
		else
		{
			if ((temperature_raw <= Hotend::max_temperature::Adc) & (target_temperature > 0_C)) { max_temp_error<Manager::Hotend>(); }
			if ((Hotend::min_temperature::Adc <= temperature_raw) & (!is_preheating()) & (target_temperature > 0_C)) { min_temp_error<Manager::Hotend>(); }
		}
#endif

#if ENABLE_ERROR_5
		if constexpr (HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP)
		{
			if ((temperature_bed_raw >= Bed::max_temperature::Adc) & (target_temperature_bed > 0_C)) { max_temp_error<Manager::Bed>(); }
			if ((Bed::min_temperature::Adc >= temperature_bed_raw) & (target_temperature_bed > 0_C)) { min_temp_error<Manager::Bed>(); }
		}
		else
		{
			if ((temperature_bed_raw <= Bed::max_temperature::Adc) & (target_temperature_bed > 0_C)) { max_temp_error<Manager::Bed>(); }
			if ((Bed::min_temperature::Adc <= temperature_bed_raw) & (target_temperature_bed > 0_C)) { min_temp_error<Manager::Bed>(); }
		}
#endif

    // reading from constexpr here.
    temperature_raw = Thermistor::clamp_adc(temperature_raw);
    temperature_bed_raw = Thermistor::clamp_adc(temperature_bed_raw);

    const auto previous_temperature = current_temperature;

		current_temperature = Temperature::analog2temp(temperature_raw);
		current_temperature_bed = Temperature::analog2tempBed(temperature_bed_raw);

    if (current_temperature >= previous_temperature)
    {
      const temp_t temperatureDiff = current_temperature - previous_temperature;
      temperatureTrendCalculator.appendValue(temperatureDiff, true);
    }
    else
    {
      const temp_t temperatureDiff = previous_temperature - current_temperature;
      temperatureTrendCalculator.appendValue(temperatureDiff, false);
    }

		// Reset the watchdog after we know we have a temperature measurement.
		Tuna::wdr();
		return true;
	}
	return false;
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
}

/**
 * Start Heating Sanity Check for hotends that are below
 * their target temperature by a configurable margin.
 * This is called when the temperature is set. (M104, M109)
 */
void Temperature::start_watching_heater() {
	if (degHotend() < degTargetHotend() - uint8(WATCH_TEMP_INCREASE + TEMP_HYSTERESIS + 1)) {
		watch_target_temp = degHotend() + uint8(WATCH_TEMP_INCREASE);
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
	if (degBed() < degTargetBed() - uint8(WATCH_BED_TEMP_INCREASE + TEMP_BED_HYSTERESIS + 1)) {
		watch_target_bed_temp = degBed() + uint8(WATCH_BED_TEMP_INCREASE);
		watch_bed_next_ms = millis() + uint8(WATCH_BED_TEMP_PERIOD) * 1000UL;
	}
	else
		watch_bed_next_ms = 0;
}

Temperature::TRState Temperature::thermal_runaway_state_machine = TRInactive;
millis_t Temperature::thermal_runaway_timer = 0;

Temperature::TRState Temperature::thermal_runaway_bed_state_machine = TRInactive;
millis_t Temperature::thermal_runaway_bed_timer;

template <Temperature::Manager manager_type> static temp_t tr_target_temperature = 0_C;

template <Temperature::Manager manager_type>
void Temperature::thermal_runaway_protection(Temperature::TRState & __restrict state, millis_t & __restrict timer, arg_type<temp_t> current, arg_type<temp_t> target, arg_type<int> period_seconds, arg_type<int> hysteresis_degc)
{
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
		state = target > 0_C ? TRFirstHeating : TRInactive;
	}

	switch (state) {
		// Inactive state waits for a target temperature to be set
	case TRInactive: break;
		// When first heating, wait for the temperature to be reached then go to Stable state
	case TRFirstHeating:
		if (current < tr_target_temperature<manager_type>) break;
		state = TRStable;
		// While the temperature is stable watch for a bad temperature
	case TRStable:
		if (current >= tr_target_temperature<manager_type> -hysteresis_degc) {
			timer = millis() + period_seconds * 1000UL;
			break;
		}
		else if (PENDING(millis(), timer)) break;
		state = TRRunaway;
	case TRRunaway:
		_temp_error<manager_type>(PSTR(MSG_T_THERMAL_RUNAWAY), PSTR(MSG_THERMAL_RUNAWAY));
	}
}

//template void Temperature::thermal_runaway_protection<Temperature::Manager::Hotend>(Temperature::TRState & __restrict state, millis_t & __restrict timer, arg_type<temp_t> current, arg_type<temp_t> target, arg_type<int> period_seconds, arg_type<int> hysteresis_degc);
//template void Temperature::thermal_runaway_protection<Temperature::Manager::Bed>(Temperature::TRState & __restrict state, millis_t & __restrict timer, arg_type<temp_t> current, arg_type<temp_t> target, arg_type<int> period_seconds, arg_type<int> hysteresis_degc);

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
void Temperature::set_current_temp_raw()
{
	Tuna::utils::critical_section _critsec;
	current_temperature_raw = raw_temp_value;
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

/**
* States for ADC reading in the ISR
*/
enum ADCSensorState : uint8 {
	PrepareTemp_0 = 0,
	MeasureTemp_0,
	PrepareTemp_BED,
	MeasureTemp_BED,
	SensorsReady, // Temperatures ready. Delay the next round of readings to let ADC pins settle.
	StartupDelay  // Startup, delay initial temp reading a tiny bit so the hardware can settle
};

static_assert(Temperature::ACTUAL_ADC_SAMPLES >= uint(SensorsReady), "bad");

void Temperature::isr() {
  // TODO : Investigate native AVR PWM pin controls
  // http://maxembedded.com/2012/01/avr-timers-pwm-mode-part-ii/
  // analogWrite
  // It would likely be simpler, though we'd have less control over frequency.


	// The stepper ISR can interrupt this ISR. When it does it re-enables this ISR
	// at the end of its run, potentially causing re-entry. This flag prevents it.
	if (in_temp_isr) return;
	in_temp_isr = true;

	// Allow UART and stepper ISRs
	CBI(TIMSK0, OCIE0B); //Disable Temperature ISR
	Tuna::sei();

	static int8 oversample_count = 0;
	static ADCSensorState adc_sensor_state = StartupDelay;
  static uint8 pwm_counter = 0;

  const uint8_t extruder_pwm = soft_pwm_amount;
  const uint8_t bed_pwm = soft_pwm_amount_bed;

  // TODO : better heater state tracking.
  WRITE_HEATER_0((pwm_counter <= extruder_pwm && extruder_pwm) ? HIGH : LOW);
  WRITE_HEATER_BED((pwm_counter <= bed_pwm && bed_pwm) ? HIGH : LOW);
  ++pwm_counter;

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

	// oversamples = 16, thus (2^16)/16 is the greatest temperature we can represent without overflowing (4096).
	// We should set a peak limit below that. 512 seems OK.
	// 512 is nice as we actually don't have to mask the lower byte, only the upper byte, as the mask is 0x1FF.
	constexpr const uint16 MaxTemp = 512_u16;
	constexpr const uint16 MaxTempMask = MaxTemp - 1_u16;
	constexpr const uint8 MaxTempMaskLower = uint8(MaxTempMask & 0xFF);
	static_assert(MaxTempMaskLower == 0xFF, "The lower byte of the MaxTempMask must be 0xFF");

	switch (adc_sensor_state) {

	case SensorsReady: {
		// All sensors have been read. Stay in this state for a few
		// ISRs to save on calls to temp update/checking code below.
		//constexpr uint8 extra_loops = ACTUAL_ADC_SAMPLES - (uint8)SensorsReady;
		//static uint8 delay_count = 0;
		//if (extra_loops > 0) {
		//	if (delay_count == 0) delay_count = extra_loops;   // Init this delay
		//	if (--delay_count)                                 // While delaying...
		//		adc_sensor_state = (ADCSensorState)(uint8(SensorsReady) - 1); // retain this state (else, next state will be 0)
		//	break;
		//}
		//else
			adc_sensor_state = (ADCSensorState)0; // Fall-through to start first sensor now
	}

	case PrepareTemp_0:
		start_adc(TEMP_0_PIN);
		break;
	case MeasureTemp_0:
	{
		raw_temp_value += ADC;
	} break;

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
	adc_sensor_state = (ADCSensorState)(uint8(uint8(adc_sensor_state) + 1) % uint8(StartupDelay));

	Tuna::cli();
	in_temp_isr = false;
	SBI(TIMSK0, OCIE0B); //re-enable Temperature ISR
}
