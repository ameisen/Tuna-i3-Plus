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
 * stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
 * Derived from Grbl
 *
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef STEPPER_H
#define STEPPER_H

#include "planner.h"
#include "speed_lookuptable.h"
#include "stepper_indirection.h"
#include "language.h"
#include "types.h"

class Stepper;
extern Stepper stepper;

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result

inline uint8 __forceinline __flatten __const MultiU16X8toH8(uint8 charIn1, uint16 intIn2)
{
  // It is faster to use uint24, shift by 8, and let the compiler do it.
  // Also note that the upper value is always going to be expressable as a single byte.
  __assume((charIn1 * intIn2) <= 0xFFFF);
  return (charIn1 * intIn2) >> 8_u8;
}

class Stepper final {

  public:

    static block_t * __restrict current_block;  // A pointer to the block currently being traced

  private:

    static uint8_t last_direction_bits;        // The next stepping-bits to be output
    static uint16_t cleaning_buffer_counter;

    // Counter variables for the Bresenham line tracer
    static int24 counter[XYZE];
    static uint24 step_events_completed; // The number of step events executed in the current block

    #if ENABLED(LIN_ADVANCE)
      static uint16_t nextMainISR, nextAdvanceISR, eISR_Rate;
      #define _NEXT_ISR(T) nextMainISR = T
      static volatile int e_steps[E_STEPPERS];
      static int final_estep_rate;
      static int current_estep_rate[E_STEPPERS]; // Actual extruder speed [steps/s]
      static int current_adv_steps[E_STEPPERS];  // The amount of current added esteps due to advance.
                                                  // i.e., the current amount of pressure applied
                                                  // to the spring (=filament).
    #else
      #define _NEXT_ISR(T) OCR1A = T
    #endif // ADVANCE or LIN_ADVANCE

    //unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
    static uint16/*24*/ acc_step_rate; // needed for deceleration start point
    static uint8_t step_loops, step_loops_nominal;
    static unsigned short OCR1A_nominal;

    static volatile int24 endstops_trigsteps[XYZ];
    static volatile int24 endstops_stepsTotal, endstops_stepsDone;

    //
    // Positions of stepper motors, in step units
    //
    static volatile int24 count_position[NUM_AXIS];

    //
    // Current direction of stepper motors (+1 or -1)
    //
    static volatile signed char count_direction[NUM_AXIS];

    //
    // Mixing extruder mix counters
    //
    #if ENABLED(MIXING_EXTRUDER)
      static long counter_m[MIXING_STEPPERS];
      #define MIXING_STEPPERS_LOOP(VAR) \
        for (uint8_t VAR = 0; VAR < MIXING_STEPPERS; VAR++) \
          if (current_block->mix_event_count[VAR])
    #endif

  public:

    //
    // Constructor / initializer
    //
    Stepper() = default;

    //
    // Initialize stepper hardware
    //
    static void init();

    //
    // Interrupt Service Routines
    //

    static void __forceinline __flatten isr();

    #if ENABLED(LIN_ADVANCE)
      static void __forceinline __flatten advance_isr();
      static void __forceinline __flatten advance_isr_scheduler();
    #endif

    //
    // Block until all buffered steps are executed
    //
    static void synchronize();

    //
    // Set the current position in steps
    //
    static void set_position(const int24 &a, const int24 &b, const int24 &c, const int24 &e);
    static void __forceinline __flatten set_position(const AxisEnum &a, const int24 &v);
    static void __forceinline __flatten set_e_position(const int24 &e);

    //
    // Set direction bits for all steppers
    //
    static void set_directions();

    //
    // Get the position of a stepper, in steps
    //
    static int24 position(AxisEnum axis);

    //
    // Report the positions of the steppers, in steps
    //
    static void report_positions();

    //
    // Get the position (mm) of an axis based on stepper position(s)
    //
    static float __forceinline __flatten get_axis_position_mm(AxisEnum axis);

    //
    // SCARA AB axes are in degrees, not mm
    //
    #if IS_SCARA
      static float __forceinline get_axis_position_degrees(AxisEnum axis) { return get_axis_position_mm(axis); }
    #endif

    //
    // The stepper subsystem goes to sleep when it runs out of things to execute. Call this
    // to notify the subsystem that it is time to go to work.
    //
    static void __forceinline __flatten wake_up();

    //
    // Wait for moves to finish and disable all steppers
    //
    static void finish_and_disable();

    //
    // Quickly stop all steppers and clear the blocks queue
    //
    static void quick_stop();

    //
    // The direction of a single motor
    //
    static inline bool __forceinline __flatten motor_direction(AxisEnum axis) { return TEST(last_direction_bits, axis); }

    #if HAS_DIGIPOTSS
      static void digitalPotWrite(const int16_t address, const int16_t value);
      static void digipot_current(const uint8_t driver, const int16_t current);
    #endif

    #if HAS_MICROSTEPS
      static void microstep_ms(const uint8_t driver, const int8_t ms1, const int8_t ms2);
      static void microstep_mode(const uint8_t driver, const uint8_t stepping);
      static void microstep_readings();
    #endif

    #if ENABLED(BABYSTEPPING)
      static void babystep(const AxisEnum axis, const bool direction); // perform a short step with a single stepper motor, outside of any convention
    #endif

    static inline void __forceinline __flatten kill_current_block() {
      step_events_completed = current_block->step_event_count;
    }

    //
    // Handle a triggered endstop
    //
    static void __forceinline __flatten endstop_triggered(AxisEnum axis);

    //
    // Triggered position of an axis in mm (not core-savvy)
    //
    static inline float __forceinline __flatten triggered_position_mm(AxisEnum axis) {
      return endstops_trigsteps[axis] * planner.steps_to_mm[axis];
    }

  private:

    static inline unsigned __forceinline __flatten short calc_timer(uint16/*24*/ step_rate) {
      unsigned short timer;

      NOMORE(step_rate, MAX_STEP_FREQUENCY);

      //step_rate = min(step_rate, 159999_u24);

      /*if (step_rate > 80000) { // If steprate > 40kHz >> step 8 times
        step_rate >>= 4;
        step_loops = 16;
      }
      else */if (step_rate > 40000) { // If steprate > 40kHz >> step 8 times
        step_rate >>= 3;
        step_loops = 8;
      }
      else if (step_rate > 20000) { // If steprate > 20kHz >> step 4 times
        step_rate >>= 2;
        step_loops = 4;
      }
      else if (step_rate > 10000) { // If steprate > 10kHz >> step 2 times
        step_rate >>= 1;
        step_loops = 2;
      }
      else {
        step_loops = 1;
      }

      NOLESS(step_rate, F_CPU / 500000);
      step_rate -= F_CPU / 500000; // Correct for minimal speed
      if (step_rate >= (8 * 256)) { // higher step rate
        unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate >> 8)][0];
        unsigned char tmp_step_rate = (step_rate & 0x00FF);
        unsigned short gain = (unsigned short)pgm_read_word_near(table_address + 2);
        timer = (unsigned short)pgm_read_word_near(table_address) - MultiU16X8toH8(tmp_step_rate, gain);
      }
      else { // lower step rates
        unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
        table_address += ((step_rate) >> 1) & 0xFFFC;
        timer = (unsigned short)pgm_read_word_near(table_address);
        timer -= (((unsigned short)pgm_read_word_near(table_address + 2) * (unsigned char)(step_rate & 0x0007)) >> 3);
      }
      if (timer < 100) { // (20kHz - this should never happen)
        timer = 100;
        MYSERIAL.print(MSG_STEPPER_TOO_HIGH);
        MYSERIAL.println(uint32(step_rate));
      }
      return timer;
    }

    // Initialize the trapezoid generator from the current block.
    // Called whenever a new block begins.
    static inline void __forceinline __flatten trapezoid_generator_reset()
    {
      __assume(current_block->active_extruder == 0);

#if EXTRUDERS > 1
      static int8_t last_extruder = -1;
#endif

      if (
           current_block->direction_bits != last_direction_bits
#if EXTRUDERS > 1
        || current_block->active_extruder != last_extruder
#endif
      ) 
      {
        last_direction_bits = current_block->direction_bits;
#if EXTRUDERS > 1
        last_extruder = current_block->active_extruder;
#endif
        set_directions();
      }

      // step_rate to timer interval
      OCR1A_nominal = calc_timer(current_block->nominal_rate);
      // make a note of the number of step loops required at nominal speed
      step_loops_nominal = step_loops;
      acc_step_rate = current_block->initial_rate;
      _NEXT_ISR(calc_timer(acc_step_rate));

      #if ENABLED(LIN_ADVANCE)
        if (current_block->use_advance_lead) {
          current_estep_rate[current_block->active_extruder] = ((unsigned long)acc_step_rate * current_block->abs_adv_steps_multiplier8) >> 17;
          final_estep_rate = (current_block->nominal_rate * current_block->abs_adv_steps_multiplier8) >> 17;
        }
      #endif

      // SERIAL_ECHO_START();
      // SERIAL_ECHOPGM("advance :");
      // SERIAL_ECHO(current_block->advance/256.0);
      // SERIAL_ECHOPGM("advance rate :");
      // SERIAL_ECHO(current_block->advance_rate/256.0);
      // SERIAL_ECHOPGM("initial advance :");
      // SERIAL_ECHO(current_block->initial_advance/256.0);
      // SERIAL_ECHOPGM("final advance :");
      // SERIAL_ECHOLN(current_block->final_advance/256.0);
    }

    #if HAS_DIGIPOTSS
      static void digipot_init();
    #endif

    #if HAS_MICROSTEPS
      static void microstep_init();
    #endif

};

#endif // STEPPER_H
