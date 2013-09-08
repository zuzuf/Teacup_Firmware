
/** \file
  \brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include "dda_lookahead.h"

#ifdef LOOKAHEAD

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include <avr/interrupt.h>

#include "dda_maths.h"
#include "dda.h"
#include "timer.h"
#include "delay.h"
#include "serial.h"
#include "sermsg.h"
#include "gcode_parse.h"
#include "dda_queue.h"
#include "debug.h"
#include "sersendf.h"
#include "pinio.h"
#include "memory_barrier.h"

extern uint8_t use_lookahead;

uint32_t lookahead_joined = 0;      // Total number of moves joined together
uint32_t lookahead_timeout = 0;     // Moves that did not compute in time to be actually joined

// Used for look-ahead debugging
#ifdef LOOKAHEAD_DEBUG_VERBOSE
  #define serprintf(...) sersendf_P(__VA_ARGS__)
#else
  #define serprintf(...)
#endif

// We also need the inverse: given a ramp length, determine the expected speed
// Note: the calculation is scaled by a factor 10000 to obtain an answer with a smaller
// rounding error.
// Warning: this is an expensive function as it requires a square root to get the result.
//
uint32_t dda_steps_to_velocity(uint32_t steps) {
  // v(t) = a*t, with v in mm/s and a = acceleration in mm/s²
  // s(t) = 1/2*a*t² with s (displacement) in mm
  // Rewriting yields v(s) = sqrt(2*a*s)
  // Rewriting into steps and seperation in constant part and dynamic part:
  // F_steps = sqrt((2000*a)/STEPS_PER_M_X) * 60 * sqrt(steps)
  static uint32_t part = 0;
//  if(part == 0)
//    part = int_sqrt((uint32_t)((2000.0f*ACCELERATION*3600.0f*10000.0f)/(float)STEPS_PER_M_X));
//  uint32_t res = int_sqrt((steps) * 10000) * part;
//  return res / 10000;
  if(part == 0)
      part = (uint32_t)sqrtf((2000.0f*3600.0f*ACCELERATION*16384.0f)/(float)STEPS_PER_M_X);
//    part = int_sqrt((uint32_t)((2000.0f*3600.0f*ACCELERATION*16384.0f)/(float)STEPS_PER_M_X));
  uint32_t res = int_sqrt((steps) << 14) * part;
  return res >> 14;
}

/**
 * Determine the 'jerk' between 2 2D vectors and their speeds. The jerk can be used to obtain an
 * acceptable speed for changing directions between moves.
 * Vector delta is in um, speed is in mm/min.
 * @param x1 x component of first vector
 * @param y1 y component of first vector
 * @param F1 feed rate of first move
 * @param x2 x component of second vector
 * @param y2 y component of second vector
 * @param F2 feed rate of second move
 */
int dda_jerk_size_2d_real(int32_t x1, int32_t y1, uint32_t F1, int32_t x2, int32_t y2, uint32_t F2) {
//  const int maxlen = 10000;
    const int32_t maxlen = 16384;
  // Normalize vectors so their length will be fixed
  // Note: approx_distance is not precise enough and may result in violent direction changes
  //sersendf_P(PSTR("Input vectors: (%ld, %ld) and (%ld, %ld)\r\n"),x1,y1,x2,y2);
  int32_t len = int_sqrt(x1*x1+y1*y1);
  x1 = (x1 * maxlen) / len;
  y1 = (y1 * maxlen) / len;
  len = int_sqrt(x2*x2+y2*y2);
  x2 = (x2 * maxlen) / len;
  y2 = (y2 * maxlen) / len;

  //sersendf_P(PSTR("Normalized vectors: (%ld, %ld) and (%ld, %ld)\r\n"),x1,y1,x2,y2);

  // Now scale the normalized vectors by their speeds
  x1 *= F1; y1 *= F1; x2 *= F2; y2 *= F2;

  //sersendf_P(PSTR("Speed vectors: (%ld, %ld) and (%ld, %ld)\r\n"),x1,y1,x2,y2);

  // The difference between the vectors actually depicts the jerk
  x1 -= x2; y1 -= y2;

  //sersendf_P(PSTR("Jerk vector: (%ld, %ld)\r\n"),x1,y1);

  return approx_distance(x1,y1) / maxlen;
}

/**
 * Determine the 'jerk' for 2 1D vectors and their speeds. The jerk can be used to obtain an
 * acceptable speed for changing directions between moves.
 * Vector delta is in um, speed is in mm/min.
 * @param x component of 1d vector - used to determine if we go back or forward
 * @param F feed rate
 */
int dda_jerk_size_1d(int32_t x1, uint32_t F1, int32_t x2, uint32_t F2) {
  if(x1 > 0) x1 = F1;
  else x1 = -F1;
  if(x2 > 0) x2 = F2;
  else x2 = -F2;

  // The difference between the vectors actually depicts the jerk
  x1 -= x2;
  if(x1 < 0) x1 = -x1;  // Make sure it remains positive

  //sersendf_P(PSTR("Jerk vector: (%ld, %ld)\r\n"),x1,y1);
  return x1;
}

/**
 * Determine the 'jerk' between 2 vectors and their speeds. The jerk can be used to obtain an
 * acceptable speed for changing directions between moves.
 * Instead of using 2 axis at once, consider the jerk for each axis individually and take the
 * upper limit between both. This ensures that each axis does not changes speed too fast.
 * Vector delta is in um, speed is in mm/min.
 * @param x1 x component of first vector
 * @param y1 y component of first vector
 * @param F1 feed rate of first move
 * @param x2 x component of second vector
 * @param y2 y component of second vector
 * @param F2 feed rate of second move
 */
int dda_jerk_size_2d(int32_t x1, int32_t y1, uint32_t F1, int32_t x2, int32_t y2, uint32_t F2) {
  return MAX(dda_jerk_size_1d(x1,F1,x2,F2),dda_jerk_size_1d(y1,F1,y2,F2));
}

/**
 * Safety procedure: if something goes wrong, for example an opto is triggered during normal movement,
 * we shut down the entire machine.
 * @param msg The reason why the machine did an emergency stop
 */
void dda_emergency_shutdown(PGM_P msg) {
  // Todo: is it smart to enable all interrupts again? e.g. can we create concurrent executions?
  sei();  // Enable interrupts to print the message
  serial_writestr_P(PSTR("error: emergency stop:"));
  if(msg!=NULL) serial_writestr_P(msg);
  serial_writestr_P(PSTR("\r\n"));
  delay_ms(20);   // Delay so the buffer can be flushed - otherwise the message is never sent
  timer_stop();
  queue_flush();
  power_off();
  cli();
  for (;;) { }
}

/**
 * Join 2 moves by removing the full stop between them, where possible.
 * To join the moves, the expected jerk - or force - of the change in direction is calculated.
 * The jerk is used to scale the common feed rate between both moves to obtain an acceptable speed
 * to transition between 'prev' and 'current'.
 *
 * Premise: we currently join the last move in the queue and the one before it (if any).
 * This means the feed rate at the end of the 'current' move is 0.
 *
 * Premise: the 'current' move is not dispatched in the queue: it should remain constant while this
 * function is running.
 *
 * Note: the planner always makes sure the movement can be stopped within the
 * last move (= 'current'); as a result a lot of small moves will still limit the speed.
 */
void dda_join_moves(DDA *prev, DDA *current) {

  // Calculating the look-ahead settings can take a while; before modifying
  // the previous move, we need to locally store any values and write them
  // when we are done (and the previous move is not already active).
  uint32_t prev_F, prev_F_in_steps, prev_F_start_in_steps, prev_F_end_in_steps, prev_rampup, prev_rampdown, prev_total_steps, prev_c0;
  uint8_t prev_id;
  // Similarly, we only want to modify the current move if we have the results of the calculations;
  // until then, we do not want to touch the current move settings.
  // Note: we assume 'current' will not be dispatched while this function runs, so we do not to
  // back up the move settings: they will remain constant.
  uint32_t this_F, this_F_in_steps, this_F_start_in_steps, this_rampup, this_rampdown, this_total_steps;
  uint8_t this_id;
  int32_t jerk, jerk_e;       // Expresses the forces if we would change directions at full speed
  static uint32_t la_cnt = 0;     // Counter: how many moves did we join?
  #ifdef LOOKAHEAD_DEBUG
  static uint32_t moveno = 0;     // Debug counter to number the moves - helps while debugging
  moveno++;
  #endif

  // Sanity: if the previous move or this one has no actual movement, bail now. (e.g. G1 F1500)
  if(prev->delta.X==0 && prev->delta.Y==0 && prev->delta.Z==0 && prev->delta.E==0) return;
  if(current->delta.X==0 && current->delta.Y==0 && current->delta.Z==0 && current->delta.E==0) return;
  if(prev->delta.X==0 && prev->delta.Y==0) return;
  if(current->delta.X==0 && current->delta.Y==0) return;

  serprintf(PSTR("Current Delta: %ld,%ld,%ld E:%ld Live:%d\r\n"), current->delta.X, current->delta.Y, current->delta.Z, current->delta.E, current->live);
  serprintf(PSTR("Prev    Delta: %ld,%ld,%ld E:%ld Live:%d\r\n"), prev->delta.X, prev->delta.Y, prev->delta.Z, prev->delta.E, prev->live);

  // Look-ahead: attempt to join moves into smooth movements
  // Note: moves are only modified after the calculations are complete.
  // Only prepare for look-ahead if we have 2 available moves to
  // join and the Z axis is unused (for now, Z axis moves are NOT joined).
  if(prev!=NULL && prev->live==0 && current->live==0 && prev->delta.Z==current->delta.Z) {
    // Calculate the jerk if the previous move and this move would be joined
    // together at full speed.
    jerk = dda_jerk_size_2d(prev->delta.X, prev->delta.Y, prev->endpoint.F,
        current->delta.X, current->delta.Y, current->endpoint.F);
    serprintf(PSTR("Jerk: %lu\r\n"), jerk);
    jerk_e = dda_jerk_size_1d(prev->delta.E, prev->endpoint.F, current->delta.E, current->endpoint.F);
    serprintf(PSTR("Jerk_e: %lu\r\n"), jerk_e);
  } else {
    // Move already executing or Z moved: abort the join
    return;
  }

  // Make sure we have 2 moves and the previous move is not already active
  if(prev!=NULL && prev->live==0 && current->live == 0) {
    // Perform an atomic copy to preserve volatile parameters during the calculations
    ATOMIC_START
      prev_id = prev->id;
      prev_c0 = prev->c0;
      prev_F = prev->endpoint.F;
      prev_F_start_in_steps = prev->F_start_in_steps;
      prev_F_end_in_steps = prev->F_end_in_steps;
      prev_rampup = prev->rampup_steps;
      prev_rampdown = prev->rampdown_steps;
      prev_total_steps = prev->total_steps;
      this_id = current->id;
      this_F = current->endpoint.F;
      this_total_steps = current->total_steps;
    ATOMIC_END
    prev_F_in_steps = ACCELERATE_RAMP_LEN(prev_F);
    this_F_in_steps = ACCELERATE_RAMP_LEN(this_F);

    // The initial crossing speed is the minimum between both target speeds
    // Note: this is a given: the start speed and end speed can NEVER be
    // higher than the target speed in a move!
    // Note 2: this provides an upper limit, if needed, the speed is lowered.
    uint32_t crossF_in_steps = MIN(prev_F_in_steps, this_F_in_steps);

    //sersendf_P(PSTR("j:%lu - XF:%lu"), jerk, crossF);

    // If the XY jerk is too big, scale the proposed cross speed
    if(jerk > LOOKAHEAD_MAX_JERK_XY) {
      serprintf(PSTR("Jerk too big: scale cross speed between moves\r\n"));
      // Get the highest speed between both moves
      uint32_t crossF = MAX(prev_F, this_F);

      // Perform an exponential scaling
      uint32_t ujerk = (uint32_t)jerk;  // Use unsigned to double the range before overflowing
      crossF = (crossF*LOOKAHEAD_MAX_JERK_XY*LOOKAHEAD_MAX_JERK_XY)/(ujerk*ujerk);

      crossF_in_steps = ACCELERATE_RAMP_LEN(crossF);
      // Safety: make sure we never exceed the maximum speed of a move
      crossF_in_steps = MIN(crossF_in_steps, prev_F_in_steps);
      crossF_in_steps = MIN(crossF_in_steps, this_F_in_steps);

      // Optimize: if the crossing speed is zero, there is no join possible between these
      // two (fast) moves. Stop calculating and leave the full stop that is currently between
      // them.
      if(crossF_in_steps == 0)
        return;

      sersendf_P(PSTR("=>F:%lu"), crossF_in_steps);
    }
    // Same to the extruder jerk: make sure we do not yank it
    if(jerk_e > LOOKAHEAD_MAX_JERK_E) {
      sersendf_P(PSTR("Jerk_e too big: scale cross speed between moves\r\n"));
      uint32_t crossF = MAX(this_F, prev_F);

      // Perform an exponential scaling
      uint32_t ujerk = (uint32_t)jerk_e;  // Use unsigned to double the range before overflowing
      crossF = (crossF*LOOKAHEAD_MAX_JERK_E*LOOKAHEAD_MAX_JERK_E)/(ujerk*ujerk);

      uint32_t crossF2_in_steps = ACCELERATE_RAMP_LEN(crossF);
      // Safety: make sure we never exceed the maximum speed of a move
      crossF2_in_steps = MIN(crossF2_in_steps, prev_F_in_steps);
      crossF2_in_steps = MIN(crossF2_in_steps, this_F_in_steps);

      // Only continue with joining if there is a feasible crossing speed
      if(crossF2_in_steps == 0) return;

      if(crossF2_in_steps > crossF_in_steps) {
        sersendf_P(PSTR("Jerk_e: %lu => crossF: %lu (original: %lu)\r\n"), jerk_e, crossF2_in_steps, crossF_in_steps);
      }

      // Pick the crossing speed for these 2 move to be within the jerk limits
      crossF_in_steps = MIN(crossF_in_steps, crossF2_in_steps);
    }

    // Compute the maximum speed we can reach for crossing
    crossF_in_steps = MIN(crossF_in_steps, this_total_steps);
    crossF_in_steps = MIN(crossF_in_steps, prev_total_steps + prev_F_start_in_steps);

    if (crossF_in_steps == 0)
        return;

    // Build ramps for previous move
    if (crossF_in_steps == prev_F_in_steps) {
        prev_rampup = prev_F_in_steps - prev_F_start_in_steps;
        prev_rampdown = 0;
    }
    else if (crossF_in_steps < prev_F_start_in_steps) {
        prev_rampup = 0;
        prev_rampdown = prev_F_start_in_steps - crossF_in_steps;
        uint32_t extra = (prev_total_steps - prev_rampdown) >> 1;
        uint32_t limit = prev_F_in_steps - prev_F_start_in_steps;
        extra = MIN(extra, limit);

        prev_rampup += extra;
        prev_rampdown += extra;
    }
    else {
        prev_rampup = crossF_in_steps - prev_F_start_in_steps;
        prev_rampdown = 0;
        uint32_t extra = (prev_total_steps - prev_rampup) >> 1;
        uint32_t limit = prev_F_in_steps - crossF_in_steps;
        extra = MIN(extra, limit);

        prev_rampup += extra;
        prev_rampdown += extra;
    }
    prev_rampdown = prev_total_steps - prev_rampdown;
    prev_F_end_in_steps = crossF_in_steps;

    // Build ramps for current move
    if (crossF_in_steps == this_F_in_steps) {
        this_rampup = 0;
        this_rampdown = crossF_in_steps;
    }
    else {
        this_rampup = 0;
        this_rampdown = crossF_in_steps;

        uint32_t extra = (this_total_steps - this_rampdown) >> 1;
        uint32_t limit = this_F_in_steps - crossF_in_steps;
        extra = MIN(extra, limit);

        this_rampup += extra;
        this_rampdown += extra;
    }
    this_rampdown = this_total_steps - this_rampdown;
    this_F_start_in_steps = crossF_in_steps;

    // Recompute ramping algorithm state values (to prevent accumulation of errors)
    uint32_t c0 = C0;
    uint32_t n0 = 1;
    uint32_t step_no = 0;
    if (prev_F_start_in_steps < this_F_start_in_steps) {
        step_no = prev_F_start_in_steps;
        n0 = 1 + (step_no << 2);
        c0 = prev_c0;
        }
    for(; step_no < this_F_start_in_steps ; ++step_no) {
        n0 += 4;
        // be careful of signedness!
        c0 = (int32_t)c0 - ((int32_t)(c0 * 2) / (int32_t)n0);
    }

    // Show the proposed crossing speed - this might get adjusted below
    serprintf(PSTR("Initial crossing speed: %lu\r\n"), crossF_in_steps);

    serprintf(PSTR("prev_F_start: %lu\r\n"), prev_F_start_in_steps);
    serprintf(PSTR("prev_F: %lu\r\n"), prev_F_in_steps);
    serprintf(PSTR("prev_rampup: %lu\r\n"), prev_rampup);
    serprintf(PSTR("prev_rampdown: %lu\r\n"), prev_total_steps - prev_rampdown);
    serprintf(PSTR("crossF: %lu\r\n"), crossF_in_steps);
    serprintf(PSTR("this_rampup: %lu\r\n"), this_rampup);
    serprintf(PSTR("this_rampdown: %lu\r\n"), this_total_steps - this_rampdown);
    serprintf(PSTR("this_F: %lu\r\n"), this_F_in_steps);

    uint8_t timeout = 0;

    ATOMIC_START
      // Evaluation: determine how we did...
      lookahead_joined++;

      // Determine if we are fast enough - if not, just leave the moves
      // Note: to test if the previous move was already executed and replaced by a new
      // move, we compare the DDA id.
      if(prev->live == 0 && prev->id == prev_id && current->live == 0 && current->id == this_id) {
        prev->F_end_in_steps = prev_F_end_in_steps;
        prev->rampup_steps = prev_rampup;
        prev->rampdown_steps = prev_rampdown;
        current->rampup_steps = this_rampup;
        current->rampdown_steps = this_rampdown;
        current->F_end_in_steps = 0;
        current->F_start_in_steps = this_F_start_in_steps;
        current->c0 = c0;
        current->n0 = n0;
        la_cnt++;
      } else
        timeout = 1;
    ATOMIC_END

    // If we were not fast enough, any feedback will happen outside the atomic block:
    if(timeout) {
      sersendf_P(PSTR("Error: look ahead not fast enough\r\n"));
      lookahead_timeout++;
    }
  }
}

#endif /* LOOKAHEAD */
