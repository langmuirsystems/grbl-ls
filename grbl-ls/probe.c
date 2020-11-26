/*
  probe.c - code pertaining to probing methods
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


// Inverts the probe pin state depending on user settings and probing cycle mode.
uint8_t probe_invert_mask;

// Whether debounce counter has started
static bool debounce_started;
// Cycle count/64 when debounce counter started
// Using count/64 so that the counter fits in a uint16_t, because uint32_t is too slow
static uint16_t debounce_start_cycle;
// Debounce threshold in CPU cycles/64
// Cached here so to minimise slow floating point operations
static uint16_t debounce_threshold;
// Whether to ignore debounce timer when probing
static volatile bool ignore_debounce;

// Probe pin initialization routine.
void probe_init()
{
  PROBE_DDR &= ~(PROBE_MASK); // Configure as input pins
  #ifdef DISABLE_PROBE_PIN_PULL_UP
    PROBE_PORT &= ~(PROBE_MASK); // Normal low operation. Requires external pull-down.
  #else
    PROBE_PORT |= PROBE_MASK;    // Enable internal pull-up resistors. Normal high operation.
  #endif
  probe_configure_invert_mask(false); // Initialize invert mask.
  probe_set_debounce(settings.probe_debounce);
}


// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
void probe_configure_invert_mask(uint8_t is_probe_away)
{
  probe_invert_mask = 0; // Initialize as zero.
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK; }
  if (is_probe_away) { probe_invert_mask ^= PROBE_MASK; }
}


// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
uint8_t probe_get_state() { return((PROBE_PIN & PROBE_MASK) ^ probe_invert_mask); }

// Reset probe debounce counter
void probe_reset_debounce()
{
  debounce_started = false;
}

// Use this to configure how the probe responds to being triggered
// If ignore is true, the probe will immediately stop when triggered
void probe_configure_ignore_debounce(bool ignore)
{
  ignore_debounce = ignore;
}

// Set probe debounce time (seconds)
void probe_set_debounce(float value)
{
  if (value > (49152.0 * 1000.0 / (float)(F_CPU / 64))) {
    // don't let debounce_threshold get close to overflowing, otherwise the
    // probe may never trigger
    debounce_threshold = 49152;
  } else {
    debounce_threshold = (uint16_t)((float)(F_CPU / 64) * (value / 1000.0));
  }
}

// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
// NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
void probe_state_monitor(uint16_t cycle_counter_div64)
{
  if (probe_get_state()) {
    if (ignore_debounce
     || (debounce_started && ((cycle_counter_div64 - debounce_start_cycle) >= debounce_threshold))) {
      debounce_started = false;
      sys_probe_state = PROBE_OFF;
      memcpy(sys_probe_position, sys_position, sizeof(sys_position));
      bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
    } else if (!debounce_started) {
      debounce_start_cycle = cycle_counter_div64;
      debounce_started = true;
    }
  } else debounce_started = false;
}
