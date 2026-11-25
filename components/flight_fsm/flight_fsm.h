#pragma once

// Flight state machine: PAD_IDLE -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED.
// Driven mainly by the estimated vertical VELOCITY from state_estimator, because
// the barometer is locked out at high speed and cannot be trusted for apogee.

#include <stdbool.h>
#include <stdint.h>
#include "state_estimator.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STATE_PAD_IDLE = 0,
    STATE_BOOST    = 1,
    STATE_COAST    = 2,
    STATE_APOGEE   = 3,
    STATE_DESCENT  = 4,
    STATE_LANDED   = 5
} flight_state_t;

#define FSM_ONE_G_MPS2          9.80665f

#define FSM_CALIBRATION_SAMPLES 50                 // pad calibration samples

// -------- BOOST: motor is burning, rocket accelerating hard upward --------
#define FSM_BOOST_ACCEL_G       2.5f               // felt accel above 2.5 g
#define FSM_BOOST_VEL_MS        15.0f              // AND climbing faster than 15 m/s
#define FSM_BOOST_HOLD_MS       100                // both held this long

// -------- COAST: motor burned out, still coasting upward --------
#define FSM_COAST_ACCEL_G       1.2f               // felt accel drops below 1.2 g
#define FSM_COAST_VEL_MS        20.0f              // while still rising faster than 20 m/s

// -------- APOGEE: top of flight, vertical speed crosses ~zero --------
// Velocity-driven so it is immune to high-speed barometer spikes. By apogee
// the rocket is well below Mach 0.75 so the barometer is already unlocked.
#define FSM_APOGEE_VEL_MS       2.0f               // speed has dropped to ~0
#define FSM_APOGEE_HOLD_MS      150                // sustained near/below zero

// -------- DESCENT: falling under parachute --------
// Entered straight after the brief APOGEE marker. By now vel_z_ms is going
// negative (falling); we stay here until the rocket is nearly still.

// -------- LANDED: on the ground, nearly still --------
#define FSM_LANDED_VEL_MS       1.0f               // |vertical speed| under 1 m/s
#define FSM_LANDED_STABLE_MS    5000               // held stable this long

typedef struct {
    flight_state_t current_state;

    float peak_velocity_ms;      // highest upward speed seen (to confirm apogee)

    uint32_t boost_hold_ms;
    uint32_t apogee_hold_ms;
    uint32_t landed_stable_ms;

    uint16_t calibration_count;
    bool     is_calibrated;
} flight_fsm_t;

// Reset to PAD_IDLE.
void flight_fsm_init(flight_fsm_t *fsm);

// Advance the machine. dt_ms = ms since the last call. Returns the new state.
// Reads vertical velocity, altitude, vertical accel and tilt from the estimator.
flight_state_t flight_fsm_update(flight_fsm_t *fsm,
                                 const state_estimator_t *est,
                                 uint32_t dt_ms);

// State name for logging.
const char *flight_fsm_state_name(flight_state_t state);

#ifdef __cplusplus
}
#endif
