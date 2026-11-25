#include "flight_fsm.h"

#include <math.h>

void flight_fsm_init(flight_fsm_t *fsm)
{
    fsm->current_state = STATE_PAD_IDLE;
    fsm->peak_velocity_ms = 0.0f;
    fsm->boost_hold_ms = 0;
    fsm->apogee_hold_ms = 0;
    fsm->landed_stable_ms = 0;
    fsm->calibration_count = 0;
    fsm->is_calibrated = false;
}

// Human-readable name for a flight state (for logs/telemetry).
const char *flight_fsm_state_name(flight_state_t state)
{
    switch (state) {
        case STATE_PAD_IDLE: return "PAD_IDLE";
        case STATE_BOOST:    return "BOOST";
        case STATE_COAST:    return "COAST";
        case STATE_APOGEE:   return "APOGEE";
        case STATE_DESCENT:  return "DESCENT";
        case STATE_LANDED:   return "LANDED";
        default:             return "UNKNOWN";
    }
}

// Advance the flight state machine one step and return the new state.
// Transitions are driven by estimated vertical velocity (vel_z_mps) first,
// with acceleration and altitude as supporting evidence.
flight_state_t flight_fsm_update(flight_fsm_t *fsm,
                                 const state_estimator_t *est,
                                 uint32_t dt_ms)
{
    // Pull the estimator outputs into plainly-named locals.
    float vel_z_ms  = est->vel_z_mps;                                  // vertical speed (up = +)
    float accel_z_g = (est->accel_z_world_mps2 + FSM_ONE_G_MPS2) / FSM_ONE_G_MPS2; // total felt accel in g

    // Remember the fastest upward speed so far (used to confirm apogee).
    if (vel_z_ms > fsm->peak_velocity_ms) {
        fsm->peak_velocity_ms = vel_z_ms;
    }

    switch (fsm->current_state) {

    case STATE_PAD_IDLE:
        // Wait out the calibration window before looking for launch.
        if (!fsm->is_calibrated) {
            fsm->calibration_count++;
            if (fsm->calibration_count >= FSM_CALIBRATION_SAMPLES) {
                fsm->is_calibrated = true;
            }
            break;
        }

        // Launch: hard acceleration AND actually moving up, held for 100 ms.
        // Requiring both avoids a false trigger from a bump or a hard knock.
        if (accel_z_g > FSM_BOOST_ACCEL_G && vel_z_ms > FSM_BOOST_VEL_MS) {
            fsm->boost_hold_ms += dt_ms;
            if (fsm->boost_hold_ms >= FSM_BOOST_HOLD_MS) {
                fsm->current_state = STATE_BOOST;
            }
        } else {
            fsm->boost_hold_ms = 0;
        }
        break;

    case STATE_BOOST:
        // Motor burnout: felt accel drops below 1.2 g while still climbing fast.
        // (The barometer may be locked out here during the transonic push.)
        if (accel_z_g < FSM_COAST_ACCEL_G && vel_z_ms > FSM_COAST_VEL_MS) {
            fsm->current_state = STATE_COAST;
        }
        break;

    case STATE_COAST:
        // Apogee: upward speed has bled off to near zero. Velocity-driven, so
        // it ignores any high-speed barometer spikes earlier in the flight.
        if (vel_z_ms <= FSM_APOGEE_VEL_MS) {
            fsm->apogee_hold_ms += dt_ms;
            if (fsm->apogee_hold_ms >= FSM_APOGEE_HOLD_MS) {
                fsm->current_state = STATE_APOGEE;
            }
        } else {
            fsm->apogee_hold_ms = 0;   // still climbing, reset the timer
        }
        break;

    case STATE_APOGEE:
        // Brief marker at the top; arm the landed timer and move to descent.
        fsm->landed_stable_ms = 0;
        fsm->current_state = STATE_DESCENT;
        break;

    case STATE_DESCENT:
        // Falling under parachute until we are nearly still for 5 seconds.
        if (fabsf(vel_z_ms) < FSM_LANDED_VEL_MS) {
            fsm->landed_stable_ms += dt_ms;
            if (fsm->landed_stable_ms >= FSM_LANDED_STABLE_MS) {
                fsm->current_state = STATE_LANDED;
            }
        } else {
            fsm->landed_stable_ms = 0;   // still moving, reset the timer
        }
        break;

    case STATE_LANDED:
        break;

    default:
        fsm->current_state = STATE_PAD_IDLE;
        break;
    }

    return fsm->current_state;
}
