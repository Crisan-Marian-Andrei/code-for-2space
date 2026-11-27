#pragma once

// Fuses the accelerometer + barometer into altitude, vertical speed and
// vertical acceleration. No attitude or heading estimation: the board has no
// gyroscope or magnetometer, so we assume the rocket's body Z axis stays
// roughly aligned with vertical and use it as the vertical acceleration axis.
//
// Key safety idea for rocketry: at transonic/supersonic speeds the shockwave
// over the static port makes the barometer read false spikes. So above a
// speed threshold we LOCK OUT the barometer and trust pure inertial
// integration (accel -> vel -> altitude). Feeds flight_fsm.h.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// -------- Barometer transonic / high-speed lockout --------
// Above this vertical speed (m/s) we stop trusting the barometer.
// 240 m/s is roughly Mach 0.70 at sea level, safely below the Mach 0.75
// transonic region where static-port errors get large.
#define BARO_LOCKOUT_SPEED_MS   240.0f
// Hysteresis: once locked out, only unlock again when we slow below this.
// The gap (240 -> 200) stops rapid flip-flopping near the threshold.
#define BARO_UNLOCK_SPEED_MS    200.0f

typedef struct {
    // -------- Vertical motion (what the flight FSM uses) --------
    float alt_m;               // altitude above the pad
    float vel_z_mps;           // vertical speed (up = +), from inertial integration
    float accel_z_world_mps2;  // vertical accel, gravity removed

    // -------- Calibration state (learned on the pad) --------
    // Measured body-Z acceleration while sitting still on the pad (~ +1 g,
    // sign depends on mounting). Subtracted at run time to remove gravity.
    float accel_z_bias_mps2;
    float baro_ground_alt_m;   // pad barometer reading
    bool  is_calibrated;

    // -------- Barometer lockout --------
    bool  is_baro_locked_out;  // true = currently ignoring the barometer
} state_estimator_t;

// Zero everything.
void state_estimator_init(state_estimator_t *st);

// Feed one pad calibration sample (sample_idx 0..total_samples-1).
// Learns the resting body-Z acceleration (gravity) and the ground altitude.
void state_estimator_calibrate_step(state_estimator_t *st,
                                    const float accel_mps2[3],
                                    float baro_alt_m,
                                    uint16_t sample_idx,
                                    uint16_t total_samples);

// Advance the estimate by dt_s seconds.
//   accel_mps2 : raw accelerometer this step (body frame, m/s^2)
//   baro_alt_m : barometric altitude (ignored while locked out)
void state_estimator_update(state_estimator_t *st,
                            const float accel_mps2[3],
                            float baro_alt_m,
                            float dt_s);

#ifdef __cplusplus
}
#endif
