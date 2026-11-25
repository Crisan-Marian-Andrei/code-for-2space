#pragma once

// Fuses IMU + magnetometer + barometer into altitude, vertical speed,
// vertical accel and a full 3D attitude (quaternion + Euler angles).
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
    float vel_z_mps;           // vertical speed (Earth Z), from inertial integration
    float accel_z_world_mps2;  // vertical accel, gravity removed

    // -------- 3D attitude --------
    float qw, qx, qy, qz;      // orientation quaternion (body -> world)
    float tilt_angle_rad;      // angle away from vertical (radians)

    // Human-readable Euler angles, filled every update:
    float roll_deg;            // rotation about the forward axis
    float pitch_deg;           // nose up/down
    float yaw_heading_deg;     // compass heading 0..360 deg (0 = North)
    float tilt_deg;            // tilt_angle_rad in degrees (angle from vertical)

    // -------- Calibration state (learned on the pad) --------
    float gyro_bias[3];        // deg/s, learned on the pad
    float baro_ground_alt_m;   // pad barometer reading
    bool  is_calibrated;

    // -------- Barometer lockout --------
    bool  is_baro_locked_out;  // true = currently ignoring the barometer
} state_estimator_t;

// Zero everything; quaternion = no rotation.
void state_estimator_init(state_estimator_t *st);

// Feed one pad calibration sample (sample_idx 0..total_samples-1).
// Also seeds the starting attitude from the accelerometer + magnetometer so
// the rocket knows which way is up and which way is North before launch.
void state_estimator_calibrate_step(state_estimator_t *st,
                                    const float accel_mps2[3],
                                    const float gyro_dps[3],
                                    const float mag_ut[3],
                                    bool mag_valid,
                                    float baro_alt_m,
                                    uint16_t sample_idx,
                                    uint16_t total_samples);

// Advance the estimate by dt_s seconds.
//   accel_mps2 / gyro_dps : raw IMU this step
//   mag_ut / mag_valid    : magnetometer this step (used to fix yaw drift)
//   baro_alt_m            : barometric altitude (ignored while locked out)
void state_estimator_update(state_estimator_t *st,
                            const float accel_mps2[3],
                            const float gyro_dps[3],
                            const float mag_ut[3],
                            bool mag_valid,
                            float baro_alt_m,
                            float dt_s);

#ifdef __cplusplus
}
#endif
