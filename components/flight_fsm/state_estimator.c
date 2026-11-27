#include "state_estimator.h"

#include <math.h>
#include <string.h>

void state_estimator_init(state_estimator_t *st)
{
    memset(st, 0, sizeof(state_estimator_t));
}

void state_estimator_calibrate_step(state_estimator_t *st,
                                    const float accel_mps2[3],
                                    float baro_alt_m,
                                    uint16_t sample_idx,
                                    uint16_t total_samples)
{
    if (sample_idx == 0) {
        st->accel_z_bias_mps2 = 0.0f;
        st->baro_ground_alt_m = 0.0f;
        st->is_calibrated = false;
    }

    // Average the resting body-Z acceleration (this is gravity on the pad)
    // and the ground altitude.
    st->accel_z_bias_mps2 += accel_mps2[2];
    st->baro_ground_alt_m += baro_alt_m;

    // Last sample: turn the sums into averages and arm the estimate.
    if (sample_idx + 1 >= total_samples && total_samples > 0) {
        float inv_n = 1.0f / (float)total_samples;
        st->accel_z_bias_mps2 *= inv_n;
        st->baro_ground_alt_m *= inv_n;
        st->alt_m = 0.0f;
        st->vel_z_mps = 0.0f;
        st->is_calibrated = true;
    }
}

void state_estimator_update(state_estimator_t *st,
                            const float accel_mps2[3],
                            float baro_alt_m,
                            float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.2f) {
        return;   // reject bad time step
    }

    // --- Step 1: vertical acceleration (gravity removed) ---
    // With no gyro we treat the body Z axis as vertical. The resting bias
    // learned on the pad is the 1 g gravity offset, so subtracting it leaves
    // the dynamic (thrust/drag) acceleration.
    float az_world = accel_mps2[2] - st->accel_z_bias_mps2;
    st->accel_z_world_mps2 = az_world;

    // --- Step 2: integrate accel -> vertical speed -> altitude ---
    // Trapezoidal integration. This inertial estimate keeps working even when
    // the barometer is locked out at high speed.
    float prev_vel = st->vel_z_mps;
    st->vel_z_mps += az_world * dt_s;
    st->alt_m     += 0.5f * (prev_vel + st->vel_z_mps) * dt_s;

    // --- Step 3: transonic / high-speed barometer lockout (hysteresis) ---
    // Above BARO_LOCKOUT_SPEED_MS the static-port pressure is unreliable, so
    // we freeze the barometer and trust inertial integration. We only unlock
    // again once speed drops below BARO_UNLOCK_SPEED_MS to avoid flip-flop.
    float speed = fabsf(st->vel_z_mps);
    if (!st->is_baro_locked_out) {
        if (speed >= BARO_LOCKOUT_SPEED_MS) {
            st->is_baro_locked_out = true;
        }
    } else {
        if (speed < BARO_UNLOCK_SPEED_MS) {
            st->is_baro_locked_out = false;
        }
    }

    if (st->is_baro_locked_out) {
        return;   // trust the inertial estimate only this step
    }

    // --- Step 4: blend the barometer back in (complementary filter) ---
    // When we trust the barometer, nudge altitude and speed toward it so the
    // inertial drift stays bounded over the flight.
    float rel_baro_alt = baro_alt_m - st->baro_ground_alt_m;
    float alt_error = rel_baro_alt - st->alt_m;

    const float kp_alt = 0.25f;
    const float kp_vel = 0.08f;

    st->alt_m     += kp_alt * alt_error * dt_s;
    st->vel_z_mps += kp_vel * alt_error * dt_s;
}
