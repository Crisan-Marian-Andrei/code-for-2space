#include "state_estimator.h"

#include <math.h>
#include <string.h>

#define G_ACCEL            9.80665f
#define DEG_TO_RAD         0.017453292519943295f
#define RAD_TO_DEG         57.29577951308232f

// ---------------------------------------------------------------------------
// Small quaternion helpers. A quaternion (qw,qx,qy,qz) stores the rocket's
// orientation (how the body frame is rotated relative to the world frame).
// Keeping these tiny and explicit so the maths below reads step by step.
// ---------------------------------------------------------------------------

// Normalise a quaternion back to unit length (integration slowly inflates it).
static void quat_normalize(state_estimator_t *st)
{
    float norm = sqrtf(st->qw * st->qw + st->qx * st->qx +
                       st->qy * st->qy + st->qz * st->qz);
    if (norm > 1e-6f) {
        float inv = 1.0f / norm;
        st->qw *= inv;
        st->qx *= inv;
        st->qy *= inv;
        st->qz *= inv;
    }
}

// Build the quaternion from roll/pitch/yaw (radians). Used on the pad to seed
// the starting orientation from the accelerometer + magnetometer.
static void quat_from_euler(state_estimator_t *st,
                            float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f),   sy = sinf(yaw * 0.5f);

    st->qw = cr * cp * cy + sr * sp * sy;
    st->qx = sr * cp * cy - cr * sp * sy;
    st->qy = cr * sp * cy + sr * cp * sy;
    st->qz = cr * cp * sy - sr * sp * cy;
    quat_normalize(st);
}

// Fill roll/pitch/yaw (degrees) and tilt from the current quaternion so the
// serial monitor and telemetry can show them directly.
static void euler_from_quat(state_estimator_t *st)
{
    float qw = st->qw, qx = st->qx, qy = st->qy, qz = st->qz;

    // Standard aerospace ZYX quaternion -> Euler conversion.
    // roll (about X)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    st->roll_deg = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // pitch (about Y), clamped to avoid NaN at straight up/down
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (sinp > 1.0f)  sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    st->pitch_deg = asinf(sinp) * RAD_TO_DEG;

    // yaw (about Z) = heading
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    float yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
    if (yaw < 0.0f) yaw += 360.0f;   // wrap to 0..360 compass heading
    st->yaw_heading_deg = yaw;

    st->tilt_deg = st->tilt_angle_rad * RAD_TO_DEG;
}

// Tilt-compensated magnetic heading (radians, 0 = North).
// We rotate the magnetometer reading flat using the known roll/pitch so that
// tilting the rocket does not change the reported compass direction.
static float mag_heading_rad(const float mag_ut[3], float roll_rad, float pitch_rad)
{
    float mx = mag_ut[0], my = mag_ut[1], mz = mag_ut[2];

    float cr = cosf(roll_rad),  sr = sinf(roll_rad);
    float cp = cosf(pitch_rad), sp = sinf(pitch_rad);

    // Project the magnetic vector onto the horizontal plane.
    float xh = mx * cp + my * sr * sp + mz * cr * sp;
    float yh = my * cr - mz * sr;

    float heading = atan2f(-yh, xh);
    if (heading < 0.0f) heading += 2.0f * (float)M_PI;
    return heading;
}

void state_estimator_init(state_estimator_t *st)
{
    memset(st, 0, sizeof(state_estimator_t));
    st->qw = 1.0f;   // (1,0,0,0) = no rotation
}

void state_estimator_calibrate_step(state_estimator_t *st,
                                    const float accel_mps2[3],
                                    const float gyro_dps[3],
                                    const float mag_ut[3],
                                    bool mag_valid,
                                    float baro_alt_m,
                                    uint16_t sample_idx,
                                    uint16_t total_samples)
{
    if (sample_idx == 0) {
        st->gyro_bias[0] = 0.0f;
        st->gyro_bias[1] = 0.0f;
        st->gyro_bias[2] = 0.0f;
        st->baro_ground_alt_m = 0.0f;
        st->is_calibrated = false;
    }

    // Average the gyro (to learn its zero-rate bias) and the ground altitude.
    st->gyro_bias[0] += gyro_dps[0];
    st->gyro_bias[1] += gyro_dps[1];
    st->gyro_bias[2] += gyro_dps[2];
    st->baro_ground_alt_m += baro_alt_m;

    // Last sample: turn the sums into averages and seed the attitude.
    if (sample_idx + 1 >= total_samples && total_samples > 0) {
        float inv_n = 1.0f / (float)total_samples;
        st->gyro_bias[0] *= inv_n;
        st->gyro_bias[1] *= inv_n;
        st->gyro_bias[2] *= inv_n;
        st->baro_ground_alt_m *= inv_n;
        st->alt_m = 0.0f;
        st->vel_z_mps = 0.0f;

        // --- Seed the starting orientation from gravity + magnetometer ---
        // On the pad the only acceleration is gravity, so the accelerometer
        // tells us the tilt (roll/pitch). The magnetometer then gives yaw.
        float ax = accel_mps2[0], ay = accel_mps2[1], az = accel_mps2[2];
        float roll  = atan2f(ay, az);
        float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

        float yaw = 0.0f;
        if (mag_valid) {
            yaw = mag_heading_rad(mag_ut, roll, pitch);
        }
        quat_from_euler(st, roll, pitch, yaw);
        euler_from_quat(st);

        st->is_calibrated = true;
    }
}

void state_estimator_update(state_estimator_t *st,
                            const float accel_mps2[3],
                            const float gyro_dps[3],
                            const float mag_ut[3],
                            bool mag_valid,
                            float baro_alt_m,
                            float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.2f) {
        return;   // reject bad time step
    }

    // --- Step 1: remove gyro bias, convert deg/s -> rad/s ---
    float gx = (gyro_dps[0] - st->gyro_bias[0]) * DEG_TO_RAD;
    float gy = (gyro_dps[1] - st->gyro_bias[1]) * DEG_TO_RAD;
    float gz = (gyro_dps[2] - st->gyro_bias[2]) * DEG_TO_RAD;

    // --- Step 2: integrate the gyro to spin the orientation forward ---
    // q_dot = 0.5 * q (x) omega. This is the fast, drift-prone attitude.
    float half_dt = 0.5f * dt_s;
    float qw = st->qw, qx = st->qx, qy = st->qy, qz = st->qz;

    st->qw += (-qx * gx - qy * gy - qz * gz) * half_dt;
    st->qx += ( qw * gx + qy * gz - qz * gy) * half_dt;
    st->qy += ( qw * gy - qx * gz + qz * gx) * half_dt;
    st->qz += ( qw * gz + qx * gy - qy * gx) * half_dt;
    quat_normalize(st);

    // --- Step 3: refresh Euler angles + tilt from the new quaternion ---
    // Bottom row of the rotation matrix gives the world-Z tilt and lets us
    // rotate the measured acceleration into the world frame.
    float r31 = 2.0f * (st->qx * st->qz - st->qw * st->qy);
    float r32 = 2.0f * (st->qy * st->qz + st->qw * st->qx);
    float r33 = 1.0f - 2.0f * (st->qx * st->qx + st->qy * st->qy);

    float clamped = (r33 > 1.0f) ? 1.0f : ((r33 < -1.0f) ? -1.0f : r33);
    st->tilt_angle_rad = acosf(clamped);
    euler_from_quat(st);

    // --- Step 4: magnetometer yaw correction (stops long-term drift) ---
    // The gyro alone drifts in yaw over time. The magnetometer gives an
    // absolute compass heading, so we gently nudge the orientation toward it.
    if (mag_valid) {
        float roll_rad  = st->roll_deg  * DEG_TO_RAD;
        float pitch_rad = st->pitch_deg * DEG_TO_RAD;
        float mag_yaw   = mag_heading_rad(mag_ut, roll_rad, pitch_rad);
        float gyro_yaw  = st->yaw_heading_deg * DEG_TO_RAD;

        // Shortest angular error between gyro yaw and magnetic yaw.
        float yaw_err = mag_yaw - gyro_yaw;
        while (yaw_err >  (float)M_PI) yaw_err -= 2.0f * (float)M_PI;
        while (yaw_err < -(float)M_PI) yaw_err += 2.0f * (float)M_PI;

        // Pull a small fraction of the error back each step (complementary
        // filter). Small gain = trust the gyro short-term, the mag long-term.
        const float kp_yaw = 0.02f;
        float d_yaw = kp_yaw * yaw_err;

        // Apply it as a tiny rotation about the WORLD vertical (Z) axis:
        //   q_new = q_deltaYaw (x) q_old
        // Doing it this way (instead of rebuilding from Euler angles) stays
        // numerically stable even when the rocket points straight up.
        float h = 0.5f * d_yaw;
        float cw = cosf(h), sw = sinf(h);   // delta-yaw quaternion = (cw,0,0,sw)
        float nw = cw * st->qw - sw * st->qz;
        float nx = cw * st->qx - sw * st->qy;
        float ny = cw * st->qy + sw * st->qx;
        float nz = cw * st->qz + sw * st->qw;
        st->qw = nw; st->qx = nx; st->qy = ny; st->qz = nz;
        quat_normalize(st);
        euler_from_quat(st);
    }

    // --- Step 5: world-frame vertical acceleration (gravity removed) ---
    float az_world = r31 * accel_mps2[0] +
                     r32 * accel_mps2[1] +
                     r33 * accel_mps2[2] - G_ACCEL;
    st->accel_z_world_mps2 = az_world;

    // --- Step 6: integrate accel -> vertical speed -> altitude ---
    // Trapezoidal integration. This is the inertial-only estimate that keeps
    // working even when the barometer is locked out at high speed.
    float prev_vel = st->vel_z_mps;
    st->vel_z_mps += az_world * dt_s;
    st->alt_m     += 0.5f * (prev_vel + st->vel_z_mps) * dt_s;

    // --- Step 7: transonic / high-speed barometer lockout (hysteresis) ---
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

    // --- Step 8: blend the barometer back in (complementary filter) ---
    // When we trust the barometer, nudge altitude and speed toward it so the
    // inertial drift stays bounded over the flight.
    float rel_baro_alt = baro_alt_m - st->baro_ground_alt_m;
    float alt_error = rel_baro_alt - st->alt_m;

    const float kp_alt = 0.25f;
    const float kp_vel = 0.08f;

    st->alt_m     += kp_alt * alt_error * dt_s;
    st->vel_z_mps += kp_vel * alt_error * dt_s;
}
