#include "flight_fsm.h"
#include <math.h>
#include <string.h>

#define G_ACCEL            9.80665f
#define TRANSONIC_ENTER_V  265.0f   // ~Mach 0.78 (intrare in lockout baro)
#define TRANSONIC_EXIT_V   255.0f   // Histerezis 10 m/s la iesire
#define SUPERSONIC_EXIT_V  415.0f   // ~Mach 1.22 (peste zona de instabilitate transonica)

void state_estimator_init(state_estimator_t *st)
{
    memset(st, 0, sizeof(state_estimator_t));
    st->qw = 1.0f;
}

void state_estimator_calibrate_step(state_estimator_t *st,
                                    const float gyro_rad_s[3],
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

    st->gyro_bias[0] += gyro_rad_s[0];
    st->gyro_bias[1] += gyro_rad_s[1];
    st->gyro_bias[2] += gyro_rad_s[2];
    st->baro_ground_alt_m += baro_alt_m;

    if (sample_idx + 1 >= total_samples && total_samples > 0) {
        float inv_n = 1.0f / (float)total_samples;
        st->gyro_bias[0] *= inv_n;
        st->gyro_bias[1] *= inv_n;
        st->gyro_bias[2] *= inv_n;
        st->baro_ground_alt_m *= inv_n;
        st->alt_m = 0.0f;
        st->vel_z_mps = 0.0f;
        st->is_calibrated = true;
    }
}

void state_estimator_update(state_estimator_t *st,
                            const float accel_mps2[3],
                            const float gyro_rad_s[3],
                            float baro_alt_m,
                            float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.2f) {
        return; // Protectie la dt invalid
    }

    /* 1. Scadem bias-ul giroscopului */
    float gx = gyro_rad_s[0] - st->gyro_bias[0];
    float gy = gyro_rad_s[1] - st->gyro_bias[1];
    float gz = gyro_rad_s[2] - st->gyro_bias[2];

    /* 2. Integrare cuaternion 3D (q_dot = 0.5 * q (x) omega) */
    float half_dt = 0.5f * dt_s;
    float qw = st->qw;
    float qx = st->qx;
    float qy = st->qy;
    float qz = st->qz;

    st->qw += (-qx * gx - qy * gy - qz * gz) * half_dt;
    st->qx += ( qw * gx + qy * gz - qz * gy) * half_dt;
    st->qy += ( qw * gy - qx * gz + qz * gx) * half_dt;
    st->qz += ( qw * gz + qx * gy - qy * gx) * half_dt;

    /* Normalizare cuaternion (evita eroarea numerica in zbor) */
    float norm = sqrtf(st->qw * st->qw + st->qx * st->qx +
                       st->qy * st->qy + st->qz * st->qz);
    if (norm > 1e-6f) {
        float inv_norm = 1.0f / norm;
        st->qw *= inv_norm;
        st->qx *= inv_norm;
        st->qy *= inv_norm;
        st->qz *= inv_norm;
    }

    /* 3. Proiectia acceleratiei din sistemul rachetei (body) pe axa Z globala (randul 3 din DCM) */
    float r31 = 2.0f * (st->qx * st->qz - st->qw * st->qy);
    float r32 = 2.0f * (st->qy * st->qz + st->qw * st->qx);
    float r33 = 1.0f - 2.0f * (st->qx * st->qx + st->qy * st->qy);

    float az_world = r31 * accel_mps2[0] +
                     r32 * accel_mps2[1] +
                     r33 * accel_mps2[2] - G_ACCEL;

    st->accel_z_world_mps2 = az_world;

    /* Unghiul de inclinare fata de verticala (util in flight_fsm pentru siguranta) */
    float clamped_r33 = (r33 > 1.0f) ? 1.0f : ((r33 < -1.0f) ? -1.0f : r33);
    st->tilt_angle_rad = acosf(clamped_r33);

    /* 4. Integrare trapezoidala pentru viteza si altitudine */
    float prev_vel = st->vel_z_mps;
    st->vel_z_mps += az_world * dt_s;
    st->alt_m     += 0.5f * (prev_vel + st->vel_z_mps) * dt_s;

    /* 5. Logica de lockout barometric in transonic (cu histerezis) */
    float speed = fabsf(st->vel_z_mps);
    if (!st->baro_locked) {
        if (speed >= TRANSONIC_ENTER_V && speed <= SUPERSONIC_EXIT_V) {
            st->baro_locked = true;
        }
    } else {
        if (speed < TRANSONIC_EXIT_V || speed > SUPERSONIC_EXIT_V) {
            st->baro_locked = false;
        }
    }

    if (st->baro_locked) {
        return; // Ramane 100% pe integrala inertiala cat timp e in transonic
    }

    /* 6. Fuziune barometrica (AGL = fata de rampa) in afara zonei transonice */
    float rel_baro_alt = baro_alt_m - st->baro_ground_alt_m;
    float alt_error = rel_baro_alt - st->alt_m;

    const float kp_alt = 0.25f;
    const float kp_vel = 0.08f;

    st->alt_m     += kp_alt * alt_error * dt_s;
    st->vel_z_mps += kp_vel * alt_error * dt_s;
}