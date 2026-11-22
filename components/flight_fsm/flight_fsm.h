#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Stare estimata (axa Z globala = verticala in sus) */
    float alt_m;              // Altitudinea estimata fata de rampa [m]
    float vel_z_mps;          // Viteza verticala estimata [m/s]
    float accel_z_world_mps2; // Accelerația verticala neta (fara gravitatie) [m/s^2]

    /* Orientare 3D (cuaternion body -> world) */
    float qw, qx, qy, qz;
    float tilt_angle_rad;     // Unghiul fata de verticala (0 = drept in sus) [rad]

    /* Bias-uri calibrate pe rampa */
    float gyro_bias[3];       // [rad/s]
    float baro_ground_alt_m;  // Altitudinea barometrica de referinta la rampa [m]
    bool  is_calibrated;

    /* Stare lockout transonic */
    bool  baro_locked;
} state_estimator_t;

void state_estimator_init(state_estimator_t *st);

void state_estimator_calibrate_step(state_estimator_t *st,
                                    const float gyro_rad_s[3],
                                    float baro_alt_m,
                                    uint16_t sample_idx,
                                    uint16_t total_samples);

void state_estimator_update(state_estimator_t *st,
                            const float accel_mps2[3],
                            const float gyro_rad_s[3],
                            float baro_alt_m,
                            float dt_s);

#ifdef __cplusplus
}
#endif