#pragma once

// Accelerometer (KXTJ3-1057) + barometer (BMP581) over I2C0 at 400 kHz.
// These are the only two I2C sensors on the board. No gyro, no magnetometer.
// Change the addresses/registers below (and the read maths in the .c) to
// retarget to another accelerometer / barometer.

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// -------- Accelerometer: Kionix KXTJ3-1057 --------
// I2C address depends on the ADDR strap pin:
//   ADDR tied low  -> 0x0E
//   ADDR tied high -> 0x0F
// Set this to match the schematic. (Confirm from the netlist.)
#ifndef ACCEL_I2C_ADDR
#define ACCEL_I2C_ADDR        0x0E
#endif

#define ACCEL_REG_WHO_AM_I    0x0F
#define ACCEL_WHO_AM_I_VALUE  0x35   // KXTJ3-1057 fixed WAI value
#define ACCEL_REG_CTRL_REG1   0x1B   // PC1 (operate), RES, GSEL, EN16G, g-range
#define ACCEL_REG_CTRL_REG2   0x1D   // soft reset (SRST = bit7)
#define ACCEL_REG_DATA_CTRL   0x21   // output data rate
#define ACCEL_REG_XOUT_L      0x06   // start of 6-byte burst: X, Y, Z (LSB first)

// CTRL_REG1 for +/-16 g in 14-bit high-resolution mode.
//   bit7 PC1   = 1  (operating mode)
//   bit6 RES   = 1  (high resolution, 14-bit)
//   bit5 DRDYE = 0  (data-ready interrupt off; we poll)
//   bit4 GSEL1 = 1  } GSEL = 11 with EN16G = 1 selects the +/-16 g,
//   bit3 GSEL0 = 1  } 14-bit range (2048 counts per g)
//   bit2 EN16G = 1  (enable the extended +/-16 g range)
//   bits[1:0]  = 0  (wake-up / unused)
// => 0b1101_1100 = 0xDC
#define ACCEL_CTRL_REG1_VALUE 0xDC
// DATA_CTRL: ODR = 100 Hz (OSA[3:0] = 0x03).
#define ACCEL_DATA_CTRL_VALUE 0x03
// Soft reset command for CTRL_REG2.
#define ACCEL_SRST_VALUE      0x80

// 14-bit high-resolution mode at +/-16 g: 2048 counts per g (KXTJ3 datasheet
// Table 1, 14-bit / 16g row). Scale raw -> g with (raw / 2048).
#define ACCEL_LSB_PER_G       2048.0f
#define EARTH_GRAVITY_MPS2    9.80665f

// -------- Barometer: Bosch BMP581 --------
// I2C address depends on the SDO strap pin:
//   SDO tied low  -> 0x46
//   SDO tied high -> 0x47
// Set this to match the schematic. (Confirm from the netlist.)
#ifndef BARO_I2C_ADDR
#define BARO_I2C_ADDR         0x46
#endif

#define BARO_REG_CHIP_ID      0x01
#define BARO_CHIP_ID_VALUE    0x50   // BMP581 fixed chip id
#define BARO_REG_CMD          0x7E   // soft reset command register
#define BARO_SOFT_RESET_CMD   0xB6
#define BARO_REG_OSR_CONFIG   0x36   // pressure + temperature oversampling, press enable
#define BARO_REG_ODR_CONFIG   0x37   // output data rate + power mode
#define BARO_REG_TEMP_DATA_XLSB 0x1D // start of 6-byte burst: T[2:0], P[2:0] (LSB first)

// OSR_CONFIG: enable pressure (PRESS_EN=bit6), press OSR x4 (bits[5:3]=010),
// temp OSR x1 (bits[2:0]=000).
#define BARO_OSR_CONFIG_VALUE 0x50
// ODR_CONFIG: normal power mode (PWR_MODE bits[1:0]=11), ODR ~50 Hz
// (ODR bits[6:2]=0x10 per datasheet table).
#define BARO_ODR_CONFIG_VALUE 0x43

// BMP581 output scaling (datasheet): pressure = raw / 2^6 Pa,
// temperature = raw / 2^16 degC. raw is a signed 24-bit (LSB-first) value.
#define BARO_PRESS_LSB_PER_PA 64.0f       // 2^6
#define BARO_TEMP_LSB_PER_C   65536.0f    // 2^16

typedef struct {
    float accel_mps2[3];   // X, Y, Z
    float pressure_pa;
    float temperature_c;
    float altitude_m;      // relative to the startup ground reference
} accel_baro_data_t;

// Set up both chips. Needs bus_manager_init first.
esp_err_t accel_baro_init(void);

// Average num_samples pressure readings and store as "0 m" (pass 0 for default).
esp_err_t accel_baro_set_ground_reference(uint16_t num_samples);

// Read everything once (accelerometer + barometer).
esp_err_t accel_baro_read(accel_baro_data_t *out);

#ifdef __cplusplus
}
#endif
