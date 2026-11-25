#include "imu_baro.h"
#include "bus_manager.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "IMU_BARO";

#define I2C_TIMEOUT  pdMS_TO_TICKS(100)

static float s_ground_pressure_pa = 101325.0f;   // sea-level default until set

// BMP280 factory calibration constants + t_fine link term (datasheet).
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t  t_fine;

// Write one byte to a device register over I2C.
static esp_err_t i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    bus_lock_i2c();
    esp_err_t ret = i2c_master_write_to_device(BOARD_I2C_PORT, dev_addr,
                                                buf, sizeof(buf), I2C_TIMEOUT);
    bus_unlock_i2c();
    return ret;
}

// Read len bytes starting at a device register over I2C.
static esp_err_t i2c_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *dst, size_t len)
{
    bus_lock_i2c();
    esp_err_t ret = i2c_master_write_read_device(BOARD_I2C_PORT, dev_addr,
                                                 &reg, 1, dst, len, I2C_TIMEOUT);
    bus_unlock_i2c();
    return ret;
}

// Combine a high and low byte into a signed 16-bit value.
static int16_t make_int16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

// Read and unpack the 24 BMP280 calibration bytes (little-endian).
static esp_err_t baro_read_calibration(void)
{
    uint8_t c[24];
    esp_err_t ret = i2c_read_regs(BARO_I2C_ADDR, BARO_REG_CALIB_00, c, sizeof(c));
    if (ret != ESP_OK) return ret;

    dig_T1 = (uint16_t)(c[1] << 8 | c[0]);
    dig_T2 = make_int16(c[3], c[2]);
    dig_T3 = make_int16(c[5], c[4]);
    dig_P1 = (uint16_t)(c[7] << 8 | c[6]);
    dig_P2 = make_int16(c[9], c[8]);
    dig_P3 = make_int16(c[11], c[10]);
    dig_P4 = make_int16(c[13], c[12]);
    dig_P5 = make_int16(c[15], c[14]);
    dig_P6 = make_int16(c[17], c[16]);
    dig_P7 = make_int16(c[19], c[18]);
    dig_P8 = make_int16(c[21], c[20]);
    dig_P9 = make_int16(c[23], c[22]);
    return ESP_OK;
}

// Configure the QMC5883L magnetometer for continuous 200 Hz output.
static esp_err_t mag_init(void)
{
    // Soft reset first so we always start from a known state.
    i2c_write_reg(MAG_I2C_ADDR, MAG_REG_CONTROL_2, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Datasheet: write 0x01 to the SET/RESET period register for normal use.
    esp_err_t ret = i2c_write_reg(MAG_I2C_ADDR, MAG_REG_SET_RESET, 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Magnetometer not responding on I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    // Continuous mode, 200 Hz, +/-8 Gauss, OSR 512.
    ret = i2c_write_reg(MAG_I2C_ADDR, MAG_REG_CONTROL_1, MAG_CONTROL_1_VALUE);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Magnetometer (QMC5883L) ready.");
    return ESP_OK;
}

esp_err_t imu_baro_init(void)
{
    esp_err_t ret;

    // IMU (MPU6050)
    uint8_t who = 0;
    ret = i2c_read_regs(IMU_I2C_ADDR, IMU_REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU not responding on I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "IMU WHO_AM_I = 0x%02X", who);
    if (who != IMU_WHO_AM_I_VALUE) {
        ESP_LOGW(TAG, "Unexpected IMU id (wanted 0x%02X).", IMU_WHO_AM_I_VALUE);
    }

    ret = i2c_write_reg(IMU_I2C_ADDR, IMU_REG_PWR_MGMT_1, 0x00);   // wake up
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    i2c_write_reg(IMU_I2C_ADDR, IMU_REG_ACCEL_CONFIG, 0x00);   // +/-2g
    i2c_write_reg(IMU_I2C_ADDR, IMU_REG_GYRO_CONFIG, 0x00);    // +/-250 dps

    // Barometer (BMP280)
    uint8_t baro_id = 0;
    ret = i2c_read_regs(BARO_I2C_ADDR, BARO_REG_ID, &baro_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Barometer not responding on I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Barometer ID = 0x%02X", baro_id);
    if (baro_id != BARO_ID_VALUE) {
        ESP_LOGW(TAG, "Unexpected barometer id (wanted 0x%02X).", BARO_ID_VALUE);
    }

    ret = baro_read_calibration();
    if (ret != ESP_OK) return ret;

    i2c_write_reg(BARO_I2C_ADDR, BARO_REG_CTRL_MEAS, 0x27);   // osr x1, normal mode
    i2c_write_reg(BARO_I2C_ADDR, BARO_REG_CONFIG, 0x10);      // small IIR filter
    vTaskDelay(pdMS_TO_TICKS(50));

    // Magnetometer (QMC5883L). If it fails we still continue: heading just
    // won't be corrected, but gyro-only attitude keeps working.
    if (mag_init() != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without magnetometer (heading will drift).");
    }

    ESP_LOGI(TAG, "IMU + barometer ready.");
    return ESP_OK;
}

// Read the MPU6050 and convert to accel (m/s^2) and gyro (deg/s).
static esp_err_t read_imu(imu_baro_data_t *out)
{
    uint8_t b[14];
    esp_err_t ret = i2c_read_regs(IMU_I2C_ADDR, IMU_REG_ACCEL_XOUT_H, b, sizeof(b));
    if (ret != ESP_OK) return ret;

    // b[0..5]=accel XYZ, b[6..7]=temp, b[8..13]=gyro XYZ.
    int16_t ax = make_int16(b[0], b[1]);
    int16_t ay = make_int16(b[2], b[3]);
    int16_t az = make_int16(b[4], b[5]);
    int16_t gx = make_int16(b[8], b[9]);
    int16_t gy = make_int16(b[10], b[11]);
    int16_t gz = make_int16(b[12], b[13]);

    out->accel_mps2[0] = ((float)ax / IMU_ACCEL_LSB_PER_G) * EARTH_GRAVITY_MPS2;
    out->accel_mps2[1] = ((float)ay / IMU_ACCEL_LSB_PER_G) * EARTH_GRAVITY_MPS2;
    out->accel_mps2[2] = ((float)az / IMU_ACCEL_LSB_PER_G) * EARTH_GRAVITY_MPS2;

    out->gyro_dps[0] = (float)gx / IMU_GYRO_LSB_PER_DPS;
    out->gyro_dps[1] = (float)gy / IMU_GYRO_LSB_PER_DPS;
    out->gyro_dps[2] = (float)gz / IMU_GYRO_LSB_PER_DPS;

    return ESP_OK;
}

// BMP280 compensation (datasheet section 3.11.3).
static esp_err_t read_baro(imu_baro_data_t *out)
{
    uint8_t b[6];
    esp_err_t ret = i2c_read_regs(BARO_I2C_ADDR, BARO_REG_PRESS_MSB, b, sizeof(b));
    if (ret != ESP_OK) return ret;

    int32_t adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
    int32_t adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    float temp_c = ((t_fine * 5 + 128) >> 8) / 100.0f;

    int64_t p_var1 = ((int64_t)t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) +
             ((p_var1 * (int64_t)dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dig_P1) >> 33;

    float pressure_pa;
    if (p_var1 == 0) {
        pressure_pa = 0.0f;
    } else {
        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - p_var2) * 3125) / p_var1;
        p_var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p_var2 = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dig_P7) << 4);
        pressure_pa = (float)p / 256.0f;
    }

    out->temperature_c = temp_c;
    out->pressure_pa = pressure_pa;

    // Barometric altitude relative to the ground reference.
    if (pressure_pa > 0.0f && s_ground_pressure_pa > 0.0f) {
        out->altitude_m = 44330.0f * (1.0f - powf(pressure_pa / s_ground_pressure_pa, 0.1903f));
    } else {
        out->altitude_m = 0.0f;
    }
    return ESP_OK;
}

// Read the QMC5883L magnetometer and convert to microtesla (uT).
// Returns ESP_OK and sets mag_valid=true only when a fresh sample was ready.
static esp_err_t read_mag(imu_baro_data_t *out)
{
    out->mag_valid = false;

    // Check the STATUS register: bit0 (DRDY) is 1 when new data is ready.
    uint8_t status = 0;
    esp_err_t ret = i2c_read_regs(MAG_I2C_ADDR, MAG_REG_STATUS, &status, 1);
    if (ret != ESP_OK) return ret;
    if ((status & 0x01) == 0) {
        return ESP_OK;   // no new sample yet; keep the old values, mark invalid
    }

    // Burst-read the 6 data bytes: X, Y, Z, each little-endian (LSB first).
    uint8_t b[6];
    ret = i2c_read_regs(MAG_I2C_ADDR, MAG_REG_DATA_X_LSB, b, sizeof(b));
    if (ret != ESP_OK) return ret;

    int16_t raw_x = (int16_t)(((uint16_t)b[1] << 8) | b[0]);
    int16_t raw_y = (int16_t)(((uint16_t)b[3] << 8) | b[2]);
    int16_t raw_z = (int16_t)(((uint16_t)b[5] << 8) | b[4]);

    // raw -> Gauss -> microtesla, then remove the hard-iron offset.
    out->mag_ut[0] = ((float)raw_x / MAG_LSB_PER_GAUSS) * MAG_GAUSS_TO_UT - MAG_HARD_IRON_X_UT;
    out->mag_ut[1] = ((float)raw_y / MAG_LSB_PER_GAUSS) * MAG_GAUSS_TO_UT - MAG_HARD_IRON_Y_UT;
    out->mag_ut[2] = ((float)raw_z / MAG_LSB_PER_GAUSS) * MAG_GAUSS_TO_UT - MAG_HARD_IRON_Z_UT;

    out->mag_valid = true;
    return ESP_OK;
}

// Read IMU and barometer in one call, filling the output struct.
esp_err_t imu_baro_read(imu_baro_data_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = read_imu(out);
    if (ret != ESP_OK) return ret;
    ret = read_baro(out);
    if (ret != ESP_OK) return ret;
    // Magnetometer failure is non-fatal: leave mag_valid=false and carry on.
    read_mag(out);
    return ESP_OK;
}

// Average several pressure samples to fix the 0 m ground reference.
esp_err_t imu_baro_set_ground_reference(uint16_t num_samples)
{
    if (num_samples == 0) {
        num_samples = 20;
    }

    float sum = 0.0f;
    uint16_t good = 0;
    for (uint16_t i = 0; i < num_samples; i++) {
        imu_baro_data_t d;
        if (read_baro(&d) == ESP_OK && d.pressure_pa > 0.0f) {
            sum += d.pressure_pa;
            good++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (good == 0) {
        ESP_LOGE(TAG, "Could not read ground pressure.");
        return ESP_FAIL;
    }

    s_ground_pressure_pa = sum / (float)good;
    ESP_LOGI(TAG, "Ground pressure set to %.1f Pa (0 m reference).", s_ground_pressure_pa);
    return ESP_OK;
}
