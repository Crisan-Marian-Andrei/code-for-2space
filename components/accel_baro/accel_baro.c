#include "accel_baro.h"
#include "bus_manager.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "ACCEL_BARO";

#define I2C_TIMEOUT  pdMS_TO_TICKS(100)

static float s_ground_pressure_pa = 101325.0f;   // sea-level default until set

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

// Sign-extend a 14-bit left-justified sample (KXTJ3 high-resolution mode).
// Bytes are LSB first; the 14 valid bits sit in the top of the 16-bit word.
static int16_t accel_make_sample(uint8_t lsb, uint8_t msb)
{
    int16_t raw = (int16_t)(((uint16_t)msb << 8) | lsb);
    return (int16_t)(raw >> 2);   // drop the 2 unused low bits
}

static esp_err_t accel_init(void)
{
    uint8_t who = 0;
    esp_err_t ret = i2c_read_regs(ACCEL_I2C_ADDR, ACCEL_REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Accelerometer not responding on I2C (addr 0x%02X): %s",
                 ACCEL_I2C_ADDR, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Accel WHO_AM_I = 0x%02X", who);
    if (who != ACCEL_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "Wrong accelerometer id: got 0x%02X, expected 0x%02X (KXTJ3-1057). "
                      "Check the chip / I2C address strap.", who, ACCEL_WHO_AM_I_VALUE);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Soft reset, then wait for it to complete.
    i2c_write_reg(ACCEL_I2C_ADDR, ACCEL_REG_CTRL_REG2, ACCEL_SRST_VALUE);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set output data rate while in standby, then enter operating mode.
    ret = i2c_write_reg(ACCEL_I2C_ADDR, ACCEL_REG_DATA_CTRL, ACCEL_DATA_CTRL_VALUE);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_reg(ACCEL_I2C_ADDR, ACCEL_REG_CTRL_REG1, ACCEL_CTRL_REG1_VALUE);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "Accelerometer (KXTJ3-1057) ready (+/-16 g, 14-bit).");
    return ESP_OK;
}

static esp_err_t baro_init(void)
{
    uint8_t id = 0;
    esp_err_t ret = i2c_read_regs(BARO_I2C_ADDR, BARO_REG_CHIP_ID, &id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Barometer not responding on I2C (addr 0x%02X): %s",
                 BARO_I2C_ADDR, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Barometer CHIP_ID = 0x%02X", id);
    if (id != BARO_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "Wrong barometer id: got 0x%02X, expected 0x%02X (BMP581). "
                      "Check the chip / I2C address strap.", id, BARO_CHIP_ID_VALUE);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Soft reset and wait for the power-on sequence.
    i2c_write_reg(BARO_I2C_ADDR, BARO_REG_CMD, BARO_SOFT_RESET_CMD);
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = i2c_write_reg(BARO_I2C_ADDR, BARO_REG_OSR_CONFIG, BARO_OSR_CONFIG_VALUE);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_reg(BARO_I2C_ADDR, BARO_REG_ODR_CONFIG, BARO_ODR_CONFIG_VALUE);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "Barometer (BMP581) ready.");
    return ESP_OK;
}

esp_err_t accel_baro_init(void)
{
    esp_err_t ret = accel_init();
    if (ret != ESP_OK) return ret;

    ret = baro_init();
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Accelerometer + barometer ready.");
    return ESP_OK;
}

// Read the KXTJ3 and convert to acceleration in m/s^2.
static esp_err_t read_accel(accel_baro_data_t *out)
{
    uint8_t b[6];
    esp_err_t ret = i2c_read_regs(ACCEL_I2C_ADDR, ACCEL_REG_XOUT_L, b, sizeof(b));
    if (ret != ESP_OK) return ret;

    int16_t ax = accel_make_sample(b[0], b[1]);
    int16_t ay = accel_make_sample(b[2], b[3]);
    int16_t az = accel_make_sample(b[4], b[5]);

    out->accel_mps2[0] = ((float)ax / ACCEL_LSB_PER_G) * EARTH_GRAVITY_MPS2;
    out->accel_mps2[1] = ((float)ay / ACCEL_LSB_PER_G) * EARTH_GRAVITY_MPS2;
    out->accel_mps2[2] = ((float)az / ACCEL_LSB_PER_G) * EARTH_GRAVITY_MPS2;
    return ESP_OK;
}

// Read the BMP581 pressure + temperature (datasheet gives the result already
// compensated, so we just scale the raw 24-bit values).
static esp_err_t read_baro(accel_baro_data_t *out)
{
    uint8_t b[6];
    esp_err_t ret = i2c_read_regs(BARO_I2C_ADDR, BARO_REG_TEMP_DATA_XLSB, b, sizeof(b));
    if (ret != ESP_OK) return ret;

    // Temperature: b[0..2], pressure: b[3..5], each LSB first, signed 24-bit.
    int32_t raw_t = (int32_t)(((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | b[0]);
    int32_t raw_p = (int32_t)(((uint32_t)b[5] << 16) | ((uint32_t)b[4] << 8) | b[3]);
    if (raw_t & 0x800000) raw_t |= ~0xFFFFFF;   // sign-extend 24 -> 32 bit
    if (raw_p & 0x800000) raw_p |= ~0xFFFFFF;

    out->temperature_c = (float)raw_t / BARO_TEMP_LSB_PER_C;
    out->pressure_pa   = (float)raw_p / BARO_PRESS_LSB_PER_PA;

    // Barometric altitude relative to the ground reference.
    if (out->pressure_pa > 0.0f && s_ground_pressure_pa > 0.0f) {
        out->altitude_m = 44330.0f *
            (1.0f - powf(out->pressure_pa / s_ground_pressure_pa, 0.1903f));
    } else {
        out->altitude_m = 0.0f;
    }
    return ESP_OK;
}

esp_err_t accel_baro_read(accel_baro_data_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = read_accel(out);
    if (ret != ESP_OK) return ret;
    return read_baro(out);
}

esp_err_t accel_baro_set_ground_reference(uint16_t num_samples)
{
    if (num_samples == 0) {
        num_samples = 20;
    }

    float sum = 0.0f;
    uint16_t good = 0;
    for (uint16_t i = 0; i < num_samples; i++) {
        accel_baro_data_t d = {0};
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
