#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_PIN_CS      4
#define ADC_PIN_DRDY    7
#define ADC_PIN_RESET   8

#define STRAIN_CHANNEL_COUNT  3

typedef enum {
    ADS124_GAIN_1   = 0x00,
    ADS124_GAIN_2   = 0x01,
    ADS124_GAIN_4   = 0x02,
    ADS124_GAIN_8   = 0x03,
    ADS124_GAIN_16  = 0x04,
    ADS124_GAIN_32  = 0x05,
    ADS124_GAIN_64  = 0x06,
    ADS124_GAIN_128 = 0x07
} ads124_gain_t;

esp_err_t ads124s06_init(spi_host_device_t spi_host, ads124_gain_t gain);

esp_err_t ads124s06_read_channel(uint8_t channel_idx, int32_t *raw_out);

esp_err_t ads124s06_read_all_channels(int32_t raw_out[STRAIN_CHANNEL_COUNT]);

esp_err_t ads124s06_tare(uint16_t num_samples);

float ads124s06_raw_to_microstrain(int32_t raw_val, float gain_val, float gauge_factor, float active_arms);

#ifdef __cplusplus
}
#endif