#pragma once

// Driver for the TI ADS124S06 24-bit strain-gauge ADC (3 bridges, SPI mode 1).
// Shares the SPI bus with LoRa; transfers use the bus_manager SPI lock.

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STRAIN_CHANNEL_COUNT  3

// PGA gain codes (multiplies the bridge voltage before conversion).
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

// Reset, verify id, select internal 2.5V ref + PGA gain. Needs bus_manager_init first.
esp_err_t ads124s06_init(spi_host_device_t spi_host, ads124_gain_t gain);

// Read one bridge (0..2), sign-extended, zero-offset removed.
esp_err_t ads124s06_read_channel(uint8_t channel_idx, int32_t *raw_out);

// Read all three bridges.
esp_err_t ads124s06_read_all_channels(int32_t raw_out[STRAIN_CHANNEL_COUNT]);

// Average num_samples readings per bridge and store as the zero (pass 0 for default).
esp_err_t ads124s06_tare(uint16_t num_samples);

// raw -> differential voltage in millivolts, given the gain multiplier.
float ads124s06_raw_to_millivolts(int32_t raw_val, float gain_multiplier);

// raw -> microstrain for a Wheatstone bridge.
float ads124s06_raw_to_microstrain(int32_t raw_val, float gain_val,
                                    float gauge_factor, float active_arms);

#ifdef __cplusplus
}
#endif
