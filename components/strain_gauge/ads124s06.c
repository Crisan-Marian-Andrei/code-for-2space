#include "ads124s06.h"
#include "board_pins.h"
#include "bus_manager.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char *TAG = "ADS124S06";

// SPI commands (datasheet names).
#define CMD_NOP      0x00
#define CMD_RESET    0x06
#define CMD_START    0x08
#define CMD_STOP     0x0A
#define CMD_RDATA    0x12
#define CMD_RREG     0x20   // OR with register number
#define CMD_WREG     0x40   // OR with register number

// Registers.
#define REG_ID       0x00
#define REG_STATUS   0x01
#define REG_INPMUX   0x02
#define REG_PGA      0x03
#define REG_DATARATE 0x04
#define REG_REF      0x05
#define REG_IDACMAG  0x06
#define REG_IDACMUX  0x07

// INPMUX value per bridge: high nibble = (+) input, low nibble = (-) input.
static const uint8_t s_channel_mux[STRAIN_CHANNEL_COUNT] = {
    0x01,   // AIN0 / AIN1
    0x23,   // AIN2 / AIN3
    0x45    // AIN4 / AIN5
};

static spi_device_handle_t s_spi = NULL;
static int32_t s_tare_offset[STRAIN_CHANNEL_COUNT] = {0, 0, 0};

// Send a single-byte SPI command to the ADC.
static esp_err_t send_command(uint8_t cmd)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_data = { cmd },
        .flags = SPI_TRANS_USE_TXDATA,
    };
    return spi_device_polling_transmit(s_spi, &t);
}

// Register write: [WREG|addr] [count-1] [value].
static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    spi_transaction_t t = {
        .length = 24,
        .tx_data = { (uint8_t)(CMD_WREG | (reg & 0x1F)), 0x00, val },
        .flags = SPI_TRANS_USE_TXDATA,
    };
    return spi_device_polling_transmit(s_spi, &t);
}

// Read one ADC register into *val.
static esp_err_t read_reg(uint8_t reg, uint8_t *val)
{
    spi_transaction_t t = {
        .length = 24,
        .tx_data = { (uint8_t)(CMD_RREG | (reg & 0x1F)), 0x00, CMD_NOP },
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    esp_err_t ret = spi_device_polling_transmit(s_spi, &t);
    if (ret == ESP_OK && val != NULL) {
        *val = t.rx_data[2];
    }
    return ret;
}

// Wait for DRDY LOW (sample ready). Returns false on timeout.
static bool wait_drdy_low(uint32_t timeout_ms)
{
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000);
    while (gpio_get_level(PIN_SG_DRDY) != 0) {
        if (esp_timer_get_time() > deadline_us) {
            return false;
        }
        esp_rom_delay_us(50);
    }
    return true;
}

esp_err_t ads124s06_init(spi_host_device_t spi_host, ads124_gain_t gain)
{
    gpio_config_t drdy_cfg = {
        .pin_bit_mask = (1ULL << PIN_SG_DRDY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&drdy_cfg);
    if (ret != ESP_OK) return ret;

    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << PIN_SG_RESET),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&rst_cfg);
    if (ret != ESP_OK) return ret;

    // Hardware reset, then wait for the internal oscillator to settle.
    gpio_set_level(PIN_SG_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(PIN_SG_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 4 * 1000 * 1000,
        .mode = 1,
        .spics_io_num = PIN_SG_CS,
        .queue_size = 2,
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = 2,
    };
    ret = spi_bus_add_device(spi_host, &dev_cfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Could not add ADS124S06 to SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    bus_lock_spi();

    send_command(CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t dev_id = 0;
    read_reg(REG_ID, &dev_id);
    ESP_LOGI(TAG, "ADS124S06 REG_ID = 0x%02X", dev_id);
    if ((dev_id & 0x07) != 0x01) {
        ESP_LOGE(TAG, "Wrong/absent chip id (wanted low-bits 0x01, got 0x%02X)", dev_id);
        bus_unlock_spi();
        return ESP_ERR_NOT_FOUND;
    }

    write_reg(REG_STATUS, 0x00);

    uint8_t pga_reg = 0x08 | ((uint8_t)gain & 0x07);   // bit3 enables PGA
    write_reg(REG_PGA, pga_reg);

    write_reg(REG_DATARATE, 0x38);   // continuous, sinc3, ~400 SPS
    write_reg(REG_REF, 0x00);        // internal 2.5V reference, always on
    write_reg(REG_IDACMAG, 0x00);    // excitation current off
    write_reg(REG_IDACMUX, 0xFF);

    uint8_t pga_check = 0;
    read_reg(REG_PGA, &pga_check);

    bus_unlock_spi();

    if (pga_check != pga_reg) {
        ESP_LOGE(TAG, "PGA readback mismatch (wrote 0x%02X, read 0x%02X)", pga_reg, pga_check);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ADS124S06 ready (gain code 0x%02X, internal 2.5V ref).", gain);
    return ESP_OK;
}

// Caller must already hold the SPI lock.
static esp_err_t read_one_channel_locked(uint8_t channel_idx, int32_t *raw_out)
{
    esp_err_t ret;

    ret = write_reg(REG_INPMUX, s_channel_mux[channel_idx]);
    if (ret != ESP_OK) return ret;

    ret = send_command(CMD_START);
    if (ret != ESP_OK) return ret;

    if (!wait_drdy_low(20)) {
        ESP_LOGW(TAG, "DRDY timeout on channel %u", channel_idx);
        return ESP_ERR_TIMEOUT;
    }

    spi_transaction_t t = {
        .length = 32,   // RDATA + 3 data bytes
        .tx_data = { CMD_RDATA, CMD_NOP, CMD_NOP, CMD_NOP },
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    ret = spi_device_polling_transmit(s_spi, &t);
    if (ret != ESP_OK) return ret;

    int32_t val = ((int32_t)t.rx_data[1] << 16) |
                  ((int32_t)t.rx_data[2] << 8)  |
                  ((int32_t)t.rx_data[3]);

    // Sign-extend 24-bit two's complement to 32-bit.
    if (val & 0x00800000) {
        val |= 0xFF000000;
    }

    *raw_out = val - s_tare_offset[channel_idx];
    return ESP_OK;
}

// Read one strain channel (takes the SPI lock for the call).
esp_err_t ads124s06_read_channel(uint8_t channel_idx, int32_t *raw_out)
{
    if (s_spi == NULL || channel_idx >= STRAIN_CHANNEL_COUNT || raw_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    bus_lock_spi();
    esp_err_t ret = read_one_channel_locked(channel_idx, raw_out);
    bus_unlock_spi();
    return ret;
}

// Read every strain channel in one locked cycle.
esp_err_t ads124s06_read_all_channels(int32_t raw_out[STRAIN_CHANNEL_COUNT])
{
    if (s_spi == NULL || raw_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Lock once for the whole cycle so LoRa cannot interleave.
    bus_lock_spi();
    esp_err_t ret = ESP_OK;
    for (uint8_t ch = 0; ch < STRAIN_CHANNEL_COUNT; ch++) {
        ret = read_one_channel_locked(ch, &raw_out[ch]);
        if (ret != ESP_OK) {
            break;
        }
    }
    bus_unlock_spi();
    return ret;
}

// Average many samples per channel to set the zero (tare) offsets.
esp_err_t ads124s06_tare(uint16_t num_samples)
{
    if (num_samples == 0) {
        num_samples = 32;
    }

    memset(s_tare_offset, 0, sizeof(s_tare_offset));

    int64_t acc[STRAIN_CHANNEL_COUNT] = {0, 0, 0};
    for (uint16_t i = 0; i < num_samples; i++) {
        int32_t sample[STRAIN_CHANNEL_COUNT];
        esp_err_t ret = ads124s06_read_all_channels(sample);
        if (ret != ESP_OK) {
            return ret;
        }
        for (uint8_t ch = 0; ch < STRAIN_CHANNEL_COUNT; ch++) {
            acc[ch] += sample[ch];
        }
    }

    for (uint8_t ch = 0; ch < STRAIN_CHANNEL_COUNT; ch++) {
        s_tare_offset[ch] = (int32_t)(acc[ch] / num_samples);
    }

    ESP_LOGI(TAG, "Tare done: CH0=%ld CH1=%ld CH2=%ld",
             (long)s_tare_offset[0], (long)s_tare_offset[1], (long)s_tare_offset[2]);
    return ESP_OK;
}

// volts = (raw / 2^23) * (Vref / gain); returned as millivolts.
float ads124s06_raw_to_millivolts(int32_t raw_val, float gain_multiplier)
{
    const float v_ref = 2.5f;
    const float full_scale = 8388608.0f;   // 2^23
    if (gain_multiplier <= 0.0f) {
        gain_multiplier = 1.0f;
    }
    float volts = ((float)raw_val / full_scale) * (v_ref / gain_multiplier);
    return volts * 1000.0f;
}

// epsilon[uE] = 4e6 * raw / (2^23 * gain * gauge_factor * active_arms)
float ads124s06_raw_to_microstrain(int32_t raw_val, float gain_val,
                                    float gauge_factor, float active_arms)
{
    const float full_scale = 8388608.0f;
    float denom = full_scale * gain_val * gauge_factor * active_arms;
    if (denom == 0.0f) {
        return 0.0f;
    }
    return (4000000.0f * (float)raw_val) / denom;
}
