#include "ads124s06.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "ADS124S06";

#define CMD_NOP     0x00
#define CMD_WAKEUP  0x02
#define CMD_POWERDN 0x04
#define CMD_RESET   0x06
#define CMD_START   0x08
#define CMD_STOP    0x0A
#define CMD_RDATA   0x12
#define CMD_RREG    0x20
#define CMD_WREG    0x40

#define REG_ID      0x00
#define REG_STATUS  0x01
#define REG_INPMUX  0x02
#define REG_PGA     0x03
#define REG_DATARATE 0x04
#define REG_REF     0x05
#define REG_IDACMAG 0x06
#define REG_IDACMUX 0x07
#define REG_VBIAS   0x08
#define REG_SYS     0x09

static const uint8_t s_channel_mux[STRAIN_CHANNEL_COUNT] = {
    0x01, // CH0: AIN0 (+) / AIN1 (-)
    0x23, // CH1: AIN2 (+) / AIN3 (-)
    0x45  // CH2: AIN4 (+) / AIN5 (-)
};

static spi_device_handle_t s_spi = NULL;
static int32_t s_tare_offset[STRAIN_CHANNEL_COUNT] = {0, 0, 0};

static esp_err_t send_command(uint8_t cmd)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_data = {cmd},
        .flags = SPI_TRANS_USE_TXDATA
    };
    return spi_device_polling_transmit(s_spi, &t);
}

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    spi_transaction_t t = {
        .length = 24,
        .tx_data = { (uint8_t)(CMD_WREG | (reg & 0x1F)), 0x00, val },
        .flags = SPI_TRANS_USE_TXDATA
    };
    return spi_device_polling_transmit(s_spi, &t);
}

static esp_err_t read_reg(uint8_t reg, uint8_t *val)
{
    spi_transaction_t t = {
        .length = 24,
        .tx_data = { (uint8_t)(CMD_RREG | (reg & 0x1F)), 0x00, CMD_NOP },
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA
    };
    esp_err_t ret = spi_device_polling_transmit(s_spi, &t);
    if (ret == ESP_OK && val != NULL) {
        *val = t.rx_data[2];
    }
    return ret;
}

static bool wait_drdy_low(uint32_t timeout_ms)
{
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000);
    while (gpio_get_level(ADC_PIN_DRDY) != 0) {
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
        .pin_bit_mask = (1ULL << ADC_PIN_DRDY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&drdy_cfg));

    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << ADC_PIN_RESET),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&rst_cfg));

    gpio_set_level(ADC_PIN_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(ADC_PIN_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10)); // Asteptam stabilizarea oscilatorului intern (~2.2 ms)

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 4 * 1000 * 1000, // 4 MHz
        .mode = 1,                         // ADS124S06 cere strict SPI Mode 1
        .spics_io_num = ADC_PIN_CS,
        .queue_size = 2,
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = 2
    };

    esp_err_t ret = spi_bus_add_device(spi_host, &dev_cfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nu pot adauga ADS124S06 pe SPI: %s", esp_err_to_name(ret));
        return ret;
    }

    send_command(CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t dev_id = 0;
    read_reg(REG_ID, &dev_id);
    ESP_LOGI(TAG, "ADS124S06 REG_ID = 0x%02X", dev_id);
    if ((dev_id & 0x07) != 0x01) {
        ESP_LOGE(TAG, "ID invalid (asteptat 0x01 pentru ADS124S06, primit 0x%02X)", dev_id);
        return ESP_ERR_NOT_FOUND;
    }

    write_reg(REG_STATUS, 0x00);

    uint8_t pga_reg = 0x08 | ((uint8_t)gain & 0x07);
    write_reg(REG_PGA, pga_reg);

    write_reg(REG_DATARATE, 0x38);

    write_reg(REG_REF, 0x00);

    write_reg(REG_IDACMAG, 0x00);
    write_reg(REG_IDACMUX, 0xFF);

    uint8_t pga_check = 0;
    read_reg(REG_PGA, &pga_check);
    if (pga_check != pga_reg) {
        ESP_LOGE(TAG, "Verificare registru PGA esuata (scris 0x%02X, citit 0x%02X)", pga_reg, pga_check);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ADS124S06 initializat cu succes (Gain code: 0x%02X, Sinc3 400 SPS)", gain);
    return ESP_OK;
}

esp_err_t ads124s06_read_channel(uint8_t channel_idx, int32_t *raw_out)
{
    if (s_spi == NULL || channel_idx >= STRAIN_CHANNEL_COUNT || raw_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(write_reg(REG_INPMUX, s_channel_mux[channel_idx]));

    ESP_ERROR_CHECK(send_command(CMD_START));

    if (!wait_drdy_low(20)) {
        ESP_LOGW(TAG, "Timeout DRDY pe canalul %u", channel_idx);
        return ESP_ERR_TIMEOUT;
    }

    spi_transaction_t t = {
        .length = 32,
        .tx_data = { CMD_RDATA, CMD_NOP, CMD_NOP, CMD_NOP },
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA
    };
    esp_err_t ret = spi_device_polling_transmit(s_spi, &t);
    if (ret != ESP_OK) {
        return ret;
    }

    int32_t val = ((int32_t)t.rx_data[1] << 16) |
                  ((int32_t)t.rx_data[2] << 8)  |
                  ((int32_t)t.rx_data[3]);

    if (val & 0x00800000) {
        val |= 0xFF000000;
    }

    *raw_out = val - s_tare_offset[channel_idx];
    return ESP_OK;
}

esp_err_t ads124s06_read_all_channels(int32_t raw_out[STRAIN_CHANNEL_COUNT])
{
    for (uint8_t ch = 0; ch < STRAIN_CHANNEL_COUNT; ch++) {
        esp_err_t ret = ads124s06_read_channel(ch, &raw_out[ch]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

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

    ESP_LOGI(TAG, "Tare complet: CH0=%ld, CH1=%ld, CH2=%ld",
             (long)s_tare_offset[0], (long)s_tare_offset[1], (long)s_tare_offset[2]);
    return ESP_OK;
}

float ads124s06_raw_to_microstrain(int32_t raw_val, float gain_val, float gauge_factor, float active_arms)
{
    /* Formula ratiometrica exacta pentru ADC delta-sigma cu V_REF == V_EXC:
       raw / 2^23 = Gain * (V_diff / V_exc)
       V_diff / V_exc = (active_arms / 4) * GF * epsilon
       => epsilon [uE] = (4.0 * 10^6 * raw) / (2^23 * Gain * GF * active_arms) */
    const float full_scale = 8388608.0f; // 2^23
    return (4000000.0f * (float)raw_val) / (full_scale * gain_val * gauge_factor * active_arms);
}