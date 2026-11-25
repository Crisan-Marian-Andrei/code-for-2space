#include "lora.h"
#include "board_pins.h"
#include "bus_manager.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "LORA";

// SX1276 registers.
#define REG_FIFO                0x00
#define REG_OP_MODE             0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_PA_CONFIG           0x09
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_IRQ_FLAGS           0x12
#define REG_PAYLOAD_LENGTH      0x22
#define REG_MODEM_CONFIG_1      0x1D
#define REG_MODEM_CONFIG_2      0x1E
#define REG_VERSION             0x42   // reads 0x12 on a real SX1276

// OP_MODE values (bit7 = LoRa mode).
#define MODE_LONG_RANGE         0x80
#define MODE_SLEEP              0x00
#define MODE_STANDBY            0x01
#define MODE_TX                 0x03

#define IRQ_TX_DONE_MASK        0x08
#define FREQ_868_FRF            0xD90000   // 868.0 MHz (Frf = freq / Fstep)

static spi_device_handle_t s_spi = NULL;

// Caller holds the SPI lock. Top address bit: 1 = write.
static void lora_write_reg(uint8_t reg, uint8_t value)
{
    spi_transaction_t t = {
        .length = 16,
        .tx_data = { (uint8_t)(reg | 0x80), value },
        .flags = SPI_TRANS_USE_TXDATA,
    };
    spi_device_polling_transmit(s_spi, &t);
}

// Read one SX1276 register (caller holds the SPI lock).
static uint8_t lora_read_reg(uint8_t reg)
{
    spi_transaction_t t = {
        .length = 16,
        .tx_data = { (uint8_t)(reg & 0x7F), 0x00 },
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    spi_device_polling_transmit(s_spi, &t);
    return t.rx_data[1];
}

esp_err_t lora_init(spi_host_device_t spi_host)
{
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << PIN_LORA_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&rst_cfg);
    if (ret != ESP_OK) return ret;

    gpio_set_level(PIN_LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // DIO0 (TxDone) read by polling.
    gpio_config_t dio0_cfg = {
        .pin_bit_mask = (1ULL << PIN_LORA_DIO0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&dio0_cfg);
    if (ret != ESP_OK) return ret;

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_LORA_CS,
        .queue_size = 2,
    };
    ret = spi_bus_add_device(spi_host, &dev_cfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Could not add LoRa to SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    bus_lock_spi();

    uint8_t version = lora_read_reg(REG_VERSION);
    ESP_LOGI(TAG, "LoRa REG_VERSION = 0x%02X (expect 0x12)", version);
    if (version != 0x12) {
        bus_unlock_spi();
        ESP_LOGE(TAG, "LoRa radio not detected.");
        return ESP_ERR_NOT_FOUND;
    }

    // Must be in sleep to switch into LoRa mode.
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));

    lora_write_reg(REG_FRF_MSB, (FREQ_868_FRF >> 16) & 0xFF);
    lora_write_reg(REG_FRF_MID, (FREQ_868_FRF >> 8) & 0xFF);
    lora_write_reg(REG_FRF_LSB, (FREQ_868_FRF >> 0) & 0xFF);

    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);   // BW 125 kHz, CR 4/5, explicit header
    lora_write_reg(REG_MODEM_CONFIG_2, 0x94);   // SF9, CRC on
    lora_write_reg(REG_PA_CONFIG, 0x8F);        // PA_BOOST near max power

    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STANDBY);

    bus_unlock_spi();

    ESP_LOGI(TAG, "LoRa ready: 868 MHz, SF9, BW 125 kHz, CRC on.");
    return ESP_OK;
}

// CCITT CRC-16 (poly 0x1021, init 0xFFFF) over a byte buffer.
uint16_t lora_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// Stamp the CRC, load the FIFO, transmit, and wait for TxDone.
esp_err_t lora_send_packet(telemetry_packet_t *packet)
{
    if (s_spi == NULL || packet == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // CRC over every byte except the crc16 field itself.
    size_t crc_covered = sizeof(telemetry_packet_t) - sizeof(packet->crc16);
    packet->crc16 = lora_crc16((const uint8_t *)packet, crc_covered);

    bus_lock_spi();

    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STANDBY);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    const uint8_t *bytes = (const uint8_t *)packet;
    for (size_t i = 0; i < sizeof(telemetry_packet_t); i++) {
        lora_write_reg(REG_FIFO, bytes[i]);
    }

    lora_write_reg(REG_PAYLOAD_LENGTH, (uint8_t)sizeof(telemetry_packet_t));
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE | MODE_TX);

    // Poll TxDone with a 500 ms timeout; release the bus while waiting.
    esp_err_t result = ESP_OK;
    int64_t deadline_us = esp_timer_get_time() + 500 * 1000;
    while ((lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
        if (esp_timer_get_time() > deadline_us) {
            result = ESP_ERR_TIMEOUT;
            break;
        }
        bus_unlock_spi();
        vTaskDelay(pdMS_TO_TICKS(2));
        bus_lock_spi();
    }

    lora_write_reg(REG_IRQ_FLAGS, 0xFF);   // clear flags

    bus_unlock_spi();

    if (result == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "TxDone timeout.");
    }
    return result;
}
