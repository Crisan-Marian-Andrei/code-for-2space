#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sd_logger.h"
#include "ads124s06.h"
#include "state_estimator.h"

static const char *TAG = "MAIN_FLIGHT";

// Pinii pentru SPI-ul comun (ADS124S06 + Ra-01H LoRa)
#define SPI_HOST_USED   SPI2_HOST
#define PIN_SPI_MOSI    11
#define PIN_SPI_SCK     12
#define PIN_SPI_MISO    13
#define PIN_LORA_CS     10

#define LOOP_PERIOD_MS  20  // 50 Hz -> destul timp pentru ADC + IMU + baro

typedef enum {
    STATE_PAD_IDLE = 0,
    STATE_FLIGHT   = 1,
    STATE_LANDED   = 2
} flight_state_t;

static bool s_adc_ready = false;

// Date dummy pana legam driverul real de IMU + baro
static void read_imu_baro_stub(float accel_mps2[3], float gyro_rad_s[3], float *baro_alt_m)
{
    accel_mps2[0] = 0.0f;
    accel_mps2[1] = 0.0f;
    accel_mps2[2] = 9.80665f;
    gyro_rad_s[0] = 0.0f;
    gyro_rad_s[1] = 0.0f;
    gyro_rad_s[2] = 0.0f;
    *baro_alt_m   = 0.0f;
}

static esp_err_t init_shared_spi_bus(void)
{
    // Tinem LoRa in HIGH ca sa nu incurce MISO cand vorbim cu ADC-ul
    gpio_config_t cs_cfg = {
        .pin_bit_mask = (1ULL << PIN_LORA_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&cs_cfg));
    gpio_set_level(PIN_LORA_CS, 1);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256
    };

    return spi_bus_initialize(SPI_HOST_USED, &bus_cfg, SPI_DMA_CH_AUTO);
}

static void flight_acq_task(void *arg)
{
    state_estimator_t est;
    state_estimator_init(&est);

    flight_state_t state = STATE_PAD_IDLE;
    float accel[3], gyro[3], baro_alt = 0.0f;
    int32_t strain_raw[STRAIN_CHANNEL_COUNT] = {0};

    // Calibrare 2 secunde pe rampa (100 esantioane la 50 Hz)
    ESP_LOGI(TAG, "Calibrare pe rampa (2s)...");
    const uint16_t cal_samples = 100;
    for (uint16_t i = 0; i < cal_samples; i++) {
        read_imu_baro_stub(accel, gyro, &baro_alt);
        state_estimator_calibrate_step(&est, gyro, baro_alt, i, cal_samples);
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }

    // Aducem marcile tensometrice la 0 inainte de lansare
    if (s_adc_ready) {
        if (ads124s06_tare(32) != ESP_OK) {
            ESP_LOGW(TAG, "Tare ADS124S06 esuat, ramanem pe offset 0");
        }
    }

    ESP_LOGI(TAG, "Gata de lansare (PAD_IDLE). Bucla ruleaza la 50 Hz.");

    int64_t last_us = esp_timer_get_time();
    uint32_t loop_counter = 0;
    uint32_t landed_stable_ticks = 0;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(LOOP_PERIOD_MS));

        int64_t now_us = esp_timer_get_time();
        float dt_s = (float)(now_us - last_us) * 1e-6f;
        last_us = now_us;

        read_imu_baro_stub(accel, gyro, &baro_alt);

        if (s_adc_ready) {
            esp_err_t adc_ret = ads124s06_read_all_channels(strain_raw);
            if (adc_ret != ESP_OK && (loop_counter % 50 == 0)) {
                ESP_LOGW(TAG, "Eroare citire ADS124S06: %s", esp_err_to_name(adc_ret));
            }
        }

        // Actualizam altitudinea si viteza (ignora automat baro in transonic)
        state_estimator_update(&est, accel, gyro, baro_alt, dt_s);

        if (state == STATE_PAD_IDLE) {
            // Peste 2.5g in sus inseamna ca a plecat de pe rampa
            if (est.accel_z_world_mps2 > 24.5f) {
                state = STATE_FLIGHT;
                ESP_LOGI(TAG, "LAUNCH DETECTED! Intrare in STATE_FLIGHT");
            }
        } else if (state == STATE_FLIGHT) {
            // Daca sta sub 15m cu viteza aproape 0 timp de 5s, a aterizat
            if (est.alt_m < 15.0f && est.vel_z_mps > -1.0f && est.vel_z_mps < 1.0f) {
                landed_stable_ticks++;
                if (landed_stable_ticks >= 250) {
                    state = STATE_LANDED;
                    ESP_LOGI(TAG, "LANDING DETECTED! Inchidem fisierul FLIGHT.BIN...");
                    sd_logger_close();
                }
            } else {
                landed_stable_ticks = 0;
            }
        }

        // Bagam datele in ring buffer cat timp suntem inca in zbor/pe rampa
        if (state != STATE_LANDED) {
            flight_log_record_t rec = {
                .timestamp_us = (uint64_t)now_us,
                .strain_ch    = { strain_raw[0], strain_raw[1], strain_raw[2] },
                .flight_state = (uint8_t)state,
                .reserved     = { 0, 0, 0 }
            };
            sd_logger_push_record(&rec);
        }

        // Print de status o data pe secunda
        if (++loop_counter % 50 == 0) {
            float ue0 = ads124s06_raw_to_microstrain(strain_raw[0], 64.0f, 2.05f, 1.0f);
            ESP_LOGI(TAG, "State=%d | Alt=%.1fm | Vz=%.1fm/s | BaroLock=%d | CH0=%.1f uE | Drop=%lu",
                     state, est.alt_m, est.vel_z_mps, est.baro_locked,
                     ue0, (unsigned long)sd_logger_get_dropped_count());
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Boot Avionica 2SPACE (ESP32-S3) ===");

    if (sd_logger_init() != ESP_OK) {
        ESP_LOGE(TAG, "Eroare critica: microSD neinitializat!");
    }

    ESP_ERROR_CHECK(init_shared_spi_bus());

    if (ads124s06_init(SPI_HOST_USED, ADS124_GAIN_64) == ESP_OK) {
        s_adc_ready = true;
    } else {
        ESP_LOGE(TAG, "Atentie: ADS124S06 nu raspunde pe SPI (continuam fara ADC)");
    }

    // Task-ul principal de achizitie sta pe Core 1 (SD-ul scrie separat pe Core 0)
    xTaskCreatePinnedToCore(
        flight_acq_task,
        "flight_acq",
        6144,
        NULL,
        6,
        NULL,
        1
    );
}