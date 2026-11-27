// 2SPACE rocket avionics firmware entry point (ESP32-S3). See README.md.
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "board_pins.h"
#include "bus_manager.h"
#include "ads124s06.h"
#include "accel_baro.h"
#include "lora.h"
#include "sd_logger.h"
#include "state_estimator.h"
#include "flight_fsm.h"

static const char *TAG = "MAIN";

#define ADC_GAIN_CODE        ADS124_GAIN_64
#define ADC_GAIN_MULTIPLIER  64.0f

#define SENSOR_LOOP_MS       20          // 50 Hz

#define LORA_PERIOD_FLIGHT_MS  200
#define LORA_PERIOD_PAD_MS     1000

// Shared data between tasks; take s_data_mutex before touching these.
static SemaphoreHandle_t s_data_mutex = NULL;

static bool            s_adc_ready = false;
static bool            s_lora_ready = false;

static flight_state_t  s_flight_state = STATE_PAD_IDLE;
static int32_t         s_strain_raw[STRAIN_CHANNEL_COUNT] = {0};
static float           s_accel_z_mps2 = 0.0f;
static float           s_altitude_m = 0.0f;
static float           s_vel_z_ms = 0.0f;
static float           s_pressure_pa = 0.0f;
static float           s_temperature_c = 0.0f;
static bool            s_baro_locked_out = false;

static void lock_data(void)   { xSemaphoreTake(s_data_mutex, portMAX_DELAY); }
static void unlock_data(void) { xSemaphoreGive(s_data_mutex); }

// Core 1: read sensors, run estimator + FSM, push a log record each loop.
static void sensor_and_fsm_task(void *arg)
{
    state_estimator_t est;
    flight_fsm_t      fsm;
    state_estimator_init(&est);
    flight_fsm_init(&fsm);

    // Pad calibration: set ground = 0 m and the strain zero.
    ESP_LOGI(TAG, "Calibrating on the pad (%d samples)...", FSM_CALIBRATION_SAMPLES);
    for (uint16_t i = 0; i < FSM_CALIBRATION_SAMPLES; i++) {
        accel_baro_data_t d = {0};
        accel_baro_read(&d);
        state_estimator_calibrate_step(&est, d.accel_mps2, d.altitude_m,
                                       i, FSM_CALIBRATION_SAMPLES);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_LOOP_MS));
    }

    if (s_adc_ready) {
        if (ads124s06_tare(32) != ESP_OK) {
            ESP_LOGW(TAG, "Strain-gauge tare failed; using raw zero.");
        }
    }
    ESP_LOGI(TAG, "Calibration done. Ready for launch (PAD_IDLE).");

    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_us = esp_timer_get_time();
    uint32_t loop_count = 0;
    static bool s_log_closed = false;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_LOOP_MS));

        int64_t now_us = esp_timer_get_time();
        float    dt_s  = (float)(now_us - last_us) * 1e-6f;
        uint32_t dt_ms = (uint32_t)((now_us - last_us) / 1000);
        last_us = now_us;

        accel_baro_data_t sensors = {0};
        accel_baro_read(&sensors);

        int32_t strain[STRAIN_CHANNEL_COUNT] = {0};
        if (s_adc_ready) {
            ads124s06_read_all_channels(strain);
        }

        state_estimator_update(&est, sensors.accel_mps2, sensors.altitude_m, dt_s);
        flight_state_t state = flight_fsm_update(&fsm, &est, dt_ms);

        lock_data();
        s_flight_state = state;
        for (int i = 0; i < STRAIN_CHANNEL_COUNT; i++) {
            s_strain_raw[i] = strain[i];
        }
        s_accel_z_mps2    = est.accel_z_world_mps2;
        s_altitude_m      = est.alt_m;
        s_vel_z_ms        = est.vel_z_mps;
        s_pressure_pa     = sensors.pressure_pa;
        s_temperature_c   = sensors.temperature_c;
        s_baro_locked_out = est.is_baro_locked_out;
        unlock_data();

        if (state != STATE_LANDED) {
            flight_log_record_t rec = {0};
            rec.timestamp    = (uint64_t)now_us;
            rec.strain_ch[0] = strain[0];
            rec.strain_ch[1] = strain[1];
            rec.strain_ch[2] = strain[2];
            rec.accel_z_mps2 = est.accel_z_world_mps2;
            rec.altitude_m   = est.alt_m;
            rec.vel_z_ms           = est.vel_z_mps;
            rec.pressure_pa        = sensors.pressure_pa;
            rec.temperature_c      = sensors.temperature_c;
            rec.flight_state       = (uint8_t)state;
            rec.is_baro_locked_out = est.is_baro_locked_out ? 1 : 0;
            sd_logger_push_record(&rec);
        }

        if (state == STATE_LANDED && !s_log_closed) {
            ESP_LOGI(TAG, "Landed. Flushing and closing the SD log.");
            sd_logger_close();
            s_log_closed = true;
        }

        if (++loop_count % 50 == 0) {
            float mv0 = ads124s06_raw_to_millivolts(strain[0], ADC_GAIN_MULTIPLIER);
            ESP_LOGI(TAG, "[%s] alt=%.1fm vz=%.1fm/s az=%.1fm/s2 P=%.0fPa T=%.1fC baro_lock=%d SG0=%.3fmV dropped=%lu",
                     flight_fsm_state_name(state), est.alt_m, est.vel_z_mps,
                     est.accel_z_world_mps2, sensors.pressure_pa, sensors.temperature_c,
                     est.is_baro_locked_out ? 1 : 0, mv0,
                     (unsigned long)sd_logger_get_dropped_count());
        }
    }
}

// Core 0: build and transmit a telemetry packet; fast in flight, slow otherwise.
static void lora_tx_task(void *arg)
{
    uint16_t counter = 0;

    while (1) {
        lock_data();
        flight_state_t state = s_flight_state;
        telemetry_packet_t pkt = {0};
        pkt.flight_state = (uint8_t)state;
        pkt.strain_raw[0] = s_strain_raw[0];
        pkt.strain_raw[1] = s_strain_raw[1];
        pkt.strain_raw[2] = s_strain_raw[2];
        pkt.accel_z_mps2 = s_accel_z_mps2;
        pkt.altitude_m   = s_altitude_m;
        pkt.vel_z_ms           = s_vel_z_ms;
        pkt.temperature_c      = s_temperature_c;
        pkt.is_baro_locked_out = s_baro_locked_out ? 1 : 0;
        unlock_data();

        pkt.packet_counter = counter++;
        pkt.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (s_lora_ready) {
            lora_send_packet(&pkt);
        }

        uint32_t period_ms;
        if (state == STATE_PAD_IDLE || state == STATE_LANDED) {
            period_ms = LORA_PERIOD_PAD_MS;
        } else {
            period_ms = LORA_PERIOD_FLIGHT_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

// Wi-Fi/BT are never started, so both are already off; make it explicit.
static void disable_wifi_and_bt(void)
{
    esp_wifi_stop();
    esp_wifi_deinit();
    ESP_LOGI(TAG, "Wi-Fi and Bluetooth left disabled (never started).");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== 2SPACE Avionics boot (ESP32-S3) ===");

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    disable_wifi_and_bt();

    s_data_mutex = xSemaphoreCreateMutex();
    if (s_data_mutex == NULL) {
        ESP_LOGE(TAG, "Could not create data mutex. Halting.");
        return;
    }

    ESP_LOGI(TAG, "Init: shared buses...");
    if (bus_manager_init() != ESP_OK) {
        ESP_LOGE(TAG, "Bus init failed. Halting.");
        return;
    }

    ESP_LOGI(TAG, "Init: microSD logger...");
    if (sd_logger_init() != ESP_OK) {
        ESP_LOGE(TAG, "microSD not available (continuing without logging).");
    }

    ESP_LOGI(TAG, "Init: strain-gauge ADC (ADS124S06)...");
    if (ads124s06_init(BOARD_SPI_HOST, ADC_GAIN_CODE) == ESP_OK) {
        s_adc_ready = true;
    } else {
        ESP_LOGE(TAG, "ADS124S06 not responding (continuing without strain).");
    }

    ESP_LOGI(TAG, "Init: accelerometer + barometer...");
    if (accel_baro_init() != ESP_OK) {
        ESP_LOGE(TAG, "Accelerometer/barometer chip-ID check failed. Halting.");
        return;
    }
    accel_baro_set_ground_reference(20);

    ESP_LOGI(TAG, "Init: LoRa radio...");
    if (lora_init(BOARD_SPI_HOST) == ESP_OK) {
        s_lora_ready = true;
    } else {
        ESP_LOGE(TAG, "LoRa not responding (continuing without telemetry).");
    }

    ESP_LOGI(TAG, "Starting tasks...");

    // Core 1: time-critical sensor + state-machine loop.
    xTaskCreatePinnedToCore(sensor_and_fsm_task, "sensor_fsm", 8192, NULL, 6, NULL, 1);

    // Core 0: housekeeping (sd_writer also runs on core 0, started in sd_logger_init).
    xTaskCreatePinnedToCore(lora_tx_task, "lora_tx", 4096, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "=== Boot complete. System running. ===");
}
