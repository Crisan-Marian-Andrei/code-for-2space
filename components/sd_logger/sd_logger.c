#include "sd_logger.h"
#include "board_pins.h"

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_rom_crc.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"

static const char *TAG = "SD_LOGGER";

#define MOUNT_POINT     "/sdcard"
#define LOG_BUFFER_SIZE 4096
#define QUEUE_LENGTH    256

static sdmmc_card_t    *s_card = NULL;
static FILE            *s_log_file = NULL;
static QueueHandle_t    s_queue = NULL;
static TaskHandle_t     s_writer_task = NULL;
static volatile bool    s_running = false;
static volatile uint32_t s_dropped_records = 0;

static uint8_t  s_log_buffer[LOG_BUFFER_SIZE];   // static, never malloc'd in a loop
static size_t   s_buffer_used = 0;

// Write the buffer to the card and force it to flash.
static void flush_buffer_to_card(void)
{
    if (s_buffer_used == 0 || s_log_file == NULL) {
        return;
    }

    size_t written = fwrite(s_log_buffer, 1, s_buffer_used, s_log_file);
    if (written != s_buffer_used) {
        ESP_LOGW(TAG, "Short write: %u of %u bytes", (unsigned)written, (unsigned)s_buffer_used);
    }

    fflush(s_log_file);
    fsync(fileno(s_log_file));

    s_buffer_used = 0;
}

// Writer task: pull records off the queue, batch into the buffer, flush when full.
static void sd_writer_task(void *arg)
{
    flight_log_record_t rec;

    while (s_running) {
        if (xQueueReceive(s_queue, &rec, pdMS_TO_TICKS(100)) == pdTRUE) {

            if (s_buffer_used + sizeof(rec) > LOG_BUFFER_SIZE) {
                flush_buffer_to_card();
            }

            memcpy(&s_log_buffer[s_buffer_used], &rec, sizeof(rec));
            s_buffer_used += sizeof(rec);

            if (s_buffer_used >= LOG_BUFFER_SIZE) {
                flush_buffer_to_card();
            }
        }
    }

    // Shutdown: flush the remainder and close.
    flush_buffer_to_card();
    if (s_log_file != NULL) {
        fclose(s_log_file);
        s_log_file = NULL;
    }
    vTaskDelete(NULL);
}

// Find the next unused FLIGHTnn.BIN.
static void pick_next_filename(char *path_out, size_t path_len)
{
    for (int i = 0; i < 100; i++) {
        snprintf(path_out, path_len, MOUNT_POINT "/FLIGHT%02d.BIN", i);
        FILE *f = fopen(path_out, "rb");
        if (f == NULL) {
            return;
        }
        fclose(f);
    }
    snprintf(path_out, path_len, MOUNT_POINT "/FLIGHT99.BIN");
}

esp_err_t sd_logger_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_1;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = PIN_SDMMC_CLK;
    slot_config.cmd = PIN_SDMMC_CMD;
    slot_config.d0  = PIN_SDMMC_D0;
    slot_config.d1  = PIN_SDMMC_D1;
    slot_config.d2  = PIN_SDMMC_D2;
    slot_config.d3  = PIN_SDMMC_D3;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config,
                                            &mount_config, &s_card);

    // Fall back to 1-bit mode if 4-bit failed.
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "4-bit mount failed (%s). Falling back to 1-bit mode.",
                 esp_err_to_name(ret));
        slot_config.width = 1;
        ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config,
                                      &mount_config, &s_card);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "microSD mount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_card_print_info(stdout, s_card);

    char path[32];
    pick_next_filename(path, sizeof(path));
    s_log_file = fopen(path, "ab");
    if (s_log_file == NULL) {
        ESP_LOGE(TAG, "Could not open %s", path);
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
        s_card = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Logging to %s", path);

    s_queue = xQueueCreate(QUEUE_LENGTH, sizeof(flight_log_record_t));
    if (s_queue == NULL) {
        ESP_LOGE(TAG, "Could not create log queue");
        fclose(s_log_file);
        s_log_file = NULL;
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
        s_card = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_buffer_used = 0;
    s_running = true;

    // Writer runs on Core 0 (sensor/FSM loop is on Core 1).
    BaseType_t ok = xTaskCreatePinnedToCore(sd_writer_task, "sd_writer",
                                            4096, NULL, 4, &s_writer_task, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Could not start sd_writer task");
        s_running = false;
        vQueueDelete(s_queue);
        s_queue = NULL;
        fclose(s_log_file);
        s_log_file = NULL;
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
        s_card = NULL;
        return ESP_FAIL;
    }

    return ESP_OK;
}

// Stamp magic + CRC and enqueue a record for the writer (non-blocking).
esp_err_t sd_logger_push_record(const flight_log_record_t *rec)
{
    if (!s_running || s_queue == NULL || rec == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    flight_log_record_t pkt = *rec;
    pkt.magic = 0x524B5432;   // "RKT2"
    pkt.crc32 = esp_rom_crc32_le(0, (const uint8_t *)&pkt,
                                 offsetof(flight_log_record_t, crc32));

    if (xQueueSend(s_queue, &pkt, 0) != pdTRUE) {
        s_dropped_records++;
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

// Stop the writer, flush the last buffer, and unmount the card.
void sd_logger_close(void)
{
    if (!s_running) {
        return;
    }
    s_running = false;

    // Let the writer flush its final buffer and close the file.
    vTaskDelay(pdMS_TO_TICKS(200));

    if (s_queue != NULL) {
        vQueueDelete(s_queue);
        s_queue = NULL;
    }
    if (s_card != NULL) {
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
        s_card = NULL;
    }
    ESP_LOGI(TAG, "SD logger closed. Records dropped: %lu",
             (unsigned long)s_dropped_records);
}

// Number of records dropped because the queue was full.
uint32_t sd_logger_get_dropped_count(void)
{
    return s_dropped_records;
}
