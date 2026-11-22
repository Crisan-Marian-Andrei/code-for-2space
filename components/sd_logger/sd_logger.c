#include "sd_logger.h"

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_rom_crc.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"

static const char *TAG = "SD_LOGGER";

#define MOUNT_POINT "/sdcard"
#define RINGBUF_SIZE_BYTES   (32 * 1024)
#define WRITE_CHUNK_BYTES    4096
#define FSYNC_INTERVAL_MS    250
#define FLIGHT_LOG_PATH MOUNT_POINT "/FLIGHT.BIN"

static sdmmc_card_t *s_card = NULL;
static FILE *s_log_file = NULL;
static RingbufHandle_t s_ringbuf = NULL;
static TaskHandle_t s_writer_task = NULL;
static volatile bool s_running = false;
static volatile uint32_t s_dropped_records = 0;

static void sd_writer_task(void *arg)
{
    uint8_t chunk[WRITE_CHUNK_BYTES];
    size_t chunk_offset = 0;
    TickType_t last_sync_tick = xTaskGetTickCount();

    while (s_running) {
        size_t item_size = 0;
        uint8_t *item = (uint8_t *)xRingbufferReceive(s_ringbuf, &item_size, pdMS_TO_TICKS(50));

        if (item != NULL) {
            if (chunk_offset + item_size <= WRITE_CHUNK_BYTES) {
                memcpy(&chunk[chunk_offset], item, item_size);
                chunk_offset += item_size;
            }
            vRingbufferReturnItem(s_ringbuf, item);
        }

        TickType_t now = xTaskGetTickCount();
        bool time_to_flush = ((now - last_sync_tick) >= pdMS_TO_TICKS(FSYNC_INTERVAL_MS)) && (chunk_offset > 0);
        bool chunk_full = (chunk_offset >= WRITE_CHUNK_BYTES);

        if ((chunk_full || time_to_flush) && s_log_file != NULL) {
            size_t written = fwrite(chunk, 1, chunk_offset, s_log_file);
            if (written != chunk_offset) {
                ESP_LOGW(TAG, "Scriere incompleta pe SD: %u / %u bytes", (unsigned)written, (unsigned)chunk_offset);
            }
            chunk_offset = 0;

            if (time_to_flush) {
                fsync(fileno(s_log_file));
                last_sync_tick = now;
            }
        }
    }

    // Golire finala la oprire
    if (chunk_offset > 0 && s_log_file != NULL) {
        fwrite(chunk, 1, chunk_offset, s_log_file);
    }
    if (s_log_file != NULL) {
        fsync(fileno(s_log_file));
        fclose(s_log_file);
        s_log_file = NULL;
    }

    vTaskDelete(NULL);
}

esp_err_t sd_logger_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED; // 40 MHz bcs of the pullup R

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = SD_PIN_CLK;
    slot_config.cmd = SD_PIN_CMD;
    slot_config.d0  = SD_PIN_D0;
    slot_config.d1  = SD_PIN_D1;
    slot_config.d2  = SD_PIN_D2;
    slot_config.d3  = SD_PIN_D3;
    // internal pullups for safety
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Montare microSD esuata: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_card_print_info(stdout, s_card);

    s_log_file = fopen(FLIGHT_LOG_PATH, "ab");
    if (s_log_file == NULL) {
        ESP_LOGE(TAG, "Nu pot deschide fisierul %s", FLIGHT_LOG_PATH);
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
        s_card = NULL;
        return ESP_FAIL;
    }
    setvbuf(s_log_file, NULL, _IONBF, 0);
    ESP_LOGI(TAG, "Fisier de zbor deschis: %s", FLIGHT_LOG_PATH);

    s_ringbuf = xRingbufferCreate(RINGBUF_SIZE_BYTES, RINGBUF_TYPE_NOSPLIT);
    if (s_ringbuf == NULL) {
        ESP_LOGE(TAG, "Alocare ring buffer esuata");
        fclose(s_log_file);
        s_log_file = NULL;
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
        s_card = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_running = true;
    // Pinning pe Core 0 (task-urile critice SPI/ADC/FSM stau pe Core 1)
    BaseType_t task_ok = xTaskCreatePinnedToCore(
        sd_writer_task,
        "sd_writer",
        4096,
        NULL,
        4,
        &s_writer_task,
        0
    );

    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Creare task sd_writer esuata");
        s_running = false;
        vRingbufferDelete(s_ringbuf);
        s_ringbuf = NULL;
        fclose(s_log_file);
        s_log_file = NULL;
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
        s_card = NULL;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t sd_logger_push_record(const flight_log_record_t *rec)
{
    if (!s_running || s_ringbuf == NULL || rec == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    flight_log_record_t pkt = *rec;
    pkt.magic = 0x524B5432; // "RKT2"
    pkt.crc32 = esp_rom_crc32_le(0, (const uint8_t *)&pkt, offsetof(flight_log_record_t, crc32));

    // Timeout 0: nu blocheaza niciodata task-ul de achizitie ADC
    if (xRingbufferSend(s_ringbuf, &pkt, sizeof(pkt), 0) != pdTRUE) {
        s_dropped_records++;
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

void sd_logger_close(void)
{
    if (!s_running) {
        return;
    }
    s_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));

    if (s_ringbuf != NULL) {
        vRingbufferDelete(s_ringbuf);
        s_ringbuf = NULL;
    }
    if (s_card != NULL) {
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
        s_card = NULL;
    }
    ESP_LOGI(TAG, "SD Logger inchis. Pachete pierdute: %lu", (unsigned long)s_dropped_records);
}

uint32_t sd_logger_get_dropped_count(void)
{
    return s_dropped_records;
}