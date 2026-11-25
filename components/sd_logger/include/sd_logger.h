#pragma once

// Logs flight records to microSD (SDMMC + FATFS), sequential FLIGHTnn.BIN files.
// Records are queued, buffered in a 4 KB block, and fsync'd when the block fills.

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// One logged sample. packed + fixed size so a PC tool can read it back.
typedef struct __attribute__((packed)) {
    uint32_t magic;          // 0x524B5432 ("RKT2")
    uint64_t timestamp;      // esp_timer_get_time() us
    int32_t  strain_ch[3];
    float    accel_z_mps2;
    float    altitude_m;
    float    vel_z_ms;            // estimated vertical speed (up = +)
    float    yaw_heading_deg;     // compass heading 0..360 deg
    uint8_t  flight_state;
    uint8_t  is_baro_locked_out;  // 1 = barometer frozen (high speed)
    uint8_t  reserved[2];
    uint32_t crc32;
} flight_log_record_t;

// Mount the card and start the writer task.
esp_err_t sd_logger_init(void);

// Queue one record (never blocks; drops + counts if the queue is full).
esp_err_t sd_logger_push_record(const flight_log_record_t *rec);

// Flush, close the file and unmount. Call at STATE_LANDED.
void sd_logger_close(void);

// Number of records dropped due to a full queue (0 = healthy).
uint32_t sd_logger_get_dropped_count(void);

#ifdef __cplusplus
}
#endif
