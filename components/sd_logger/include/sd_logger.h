#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SD_PIN_CMD   35
#define SD_PIN_CLK   36
#define SD_PIN_D0    37
#define SD_PIN_D1    38
#define SD_PIN_D2    39
#define SD_PIN_D3    40

typedef struct __attribute__((packed)) {
    uint32_t magic;          // 0x524B5432 ("RKT2") pentru sincronizare la parsare
    uint64_t timestamp;   // esp_timer_get_time()
    int32_t  strain_ch[3];   // Valorile brute pe 24-bit (extinse pe 32-bit) de la ADS124S06
    uint8_t  flight_state;   // Starea curenta din flight_fsm
    uint8_t  reserved[3];    // Padding pentru aliniere la 32 bytes
    uint32_t crc32;          // Checksum CRC32 al primilor 28 bytes
} flight_log_record_t;

_Static_assert(sizeof(flight_log_record_t) == 32, "flight_log_record_t trebuie sa aiba exact 32 bytes");

esp_err_t sd_logger_init(void);

esp_err_t sd_logger_push_record(const flight_log_record_t *rec);

void sd_logger_close(void);

uint32_t sd_logger_get_dropped_count(void);

#ifdef __cplusplus
}
#endif