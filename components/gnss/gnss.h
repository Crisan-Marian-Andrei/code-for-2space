#pragma once

// Reads NMEA from a GPS module on UART1 (9600 baud) and parses the GGA sentence.

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double  latitude_deg;    // + North, - South
    double  longitude_deg;   // + East,  - West
    float   altitude_m;      // metres above mean sea level
    uint8_t satellites;
    bool    has_fix;
} gnss_data_t;

// Clear internal state (UART was started by bus_manager_init).
esp_err_t gnss_init(void);

// Drain the UART, parse any complete GGA line, fill out. Returns ESP_OK if a
// fresh line was parsed, ESP_ERR_NOT_FOUND otherwise. out always holds the
// latest known values.
esp_err_t gnss_read(gnss_data_t *out);

#ifdef __cplusplus
}
#endif
