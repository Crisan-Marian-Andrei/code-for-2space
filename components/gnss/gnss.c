#include "gnss.h"
#include "bus_manager.h"

#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "GNSS";

#define GNSS_LINE_MAX 128
static char    s_line[GNSS_LINE_MAX];
static uint8_t s_line_len = 0;

static gnss_data_t s_last = {0};   // most recent parsed values

esp_err_t gnss_init(void)
{
    s_line_len = 0;
    memset(&s_last, 0, sizeof(s_last));
    ESP_LOGI(TAG, "GNSS reader ready (UART %d, 9600 baud).", BOARD_GNSS_UART_PORT);
    return ESP_OK;
}

// NMEA "ddmm.mmmm" + hemisphere -> decimal degrees.
static double nmea_to_degrees(const char *field, char hemisphere, int degree_digits)
{
    if (field == NULL || field[0] == '\0') {
        return 0.0;
    }

    double raw = atof(field);
    int    deg = (int)(raw / 100.0);
    double min = raw - (deg * 100.0);
    double value = deg + (min / 60.0);

    (void)degree_digits;
    if (hemisphere == 'S' || hemisphere == 'W') {
        value = -value;
    }
    return value;
}

// Parse a GGA line. Fields: 2=lat,3=N/S,4=lon,5=E/W,6=fix,7=sats,9=alt.
static bool parse_gga(char *line)
{
    if (strncmp(line, "$GPGGA", 6) != 0 && strncmp(line, "$GNGGA", 6) != 0) {
        return false;
    }

    char *fields[16] = {0};
    int   count = 0;
    char *cursor = line;
    while (count < 16) {
        char *piece = strsep(&cursor, ",");
        if (piece == NULL) {
            break;
        }
        fields[count++] = piece;
    }

    if (count < 10) {
        return false;
    }

    int fix_quality = (fields[6][0] != '\0') ? atoi(fields[6]) : 0;
    if (fix_quality == 0) {
        s_last.has_fix = false;
        return true;
    }

    s_last.latitude_deg  = nmea_to_degrees(fields[2], fields[3][0], 2);
    s_last.longitude_deg = nmea_to_degrees(fields[4], fields[5][0], 3);
    s_last.satellites    = (uint8_t)atoi(fields[7]);
    s_last.altitude_m    = (float)atof(fields[9]);
    s_last.has_fix       = true;
    return true;
}

// Drain the UART, parse any complete GGA lines, return the latest fix.
esp_err_t gnss_read(gnss_data_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bool got_new_line = false;

    uint8_t chunk[64];
    int n;
    do {
        n = uart_read_bytes(BOARD_GNSS_UART_PORT, chunk, sizeof(chunk), 0);
        for (int i = 0; i < n; i++) {
            char c = (char)chunk[i];

            if (c == '\n' || c == '\r') {
                if (s_line_len > 0) {
                    s_line[s_line_len] = '\0';
                    if (parse_gga(s_line)) {
                        got_new_line = true;
                    }
                }
                s_line_len = 0;
            } else if (s_line_len < (GNSS_LINE_MAX - 1)) {
                s_line[s_line_len++] = c;
            } else {
                s_line_len = 0;   // overlong line, resync
            }
        }
    } while (n > 0);

    *out = s_last;

    return got_new_line ? ESP_OK : ESP_ERR_NOT_FOUND;
}
