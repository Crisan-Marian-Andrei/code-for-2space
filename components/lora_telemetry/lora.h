#pragma once

// SX1276/SX1278 (Ra-01H) LoRa transmitter: 868 MHz, SF9, BW125, CRC on.
// Shares the SPI bus with the strain ADC; transfers use the bus_manager lock.

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// Keep this struct identical on the ground station. packed = no padding.
typedef struct __attribute__((packed)) {
    uint16_t packet_counter;
    uint32_t timestamp_ms;
    uint8_t  flight_state;
    int32_t  strain_raw[3];
    float    accel_z_mps2;
    float    altitude_m;
    float    vel_z_ms;            // estimated vertical speed (up = +)
    float    temperature_c;       // barometer temperature
    uint8_t  is_baro_locked_out;  // 1 = barometer frozen (high speed)
    uint16_t crc16;
} telemetry_packet_t;

// Configure the radio. Needs bus_manager_init first.
esp_err_t lora_init(spi_host_device_t spi_host);

// Fill packet->crc16 and transmit. Blocks until TxDone or timeout.
esp_err_t lora_send_packet(telemetry_packet_t *packet);

// CRC-16/CCITT (poly 0x1021, init 0xFFFF).
uint16_t lora_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
