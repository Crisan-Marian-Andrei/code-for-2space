#pragma once

// Owns the shared SPI bus and I2C bus, plus the bus locks.
// Usage: bus_lock_spi(); ...transfers...; bus_unlock_spi();  (same for i2c)

#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_SPI_HOST        SPI2_HOST
#define BOARD_I2C_PORT        I2C_NUM_0
#define BOARD_I2C_FREQ_HZ     400000

// Init SPI2, I2C0 and the two mutexes. Call once, before drivers.
esp_err_t bus_manager_init(void);

// SPI bus lock (strain ADC + LoRa share it).
void bus_lock_spi(void);
void bus_unlock_spi(void);

// I2C bus lock (accelerometer + barometer share it).
void bus_lock_i2c(void);
void bus_unlock_i2c(void);

#ifdef __cplusplus
}
#endif
