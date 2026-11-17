#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_SPI_HOST SPI2_HOST
#define BOARD_I2C_PORT I2C_NUM_0
#define BOARD_I2C_FREQ_HZ 400000
#define BOARD_GNSS_UART_PORT UART_NUM_1

esp_err_t bus_manager_init(void);

esp_err_t bus_manager_spi_acquire(spi_device_handle_t dev_handle, TickType_t wait_ticks);
void bus_manager_spi_release(spi_device_handle_t dev_handle);

#ifdef __cplusplus
}

#endif