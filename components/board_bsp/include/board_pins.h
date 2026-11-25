#pragma once

// All board GPIO numbers. Change a pin here only; drivers never hardcode pins.
// Board: ESP32-S3-WROOM-1-N16 (no PSRAM). See README for the pin audit.
// NOTE: GPIO35-40 (SDMMC) are valid ONLY because PSRAM is disabled.

#ifdef __cplusplus
extern "C" {
#endif

// Shared SPI bus (SPI2_HOST): strain ADC + LoRa
#define PIN_SPI_MOSI            11
#define PIN_SPI_SCK             12
#define PIN_SPI_MISO            13

// Strain-gauge ADC (ADS124S06), SPI device 1
#define PIN_SG_CS               14      // active LOW
#define PIN_SG_DRDY             7       // data ready, active LOW
#define PIN_SG_RESET            33      // active LOW

// LoRa radio (Ra-01H / SX1276), SPI device 2
#define PIN_LORA_CS             10      // active LOW
#define PIN_LORA_RST            5       // active LOW
#define PIN_LORA_DIO0           6       // TxDone

// I2C bus (I2C_NUM_0): IMU + barometer
#define PIN_I2C_SDA             8
#define PIN_I2C_SCL             9
#define PIN_IMU_INT1            15
#define PIN_IMU_INT2            21

// GNSS (UART_NUM_1)
#define PIN_GNSS_UART_TX        17      // ESP TX -> GNSS RX
#define PIN_GNSS_UART_RX        18      // ESP RX <- GNSS TX
#define PIN_GNSS_RST            16

// microSD (SDMMC slot 1, 4-bit)
#define PIN_SDMMC_CMD           35
#define PIN_SDMMC_CLK           36
#define PIN_SDMMC_D0            37
#define PIN_SDMMC_D1            38
#define PIN_SDMMC_D2            39
#define PIN_SDMMC_D3            40

// CAN / TWAI (unused on payload)
#define PIN_TWAI_TX             41
#define PIN_TWAI_RX             42

// Analog sense + pyro (unused on payload)
#define PIN_VBATT_SENSE         4       // ADC1_CH3
#define PIN_PYRO1_SENSE         1
#define PIN_PYRO2_SENSE         2
#define PIN_IGNITE1             47
#define PIN_IGNITE2             48

// System (boot button, USB, debug UART0) - do not reuse
#define PIN_BOOT_BTN            0       // strapping pin
#define PIN_USB_DN              19      // native USB
#define PIN_USB_DP              20      // native USB
#define PIN_UART0_TX            43
#define PIN_UART0_RX            44

#ifdef __cplusplus
}
#endif
