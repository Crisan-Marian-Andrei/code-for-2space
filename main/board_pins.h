#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// 1. ADC 24-Bit Strain Gauges (ADS124S06IPBS - U1) & Magistrala SPI

#define PIN_SPI_MOSI            11      // IO11
#define PIN_SPI_SCK             12      // IO12
#define PIN_SPI_MISO            13      // IO13

#define PIN_SG_CS               14      // IO14 (Active LOW, pull-up extern 10k)
#define PIN_SG_DRDY             7       // IO7  (Data Ready, Active LOW)

// 2. Transceiver LoRa (Partajeaza magistrala SPI)

#define PIN_LORA_CS             10      // IO10
#define PIN_LORA_RST            5       // IO5
#define PIN_LORA_DIO0           6       // IO6

// 3. Senzori I2C & IMU (Intreruperi)

#define PIN_I2C_SDA             8       // IO8
#define PIN_I2C_SCL             9       // IO9

#define PIN_IMU_INT1            15      // IO15
#define PIN_IMU_INT2            21      // IO21

// 4. Modul GNSS / GPS (UART)

#define PIN_GNSS_UART_TX        17      // IO17 (ESP TX -> GNSS RX)
#define PIN_GNSS_UART_RX        18      // IO18 (ESP RX <- GNSS TX)
#define PIN_GNSS_RST            16      // IO16

// 5. Card SD (Interfata SDMMC 4-bit)

#define PIN_SDMMC_CMD           35      // IO35
#define PIN_SDMMC_CLK           36      // IO36
#define PIN_SDMMC_D0            37      // IO37
#define PIN_SDMMC_D1            38      // IO38
#define PIN_SDMMC_D2            39      // IO39
#define PIN_SDMMC_D3            40      // IO40

// 6. Bus CAN / TWAI

#define PIN_TWAI_TX             41      // IO41
#define PIN_TWAI_RX             42      // IO42

// 7. Masuratori Analogice & Pirotehnie (Neutilizate pe payload)

#define PIN_VBATT_SENSE         4       // IO4 (ADC1_CH3)

#define PIN_PYRO1_SENSE         1       // IO1
#define PIN_PYRO2_SENSE         2       // IO2
#define PIN_IGNITE1             47      // IO47
#define PIN_IGNITE2             48      // IO48

// 8. Interfete Sistem (Boot, USB, UART0 Debug)
#define PIN_BOOT_BTN            0       // IO0 (Strapping pin / SW2)
#define PIN_USB_DN              19      // IO19 (J11 D-)
#define PIN_USB_DP              20      // IO20 (J11 D+)
#define PIN_UART0_TX            43      // TXD0 (J10)
#define PIN_UART0_RX            44      // RXD0 (J10)

#ifdef __cplusplus
}
#endif