# 2SPACE Rocket Avionics Firmware (ESP32-S3)

ESP-IDF v5.x firmware for the 2SPACE payload avionics board. Reads strain
gauges, an accelerometer and a barometer; runs a flight state machine; logs to
microSD and transmits telemetry over LoRa.

## Build

```sh
. ~/esp/esp-idf/export.sh
idf.py build
idf.py -p <PORT> flash monitor
```

## Hardware

Only these blocks exist on the board (per the schematic block diagram):

- **MCU:** ESP32-S3-WROOM-1-N16 (16 MB flash, **no PSRAM**), 3.3 V supply.
- **Accelerometer:** KXTJ3-1057 (I2C).
- **Barometer:** BMP581 (I2C).
- **Strain-gauge ADC:** TI ADS124S06IPBS (SPI).
- **LoRa radio:** Ra-01H / SX1276 (SPI).
- **Data storage:** microSD (SDMMC).

There is no gyroscope, magnetometer, GPS, gas sensor or pyro channel on this
board, and none is used in firmware. Wi-Fi and Bluetooth are left off (never
started) to save power and avoid RF interference with the strain and LoRa
circuits.

### Pinout (`components/board_bsp/include/board_pins.h`)

Every GPIO number lives in `board_pins.h` and nowhere else. Moving a trace on
the PCB is a one-line edit.

| Function | Pins |
|---|---|
| Shared SPI2 | MOSI=11, SCK=12, MISO=13 |
| ADS124S06 strain ADC | CS=14, DRDY=7 (RESET: software SPI command, not wired) |
| Ra-01H LoRa radio | CS=10, RESET=5, DIO0=6 |
| I2C0 (accel + baro, 400 kHz) | SDA=8, SCL=9 |
| microSD SDMMC slot 1 (4-bit) | CMD=35, CLK=36, D0=37, D1=38, D2=39, D3=40 |

### Pin safety notes

The ESP32-S3 has special pins to keep clear of normal peripherals:

- **Strapping pins** (GPIO0, 3, 45, 46): read at power-up to pick boot mode /
  flash voltage. GPIO0 is used only as the BOOT button; the rest are unused.
- **Native USB** (GPIO19, 20): left for USB only.
- **Internal flash** (GPIO26–32): wired to the module's internal SPI flash,
  never usable.

No peripheral in this firmware uses any of those.

**PSRAM / SDMMC caveat:** On parts with Octal PSRAM enabled, GPIO35/36/37 are
taken by the PSRAM and cannot drive the SD card. This board is the N16 variant
with **no PSRAM** (`CONFIG_SPIRAM` is not set), so GPIO35–40 are free and the
schematic's SDMMC wiring is valid. If you ever move to a PSRAM part (N16R8),
the SDMMC pins must move off 35/36/37.

The ADS124S06 RESET pin is not wired to the MCU on this board (and GPIO33 does
not exist on ESP32-S3-WROOM-1). The driver resets the ADC in software with the
SPI RESET command (0x06), then waits ~5 ms before the first register access.

## Software layout

The firmware is split into one app (`main/`) plus components in `components/`.

### `board_bsp`

Header-only. Holds `board_pins.h`: all pin numbers in one place.

### `bus_manager`

Brings up the two shared buses once at startup (SPI2, I2C0) and hands out
simple mutex locks. Two chips (the strain ADC and the LoRa radio) share the SPI
wires, so any task must hold the SPI lock around its transfers:

```c
bus_lock_spi();
/* ... SPI transfers ... */
bus_unlock_spi();
```

`bus_lock_i2c()` / `bus_unlock_i2c()` do the same for the accelerometer +
barometer. Before anything talks on SPI, both chip-select lines are parked HIGH
so a floating CS cannot corrupt the bus.

### `strain_gauge` — TI ADS124S06 (24-bit)

Reads three Wheatstone strain-gauge bridges wired to the ADC inputs:

- Bridge 1 (J3) → AIN0 (+) / AIN1 (−)
- Bridge 2 (J4) → AIN2 (+) / AIN3 (−)
- Bridge 3 (J5) → AIN4 (+) / AIN5 (−)

Each read points the input MUX at a bridge, issues START, waits for DRDY to go
low, then RDATA clocks out three bytes. The 24-bit two's-complement result is
sign-extended into an `int32_t`, with the startup zero-offset (set by
`ads124s06_tare()`) subtracted.

Configured for the internal 2.5 V reference and 64× PGA gain. SPI mode 1.

Conversions:
- `ads124s06_raw_to_millivolts(raw, gain)` = `(raw / 2^23) * (2.5 V / gain)`, ×1000.
- `ads124s06_raw_to_microstrain(raw, gain, gauge_factor, active_arms)` =
  `4e6 * raw / (2^23 * gain * gauge_factor * active_arms)`.

### `accel_baro` — accelerometer + barometer over I2C

A Kionix KXTJ3-1057 accelerometer (addr 0x0E) and a Bosch BMP581 barometer
(addr 0x47) on I2C0 at 400 kHz. All register and address defines are grouped at
the top of `accel_baro.h`.

Returns acceleration (m/s²), pressure (Pa), temperature (°C) and altitude (m).
There is no gyroscope or magnetometer on the board, so no rotation rate or
heading is produced. Altitude uses the barometric formula
`44330 * (1 - (P / P_ground)^0.1903)` where `P_ground` is measured once on the
pad by `accel_baro_set_ground_reference()`.

The KXTJ3 is read in 14-bit high-resolution mode at ±2 g (4096 counts/g). The
BMP581 returns pressure and temperature already compensated, so the driver only
scales the raw 24-bit values (pressure = raw / 2⁶ Pa, temperature = raw / 2¹⁶ °C).

### `lora_telemetry` — SX1276/SX1278 (Ra-01H, 868 MHz)

Transmit-only setup: 868.0 MHz, spreading factor 9, 125 kHz bandwidth,
coding rate 4/5, CRC on, PA_BOOST near max power. SPI mode 0. DIO0 (TxDone) is
polled rather than used as an interrupt to keep the driver simple.

`lora_send_packet()` computes a CRC-16/CCITT (poly 0x1021, init 0xFFFF) over the
packed `telemetry_packet_t`, writes the packet into the radio FIFO, starts the
transmit and waits for TxDone (with a 500 ms timeout). The packet carries a
counter, timestamp, flight state, the three strain values, Z acceleration,
altitude, vertical speed, temperature and the barometer-lockout flag. Keep the
struct identical on the ground station. The exact byte layout, CRC and enum
values are documented in [`docs/lora_packet.md`](docs/lora_packet.md).

### `sd_logger` — microSD logging (SDMMC + FATFS)

Mounts `/sdcard` over SDMMC slot 1 in 4-bit mode, falling back automatically to
1-bit if 4-bit fails. Opens the next free sequential file
(`/sdcard/FLIGHT00.BIN`, `FLIGHT01.BIN`, …) so old flights are never overwritten.

Records are queued (never blocking the sensor loop) and copied into a static
4096-byte buffer by a writer task pinned to Core 0. The buffer is written with
`fwrite` + `fflush` + `fsync` whenever it fills, so at most one not-yet-full
buffer can be lost on a power cut. Each record is tagged with a magic marker
(`"RKT2"`) and a CRC32 for the PC-side reader.

### `flight_fsm` — state estimator + flight state machine

Two files, two jobs:

- **`state_estimator.c/.h`** does the maths. It fuses the accelerometer +
  barometer into clean altitude, vertical speed and vertical acceleration. With
  no gyroscope it does not estimate attitude or heading; it assumes the body Z
  axis is vertical and subtracts the gravity offset learned on the pad. It
  briefly ignores the barometer above ~240 m/s (transonic band) where pressure
  readings are unreliable, using the integrated inertial estimate instead.
- **`flight_fsm.c/.h`** makes the decisions. A plain `switch (current_state)`
  moves through the flight phases using the estimator's numbers. All trigger
  thresholds are `#define`s at the top of `flight_fsm.h`.

Flight phases:

| State | Enter condition |
|---|---|
| `STATE_PAD_IDLE` | boot; averages the first 50 samples to set ground = 0 m and the strain zero |
| `STATE_BOOST` | felt accel > 2.5 g **and** climbing > 15 m/s, held ≥ 100 ms |
| `STATE_COAST` | felt accel drops below 1.2 g while still rising > 20 m/s (motor burnout) |
| `STATE_APOGEE` | vertical speed drops to ≤ 2 m/s, held ≥ 150 ms |
| `STATE_DESCENT` | parachute descent (entered immediately after apogee) |
| `STATE_LANDED` | \|vertical speed\| < 1 m/s for 5 s; slows LoRa, closes the SD log |

### `main` — application

1. Init NVS, leave Wi-Fi/BT off.
2. Init buses, then each driver in order, with an `ESP_LOGI` status line per step.
3. Start three FreeRTOS tasks across the two cores:
   - **Core 1:** `sensor_and_fsm_task` — reads the ADC + accel/baro at 50 Hz,
     runs the estimator and state machine, pushes a record to the SD queue.
   - **Core 0:** `sd_writer_task` (inside `sd_logger`) and `lora_tx_task`
     (every 200 ms in flight, 1000 ms on pad / after landing).

Tasks share the latest sensor values and flight state through a few globals
guarded by one mutex.
