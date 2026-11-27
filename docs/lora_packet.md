# LoRa telemetry packet layout

Defined by `telemetry_packet_t` in `components/lora_telemetry/lora.h`.
The struct is `__attribute__((packed))` (no padding) and transmitted
**little-endian** (ESP32-S3 byte order). Total size: **38 bytes**.

Radio settings (for the receiver): 868.0 MHz, LoRa, SF9, BW 125 kHz,
coding rate 4/5, explicit header, CRC on.

## Field table

| Offset | Size | Type       | Field                | Meaning |
|-------:|-----:|------------|----------------------|---------|
| 0      | 2    | `uint16_t` | `packet_counter`     | Increments per transmitted packet (wraps at 65535). |
| 2      | 4    | `uint32_t` | `timestamp_ms`       | Milliseconds since boot (`esp_timer` / 1000). |
| 6      | 1    | `uint8_t`  | `flight_state`       | FSM state enum (see below). |
| 7      | 12   | `int32_t[3]` | `strain_raw[3]`    | Raw sign-extended ADS124S06 counts, bridges 0/1/2. |
| 19     | 4    | `float`    | `accel_z_mps2`       | Vertical acceleration, gravity removed (m/s²). |
| 23     | 4    | `float`    | `altitude_m`         | Estimated altitude above the pad (m). |
| 27     | 4    | `float`    | `vel_z_ms`           | Estimated vertical speed, up = + (m/s). |
| 31     | 4    | `float`    | `temperature_c`      | Barometer temperature (°C). |
| 35     | 1    | `uint8_t`  | `is_baro_locked_out` | 1 = barometer frozen (high-speed/transonic lockout), else 0. |
| 36     | 2    | `uint16_t` | `crc16`              | CRC-16/CCITT over bytes 0..35. |

`float` is IEEE-754 single precision, little-endian. `int32_t[3]` is three
consecutive little-endian 32-bit signed integers.

## CRC

- Algorithm: CRC-16/CCITT-FALSE — polynomial `0x1021`, init `0xFFFF`, no
  reflection, no final XOR.
- Coverage: the first 36 bytes (offset 0 through 35 inclusive), i.e. every
  field **except** `crc16` itself.
- Transmitted at offset 36 as a little-endian `uint16_t`.

Reference implementation (matches `lora_crc16()` in `lora.c`):

```c
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
```

## flight_state enum

From `flight_fsm.h`:

| Value | Name |
|------:|------|
| 0 | `STATE_PAD_IDLE` |
| 1 | `STATE_BOOST` |
| 2 | `STATE_COAST` |
| 3 | `STATE_APOGEE` |
| 4 | `STATE_DESCENT` |
| 5 | `STATE_LANDED` |

## Decoding note

On a little-endian host with the same packed layout you can memcpy the 38 bytes
straight onto the struct. On a big-endian host, byte-swap each multi-byte field
individually. Validate `crc16` before trusting a packet.
