# M5Stack Core2 v1.1 Demo (Rust / esp-hal)

A feature-rich demo application for the [M5Stack Core2 v1.1](https://docs.m5stack.com/en/core/Core2%20v1.1) running bare-metal Rust with [esp-hal](https://github.com/esp-rs/esp-hal) and [Embassy](https://embassy.dev/) async runtime.

## Features

- **Slint UI** with tabbed interface (Status, Controls, Animations, System)
- **ILI9342C display** over SPI with DMA (320x240, 30 MHz)
- **FT6336U capacitive touch** with Slint event dispatching
- **AXP2101 PMIC** — battery monitoring, voltage rails, backlight control
- **INA3221** 3-channel power monitor (battery, USB, system rail)
- **BM8563 RTC** with clock display
- **BLE GATT server** (trouble-host) for setting RTC time wirelessly
- **PSRAM** — 4 MB external RAM for the framebuffer
- **Live stats** — FPS, frame time, render time, heap usage, uptime

## Hardware

| Component | Chip | Interface |
|-----------|------|-----------|
| MCU | ESP32 (240 MHz, 520 KB SRAM, 4 MB PSRAM) | — |
| Display | ILI9342C 320x240 | SPI + DMA |
| Touch | FT6336U | I2C (0x38) |
| PMIC | AXP2101 | I2C (0x34) |
| Power Monitor | INA3221 | I2C (0x40) |
| RTC | BM8563 (PCF8563 compatible) | I2C (0x51) |
| Radio | ESP32 BLE | — |

## Memory Layout

The ESP32 DRAM is split across regions. With BLE enabled, 64 KB is reserved for the Bluetooth controller:

| Region | Size | Usage |
|--------|------|-------|
| DRAM (main) | 128 KB | BSS (statics, DMA buffers) + stack (~72 KB) |
| DRAM2 (reclaimed) | 96 KB | Heap (general allocations, Slint, BLE) |
| PSRAM | 4 MB | Framebuffer (150 KB) |

## Building

Requires the Xtensa Rust toolchain. Install via [espup](https://github.com/esp-rs/espup):

```bash
espup install
```

Build and flash:

```bash
cargo run --release
```

## BLE Time Sync

The device advertises as **"M5Core2"** with a writable GATT characteristic for setting the RTC. A Python script is included:

```bash
uv run scripts/set_time.py
```

This connects via BLE and writes the current system time to the RTC.

## Project Structure

```
src/
  bin/main.rs        — Application entry point, peripheral init, main loop
  lib.rs             — Library root
  ble.rs             — BLE GATT server (trouble-host)
  pmic.rs            — AXP2101 PMIC initialization and helpers
  slint_platform.rs  — Slint platform backend for ESP32
  display.rs         — Display utilities
ui/
  main.slint         — Slint UI definition (tabs, widgets, animations)
scripts/
  set_time.py        — Python BLE time sync script (bleak)
```

## Dependencies

All driver crates are published on crates.io:

- [axp2101-dd](https://crates.io/crates/axp2101-dd) — AXP2101 PMIC driver
- [lcd-async](https://crates.io/crates/lcd-async) — Async display driver
- [ft6336u-dd](https://crates.io/crates/ft6336u-dd) — FT6336U touch driver
- [ina3221-dd](https://crates.io/crates/ina3221-dd) — INA3221 power monitor driver
- [pcf8563-dd](https://crates.io/crates/pcf8563-dd) — PCF8563/BM8563 RTC driver
- [trouble-host](https://crates.io/crates/trouble-host) — BLE GATT host stack

esp-hal ecosystem crates are pinned to git rev `04c4243c`.

## License

MIT OR Apache-2.0
