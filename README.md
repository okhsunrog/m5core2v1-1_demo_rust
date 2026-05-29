# M5Stack Core2 v1.1 Demo (Rust / esp-hal)

A feature-rich demo application for the [M5Stack Core2 v1.1](https://docs.m5stack.com/en/core/Core2%20v1.1) running bare-metal Rust with [esp-hal](https://github.com/esp-rs/esp-hal) and [Embassy](https://embassy.dev/) async runtime.

## Screenshots

| Status | Controls | Animations |
|--------|----------|------------|
| ![Status tab](images/photo_2026-02-27_00-10-48.jpg) | ![Controls tab](images/photo_2026-02-27_00-11-05.jpg) | ![Animations tab](images/photo_2026-02-27_00-11-10.jpg) |

| System (hardware) | System (controls) |
|--------------------|--------------------|
| ![System tab top](images/photo_2026-02-27_00-11-18.jpg) | ![System tab bottom](images/photo_2026-02-27_00-11-15.jpg) |

## Features

- **Slint UI** with tabbed interface (Status, Controls, Animations, System)
- **ILI9342C display** over SPI + DMA (320x240, 30 MHz) — line-by-line DMA
  streaming with ping-pong tile buffers, so rendering overlaps with SPI
  transmission. No full framebuffer in RAM.
- **Dual-core split** — render loop pinned to core 1 via `esp_rtos::start_second_core`,
  BLE / I2C / touch on core 0. Heavy animation frames can't starve input.
- **FT6336U capacitive touch** with Slint event dispatching
- **AXP2101 PMIC** — battery monitoring, voltage rails, backlight control
- **INA3221** 3-channel power monitor (battery, USB, system rail)
- **BM8563 RTC** with clock display
- **I2S audio** via NS4168 speaker amplifier — IMA ADPCM decoded on the fly
  into esp-hal's async circular DMA stream (`DmaTxStreamBuf`), gap-free with
  no full PCM buffer in RAM.
- **BLE GATT server** (trouble-host) for setting RTC time wirelessly
- **Persistent settings** — backlight brightness and Ext 5V state stored
  in flash via [`sequential-storage`](https://crates.io/crates/sequential-storage)
  with postcard serialization. Backlight writes are debounced 3 s so dragging
  the slider doesn't hammer flash.
- **Live stats** — FPS, frame time, render time, heap usage, uptime

## Hardware

| Component | Chip | Interface |
|-----------|------|-----------|
| MCU | ESP32 (240 MHz, 520 KB SRAM) | — |
| Display | ILI9342C 320x240 | SPI + DMA |
| Touch | FT6336U | I2C (0x38) |
| PMIC | AXP2101 | I2C (0x34) |
| Power Monitor | INA3221 | I2C (0x40) |
| RTC | BM8563 (PCF8563 compatible) | I2C (0x51) |
| Speaker | NS4168 (class-D amp) | I2S1 (BCLK=12, LRCK=0, DOUT=2) |
| Radio | ESP32 BLE | — |

## Memory Layout

The ESP32 DRAM is split across regions. With BLE enabled, 64 KB is reserved for the Bluetooth controller. Line-by-line DMA streaming means no full framebuffer is needed, so everything fits in internal SRAM:

| Region | Size | Usage |
|--------|------|-------|
| DRAM (main) | 128 KB | BSS (statics, DMA tile buffers, stacks) |
| DRAM2 (reclaimed) | 96 KB | Heap (Slint, BLE, general allocations) |

## Building

Requires the Xtensa Rust toolchain. Install via [espup](https://github.com/esp-rs/espup):

```bash
espup install
```

Build and flash:

```bash
cargo run --release
```

## Audio Encoding

Sound clips in `sounds/*.adpcm` are raw continuous IMA ADPCM nibble streams (no
WAV container, no block headers — see [src/audio.rs](src/audio.rs)). To add or
re-encode a clip:

```bash
scripts/encode_adpcm.rs input.ogg sounds/myname.adpcm
# override sample rate (default 44100):
scripts/encode_adpcm.rs input.wav sounds/myname.adpcm --rate 22050
```

Requirements:

- `ffmpeg` on PATH (decodes any audio format to raw PCM)
- `rust-script` — install with `cargo install rust-script`

The script pipes ffmpeg's PCM output through
[`audio-codec-algorithms`](https://crates.io/crates/audio-codec-algorithms),
the same crate the firmware uses to decode at runtime, so round-trips are
guaranteed byte-correct. Compression is 4:1 vs 16-bit PCM. Note that
`sox -e ima-adpcm` and `ffmpeg -acodec adpcm_ima_wav` produce **block-formatted**
ADPCM that is not playable by this decoder — use the script.

Why IMA ADPCM and not a newer codec? See the on-device codec evaluation
(ADPCM vs sea-codec vs LC3 — flash size, decode CPU, heap, quality) on the
`local-esp-hal-test` branch: `CODEC_BENCHMARK.md`.

## BLE Time Sync

The device advertises as **"M5Core2"** with a writable GATT characteristic for setting the RTC. A Python script is included:

```bash
uv run scripts/set_time.py
```

This connects via BLE and writes the current system time to the RTC.

## Project Structure

```
src/
  bin/main.rs        — Entry point, peripheral init, core-0 tasks + core-1 spawn
  lib.rs             — Library root
  audio.rs           — I2S audio via esp-hal async DMA streaming + IMA ADPCM
  ble.rs             — BLE GATT server (trouble-host)
  config_store.rs    — Persistent settings (backlight, Ext 5V) in flash via
                       sequential-storage + postcard, with debounced writes
  display_dma.rs     — Per-line DMA display driver with ping-pong tile buffers
                       and `LineBufferProvider` for Slint's `render_by_line`
  pmic.rs            — AXP2101 PMIC initialization and helpers
  slint_platform.rs  — Slint platform backend for ESP32
ui/
  main.slint         — Slint UI definition (tabs, widgets, animations)
sounds/
  *.adpcm            — IMA ADPCM notification sounds (44.1 kHz mono, 4:1)
scripts/
  encode_adpcm.rs    — rust-script: encode any audio to raw IMA ADPCM (ffmpeg)
  set_time.py        — Python BLE time sync script (bleak)
```

## Dependencies

Driver crates from crates.io:

- [axp2101-dd](https://crates.io/crates/axp2101-dd) — AXP2101 PMIC driver
- [mipidsi](https://github.com/almindor/mipidsi) — MIPI DCS display driver
  (only used during boot to initialize the ILI9342C; the steady-state render
  path is the custom DMA driver in `src/display_dma.rs`)
- [ft6336u-dd](https://crates.io/crates/ft6336u-dd) — FT6336U touch driver
- [ina3221-dd](https://crates.io/crates/ina3221-dd) — INA3221 power monitor driver
- [pcf8563-dd](https://crates.io/crates/pcf8563-dd) — PCF8563/BM8563 RTC driver
- [audio-codec-algorithms](https://crates.io/crates/audio-codec-algorithms) —
  IMA ADPCM decoder (no_std, no alloc, 3 bytes state)
- [esp-storage](https://crates.io/crates/esp-storage) +
  [sequential-storage](https://crates.io/crates/sequential-storage) — wear-leveled
  flash KV storage for persisted settings (postcard-serialized)

Git dependencies:

- [trouble-host](https://github.com/embassy-rs/trouble) — BLE GATT host stack (master branch)
- [slint](https://github.com/okhsunrog/slint/tree/rgb565-be) — fork with `Rgb565PixelBE` so
  the software renderer writes big-endian pixels directly for the ILI9342C, avoiding a
  post-render byte swap. Tracks upstream.

## License

MIT OR Apache-2.0
