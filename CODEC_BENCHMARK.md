# Audio codec comparison on ESP32 (M5Stack Core2 v1.1)

Evaluation of three `no_std` audio codecs for storing/playing notification
sounds on the ESP32, across the three axes that matter on an MCU: **flash
size**, **decode CPU**, and **decode RAM**. Measured on real hardware.

- **Codecs:** IMA ADPCM ([`audio-codec-algorithms`](https://crates.io/crates/audio-codec-algorithms), current),
  [`sea-codec`](https://crates.io/crates/sea-codec) 0.7, [`lc3-codec`](https://crates.io/crates/lc3-codec) 0.2
- **Test clip:** `ping` (KDE Oxygen `desktop-login-long.ogg`) — 593,069 mono
  samples, **13.45 s @ 44.1 kHz**, 1,186,138 B as raw `s16` PCM
- **Hardware:** ESP32 (Xtensa LX6, dual-core) @ 240 MHz, built at the project's
  `opt-level = "s"`
- **Bench source:** [`src/bin/bench_codecs.rs`](src/bin/bench_codecs.rs)
  (`cargo run --release --bin bench_codecs`)

## Summary

| Codec | Flash (clip) | vs ADPCM | Decode ×realtime | CPU @ realtime¹ | Decode RAM | SNR² |
|-------|-------------:|---------:|-----------------:|----------------:|-----------:|-----:|
| **IMA ADPCM** (4-bit) | 296.5 KB | 1.00× | **54×** | ~1.9% | **~0** (stack only) | 41.8 dB |
| **sea-codec** CBR 3-bit | 239.6 KB | 0.81× | **16×** | ~6.3% | ~37–47 KB | **45.5 dB** |
| **lc3-codec** 80 B/frame | 98.9 KB | 0.33× | **4.0×** | ~25% | **30.6 KB** fixed | 20.7 dB² |

¹ fraction of one 240 MHz core to decode in real time.
² SNR vs the source waveform — **valid only for the waveform codecs** (ADPCM, sea).
LC3 is an MDCT transform codec that shapes quantization noise psychoacoustically,
so its low SNR is *not* a quality verdict; judge LC3 by ear (see Quality below).

**Verdict for this project (3 short notification sounds):** keep **IMA ADPCM** —
zero RAM, negligible CPU, and the clip already sounds fine. If a principled upgrade
is wanted, **sea-codec @ 3-bit** is both smaller *and* measurably cleaner than ADPCM
(45.5 vs 41.8 dB) — the best quality move, at ~6% CPU but a non-trivial ~37 KB heap.
**LC3** compresses 3× better and likely sounds best, but ~31 KB RAM + ~25% of a core
is a poor fit for short SFX on a core/heap shared with Slint + BLE.

## 1. Flash size (compression)

Encoder output for the same 13.45 s clip. ADPCM and sea are time-domain; LC3 is a
transform codec (MDCT), far more efficient per bit.

| Encoding | Size | Bitrate | vs PCM | vs ADPCM |
|----------|-----:|--------:|-------:|---------:|
| raw PCM `s16` | 1,186,138 B | 706 kbps | 1.0× | 4.0× larger |
| IMA ADPCM (current) | 296,535 B | 176 kbps | 4.0× | — |
| sea-codec CBR 2-bit | 165,437 B | 98 kbps | 7.2× | 0.56× |
| sea-codec VBR 3.0-bit | 222,658 B | 133 kbps | 5.3× | 0.75× |
| sea-codec CBR 3-bit | 239,570 B | 143 kbps | 5.0× | 0.81× |
| sea-codec CBR 4-bit | 313,704 B | 187 kbps | 3.8× | 1.06× |
| lc3-codec 40 B/frame | 49,440 B | 29 kbps | 24.0× | 0.17× |
| lc3-codec 60 B/frame | 74,160 B | 44 kbps | 16.0× | 0.25× |
| lc3-codec 80 B/frame | 98,880 B | 59 kbps | 12.0× | 0.33× |
| lc3-codec 120 B/frame | 148,320 B | 88 kbps | 8.0× | 0.50× |

IMA ADPCM is a fixed 4 bits/sample (4:1). sea-codec's knob is `residual_bits`
(1–8). LC3 is CBR via bytes-per-frame (10 ms frames → 441 samples @ 44.1 kHz).

## 2. Decode CPU (on-device)

Whole clip decoded into small reusable buffers (the full PCM is too big for RAM);
a checksum of the output prevents the optimizer from eliding the work.

| Codec | Decode time | ×realtime | µs / 1k samples | ~cycles/sample |
|-------|------------:|----------:|----------------:|---------------:|
| IMA ADPCM | 0.250 s | 53.8× | 421 | ~101 |
| sea-codec 3-bit | 0.851 s | 15.8× | 1,435 | ~344 |
| lc3-codec 80 B/frame | 3.376 s | 4.0× | 5,691 | ~1,366 |

All decode faster than real time, so all are *usable*; the headroom differs by
~14× between ADPCM and LC3. sea-codec's per-sample cost is roughly independent of
bitrate (the LMS filter runs regardless), so **2-bit sea ≈ same CPU as 3-bit** but
at 165 KB — a strictly better size/CPU point.

## 3. Decode RAM (heap, via `esp_alloc::HEAP`)

| Codec | Heap | Notes |
|-------|-----:|-------|
| IMA ADPCM | ~0 B | `AdpcmImaState` is ~4 B on the stack |
| sea-codec 3-bit | ~37–47 KB | a ~36 KB fixed decoder floor + ~2 B/frame staging (see below) |
| lc3-codec 80 B/frame | 30.6 KB fixed | 19.9 KB scaler (`f32`) + 7.7 KB complex (FFT) + ~3 KB channel state/output — mandatory MDCT/FFT scratch, no per-frame churn |

Real RAM floor: **ADPCM (≈0) ≪ LC3 (~31 KB fixed) ≈ sea-codec (~36–47 KB)**.

> **Correction:** an earlier draft claimed sea-codec needs only "a few KB,
> tunable." That was wrong — see the block-size sweep below. The decoder has a
> hard ~36 KB floor that block size cannot remove.

### sea-codec heap vs block size (`frames_per_chunk`)

Measured with a counting allocator on the host (allocation byte-requests are
platform-independent; the resident number was validated against the on-device
`esp_alloc::HEAP.used()` — 46.6 KB host vs 45 KB device at chunk 5120).

| `frames_per_chunk` | encoded size | resident heap | peak (incl. realloc) |
|-------------------:|-------------:|--------------:|---------------------:|
| 100 | 361.8 KB | **36.6 KB** | 37.8 KB |
| 200 | 296.6 KB | 36.8 KB | 38.0 KB |
| 500 | 262.2 KB | 37.4 KB | 38.5 KB |
| 1000 | 249.1 KB | 38.4 KB | 40.5 KB |
| 2000 | 243.2 KB | 40.4 KB | 44.5 KB |
| 5120 (default) | 239.6 KB | 46.6 KB | 61.4 KB |

Two takeaways: (1) the ~36 KB decoder floor barely moves with block size; (2)
smaller blocks *grow* the file (more per-chunk header overhead), so chunk size is
a RAM-vs-flash trade with a hard RAM floor. The sweet spot is ~500–1000 frames
(≈37–38 KB resident, 249–262 KB flash).

## 4. Quality (SNR vs source)

SNR of each decoded clip against the original PCM, best-offset aligned (LC3's
~120-sample MDCT delay removed before scoring).

| Codec | Flash | SNR |
|-------|------:|----:|
| IMA ADPCM 4-bit (current) | 296.5 KB | 41.8 dB |
| sea-codec 2-bit | 165.4 KB | 37.6 dB |
| **sea-codec 3-bit** | 239.6 KB | **45.5 dB** |
| sea-codec 4-bit | 313.7 KB | 51.0 dB |
| lc3 40 B/frame | 49.4 KB | 17.6 dB† |
| lc3 80 B/frame | 98.9 KB | 20.7 dB† |

**sea-codec 3-bit is both smaller than ADPCM *and* objectively cleaner** (45.5 vs
41.8 dB) — a strict win on both axes. † **LC3's SNR is not a quality verdict:**
MDCT codecs shape quantization noise to be perceptually masked, so a low waveform
SNR can still sound good. LC3 must be judged by ear, not by this number.

## 5. ESP32 internal-RAM budget (can the heap grow?)

From the linked main firmware (`xtensa-esp32-elf-{size,nm}`):

| Region | Range | Size | Contents |
|--------|-------|-----:|----------|
| `dram_seg` | `0x3FFB0000`–`0x3FFE0000` | 192 KB | `.data` 16 KB + `.rwtext(.wifi)` ~52 KB + `.bss` ~90 KB + **cpu0 stack 23 KB** → effectively full |
| reserved ROM | `0x3FFE0000`–`0x3FFE7E30` | ~32 KB | ROM data (~2 KB) + **ROM stacks ~22.5 KB (reclaimable after dual-core boot)** |
| `dram2_seg` (heap) | `0x3FFE7E30`–`0x3FFF0000` | 96.5 KB | the `#[ram(reclaimed)]` heap (96 KB) → **maxed** (~464 B spare) |

- `RESERVE_DRAM = 0` in this build — esp-hal does **not** reserve 64 KB for BLE at
  link time; esp-radio's controller takes its memory from the heap at runtime
  (which is why ~55 KB of the 96 KB heap is already used, leaving ~41 KB free).
- Biggest `.bss` consumers: core-1 `APP_CORE_STACK` 32 KB, three display tile
  buffers 3×10 KB, the audio DMA stream buffer 16 KB (a `static`, **not** heap),
  BLE/embassy pools ~15 KB.
- **The heap can't grow for free:** `dram2_seg` is maxed and `dram_seg` is full.
  Levers (no PSRAM — it's slow and unneeded here):
  - **Reclaim the ~22.5 KB ROM stacks** by extending `dram2_seg` in the forked
    `esp-hal/ld/esp32/memory.x` (ESP-IDF does this as standard): heap 96 → ~118 KB,
    free 41 → ~63 KB. Lowest-effort, low-risk.
  - Shrink the 32 KB core-1 stack (needs a render stack-usage check) and add the
    saving as a second `esp_alloc` region.
- **But for audio you likely don't need to:** the 16 KB stream buffer is already a
  `static`, so the ~41 KB free heap covers a codec working set as-is — LC3 (~31 KB)
  fits with margin, sea-codec 3-bit (~37 KB) fits tightly, ADPCM is free.

## FPU note

The ESP32 (LX6) has a **single-precision (`f32`) hardware FPU**; `f64` is
software-emulated. lc3-codec's per-frame DSP is `f32` (`Scaler = f32`), so it uses
the FPU; its only `f64` use is one-time FFT/DCT twiddle-table precompute. The cost
is volume (MDCT + FFT per frame), not precision.

## Xtensa portability gotchas

To build `lc3-codec` and `sea-codec` for `xtensa-esp32-none-elf`, two transitive
issues needed local patches (vendored under [`vendor/`](vendor/)):

- **`radium` 0.7.0** (via `bitvec` → lc3-codec) assumes 64-bit atomics exist; its
  `build.rs` only downgrades for a hardcoded target list that omits Xtensa, so it
  references the nonexistent `AtomicU64`. Fix: add an `"xtensa"` arm setting
  `has_64 = false`. (radium ≥ 1.x uses `portable-atomic` and avoids this, but
  `bitvec` 1.0 pins radium `^0.7`.)
- **`sea-codec`** declares `crate-type = ["cdylib", "staticlib", "rlib"]`; the
  `cdylib`/`staticlib` artifacts require a `#[panic_handler]` + `#[global_allocator]`,
  fatal on `no_std`. Fix: vendor with `crate-type = ["rlib"]`.

## Encoding tools

Offline encoders (rust-script + ffmpeg, like the firmware's
[`scripts/encode_adpcm.rs`](scripts/encode_adpcm.rs)):

```bash
scripts/encode_adpcm.rs in.ogg out.adpcm                    # firmware codec (IMA ADPCM)
scripts/encode_sea.rs   in.ogg out.sea  --bitrate 3         # comparison: sea-codec
scripts/encode_lc3.rs   in.ogg out.lc3  --bytes-per-frame 80 # comparison: LC3
```

Each pipes ffmpeg PCM (mono, `--rate`) through the codec's own encoder, so the
output round-trips byte-identically with what the decoder reads. `.sea`/`.lc3` are
comparison assets; the firmware ships ADPCM.

## Caveats

- Built at `opt-level = "s"` (the project default). `opt-level = 3` would likely
  speed up LC3/sea (FFT/filter loops) more than ADPCM.
- CPU/RAM figures are decode-only; encoding is done offline on the host.
- SNR (§4) is objective but waveform-only — valid for ADPCM/sea, *not* for LC3
  (transform codec; judge by ear). No formal perceptual (PEAQ/MUSHRA) test done.
