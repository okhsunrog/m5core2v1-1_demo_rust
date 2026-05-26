# ESP32 I2S DMA Workarounds

This project bypasses `esp-hal`'s I2S DMA layer and drives the ESP32 PDMA
directly via the PAC. Three separate bugs in `esp-hal` 1.1.1 (and the
`esp32` PAC 0.40) prevent the standard API from working on ESP32.

## Bug 1: `reset_tx()` too fast — `write_words` hangs on second call

**Symptom:** `I2sTx::write_words()` succeeds once, then the second call
hangs forever in `tx_start()` (spinning on `tx_idle`).

**Root cause:** `esp-hal`'s `reset_tx()` uses the `toggle()` helper, which
asserts and deasserts `tx_reset` / `tx_fifo_reset` / `out_rst` in
back-to-back register writes with no delay between them. The ESP32 I2S
hardware does not register the reset pulse — the peripheral remains in its
post-transfer state, and `tx_start()` cannot bring it out of idle.

**ESP-IDF comparison:** ESP-IDF's `i2s_ll_tx_reset()` also does back-to-back
set/clear with no delay. However, ESP-IDF only calls `reset` during
initial configuration — it never resets between writes. `i2s_write()`
pushes data into a continuously-running circular DMA ring. `esp-hal`'s
`write_words()` calls `reset_tx()` + `start_tx_transfer()` on every
invocation, which triggers the bug.

**Proof:** Adding a small delay (~100 iterations of `black_box(0)`) between
the assert and deassert register writes makes 20+ consecutive
`write_words()` calls succeed.

**Our fix:** We skip `tx_reset` entirely. Before each playback we only reset
`out_rst` (DMA outlink) and `tx_fifo_reset` (FIFO), matching what ESP-IDF
does in `i2s_tx_channel_start()`. The TX core state machine is never
reset after initial configuration.

**esp-hal code (broken):**
```rust
// esp-hal 1.1.1: src/i2s/master.rs, reset_tx()
self.regs().conf().toggle(|w, bit| {
    w.tx_reset().bit(bit);       // set then immediately clear
    w.tx_fifo_reset().bit(bit);  // same — too fast for hardware
});
self.regs().lc_conf().toggle(|w, bit| w.out_rst().bit(bit));
```

**Related esp-hal issue:** [#2440](https://github.com/esp-rs/esp-hal/issues/2440)
(I2S tests disabled on ESP32 due to reliability issues).

---

## Bug 2: Async circular DMA never wakes on ESP32

**Symptom:** `I2sWriteDmaTransferAsync::push().await` never completes. The
circular DMA transfer runs (speaker outputs sound from the initial buffer
fill), but the async task is never woken to refill buffers — resulting in
a short loop of the initial data.

**Root cause:** The ESP32 uses PDMA (not GDMA). The async waker
registration for PDMA DMA-done interrupts on Xtensa does not work
correctly. A contributor on [#2440](https://github.com/esp-rs/esp-hal/issues/2440)
noted: *"the first transfer is sent — it seems like the task is never
awakened from the interrupt."*

**Our fix:** We do not use `esp-hal`'s async I2S API at all. Instead we
manage the circular DMA descriptor ring via PAC and poll `int_raw` for
the completion flag in a blocking loop (see Bug 3 for why we poll
`int_raw` instead of `int_st`).

---

## Pitfall 3: `out_eof` vs `out_total_eof` naming confusion

**Symptom:** Polling `int_raw.out_total_eof()` never fires during circular
DMA, even though descriptors are being consumed.

**Root cause:** The ESP32 I2S has two distinct EOF interrupt flags:

| PAC accessor     | Bit | Meaning |
|------------------|-----|---------|
| `out_eof`        | 12  | A descriptor with `eof=1` was completed |
| `out_total_eof`  | 16  | The entire descriptor chain terminated |

For **circular DMA** (where the chain loops forever), `out_total_eof`
never fires. The correct flag is `out_eof` — it fires once per
descriptor since all our descriptors have `eof=1`.

The PAC bit positions are correct. The confusion arises because ESP-IDF
names the same bit 12 as `I2S_LL_EVENT_TX_EOF`, which looks like it
should map to `out_total_eof` but actually corresponds to `out_eof`.

**Fix:** Use `out_eof` (bit 12), not `out_total_eof` (bit 16):
```rust
// Poll for per-descriptor completion
if r.int_raw().read().out_eof().bit_is_set() { ... }
r.int_clr().write(|w| w.out_eof().clear_bit_by_one());
```

---

## Our DMA implementation

Since bugs 1 and 2 block normal `esp-hal` I2S usage on ESP32, we
implement the DMA ring ourselves in `src/audio.rs`, following ESP-IDF's
pattern (`components/esp_driver_i2s/i2s_common.c`):

1. **Use `esp-hal` only for I2S clock/pin configuration** — `I2s::new()`
   sets up clock dividers, Philips standard framing, and GPIO matrix
   routing. We call `.build()` to keep the `PeripheralGuard` alive, but
   never use `write_words()` or any DMA methods.

2. **Set up a circular DMA descriptor ring via PAC** — 4 descriptors ×
   2048 bytes each, linked in a circle, all with `eof=1`. Total static
   cost: ~8 KB for buffers + 48 bytes for descriptors.

3. **Start DMA once per playback** — reset only `out_rst` + `tx_fifo_reset`
   (not `tx_reset`), write the descriptor address to `out_link.addr`,
   set `out_link.start`, then set `tx_start`.

4. **Poll `int_raw` bit 12** for descriptor completion. When a descriptor
   finishes, read `out_eof_des_addr` to find which one, refill its
   buffer with the next chunk of decoded ADPCM audio.

5. **Stop DMA after playback** — clear `tx_start`, wait for `tx_idle`,
   set `out_link.stop`.

This approach uses zero heap allocation, produces gap-free audio, and
allows repeated playback without the reset bug.

---

## Hardware details (M5Stack Core2 v1.1)

| Signal | GPIO | Notes |
|--------|------|-------|
| BCLK   | 12   | I2S bit clock |
| LRCK   | 0    | I2S word select (strapping pin — safe after boot) |
| DOUT   | 2    | I2S data out (strapping pin — safe after boot) |
| SPK_EN | —    | AXP2101 ALDO3 @ 3.3V (NS4168 CTRL pin → selects right channel) |

- **I2S peripheral:** I2S1 (matching M5Unified's configuration)
- **NS4168 format:** Standard I2S Philips, 16-bit, stereo
- **ADPCM encoding:** IMA ADPCM at 8 kHz, 4 bits/sample (4:1 compression)
