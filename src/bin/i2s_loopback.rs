//! Serial (no-JTAG) ESP32 I2S reset_tx regression repro.
//!
//! Sends N consecutive one-shot `write()`s over signal loopback, each filled
//! with a unique marker, and checks every transfer's marker comes back. The
//! reset_tx defect (too-short reset pulse) leaves the TX core unable to restart,
//! so later transfers silently stop transmitting; with a working reset all N
//! transmit. RX is drained to idle before tallying, so capture lag is not
//! mistaken for a dropped transfer.
//!
//!   "i2s_test: RESULT PASS (100/100, .. ms)"       -> fixed reset_tx
//!   "i2s_test: RESULT FAIL (k/100 dropped, .. ms)"  -> broken reset_tx
//!
//! Point the local esp-hal patch (Cargo.toml) at a broken vs fixed build, then:
//!   cargo run --release --bin i2s_loopback

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::dma::DmaTxBuf;
use esp_hal::dma_descriptors;
use esp_hal::dma_rx_stream_buffer;
use esp_hal::gpio::{NoPin, Pin};
use esp_hal::i2s::master::{Channels, Config, DataFormat, I2s};
use esp_hal::time::Rate;
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

const N: usize = 100;
const BASE: u16 = 0x2000; // transfer i -> marker BASE + i

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let p = esp_hal::init(config);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 96 * 1024);
    let timg0 = esp_hal::timer::timg::TimerGroup::new(p.TIMG0);
    let sw = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw.software_interrupt0);

    let i2s = I2s::new(
        p.I2S1,
        p.DMA_I2S1,
        Config::new_tdm_philips()
            .with_sample_rate(Rate::from_hz(16000))
            .with_data_format(DataFormat::Data16Channel16)
            .with_channels(Channels::STEREO)
            .with_signal_loopback(true),
    )
    .unwrap();

    let (din, dout) = unsafe { p.GPIO2.degrade().split() };
    let i2s_tx = i2s.i2s_tx.with_bclk(NoPin).with_ws(NoPin).with_dout(dout).build();
    let i2s_rx = i2s.i2s_rx.with_bclk(NoPin).with_ws(NoPin).with_din(din).build();

    let rx_buffer = dma_rx_stream_buffer!(16384, 1024);
    let mut rx = i2s_rx.read(rx_buffer).map_err(|(e, _, _)| e).unwrap();

    let buffer = {
        static mut BUF: [u8; 256] = [0u8; 256];
        unsafe { &mut *core::ptr::addr_of_mut!(BUF) }
    };
    let descr = dma_descriptors!(0, 256).1;
    let mut tx_buffer = Some(DmaTxBuf::new(descr, buffer).unwrap());
    let mut tx = Some(i2s_tx);

    let mut seen = [false; N];
    let mut tmp = [0u8; 4096];

    macro_rules! scan {
        () => {{
            let avail = rx.available_bytes().min(tmp.len());
            if avail > 0 {
                let n = rx.pop(&mut tmp[..avail]);
                for w in tmp[..n].chunks_exact(2) {
                    let v = u16::from_le_bytes([w[0], w[1]]);
                    if v >= BASE && (v - BASE) < N as u16 {
                        seen[(v - BASE) as usize] = true;
                    }
                }
            }
        }};
    }

    let t0 = embassy_time::Instant::now();
    for i in 0..N {
        let mut b = tx_buffer.take().unwrap();
        let bytes = (BASE + i as u16).to_le_bytes();
        for chunk in b.as_mut_slice().chunks_mut(2) {
            chunk.copy_from_slice(&bytes);
        }
        let transfer = tx.take().unwrap().write(b).map_err(|(e, _, _)| e).unwrap();
        for _ in 0..1500 {
            scan!();
        }
        let (res, tx_back, buf_back) = transfer.wait();
        if res.is_err() {
            println!("i2s_test: RESULT FAIL (write {} errored: {:?})", i, res.err());
            loop {}
        }
        tx = Some(tx_back);
        tx_buffer = Some(buf_back);
    }
    // Exhaustive final drain: after TX stops the loopback clock, RX produces no
    // new data, so once it stays empty for a long streak everything transmitted
    // has been captured. Eliminates "tail" false-drops from capture lag.
    let mut empty_streak = 0u32;
    let mut guard = 0u32;
    while empty_streak < 200_000 && guard < 50_000_000 {
        guard += 1;
        let avail = rx.available_bytes().min(tmp.len());
        if avail > 0 {
            let n = rx.pop(&mut tmp[..avail]);
            for w in tmp[..n].chunks_exact(2) {
                let v = u16::from_le_bytes([w[0], w[1]]);
                if v >= BASE && (v - BASE) < N as u16 {
                    seen[(v - BASE) as usize] = true;
                }
            }
            empty_streak = 0;
        } else {
            empty_streak += 1;
        }
    }

    let ms = t0.elapsed().as_millis();
    let dropped = seen.iter().filter(|&&s| !s).count();
    if dropped == 0 {
        println!("i2s_test: RESULT PASS ({}/{}, {} ms)", N, N, ms);
    } else {
        println!("i2s_test: RESULT FAIL ({}/{} dropped, {} ms)", dropped, N, ms);
        // list first few missing transfer indices
        let mut listed = 0;
        for (i, &s) in seen.iter().enumerate() {
            if !s && listed < 12 {
                println!("  transfer {} (marker {:#06x}) not received", i, BASE + i as u16);
                listed += 1;
            }
        }
    }
    loop {}
}
