//! Decode-cost benchmark for the Ping clip on real ESP32 hardware.
//!
//! Compares IMA ADPCM (current), sea-codec, and lc3-codec by decoding the
//! whole clip into small reusable buffers (the full clip is ~1.18 MB PCM,
//! too big for RAM) and timing each with the microsecond clock at 240 MHz.
//!
//! Run with: cargo run --release --bin bench_codecs

#![no_std]
#![no_main]

extern crate alloc;
use alloc::{vec, vec::Vec};

use audio_codec_algorithms::{AdpcmImaState, decode_adpcm_ima};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::time::Instant;
use esp_println::println;
use lc3_codec::common::complex::Complex;
use lc3_codec::common::config::{FrameDuration, Lc3Config, SamplingFrequency};
use lc3_codec::decoder::lc3_decoder::Lc3Decoder;
use sea_codec::decoder::SeaDecoder;

esp_bootloader_esp_idf::esp_app_desc!();

static PING_ADPCM: &[u8] = include_bytes!("../../sounds/ping.adpcm");
static PING_SEA: &[u8] = include_bytes!("../../sounds/ping.sea");
static PING_LC3: &[u8] = include_bytes!("../../sounds/ping.lc3");

const RATE: u32 = 44_100;
const LC3_BYTES_PER_FRAME: usize = 80;

fn report(name: &str, micros: u64, samples: u32) {
    // Clip duration in micros vs decode time → realtime headroom.
    let clip_us = samples as u64 * 1_000_000 / RATE as u64;
    let xrt = clip_us as f32 / micros as f32;
    let us_per_kspl = micros as f32 * 1000.0 / samples as f32;
    println!(
        "{:<20} {:>8} us  {:>7} samples  {:>7.1}x realtime  {:>6.2} us/1k-samples",
        name, micros, samples, xrt, us_per_kspl
    );
}

#[esp_hal::main]
fn main() -> ! {
    let _p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 100 * 1024);

    let base = esp_alloc::HEAP.used();
    println!("\n=== codec decode bench — ESP32 @ 240 MHz, Ping clip ===");
    println!("(decode whole clip into small buffers; checksum prevents elision)");
    println!("heap: {} B free, {} B used at baseline\n", esp_alloc::HEAP.free(), base);

    // --- IMA ADPCM (current) ---
    {
        let t = Instant::now();
        let mut st = AdpcmImaState::new();
        let mut acc: i64 = 0;
        for &byte in PING_ADPCM {
            acc += decode_adpcm_ima(byte & 0x0f, &mut st) as i64;
            acc += decode_adpcm_ima(byte >> 4, &mut st) as i64;
        }
        let us = t.elapsed().as_micros();
        report("IMA ADPCM", us, (PING_ADPCM.len() * 2) as u32);
        // AdpcmImaState is ~4 bytes on the stack; zero heap.
        println!("  heap: +{} B   (chk {acc})", esp_alloc::HEAP.used() - base);
    }

    // --- sea-codec (3-bit CBR) ---
    {
        let mut dec = SeaDecoder::from_slice(PING_SEA).unwrap();
        let setup = esp_alloc::HEAP.used() - base;
        let mut frame: Vec<i16> = Vec::new();
        let mut acc: i64 = 0;
        let mut samples: u32 = 0;
        let mut peak = setup;
        let t = Instant::now();
        loop {
            frame.clear();
            let more = dec.decode_frame(&mut frame).unwrap();
            for &s in &frame {
                acc += s as i64;
            }
            samples += frame.len() as u32;
            if !more {
                break;
            }
        }
        let us = t.elapsed().as_micros();
        peak = peak.max(esp_alloc::HEAP.used() - base); // frame Vec at full capacity
        report("sea-codec 3-bit", us, samples);
        println!("  heap: decoder +{setup} B, peak +{peak} B   (chk {acc})");
    }

    // --- lc3-codec (80 B/frame) ---
    {
        let freq = SamplingFrequency::Hz44100;
        let dur = FrameDuration::TenMs;
        let nf = Lc3Config::new(freq, dur).nf;
        let (slen, clen) = Lc3Decoder::calc_working_buffer_lengths(1, dur, freq);
        let mut sbuf = vec![0.0; slen];
        let mut cbuf = vec![Complex::default(); clen];
        let mut dec = Lc3Decoder::new(1, dur, freq, &mut sbuf, &mut cbuf);
        let mut out = vec![0i16; nf];
        let setup = esp_alloc::HEAP.used() - base;

        let t = Instant::now();
        let mut acc: i64 = 0;
        let mut samples: u32 = 0;
        for chunk in PING_LC3.chunks(LC3_BYTES_PER_FRAME) {
            dec.decode_frame(16, 0, chunk, &mut out).unwrap();
            for &s in &out {
                acc += s as i64;
            }
            samples += nf as u32;
        }
        let us = t.elapsed().as_micros();
        let peak = esp_alloc::HEAP.used() - base;
        report("lc3-codec 80B/frame", us, samples);
        println!(
            "  heap: total +{peak} B  (scaler {} B f32 + complex {} B + chans/out)   (chk {acc})",
            slen * 4,
            clen * core::mem::size_of::<Complex>()
        );
    }

    println!("\n=== done ===");
    loop {}
}
