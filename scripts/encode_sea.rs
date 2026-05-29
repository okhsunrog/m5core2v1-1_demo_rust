#!/usr/bin/env rust-script
//! Encode any ffmpeg-readable audio to a sea-codec `.sea` stream.
//!
//! A comparison/experiment tool for CODEC_BENCHMARK.md — the firmware currently
//! plays IMA ADPCM (see scripts/encode_adpcm.rs). ffmpeg decodes + resamples +
//! downmixes to mono s16le; sea-codec's own `sea_encode` writes the `seac`
//! container, byte-identical to what `sea_codec::decoder` reads back.
//!
//! Usage: scripts/encode_sea.rs <input> <output.sea> [--rate 44100]
//!                              [--bitrate 3] [--chunk-size 1000]
//!
//! `--rate` must match the playback rate (sea-codec does not resample on
//! decode). `--bitrate` is residual bits/sample (1..=8; 3 ≈ ADPCM size, cleaner;
//! 2 ≈ smaller). `--chunk-size` must be 200..=32000 and a multiple of 20; it
//! trades file size against decoder RAM (smaller chunk → less heap but bigger
//! file; the decoder has a ~36 KB floor regardless — see CODEC_BENCHMARK.md).
//!
//! ```cargo
//! [package]
//! edition = "2021"
//!
//! [dependencies]
//! sea-codec = "0.7"
//! ```

use sea_codec::encoder::EncoderSettings;
use std::process::{Command, Stdio, exit};

fn usage() -> ! {
    eprintln!(
        "usage: encode_sea.rs <input> <output.sea> [--rate <Hz>] [--bitrate <1-8>] [--chunk-size <n>]"
    );
    exit(2);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 2 {
        usage();
    }
    let input = &args[0];
    let output = &args[1];
    let mut rate: u32 = 44_100;
    let mut bitrate: f32 = 3.0;
    let mut chunk_size: u16 = 1000;
    let mut i = 2;
    while i < args.len() {
        let val = || args.get(i + 1).cloned().unwrap_or_else(|| usage());
        match args[i].as_str() {
            "--rate" => rate = val().parse().unwrap_or_else(|_| usage()),
            "--bitrate" => bitrate = val().parse().unwrap_or_else(|_| usage()),
            "--chunk-size" => chunk_size = val().parse().unwrap_or_else(|_| usage()),
            _ => usage(),
        }
        i += 2;
    }

    // Match sea-codec's encoder validation up front, with clearer messages.
    if !(1.0..=8.0).contains(&bitrate) {
        eprintln!("--bitrate must be in 1..=8 (got {bitrate})");
        exit(2);
    }
    if !(200..=32_000).contains(&chunk_size) || chunk_size % 20 != 0 {
        eprintln!("--chunk-size must be 200..=32000 and a multiple of 20 (got {chunk_size})");
        exit(2);
    }

    // ffmpeg → raw signed-16 LE, mono, resampled to `rate`.
    let pcm = Command::new("ffmpeg")
        .args([
            "-hide_banner", "-loglevel", "error", "-i", input,
            "-f", "s16le", "-ac", "1", "-af", &format!("aresample={rate}:resampler=soxr"), "-",
        ])
        .stdout(Stdio::piped())
        .output()
        .expect("failed to spawn ffmpeg");
    if !pcm.status.success() {
        eprintln!("ffmpeg failed: {}", String::from_utf8_lossy(&pcm.stderr));
        exit(1);
    }

    let samples: Vec<i16> = pcm
        .stdout
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect();

    let settings = EncoderSettings {
        residual_bits: bitrate,
        frames_per_chunk: chunk_size,
        vbr: false,
        ..Default::default() // scale_factor_bits: 4, scale_factor_frames: 20
    };
    let encoded = sea_codec::sea_encode(&samples, rate, 1, settings);

    std::fs::write(output, &encoded).expect("write failed");
    let secs = samples.len() as f32 / rate as f32;
    let pcm_bytes = samples.len() * 2;
    println!(
        "{input}: {} samples ({secs:.2}s @ {rate} Hz mono), {pcm_bytes} -> {} bytes ({:.1}x) at {bitrate}-bit",
        samples.len(),
        encoded.len(),
        pcm_bytes as f32 / encoded.len().max(1) as f32,
    );
}
