#!/usr/bin/env rust-script
//! Convert any ffmpeg-readable audio file to a raw IMA ADPCM nibble stream
//! playable by src/audio.rs (continuous stream, one AdpcmImaState per file —
//! NOT WAV block-formatted, which ffmpeg's adpcm_ima_wav codec produces).
//!
//! Uses the same audio_codec_algorithms::encode_adpcm_ima the firmware decodes
//! with, so the round-trip is guaranteed byte-correct.
//!
//! Usage: scripts/encode_adpcm.rs <input> <output.adpcm> [--rate 44100]
//!
//! ```cargo
//! [package]
//! edition = "2024"
//!
//! [dependencies]
//! audio-codec-algorithms = "0.8"
//! ```

use audio_codec_algorithms::{AdpcmImaState, encode_adpcm_ima};
use std::process::{Command, Stdio, exit};

fn usage() -> ! {
    eprintln!("usage: encode_adpcm.rs <input> <output.adpcm> [--rate <Hz>]");
    exit(2);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 2 {
        usage();
    }
    let input = &args[0];
    let output = &args[1];
    let mut rate: u32 = 44100;
    let mut i = 2;
    while i < args.len() {
        match args[i].as_str() {
            "--rate" => {
                rate = args.get(i + 1).and_then(|s| s.parse().ok()).unwrap_or_else(|| usage());
                i += 2;
            }
            _ => usage(),
        }
    }

    let pcm = Command::new("ffmpeg")
        .args(["-hide_banner", "-loglevel", "error", "-i", input,
               "-f", "s16le", "-ac", "1", "-ar", &rate.to_string(), "-"])
        .stdout(Stdio::piped())
        .output()
        .expect("failed to spawn ffmpeg");
    if !pcm.status.success() {
        eprintln!("ffmpeg failed: {}", String::from_utf8_lossy(&pcm.stderr));
        exit(1);
    }
    let pcm = pcm.stdout;

    let mut state = AdpcmImaState::new();
    let n_samples = pcm.len() / 2;
    let mut out = Vec::with_capacity(n_samples.div_ceil(2));
    let mut pending: Option<u8> = None;
    for chunk in pcm.chunks_exact(2) {
        let sample = i16::from_le_bytes([chunk[0], chunk[1]]);
        let nibble = encode_adpcm_ima(sample, &mut state);
        match pending.take() {
            None => pending = Some(nibble),
            Some(lo) => out.push(lo | (nibble << 4)),
        }
    }
    if let Some(lo) = pending {
        out.push(lo);
    }

    std::fs::write(output, &out).expect("write failed");
    let secs = n_samples as f32 / rate as f32;
    println!(
        "{input}: {n_samples} samples ({secs:.2}s @ {rate} Hz), {} -> {} bytes ({:.1}x)",
        pcm.len(),
        out.len(),
        pcm.len() as f32 / out.len() as f32,
    );
}
