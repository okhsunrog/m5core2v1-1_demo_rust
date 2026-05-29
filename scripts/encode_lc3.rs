#!/usr/bin/env rust-script
//! Encode any ffmpeg-readable audio to a raw LC3 frame stream (`.lc3`).
//!
//! A comparison/experiment tool for CODEC_BENCHMARK.md — the firmware currently
//! plays IMA ADPCM (see scripts/encode_adpcm.rs). Output is simply each LC3
//! frame's bytes concatenated (no container): fixed `--bytes-per-frame` CBR,
//! 10 ms frames, which is exactly what src/bin/bench_codecs.rs decodes back by
//! chunking the file into equal frames.
//!
//! Usage: scripts/encode_lc3.rs <input> <output.lc3> [--rate 44100]
//!                              [--bytes-per-frame 80]
//!
//! Bitrate = bytes-per-frame * 800 bits/s (10 ms frames). e.g. 80 B → 64 kbps.
//! `--rate` must be an LC3 rate: 8000/16000/24000/32000/44100/48000.
//!
//! ```cargo
//! [package]
//! edition = "2021"
//!
//! [dependencies]
//! lc3-codec = "0.2"
//! ```

use lc3_codec::common::complex::Complex;
use lc3_codec::common::config::{FrameDuration, Lc3Config, SamplingFrequency};
use lc3_codec::encoder::lc3_encoder::Lc3Encoder;
use std::process::{Command, Stdio, exit};

fn usage() -> ! {
    eprintln!("usage: encode_lc3.rs <input> <output.lc3> [--rate <Hz>] [--bytes-per-frame <n>]");
    exit(2);
}

fn freq_of(rate: u32) -> SamplingFrequency {
    match rate {
        8000 => SamplingFrequency::Hz8000,
        16000 => SamplingFrequency::Hz16000,
        24000 => SamplingFrequency::Hz24000,
        32000 => SamplingFrequency::Hz32000,
        44100 => SamplingFrequency::Hz44100,
        48000 => SamplingFrequency::Hz48000,
        _ => {
            eprintln!("--rate must be 8000/16000/24000/32000/44100/48000 (got {rate})");
            exit(2);
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 2 {
        usage();
    }
    let input = &args[0];
    let output = &args[1];
    let mut rate: u32 = 44_100;
    let mut bpf: usize = 80;
    let mut i = 2;
    while i < args.len() {
        let val = || args.get(i + 1).cloned().unwrap_or_else(|| usage());
        match args[i].as_str() {
            "--rate" => rate = val().parse().unwrap_or_else(|_| usage()),
            "--bytes-per-frame" => bpf = val().parse().unwrap_or_else(|_| usage()),
            _ => usage(),
        }
        i += 2;
    }

    let freq = freq_of(rate);
    let dur = FrameDuration::TenMs;

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

    let nf = Lc3Config::new(freq, dur).nf; // samples per 10 ms frame
    let (i_len, s_len, c_len) = Lc3Encoder::calc_working_buffer_lengths(1, dur, freq);
    let mut ibuf = vec![0; i_len];
    let mut sbuf = vec![0.0; s_len];
    let mut cbuf = vec![Complex::default(); c_len];
    let mut enc = Lc3Encoder::new(1, dur, freq, &mut ibuf, &mut sbuf, &mut cbuf);

    let mut frame = vec![0i16; nf];
    let mut out_frame = vec![0u8; bpf];
    let mut encoded = Vec::new();
    for chunk in samples.chunks(nf) {
        frame[..chunk.len()].copy_from_slice(chunk);
        frame[chunk.len()..].fill(0); // zero-pad the last partial frame
        enc.encode_frame(0, &frame, &mut out_frame).unwrap();
        encoded.extend_from_slice(&out_frame);
    }

    std::fs::write(output, &encoded).expect("write failed");
    let secs = samples.len() as f32 / rate as f32;
    let pcm_bytes = samples.len() * 2;
    let kbps = bpf as f32 * 800.0 / 1000.0;
    println!(
        "{input}: {} samples ({secs:.2}s @ {rate} Hz mono), {pcm_bytes} -> {} bytes ({:.1}x) at {bpf} B/frame ({kbps:.0} kbps)",
        samples.len(),
        encoded.len(),
        pcm_bytes as f32 / encoded.len().max(1) as f32,
    );
}
