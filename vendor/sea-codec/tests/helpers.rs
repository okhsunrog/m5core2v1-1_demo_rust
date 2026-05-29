use sea_codec::{encoder::EncoderSettings, sea_decode, sea_encode};

pub struct EncodeDecodeOutput {
    pub encoded: Vec<u8>,
    pub decoded: Vec<i16>,
    pub compression_ratio: f64,
}

pub fn encode_decode(
    input_samples: &[i16],
    sample_rate: u32,
    channels: u32,
    settings: EncoderSettings,
) -> EncodeDecodeOutput {
    let encoded = sea_encode(input_samples, sample_rate, channels, settings);

    let encoded_len = encoded.len();
    let decoded = sea_decode(&encoded);

    EncodeDecodeOutput {
        encoded,
        decoded: decoded.samples,
        compression_ratio: (input_samples.len() * 2) as f64 / encoded_len as f64,
    }
}

pub const TEST_SAMPLE_RATE: u32 = 44100;

fn write_square_wave(signal: &mut [f32], gain: f32, frequency: f32) {
    let period = TEST_SAMPLE_RATE as f32 / frequency;
    for (i, s) in signal.iter_mut().enumerate() {
        if (i % period as usize) < (period / 2.0) as usize {
            *s += gain * 1.0;
        } else {
            *s += gain * -1.0;
        }
    }
}

fn write_sine_wave(signal: &mut [f32], gain: f32, frequency: f32) {
    let angular_frequency = 2.0 * std::f32::consts::PI * frequency / TEST_SAMPLE_RATE as f32;

    for (i, sample) in signal.iter_mut().enumerate() {
        let sine_value = (angular_frequency * i as f32).sin();
        *sample += gain * sine_value;
    }
}

fn signal_chunk(signal: &mut [f32], start_percent: f32, end_percent: f32) -> &mut [f32] {
    assert!((0.0..=1.0).contains(&start_percent));
    assert!((0.0..=1.0).contains(&end_percent));
    assert!(start_percent <= end_percent);

    let start_index = (signal.len() as f32 * start_percent) as usize;
    let end_index = (signal.len() as f32 * end_percent) as usize;

    &mut signal[start_index..end_index]
}

fn mono_to_multi(mono_signal: &[f32], channels: u32) -> Vec<f32> {
    let channel_delay = TEST_SAMPLE_RATE / 25;

    let total_samples = mono_signal.len() + (channels as usize - 1) * channel_delay as usize;
    let mut multi_signal = vec![0.0; total_samples * channels as usize];

    for (i, &sample) in mono_signal.iter().enumerate() {
        for channel in 0..channels {
            let delay = channel_delay * channel;
            let index = (i + delay as usize) * channels as usize + channel as usize;
            if index < multi_signal.len() {
                multi_signal[index] = sample;
            }
        }
    }

    multi_signal
}

pub fn gen_test_signal(channels: u32, samples: usize) -> Vec<i16> {
    let mono_signal: &mut [f32] = &mut vec![0f32; samples];
    write_square_wave(signal_chunk(mono_signal, 0.0, 0.3), 0.5, 440.0);
    write_square_wave(signal_chunk(mono_signal, 0.1, 0.2), 0.3, 2150.1);
    write_sine_wave(signal_chunk(mono_signal, 0.1, 0.7), 0.5, 105.0);
    write_square_wave(signal_chunk(mono_signal, 0.6, 0.7), 0.5, 14000.0);
    write_sine_wave(signal_chunk(mono_signal, 0.5, 0.8), 0.8, 12000.0);
    write_sine_wave(signal_chunk(mono_signal, 0.8, 0.9), 1.0, 440.0);

    let multi_signal = mono_to_multi(mono_signal, channels);
    multi_signal
        .iter()
        .map(|s| ((*s).clamp(-1.0, 1.0) * i16::MAX as f32) as i16)
        .collect()
}

#[derive(Debug)]
pub struct AudioQualityStats {
    pub rms: f64,
    pub psnr: f64,
}

pub fn get_audio_quality(a: &[i16], b: &[i16]) -> AudioQualityStats {
    assert_eq!(a.len(), b.len());

    let mut sum = 0.0f64;
    for i in 0..a.len() {
        let a_float = a[i] as f64 / i16::MAX as f64;
        let b_float = b[i] as f64 / i16::MAX as f64;
        let diff = a_float - b_float;
        sum += diff * diff;
    }

    let rms: f64 = (sum / a.len() as f64).sqrt();
    let psnr: f64 = -20.0 * (2.0 / rms).log10();

    AudioQualityStats { rms, psnr }
}
