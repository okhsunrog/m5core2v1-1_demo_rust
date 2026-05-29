use std::{error::Error, fs::File, path::Path};

use hound::{SampleFormat, WavReader, WavSpec, WavWriter};

pub struct Wave {
    pub samples: Vec<i16>,
    pub channels: u32,
    pub sample_rate: u32,
}

pub fn read_wav(path: &Path) -> Result<Wave, Box<dyn Error>> {
    let mut reader = WavReader::open(path)?;
    let spec = reader.spec();

    if spec.channels > 2 {
        return Err("More than 2 channels are not supported".into());
    }

    let samples_iter: Box<dyn Iterator<Item = i16>> =
        match (spec.sample_format, spec.bits_per_sample) {
            (SampleFormat::Int, 8) => Box::new(reader.samples::<i8>().map(|s| {
                let s = s.unwrap();
                (s as i16) << 8
            })),
            (SampleFormat::Int, 16) => Box::new(reader.samples::<i16>().map(|s| s.unwrap())),
            (SampleFormat::Int, 24) => Box::new(reader.samples::<i32>().map(|s| {
                let s = s.unwrap();
                ((s as f32 / (1 << 23) as f32) * i16::MAX as f32).round() as i16
            })),
            (SampleFormat::Int, 32) => Box::new(reader.samples::<i32>().map(|s| {
                let s = s.unwrap();
                ((s as f32 / i32::MAX as f32) * i16::MAX as f32).round() as i16
            })),
            (SampleFormat::Float, 32) => Box::new(reader.samples::<f32>().map(|s| {
                let s = s.unwrap();
                (s * i16::MAX as f32).round() as i16
            })),
            (format, bits) => {
                return Err(format!("Unsupported format: {:?} with {} bits", format, bits).into())
            }
        };

    let samples: Vec<i16> = samples_iter.collect();

    Ok(Wave {
        samples,
        channels: spec.channels as u32,
        sample_rate: spec.sample_rate,
    })
}

pub fn write_wav(
    wave: &[i16],
    channels: u16,
    sample_rate: u32,
    output_path: &str,
) -> Result<(), Box<dyn Error>> {
    let spec = WavSpec {
        channels,
        sample_rate,
        bits_per_sample: 16,
        sample_format: SampleFormat::Int,
    };

    let mut writer: WavWriter<std::io::BufWriter<File>> =
        WavWriter::create(output_path, spec).unwrap();
    // let start = sample.get_start() as usize;
    // let end = sample.get_end() as usize;
    for item in wave {
        writer.write_sample(*item)?;
    }
    writer.finalize()?;

    Ok(())
}
