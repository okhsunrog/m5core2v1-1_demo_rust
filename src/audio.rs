//! I2S audio playback via esp-hal's async DMA streaming API.
//!
//! Notification sounds are stored as IMA ADPCM (4:1 vs PCM) and decoded on the
//! fly into a circular `DmaTxStreamBuf`. A single continuous DMA transfer is
//! kept fed via `wait_for_available_async()` + `push_with()`, so playback is
//! gap-free without a full PCM buffer in RAM.

use audio_codec_algorithms::{AdpcmImaState, decode_adpcm_ima};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use esp_hal::dma_tx_stream_buffer;
use esp_hal::i2s::master::{Config, DataFormat, I2s};
use esp_hal::time::Rate;
use log::info;

static SOUND_CHIME: &[u8] = include_bytes!("../sounds/chime.adpcm");
static SOUND_INFO: &[u8] = include_bytes!("../sounds/info.adpcm");
static SOUND_PING: &[u8] = include_bytes!("../sounds/ping.adpcm");

#[derive(Clone, Copy)]
pub enum Sfx {
    Chime,
    Info,
    Ping,
}

pub static PLAY_SIGNAL: Signal<CriticalSectionRawMutex, Sfx> = Signal::new();

const SAMPLE_RATE: u32 = 44_100;

/// Streaming ADPCM decoder: yields one mono i16 sample per call.
struct Source {
    raw: &'static [u8],
    pos: usize,
    high_pending: bool,
    state: AdpcmImaState,
}

impl Source {
    fn new(raw: &'static [u8]) -> Self {
        Self { raw, pos: 0, high_pending: false, state: AdpcmImaState::new() }
    }

    fn finished(&self) -> bool {
        self.pos >= self.raw.len()
    }

    fn next_sample(&mut self) -> Option<i16> {
        if self.pos >= self.raw.len() {
            return None;
        }
        let byte = self.raw[self.pos];
        let nibble = if self.high_pending {
            self.pos += 1;
            self.high_pending = false;
            byte >> 4
        } else {
            self.high_pending = true;
            byte & 0x0f
        };
        Some(decode_adpcm_ima(nibble, &mut self.state))
    }
}

/// Decode samples into `buf` as stereo 16-bit LE frames (mono duplicated to
/// both channels). Returns bytes written, 0 once the clip is exhausted.
fn fill(buf: &mut [u8], src: &mut Source) -> usize {
    let mut n = 0;
    while n + 4 <= buf.len() {
        match src.next_sample() {
            Some(s) => {
                let b = s.to_le_bytes();
                buf[n] = b[0];
                buf[n + 1] = b[1];
                buf[n + 2] = b[0];
                buf[n + 3] = b[1];
                n += 4;
            }
            None => break,
        }
    }
    n
}

#[embassy_executor::task]
pub async fn task(
    i2s: esp_hal::peripherals::I2S1<'static>,
    dma: esp_hal::peripherals::DMA_I2S1<'static>,
    bclk: esp_hal::peripherals::GPIO12<'static>,
    ws: esp_hal::peripherals::GPIO0<'static>,
    dout: esp_hal::peripherals::GPIO2<'static>,
) {
    let i2s = I2s::new(
        i2s,
        dma,
        Config::new_tdm_philips()
            .with_sample_rate(Rate::from_hz(SAMPLE_RATE))
            .with_data_format(DataFormat::Data16Channel16),
    )
    .unwrap()
    .into_async();

    let i2s_tx = i2s.i2s_tx.with_bclk(bclk).with_ws(ws).with_dout(dout).build();

    // write() consumes the channel + buffer; the transfer hands them back on
    // completion, so we thread both through the playback loop.
    let mut tx = Some(i2s_tx);
    let mut stream_buf = Some(dma_tx_stream_buffer!(4092 * 4, 2048));

    info!("I2S audio ready");

    loop {
        let sfx = PLAY_SIGNAL.wait().await;
        let mut src = Source::new(match sfx {
            Sfx::Chime => SOUND_CHIME,
            Sfx::Info => SOUND_INFO,
            Sfx::Ping => SOUND_PING,
        });

        let mut buf = stream_buf.take().unwrap();
        buf.push_with(|b| fill(b, &mut src)); // prefill so DMA starts with audio

        let mut transfer = tx.take().unwrap().write(buf).map_err(|(e, _, _)| e).unwrap();
        while !src.finished() {
            transfer.wait_for_available_async().await.unwrap();
            transfer.push_with(|b| fill(b, &mut src));
        }

        // Stop refilling; the remaining queued descriptors drain and the
        // transfer completes, returning the channel + buffer for reuse.
        let (res, tx_back, buf_back) = transfer.wait_async().await;
        if let Err(e) = res {
            info!("I2S transfer error: {:?}", e);
        }
        tx = Some(tx_back);
        stream_buf = Some(buf_back);
    }
}
