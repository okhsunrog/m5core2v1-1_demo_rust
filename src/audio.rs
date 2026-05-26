use audio_codec_algorithms::{AdpcmImaState, decode_adpcm_ima};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use esp_hal::dma_descriptors;
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
const NUM_DESC: usize = 4;
const DESC_BUF_SIZE: usize = 2048;

// DMA descriptor matching ESP32 PDMA lldesc_t layout.
#[repr(C, align(4))]
struct LlDesc {
    // [11:0]=size [23:12]=length [28:24]=offset [29]=sosf [30]=eof [31]=owner
    flags: u32,
    buf: *const u8,
    next: *const LlDesc,
}

unsafe impl Sync for LlDesc {}

impl LlDesc {
    const fn zeroed() -> Self {
        Self {
            flags: 0,
            buf: core::ptr::null(),
            next: core::ptr::null(),
        }
    }

    fn configure(&mut self, buf: *const u8, size: usize, next: *const LlDesc) {
        self.flags = (size as u32 & 0xFFF)
            | (((size as u32) & 0xFFF) << 12)
            | (1 << 30) // eof=1: interrupt on each descriptor
            | (1 << 31); // owner=1: hardware owns
        self.buf = buf;
        self.next = next;
    }
}

#[repr(C, align(4))]
struct AlignedBuf([u8; DESC_BUF_SIZE]);

static mut DESCS: [LlDesc; NUM_DESC] = [const { LlDesc::zeroed() }; NUM_DESC];
static mut BUFS: [AlignedBuf; NUM_DESC] = [const { AlignedBuf([0u8; DESC_BUF_SIZE]) }; NUM_DESC];

struct Source {
    raw: &'static [u8],
    pos: usize,
    high_pending: bool,
    state: AdpcmImaState,
}

impl Source {
    fn new(raw: &'static [u8]) -> Self {
        Self {
            raw,
            pos: 0,
            high_pending: false,
            state: AdpcmImaState::new(),
        }
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

    fn fill_stereo_buf(&mut self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        while pos + 3 < buf.len() {
            match self.next_sample() {
                Some(sample) => {
                    let b = sample.to_le_bytes();
                    buf[pos] = b[0];
                    buf[pos + 1] = b[1];
                    buf[pos + 2] = b[0];
                    buf[pos + 3] = b[1];
                    pos += 4;
                }
                None => break,
            }
        }
        if pos < buf.len() {
            buf[pos..].fill(0);
        }
        pos
    }
}

fn regs() -> &'static esp32::i2s1::RegisterBlock {
    unsafe { &*esp32::I2S1::PTR }
}

fn init_dma_ring() {
    unsafe {
        let descs = &raw mut DESCS;
        let bufs = &raw mut BUFS;
        for i in 0..NUM_DESC {
            let next = &(*descs)[(i + 1) % NUM_DESC] as *const LlDesc;
            (*descs)[i].configure((*bufs)[i].0.as_ptr(), DESC_BUF_SIZE, next);
        }
    }
}

fn start_dma_tx() {
    let r = regs();

    // Reset DMA outlink + FIFO (NOT tx_reset — that's what breaks esp-hal).
    // Matches ESP-IDF: i2s_hal_tx_reset_dma + i2s_hal_tx_reset_fifo.
    r.lc_conf().modify(|_, w| w.out_rst().set_bit());
    r.conf().modify(|_, w| w.tx_fifo_reset().set_bit());
    for _ in 0..100 {
        core::hint::black_box(0u32);
    }
    r.lc_conf().modify(|_, w| w.out_rst().clear_bit());
    r.conf().modify(|_, w| w.tx_fifo_reset().clear_bit());

    r.int_clr().write(|w| {
        w.out_done()
            .clear_bit_by_one()
            .out_eof()
            .clear_bit_by_one()
            .out_total_eof()
            .clear_bit_by_one()
    });

    // Point DMA at our descriptor ring and start
    let desc_addr = (&raw const DESCS) as u32;
    r.out_link()
        .modify(|_, w| unsafe { w.outlink_addr().bits(desc_addr & 0xF_FFFF) });
    r.out_link().modify(|_, w| w.outlink_start().set_bit());

    // Start I2S TX
    r.conf().modify(|_, w| w.tx_start().set_bit());
}

fn stop_dma_tx() {
    let r = regs();
    r.conf().modify(|_, w| w.tx_start().clear_bit());
    // Wait for TX to go idle
    while !r.state().read().tx_idle().bit_is_set() {
        core::hint::black_box(0u32);
    }
    r.out_link().modify(|_, w| w.outlink_stop().set_bit());
}

/// Poll for `out_eof` with async yield — fires each time DMA finishes a
/// descriptor with eof=1. Yields to the embassy executor between polls
/// so other Core 0 tasks (touch, BLE, I2C) stay responsive.
async fn wait_eof() -> u32 {
    let r = regs();
    loop {
        if r.int_raw().read().out_eof().bit_is_set() {
            let addr = r.out_eof_des_addr().read().bits();
            r.int_clr().write(|w| w.out_eof().clear_bit_by_one());
            return addr;
        }
        embassy_futures::yield_now().await;
    }
}

fn desc_index(eof_addr: u32) -> usize {
    let base = (&raw const DESCS) as u32;
    let offset = eof_addr.wrapping_sub(base) as usize;
    offset / core::mem::size_of::<LlDesc>()
}

#[embassy_executor::task]
pub async fn task(
    i2s: esp_hal::peripherals::I2S1<'static>,
    dma: esp_hal::peripherals::DMA_I2S1<'static>,
    bclk: esp_hal::peripherals::GPIO12<'static>,
    ws: esp_hal::peripherals::GPIO0<'static>,
    dout: esp_hal::peripherals::GPIO2<'static>,
) {
    info!("Initializing I2S audio...");

    // Use esp-hal for clock calc, pin routing, and peripheral guard only.
    let i2s = I2s::new(
        i2s,
        dma,
        Config::new_tdm_philips()
            .with_sample_rate(Rate::from_hz(SAMPLE_RATE))
            .with_data_format(DataFormat::Data16Channel16),
    )
    .unwrap();

    let (_, tx_desc) = dma_descriptors!(0, 4);
    let _i2s_tx = i2s
        .i2s_tx
        .with_bclk(bclk)
        .with_ws(ws)
        .with_dout(dout)
        .build(tx_desc);

    info!("I2S audio ready");

    loop {
        let sfx = PLAY_SIGNAL.wait().await;
        let adpcm = match sfx {
            Sfx::Chime => SOUND_CHIME,
            Sfx::Info => SOUND_INFO,
            Sfx::Ping => SOUND_PING,
        };

        play_clip(adpcm).await;
    }
}

async fn play_clip(adpcm_data: &'static [u8]) {
    let mut source = Source::new(adpcm_data);

    for i in 0..NUM_DESC {
        #[allow(clippy::deref_addrof)]
        let buf = unsafe { &mut (*(&raw mut BUFS))[i].0 };
        source.fill_stereo_buf(buf);
    }

    init_dma_ring();
    start_dma_tx();
    info!("DMA started");

    let mut refills = 0u32;
    while !source.finished() {
        let eof_addr = wait_eof().await;
        let idx = desc_index(eof_addr);
        let buf = unsafe { &mut BUFS[idx].0 };
        source.fill_stereo_buf(buf);
        refills += 1;
    }

    for _ in 0..NUM_DESC {
        wait_eof().await;
    }

    stop_dma_tx();
    info!("Done ({} refills)", refills);
}
