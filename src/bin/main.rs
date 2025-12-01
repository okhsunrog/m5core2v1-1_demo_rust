#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use log::info;

use m5core2v1_1_esp_hal_demo::display::{colors, DisplayDma};
use m5core2v1_1_esp_hal_demo::pmic;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    // M5Stack Core2 v1.1: GPIO21 (SDA) and GPIO22 (SCL) for internal I2C bus
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22)
        .into_async();

    // Initialize PMIC
    info!("Initializing PMIC...");
    let mut axp = pmic::init_pmic(i2c).await.unwrap();

    // Configure all power rails (including display power and backlight)
    pmic::configure_all_rails(&mut axp).await.unwrap();

    // Give power rails time to stabilize
    Timer::after(Duration::from_millis(50)).await;

    // Initialize SPI for display
    // M5Stack Core2 v1.1 SPI pins:
    // - GPIO18: SCLK
    // - GPIO23: MOSI
    // - GPIO38: MISO (not used for display, but needed for SPI bus)
    // - GPIO5:  LCD CS
    // - GPIO15: LCD DC (Data/Command)
    info!("Initializing SPI for display...");

    let spi_config = SpiConfig::default()
        .with_frequency(Rate::from_mhz(20))
        .with_mode(SpiMode::_0);

    // Create DMA buffers for SPI
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(1024);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = Spi::new(peripherals.SPI2, spi_config)
        .unwrap()
        .with_sck(peripherals.GPIO18)
        .with_mosi(peripherals.GPIO23)
        .with_miso(peripherals.GPIO38)
        .with_dma(peripherals.DMA_SPI2)
        .with_buffers(dma_rx_buf, dma_tx_buf)
        .into_async();

    // LCD control pins
    let cs = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let dc = Output::new(peripherals.GPIO15, Level::High, OutputConfig::default());

    // Create display driver
    let mut display = DisplayDma::new(spi, cs, dc);

    // Initialize display
    display.init().await;

    // Draw test pattern
    info!("Drawing test pattern...");

    // Fill screen black first
    display.fill_screen(colors::BLACK).await;
    Timer::after(Duration::from_millis(100)).await;

    // Draw colored rectangles
    display.fill_rect(20, 20, 80, 60, colors::RED).await;
    display.fill_rect(120, 20, 80, 60, colors::GREEN).await;
    display.fill_rect(220, 20, 80, 60, colors::BLUE).await;

    display.fill_rect(20, 100, 80, 60, colors::YELLOW).await;
    display.fill_rect(120, 100, 80, 60, colors::CYAN).await;
    display.fill_rect(220, 100, 80, 60, colors::MAGENTA).await;

    display.fill_rect(20, 180, 280, 40, colors::WHITE).await;

    info!("Display test complete!");

    // TODO: Spawn some tasks
    let _ = spawner;

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}
