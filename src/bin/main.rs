#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use alloc::vec;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_graphics::mono_font::{MonoTextStyle, ascii::FONT_10X20};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use esp_backtrace as _;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use heapless::String;
use lcd_async::models::ILI9342CRgb565;
use lcd_async::options::{ColorInversion, Orientation, Rotation};
use lcd_async::raw_framebuf::RawFrameBuf;
use lcd_async::{Builder, interface};
use log::info;
use static_cell::StaticCell;

use core::fmt::Write;
use m5core2v1_1_esp_hal_demo::pmic::{self, set_backlight_brightness};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// Display parameters
const WIDTH: u16 = 320;
const HEIGHT: u16 = 240;

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Use PSRAM for heap (Core2 has 8MB PSRAM, but ESP32 can only map 4MB)
    esp_alloc::psram_allocator!(&peripherals.PSRAM, esp_hal::psram);

    let (psram_ptr, psram_size) = esp_hal::psram::psram_raw_parts(&peripherals.PSRAM);
    info!(
        "PSRAM mapped: {} KB ({} bytes) at {:?}",
        psram_size / 1024,
        psram_size,
        psram_ptr
    );

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_ints =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

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

    // Set backlight to 50% brightness
    set_backlight_brightness(&mut axp, 50).await.unwrap();

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

    // Create DMA buffers for SPI - need larger buffer for framebuffer transfers
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4, 32_000);
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
    let dc = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());

    // Create shared SPI bus for lcd-async
    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, SpiDmaBus<'static, esp_hal::Async>>> =
        StaticCell::new();
    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);
    let spi_device = SpiDevice::new(spi_bus, cs);

    // Create display interface
    let di = interface::SpiInterface::new(spi_device, dc);
    let mut delay = embassy_time::Delay;

    // Initialize the display with lcd-async
    info!("Initializing display with lcd-async...");
    let mut display = Builder::new(ILI9342CRgb565, di)
        .display_size(WIDTH, HEIGHT)
        .orientation(Orientation {
            rotation: Rotation::Deg0,
            mirrored: false,
        })
        .invert_colors(ColorInversion::Inverted)
        .init(&mut delay)
        .await
        .unwrap();

    info!("Display initialized!");

    // Allocate frame buffer on heap (320 * 240 * 2 = 153600 bytes)
    let frame_size = (WIDTH as usize) * (HEIGHT as usize) * 2;
    let mut frame_buffer = vec![0u8; frame_size];

    // Text style for display
    let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    let title_style = MonoTextStyle::new(&FONT_10X20, Rgb565::CYAN);

    // TODO: Spawn some tasks
    let _ = spawner;

    info!("Starting PMIC monitoring loop...");

    loop {
        // Read PMIC values
        let battery_mv = axp.get_battery_voltage_mv().await.unwrap_or(0);
        let vbus_mv = axp.get_vbus_voltage_mv().await.unwrap_or(0);
        let vsys_mv = axp.get_vsys_voltage_mv().await.unwrap_or(0);
        let temp_c = axp.get_die_temperature_c().await.unwrap_or(0.0);

        // Read battery percentage
        let soc = axp
            .ll
            .battery_percentage()
            .read_async()
            .await
            .map(|s| s.percentage())
            .unwrap_or(0);

        // Create a framebuffer for drawing
        let mut raw_fb =
            RawFrameBuf::<Rgb565, _>::new(&mut frame_buffer[..], WIDTH.into(), HEIGHT.into());

        // Clear the framebuffer to black
        raw_fb.clear(Rgb565::BLACK).unwrap();

        // Draw title
        Text::new("M5Stack Core2 v1.1", Point::new(40, 30), title_style)
            .draw(&mut raw_fb)
            .unwrap();

        // Format and draw battery percentage
        let mut line: String<64> = String::new();
        write!(line, "Battery: {}%", soc).unwrap();
        Text::new(&line, Point::new(20, 70), text_style)
            .draw(&mut raw_fb)
            .unwrap();

        // Format and draw battery voltage
        line.clear();
        write!(line, "Battery: {} mV", battery_mv).unwrap();
        Text::new(&line, Point::new(20, 100), text_style)
            .draw(&mut raw_fb)
            .unwrap();

        // Format and draw VBUS voltage
        line.clear();
        write!(line, "VBUS:    {} mV", vbus_mv).unwrap();
        Text::new(&line, Point::new(20, 130), text_style)
            .draw(&mut raw_fb)
            .unwrap();

        // Format and draw VSYS voltage
        line.clear();
        write!(line, "VSYS:    {} mV", vsys_mv).unwrap();
        Text::new(&line, Point::new(20, 160), text_style)
            .draw(&mut raw_fb)
            .unwrap();

        // Format and draw temperature
        line.clear();
        write!(line, "Temp:    {:.1} C", temp_c).unwrap();
        Text::new(&line, Point::new(20, 190), text_style)
            .draw(&mut raw_fb)
            .unwrap();

        // Send the framebuffer to the display
        display
            .show_raw_data(0, 0, WIDTH, HEIGHT, &frame_buffer)
            .await
            .unwrap();

        Timer::after(Duration::from_secs(1)).await;
    }
}
