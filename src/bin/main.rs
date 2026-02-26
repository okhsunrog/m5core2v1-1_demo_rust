#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use alloc::boxed::Box;
use alloc::format;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use ft6336u_dd::{Ft6336uAsync, TouchStatus};
use lcd_async::models::ILI9342CRgb565;
use lcd_async::options::{ColorInversion, Orientation, Rotation};
use lcd_async::{Builder, interface};
use log::info;
use slint::platform::software_renderer::Rgb565Pixel;
use slint::platform::{PointerEventButton, WindowEvent};
use slint::PhysicalPosition;
use static_cell::StaticCell;

use m5core2v1_1_esp_hal_demo::pmic::{self, set_backlight_brightness};
use m5core2v1_1_esp_hal_demo::slint_platform::EspPlatform;

extern crate alloc;

slint::include_modules!();

esp_bootloader_esp_idf::esp_app_desc!();

const WIDTH: u16 = 320;
const HEIGHT: u16 = 240;

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
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

    // --- Set up Slint platform ---
    let slint_window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );
    slint::platform::set_platform(Box::new(EspPlatform::new(slint_window.clone()))).unwrap();

    // --- I2C shared bus ---
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22)
        .into_async();

    static I2C_BUS: StaticCell<Mutex<NoopRawMutex, I2c<'static, esp_hal::Async>>> =
        StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));
    let i2c_pmic = I2cDevice::new(i2c_bus);
    let i2c_touch = I2cDevice::new(i2c_bus);

    // --- PMIC init ---
    info!("Initializing PMIC...");
    let mut axp = pmic::init_pmic(i2c_pmic).await.unwrap();
    pmic::configure_all_rails(&mut axp).await.unwrap();
    set_backlight_brightness(&mut axp, 50).await.unwrap();
    Timer::after(Duration::from_millis(300)).await;

    // --- Touch controller init ---
    info!("Initializing touch controller...");
    let mut touch = Ft6336uAsync::new(i2c_touch);
    let chip_id = touch.read_chip_id().await.unwrap();
    info!("FT6336U Chip ID: 0x{:02X}", chip_id);

    // --- SPI display init ---
    info!("Initializing SPI for display...");
    let spi_config = SpiConfig::default()
        .with_frequency(Rate::from_mhz(20))
        .with_mode(SpiMode::_0);

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

    let cs = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let dc = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());

    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, SpiDmaBus<'static, esp_hal::Async>>> =
        StaticCell::new();
    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);
    let spi_device = SpiDevice::new(spi_bus, cs);

    let di = interface::SpiInterface::new(spi_device, dc);
    let mut delay = embassy_time::Delay;

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

    // --- Create Slint UI ---
    let ui = MainWindow::new().unwrap();
    slint_window.set_size(slint::PhysicalSize::new(WIDTH as u32, HEIGHT as u32));

    // Allocate pixel buffer on PSRAM
    let num_pixels = (WIDTH as usize) * (HEIGHT as usize);
    let mut pixel_buf = alloc::vec![Rgb565Pixel(0); num_pixels];

    let _ = spawner;

    info!("Starting Slint main loop...");

    // Track touch state for Slint event dispatching
    let mut last_touch_pos: Option<slint::LogicalPosition> = None;
    // PMIC read counter - read every ~1 second (every 60 frames at ~16ms)
    let mut frame_count: u32 = 0;

    loop {
        // Update Slint timers and animations
        slint::platform::update_timers_and_animations();

        // --- Poll touch ---
        if let Ok(touch_data) = touch.scan().await {
            if touch_data.touch_count > 0 {
                let p = &touch_data.points[0];
                if p.status != TouchStatus::Release {
                    let pos = PhysicalPosition::new(p.x as i32, p.y as i32)
                        .to_logical(ui.window().scale_factor());

                    if let Some(prev) = last_touch_pos {
                        if prev != pos {
                            let _ = ui.window().dispatch_event(WindowEvent::PointerMoved {
                                position: pos,
                            });
                        }
                    } else {
                        let _ = ui.window().dispatch_event(WindowEvent::PointerPressed {
                            position: pos,
                            button: PointerEventButton::Left,
                        });
                    }
                    last_touch_pos = Some(pos);

                    ui.set_touch_info(format!("({}, {})", p.x, p.y).into());
                } else if let Some(pos) = last_touch_pos.take() {
                    let _ = ui.window().dispatch_event(WindowEvent::PointerReleased {
                        position: pos,
                        button: PointerEventButton::Left,
                    });
                    let _ = ui.window().dispatch_event(WindowEvent::PointerExited);
                    ui.set_touch_info("none".into());
                }
            } else if let Some(pos) = last_touch_pos.take() {
                let _ = ui.window().dispatch_event(WindowEvent::PointerReleased {
                    position: pos,
                    button: PointerEventButton::Left,
                });
                let _ = ui.window().dispatch_event(WindowEvent::PointerExited);
                ui.set_touch_info("none".into());
            }
        }

        // --- Update PMIC data periodically (~1s) ---
        frame_count += 1;
        if frame_count >= 60 {
            frame_count = 0;

            let battery_mv = axp.get_battery_voltage_mv().await.unwrap_or(0);
            let vbus_mv = axp.get_vbus_voltage_mv().await.unwrap_or(0);
            let vsys_mv = axp.get_vsys_voltage_mv().await.unwrap_or(0);
            let temp_c = axp.get_die_temperature_c().await.unwrap_or(0.0);
            let soc = axp
                .ll
                .battery_percentage()
                .read_async()
                .await
                .map(|s| s.percentage())
                .unwrap_or(0);

            ui.set_battery_percent(format!("{}", soc).into());
            ui.set_battery_voltage(format!("{}", battery_mv).into());
            ui.set_vbus_voltage(format!("{}", vbus_mv).into());
            ui.set_vsys_voltage(format!("{}", vsys_mv).into());
            ui.set_temperature(format!("{:.1}", temp_c).into());
        }

        // --- Render ---
        slint_window.draw_if_needed(|renderer| {
            renderer.render(&mut pixel_buf, WIDTH as usize);
        });

        // Swap bytes: Slint renders native (little-endian), ILI9342C expects big-endian RGB565
        for pixel in pixel_buf.iter_mut() {
            pixel.0 = pixel.0.swap_bytes();
        }

        // Send framebuffer to display
        let raw_bytes: &[u8] = unsafe {
            core::slice::from_raw_parts(pixel_buf.as_ptr() as *const u8, pixel_buf.len() * 2)
        };
        display
            .show_raw_data(0, 0, WIDTH, HEIGHT, raw_bytes)
            .await
            .unwrap();

        // Swap back so Slint's dirty tracking works correctly with ReusedBuffer
        for pixel in pixel_buf.iter_mut() {
            pixel.0 = pixel.0.swap_bytes();
        }

        Timer::after(Duration::from_millis(16)).await;
    }
}
