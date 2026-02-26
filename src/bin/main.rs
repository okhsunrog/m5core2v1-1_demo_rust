#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use alloc::boxed::Box;
use alloc::format;
use bt_hci::controller::ExternalController;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use ft6336u_dd::Ft6336uAsync;
use ina3221_dd::{ChannelId, INA3221_I2C_ADDR_GND, Ina3221Async};
use lcd_async::models::ILI9342CRgb565;
use lcd_async::options::{ColorInversion, Orientation, Rotation};
use lcd_async::{Builder, interface};
use log::info;
use pcf8563_dd::Pcf8563Async;
use slint::PhysicalPosition;
use slint::platform::software_renderer::Rgb565Pixel;
use slint::platform::{PointerEventButton, WindowEvent};
use static_cell::StaticCell;

use m5core2v1_1_esp_hal_demo::ble;
use m5core2v1_1_esp_hal_demo::pd;
use m5core2v1_1_esp_hal_demo::pmic::{self, Backlight, set_backlight};
use m5core2v1_1_esp_hal_demo::slint_platform::EspPlatform;
use usbpd::protocol_layer::message::data::source_capabilities::{Augmented, PowerDataObject};

extern crate alloc;

slint::include_modules!();

esp_bootloader_esp_idf::esp_app_desc!();

const WIDTH: u16 = 320;
const HEIGHT: u16 = 240;
const SPI_FREQ_MHZ: u32 = 30;
const DMA_BUF_SIZE: usize = 32_000;
const SHUNT_RESISTOR_MOHMS: f32 = 10.0;

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Heap in reclaimed DRAM2 only — keeps main DRAM free for stack
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 96 * 1024);

    // PSRAM
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

    info!("Initializing BLE...");
    let connector = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    let controller: ExternalController<_, 1> = ExternalController::new(connector);
    info!("BLE initialized!");

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
    let i2c_ina = I2cDevice::new(i2c_bus);
    let i2c_rtc = I2cDevice::new(i2c_bus);

    // --- External I2C bus (I2C1) for FUSB302B ---
    let i2c1_config = I2cConfig::default().with_frequency(Rate::from_khz(100));
    let i2c1 = I2c::new(peripherals.I2C1, i2c1_config)
        .unwrap()
        .with_sda(peripherals.GPIO32)
        .with_scl(peripherals.GPIO33)
        .into_async();

    // --- PMIC init ---
    info!("Initializing PMIC...");
    let mut axp = pmic::init_pmic(i2c_pmic).await.unwrap();
    pmic::configure_all_rails(&mut axp).await.unwrap();
    set_backlight(&mut axp, Backlight::On(50)).await.unwrap();

    // Short vibration on boot
    axp.set_ldo_voltage_mv(axp2101_dd::LdoId::Dldo1, 3300)
        .await
        .unwrap();
    axp.set_ldo_enable(axp2101_dd::LdoId::Dldo1, true)
        .await
        .unwrap();
    Timer::after(Duration::from_millis(200)).await;
    axp.set_ldo_enable(axp2101_dd::LdoId::Dldo1, false)
        .await
        .unwrap();

    Timer::after(Duration::from_millis(300)).await;

    // --- Touch controller init ---
    info!("Initializing touch controller...");
    let mut touch = Ft6336uAsync::new(i2c_touch);
    let chip_id = touch.read_chip_id().await.unwrap();
    info!("FT6336U Chip ID: 0x{:02X}", chip_id);

    // Set interrupt trigger mode — INT pin (GPIO39) goes low on touch
    // G_MODE register (0xA4): 0x00 = polling, 0x01 = trigger
    // Note: INT pin (GPIO39) has no pull-up on PCB and ESP32 GPIO34-39 lack internal pull-ups,
    // so interrupt-driven touch is not possible without hardware modification. Using polling.

    // --- INA3221 power monitor init ---
    info!("Initializing INA3221...");
    let mut ina = Ina3221Async::new(i2c_ina, INA3221_I2C_ADDR_GND);
    let mfr_id = ina.get_manufacturer_id().await.unwrap();
    let die_id = ina.get_die_id().await.unwrap();
    info!(
        "INA3221 Manufacturer ID: 0x{:04X}, Die ID: 0x{:04X}",
        mfr_id, die_id
    );

    // --- RTC (BM8563/PCF8563) init ---
    info!("Initializing RTC...");
    let mut rtc = Pcf8563Async::new(i2c_rtc);
    let clock_valid = rtc.is_clock_valid().await.unwrap_or(false);
    info!("RTC clock valid: {}", clock_valid);
    if let Ok(dt) = rtc.get_datetime().await {
        info!(
            "RTC time: 20{:02}-{:02}-{:02} {:02}:{:02}:{:02}",
            dt.year, dt.month, dt.day, dt.hours, dt.minutes, dt.seconds
        );
    }

    // --- SPI display init ---
    info!("Initializing SPI for display...");
    let spi_config = SpiConfig::default()
        .with_frequency(Rate::from_mhz(SPI_FREQ_MHZ))
        .with_mode(SpiMode::_0);

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4, DMA_BUF_SIZE);
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
        .color_order(lcd_async::options::ColorOrder::Bgr)
        .invert_colors(ColorInversion::Inverted)
        .init(&mut delay)
        .await
        .unwrap();

    info!("Display initialized!");

    // --- Create Slint UI ---
    let ui = MainWindow::new().unwrap();
    slint_window.set_size(slint::PhysicalSize::new(WIDTH as u32, HEIGHT as u32));

    // Set static system info
    ui.set_cpu_freq("240 MHz".into());
    ui.set_psram_size(format!("{} KB", psram_size / 1024).into());
    ui.set_display_res(format!("{}x{}", WIDTH, HEIGHT).into());
    ui.set_spi_freq(format!("{} MHz", SPI_FREQ_MHZ).into());
    ui.set_i2c_freq("400 kHz".into());
    ui.set_touch_chip(format!("FT6336U (0x{:02X})", chip_id).into());
    ui.set_pmic_chip("AXP2101".into());
    ui.set_dma_buf_size(format!("{} KB", DMA_BUF_SIZE / 1024).into());
    ui.set_backlight_value(0.5);

    // Allocate pixel buffer on PSRAM (150KB — too large for SRAM)
    let num_pixels = (WIDTH as usize) * (HEIGHT as usize);
    let mut pixel_buf =
        allocator_api2::vec::Vec::with_capacity_in(num_pixels, esp_alloc::ExternalMemory);
    pixel_buf.resize(num_pixels, Rgb565Pixel(0));

    // Spawn USB PD task on external I2C bus
    spawner.must_spawn(pd::pd_task(i2c1));

    info!("Starting Slint main loop...");

    // Run BLE stack and app loop concurrently
    let ble_future = ble::run(controller);

    let app_future = async {
        // Track touch state for Slint event dispatching
        let mut last_touch_pos: Option<slint::LogicalPosition> = None;
        // PMIC read counter - read every ~1 second
        let mut pmic_timer = Instant::now();
        // FPS tracking
        let mut fps_timer = Instant::now();
        let mut fps_frame_count: u32 = 0;
        // Current backlight
        let mut current_backlight: f32 = 0.5;
        // External 5V output state
        let mut current_ext5v: bool = false;
        // PD init status tracked
        let mut pd_init_checked: bool = false;

        loop {
            let frame_start = Instant::now();

            // Update Slint timers and animations
            slint::platform::update_timers_and_animations();

            // --- Poll touch ---
            if let Ok(touch_data) = touch.scan().await {
                if touch_data.touch_count > 0 {
                    let p = &touch_data.points[0];
                    let pos = PhysicalPosition::new(p.x as i32, p.y as i32)
                        .to_logical(ui.window().scale_factor());

                    if let Some(prev) = last_touch_pos {
                        if prev != pos {
                            ui.window()
                                .dispatch_event(WindowEvent::PointerMoved { position: pos });
                        }
                    } else {
                        ui.window().dispatch_event(WindowEvent::PointerPressed {
                            position: pos,
                            button: PointerEventButton::Left,
                        });
                    }
                    last_touch_pos = Some(pos);
                    ui.set_touch_info(format!("({}, {})", p.x, p.y).into());
                } else if let Some(pos) = last_touch_pos.take() {
                    ui.window().dispatch_event(WindowEvent::PointerReleased {
                        position: pos,
                        button: PointerEventButton::Left,
                    });
                    ui.window().dispatch_event(WindowEvent::PointerExited);
                    ui.set_touch_info("none".into());
                }
            }

            // Handle backlight changes from UI slider
            let new_bl = ui.get_backlight_value();
            if (new_bl - current_backlight).abs() > 0.01 {
                current_backlight = new_bl;
                let brightness = (current_backlight * 100.0) as u8;
                let _ = set_backlight(&mut axp, Backlight::On(brightness)).await;
            }

            // Handle external 5V output toggle (BLDO2 → AXP_BoostEN)
            let new_ext5v = ui.get_ext5v_enabled();
            if new_ext5v != current_ext5v {
                current_ext5v = new_ext5v;
                let _ = axp
                    .set_ldo_enable(axp2101_dd::LdoId::Bldo2, current_ext5v)
                    .await;
                info!("External 5V output: {}", if current_ext5v { "ON" } else { "OFF" });
            }

            // --- Update PMIC data periodically (~1s) ---
            if pmic_timer.elapsed() >= Duration::from_secs(1) {
                pmic_timer = Instant::now();

                let battery_mv = axp.get_battery_voltage_mv().await.unwrap_or(0);
                let vbus_good = axp.is_vbus_good().await.unwrap_or(false);
                let vbus_mv = if vbus_good {
                    axp.get_vbus_voltage_mv().await.unwrap_or(0)
                } else {
                    0
                };
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

                // INA3221 readings
                for (ch, set_v, set_i) in [
                    (
                        ChannelId::Channel1,
                        MainWindow::set_ina_ch1_voltage as fn(&MainWindow, slint::SharedString),
                        MainWindow::set_ina_ch1_current as fn(&MainWindow, slint::SharedString),
                    ),
                    (
                        ChannelId::Channel2,
                        MainWindow::set_ina_ch2_voltage,
                        MainWindow::set_ina_ch2_current,
                    ),
                    (
                        ChannelId::Channel3,
                        MainWindow::set_ina_ch3_voltage,
                        MainWindow::set_ina_ch3_current,
                    ),
                ] {
                    let v = ina.get_bus_voltage_mv(ch).await.unwrap_or(0.0);
                    let i = ina
                        .get_current_ma(ch, SHUNT_RESISTOR_MOHMS)
                        .await
                        .unwrap_or(0.0);
                    set_v(&ui, format!("{:.0} mV", v).into());
                    set_i(&ui, format!("{:.1} mA", i).into());
                }

                // RTC time
                if let Ok(dt) = rtc.get_datetime().await {
                    ui.set_clock_text(
                        format!("{:02}:{:02}:{:02}", dt.hours, dt.minutes, dt.seconds).into(),
                    );
                }

                // Check for BLE time sync
                if let Ok(buf) = ble::TIME_CHANNEL.try_receive() {
                    let dt = pcf8563_dd::DateTime {
                        year: buf[0],
                        month: buf[1],
                        day: buf[2],
                        weekday: buf[3],
                        hours: buf[4],
                        minutes: buf[5],
                        seconds: buf[6],
                    };
                    info!(
                        "Setting RTC from BLE: 20{:02}-{:02}-{:02} {:02}:{:02}:{:02}",
                        dt.year, dt.month, dt.day, dt.hours, dt.minutes, dt.seconds
                    );
                    if let Err(e) = rtc.set_datetime(&dt).await {
                        info!("Failed to set RTC: {:?}", e);
                    }
                }

                // Check PD init status
                if !pd_init_checked {
                    if let Ok(ok) = pd::INIT_OK_CHANNEL.try_receive() {
                        pd_init_checked = true;
                        if ok {
                            ui.set_pd_status("Waiting...".into());
                        } else {
                            ui.set_pd_status("No chip".into());
                        }
                    }
                }

                // Check for USB PD source capabilities
                if let Ok(caps) = pd::CAPS_CHANNEL.try_receive() {
                    ui.set_pd_status("Connected".into());
                    let pdos = caps.pdos();
                    let pdo_setters: [fn(&MainWindow, slint::SharedString); 7] = [
                        MainWindow::set_pd_pdo1,
                        MainWindow::set_pd_pdo2,
                        MainWindow::set_pd_pdo3,
                        MainWindow::set_pd_pdo4,
                        MainWindow::set_pd_pdo5,
                        MainWindow::set_pd_pdo6,
                        MainWindow::set_pd_pdo7,
                    ];
                    for (i, setter) in pdo_setters.iter().enumerate() {
                        if i < pdos.len() {
                            setter(&ui, format_pdo(&pdos[i]).into());
                        } else {
                            setter(&ui, "".into());
                        }
                    }
                }

                // Update free heap stats
                let free_sram = esp_alloc::HEAP
                    .free_caps(esp_alloc::MemoryCapability::Internal.into());
                let free_psram = esp_alloc::HEAP
                    .free_caps(esp_alloc::MemoryCapability::External.into());
                ui.set_free_sram(format!("{} KB", free_sram / 1024).into());
                ui.set_free_psram(format!("{} KB", free_psram / 1024).into());

                // Update uptime
                let up_secs = Instant::now().as_millis() / 1000;
                let mins = up_secs / 60;
                let secs = up_secs % 60;
                ui.set_uptime(format!("{}m {}s", mins, secs).into());
            }

            // --- Render ---
            let render_start = Instant::now();
            slint_window.draw_if_needed(|renderer| {
                renderer.render(&mut pixel_buf, WIDTH as usize);
            });
            let render_ms = render_start.elapsed().as_millis();

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

            // --- FPS calculation ---
            fps_frame_count += 1;
            if fps_timer.elapsed() >= Duration::from_secs(1) {
                let elapsed_ms = fps_timer.elapsed().as_millis().max(1);
                let fps = (fps_frame_count as u64 * 1000) / elapsed_ms;
                ui.set_fps_text(format!("{}", fps).into());

                let frame_ms = elapsed_ms / fps_frame_count.max(1) as u64;
                ui.set_frame_time(format!("{} ms", frame_ms).into());
                ui.set_render_time(format!("{} ms", render_ms).into());

                fps_frame_count = 0;
                fps_timer = Instant::now();
            }

            // Cap at ~60fps
            let elapsed = frame_start.elapsed();
            if elapsed < Duration::from_millis(16) {
                Timer::after(Duration::from_millis(16) - elapsed).await;
            }
        }
    }; // end app_future

    embassy_futures::join::join(ble_future, app_future).await;
    unreachable!()
}

fn format_pdo(pdo: &PowerDataObject) -> alloc::string::String {
    match pdo {
        PowerDataObject::FixedSupply(f) => {
            format!(
                "FIXED {}mV {}mA",
                f.voltage()
                    .get::<uom::si::electric_potential::millivolt>(),
                f.max_current()
                    .get::<uom::si::electric_current::milliampere>()
            )
        }
        PowerDataObject::VariableSupply(v) => {
            format!(
                "VAR {}-{}mV {}mA",
                v.min_voltage()
                    .get::<uom::si::electric_potential::millivolt>(),
                v.max_voltage()
                    .get::<uom::si::electric_potential::millivolt>(),
                v.max_current()
                    .get::<uom::si::electric_current::milliampere>()
            )
        }
        PowerDataObject::Battery(b) => {
            format!(
                "BATT {}-{}mV {}mW",
                b.min_voltage()
                    .get::<uom::si::electric_potential::millivolt>(),
                b.max_voltage()
                    .get::<uom::si::electric_potential::millivolt>(),
                b.max_power().get::<uom::si::power::milliwatt>()
            )
        }
        PowerDataObject::Augmented(a) => match a {
            Augmented::Spr(pps) => {
                format!(
                    "PPS {}-{}mV {}mA",
                    pps.min_voltage()
                        .get::<uom::si::electric_potential::millivolt>(),
                    pps.max_voltage()
                        .get::<uom::si::electric_potential::millivolt>(),
                    pps.max_current()
                        .get::<uom::si::electric_current::milliampere>()
                )
            }
            _ => format!("EPR PDO"),
        },
        _ => format!("Unknown PDO"),
    }
}
