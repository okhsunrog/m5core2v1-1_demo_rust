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
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::system::Stack as AppCoreStack;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp_rtos::embassy::Executor;
use ft6336u_dd::Ft6336uAsync;
use ina3221_dd::{ChannelId, INA3221_I2C_ADDR_GND, Ina3221Async};
use log::info;
use mipidsi::models::ILI9342CRgb565;
use mipidsi::options::{ColorInversion, Orientation, Rotation};
use mipidsi::{Builder, interface};
use pcf8563_dd::{DateTime, Pcf8563Async};
use slint::PhysicalPosition;
use slint::platform::{PointerEventButton, WindowEvent};
use static_cell::StaticCell;

use m5core2v1_1_esp_hal_demo::ble;
use m5core2v1_1_esp_hal_demo::display_dma::{DmaLineDisplay, InitSpiDevice};
use m5core2v1_1_esp_hal_demo::pmic::{self, Backlight, set_backlight};
use m5core2v1_1_esp_hal_demo::slint_platform::EspPlatform;

extern crate alloc;

slint::include_modules!();

esp_bootloader_esp_idf::esp_app_desc!();

const WIDTH: u16 = 320;
const HEIGHT: u16 = 240;
const WIDTH_USIZE: usize = WIDTH as usize;
const SPI_FREQ_MHZ: u32 = 30;
const DISPLAY_TILE_LINES: usize = 16;
const DMA_BUF_SIZE: usize = WIDTH_USIZE * DISPLAY_TILE_LINES * 2;
const SHUNT_RESISTOR_MOHMS: f32 = 10.0;

type SharedI2cBus = Mutex<NoopRawMutex, I2c<'static, esp_hal::Async>>;

#[derive(Clone, Copy, Debug)]
struct I2cInitInfo {
    touch_chip_id: u8,
    rtc_clock_valid: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TouchState {
    None,
    Pressed { x: u16, y: u16 },
}

#[derive(Clone, Copy, Debug, Default)]
struct I2cMeasurements {
    battery_mv: u16,
    vbus_mv: u16,
    vsys_mv: u16,
    temp_c: f32,
    soc: u8,
    ina_voltage_mv: [f32; 3],
    ina_current_ma: [f32; 3],
    rtc_time: Option<DateTime>,
}

static I2C_INIT_SIGNAL: Signal<CriticalSectionRawMutex, I2cInitInfo> = Signal::new();
static TOUCH_SIGNAL: Signal<CriticalSectionRawMutex, TouchState> = Signal::new();
static I2C_MEASUREMENTS_SIGNAL: Signal<CriticalSectionRawMutex, I2cMeasurements> = Signal::new();
static BACKLIGHT_SIGNAL: Signal<CriticalSectionRawMutex, u8> = Signal::new();
static EXT5V_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static RTC_SET_SIGNAL: Signal<CriticalSectionRawMutex, DateTime> = Signal::new();

// Raw FT6336U coordinates are used as PhysicalPosition directly. This is only
// correct because the display is initialized with `Rotation::Deg0` and no
// mirroring. If the display orientation changes, touch coordinates must be
// transformed to match (rotated/mirrored to the new display frame).
fn dispatch_touch_state(
    ui: &MainWindow,
    last_touch_pos: &mut Option<slint::LogicalPosition>,
    state: TouchState,
) {
    match state {
        TouchState::Pressed { x, y } => {
            let pos =
                PhysicalPosition::new(x as i32, y as i32).to_logical(ui.window().scale_factor());

            if let Some(prev) = *last_touch_pos {
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

            *last_touch_pos = Some(pos);
            ui.set_touch_info(format!("({}, {})", x, y).into());
        }
        TouchState::None => {
            if let Some(pos) = last_touch_pos.take() {
                ui.window().dispatch_event(WindowEvent::PointerReleased {
                    position: pos,
                    button: PointerEventButton::Left,
                });
                ui.window().dispatch_event(WindowEvent::PointerExited);
                ui.set_touch_info("none".into());
            }
        }
    }
}

#[embassy_executor::task]
async fn ble_task(controller: ExternalController<BleConnector<'static>, 1>) {
    ble::run(controller).await;
}

#[embassy_executor::task]
async fn i2c_service_task(i2c_bus: &'static SharedI2cBus) {
    let i2c_pmic = I2cDevice::new(i2c_bus);
    let i2c_touch = I2cDevice::new(i2c_bus);
    let i2c_ina = I2cDevice::new(i2c_bus);
    let i2c_rtc = I2cDevice::new(i2c_bus);

    info!("Initializing PMIC...");
    let mut axp = pmic::init_pmic(i2c_pmic).await.unwrap();
    pmic::configure_all_rails(&mut axp).await.unwrap();
    set_backlight(&mut axp, Backlight::On(50)).await.unwrap();

    // Short vibration on boot.
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

    info!("Initializing touch controller...");
    let mut touch = Ft6336uAsync::new(i2c_touch);
    let chip_id = touch.read_chip_id().await.unwrap();
    info!("FT6336U Chip ID: 0x{:02X}", chip_id);

    info!("Initializing INA3221...");
    let mut ina = Ina3221Async::new(i2c_ina, INA3221_I2C_ADDR_GND);
    let mfr_id = ina.get_manufacturer_id().await.unwrap();
    let die_id = ina.get_die_id().await.unwrap();
    info!(
        "INA3221 Manufacturer ID: 0x{:04X}, Die ID: 0x{:04X}",
        mfr_id, die_id
    );

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

    I2C_INIT_SIGNAL.signal(I2cInitInfo {
        touch_chip_id: chip_id,
        rtc_clock_valid: clock_valid,
    });

    let mut last_touch_state = TouchState::None;
    let mut measurement_timer = Instant::now();

    loop {
        if let Some(brightness) = BACKLIGHT_SIGNAL.try_take() {
            let _ = set_backlight(&mut axp, Backlight::On(brightness)).await;
        }

        if let Some(enabled) = EXT5V_SIGNAL.try_take() {
            let _ = axp.set_ldo_enable(axp2101_dd::LdoId::Bldo2, enabled).await;
            info!("External 5V output: {}", if enabled { "ON" } else { "OFF" });
        }

        if let Some(dt) = RTC_SET_SIGNAL.try_take() {
            info!(
                "Setting RTC from BLE: 20{:02}-{:02}-{:02} {:02}:{:02}:{:02}",
                dt.year, dt.month, dt.day, dt.hours, dt.minutes, dt.seconds
            );
            if let Err(e) = rtc.set_datetime(&dt).await {
                info!("Failed to set RTC: {:?}", e);
            }
        }

        if let Ok(touch_data) = touch.scan().await {
            let touch_state = if touch_data.touch_count > 0 {
                let p = &touch_data.points[0];
                TouchState::Pressed { x: p.x, y: p.y }
            } else {
                TouchState::None
            };

            if touch_state != last_touch_state {
                TOUCH_SIGNAL.signal(touch_state);
                last_touch_state = touch_state;
            }
        }

        if measurement_timer.elapsed() >= Duration::from_secs(1) {
            measurement_timer = Instant::now();

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

            let mut measurements = I2cMeasurements {
                battery_mv,
                vbus_mv,
                vsys_mv,
                temp_c,
                soc,
                ..Default::default()
            };

            for (idx, ch) in [
                ChannelId::Channel1,
                ChannelId::Channel2,
                ChannelId::Channel3,
            ]
            .into_iter()
            .enumerate()
            {
                measurements.ina_voltage_mv[idx] = ina.get_bus_voltage_mv(ch).await.unwrap_or(0.0);
                measurements.ina_current_ma[idx] = ina
                    .get_current_ma(ch, SHUNT_RESISTOR_MOHMS)
                    .await
                    .unwrap_or(0.0);
            }

            measurements.rtc_time = rtc.get_datetime().await.ok();
            I2C_MEASUREMENTS_SIGNAL.signal(measurements);
        }

        Timer::after(Duration::from_millis(8)).await;
    }
}

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Heap in reclaimed DRAM2 only — keeps main DRAM free for stack
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 96 * 1024);

    // PSRAM setup — not needed for now since we switched to `render_by_line`,
    // which streams the framebuffer line-by-line via DMA instead of holding the
    // full ~150 KB frame in memory. Kept here in case we want to re-enable it
    // for full-framebuffer rendering or other large allocations.
    //
    // let psram = esp_hal::psram::Psram::new(peripherals.PSRAM, Default::default());
    // esp_alloc::psram_allocator!(&psram);
    // let (psram_ptr, psram_size) = psram.raw_parts();
    // info!(
    //     "PSRAM mapped: {} KB ({} bytes) at {:?}",
    //     psram_size / 1024,
    //     psram_size,
    //     psram_ptr
    // );
    let psram_size = 0usize;

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_ints =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    info!("Embassy initialized!");

    info!("Initializing BLE...");
    let connector = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    let controller: ExternalController<_, 1> = ExternalController::new(connector);
    info!("BLE initialized!");

    // --- I2C shared bus (core 0) ---
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22)
        .into_async();

    static I2C_BUS: StaticCell<SharedI2cBus> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    spawner.spawn(i2c_service_task(i2c_bus).unwrap());
    spawner.spawn(ble_task(controller).unwrap());

    // Move display + Slint render loop onto the second core so heavy frames
    // (e.g. animation screens) can't starve BLE / touch on core 0.
    static APP_CORE_STACK: StaticCell<AppCoreStack<32768>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(AppCoreStack::new());

    let spi_periph = peripherals.SPI2;
    let dma_spi2 = peripherals.DMA_SPI2;
    let gpio_sck = peripherals.GPIO18;
    let gpio_mosi = peripherals.GPIO23;
    let gpio_miso = peripherals.GPIO38;
    let gpio_cs = peripherals.GPIO5;
    let gpio_dc = peripherals.GPIO15;

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        sw_ints.software_interrupt1,
        app_core_stack,
        move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                spawner.spawn(
                    render_task(
                        spi_periph, dma_spi2, gpio_sck, gpio_mosi, gpio_miso, gpio_cs, gpio_dc,
                        psram_size,
                    )
                    .unwrap(),
                );
            });
        },
    );

    // Core 0 just stays alive to keep its tasks running.
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

/// Format into a fixed stack buffer and return a Slint SharedString.
/// Avoids the per-frame `alloc::String` allocation that `format!` causes.
fn sstr<const N: usize>(args: core::fmt::Arguments) -> slint::SharedString {
    use core::fmt::Write as _;
    let mut s = heapless::String::<N>::new();
    let _ = s.write_fmt(args);
    slint::SharedString::from(s.as_str())
}

macro_rules! sstr {
    ($($arg:tt)*) => {
        sstr::<32>(format_args!($($arg)*))
    };
}

#[embassy_executor::task]
async fn render_task(
    spi_periph: esp_hal::peripherals::SPI2<'static>,
    dma_spi2: esp_hal::peripherals::DMA_SPI2<'static>,
    gpio_sck: esp_hal::peripherals::GPIO18<'static>,
    gpio_mosi: esp_hal::peripherals::GPIO23<'static>,
    gpio_miso: esp_hal::peripherals::GPIO38<'static>,
    gpio_cs: esp_hal::peripherals::GPIO5<'static>,
    gpio_dc: esp_hal::peripherals::GPIO15<'static>,
    psram_size: usize,
) {
    info!("[core1] render task starting");

    // --- Slint platform (per-core thread-local state) ---
    let slint_window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );
    slint::platform::set_platform(Box::new(EspPlatform::new(slint_window.clone()))).unwrap();

    // Wait for the I2C service on core 0 to finish probing peripherals before
    // we set up the UI (we need the touch chip ID for a status string).
    let i2c_init = I2C_INIT_SIGNAL.wait().await;
    info!(
        "[core1] I2C service ready; RTC clock valid: {}",
        i2c_init.rtc_clock_valid
    );

    // --- SPI display init ---
    info!("[core1] Initializing SPI for display...");
    let spi_config = SpiConfig::default()
        .with_frequency(Rate::from_mhz(SPI_FREQ_MHZ))
        .with_mode(SpiMode::_0);

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4, DMA_BUF_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = Spi::new(spi_periph, spi_config)
        .unwrap()
        .with_sck(gpio_sck)
        .with_mosi(gpio_mosi)
        .with_miso(gpio_miso)
        .with_dma(dma_spi2)
        .with_buffers(dma_rx_buf, dma_tx_buf);

    let cs = Output::new(gpio_cs, Level::High, OutputConfig::default());
    let dc = Output::new(gpio_dc, Level::Low, OutputConfig::default());

    let spi_device = InitSpiDevice::new(spi, cs).unwrap();
    let mut init_buffer = [0_u8; 512];
    let di = interface::SpiInterface::new(spi_device, dc, &mut init_buffer);
    let mut delay = Delay::new();

    let display = Builder::new(ILI9342CRgb565, di)
        .display_size(WIDTH, HEIGHT)
        .orientation(Orientation {
            rotation: Rotation::Deg0,
            mirrored: false,
        })
        .color_order(mipidsi::options::ColorOrder::Bgr)
        .invert_colors(ColorInversion::Inverted)
        .init(&mut delay)
        .unwrap();

    let (di, _, _) = display.release();
    let (spi_device, dc) = di.release();
    let (spi, cs) = spi_device.release();
    let (spi, _dma_rx_buf, first_tx_buf) = spi.split();
    let second_tx_buf = esp_hal::dma_tx_buffer!(DMA_BUF_SIZE).unwrap();
    let mut display = DmaLineDisplay::new(
        spi,
        cs,
        dc,
        first_tx_buf,
        second_tx_buf,
        WIDTH,
        HEIGHT,
        DISPLAY_TILE_LINES,
    )
    .unwrap();

    info!("[core1] Display initialized!");

    // --- Create Slint UI ---
    let ui = MainWindow::new().unwrap();
    slint_window.set_size(slint::PhysicalSize::new(WIDTH as u32, HEIGHT as u32));

    ui.set_cpu_freq("240 MHz".into());
    ui.set_psram_size(format!("{} KB", psram_size / 1024).into());
    ui.set_display_res(format!("{}x{}", WIDTH, HEIGHT).into());
    ui.set_spi_freq(format!("{} MHz", SPI_FREQ_MHZ).into());
    ui.set_i2c_freq("400 kHz".into());
    ui.set_touch_chip(format!("FT6336U (0x{:02X})", i2c_init.touch_chip_id).into());
    ui.set_pmic_chip("AXP2101".into());
    ui.set_dma_buf_size(format!("{} KB", DMA_BUF_SIZE / 1024).into());
    ui.set_backlight_value(0.5);

    info!("[core1] Starting Slint main loop...");

    let app_future = async {
        // Track touch state for Slint event dispatching
        let mut last_touch_pos: Option<slint::LogicalPosition> = None;
        // Periodic non-I2C stats.
        let mut stats_timer = Instant::now();
        // FPS tracking
        let mut fps_timer = Instant::now();
        let mut fps_frame_count: u32 = 0;
        let mut render_ms_accum: u64 = 0;
        // Current backlight
        let mut current_backlight: f32 = 0.5;
        // External 5V output state
        let mut current_ext5v: bool = false;

        loop {
            let frame_start = Instant::now();

            // Update Slint timers and animations
            slint::platform::update_timers_and_animations();

            if let Some(touch_state) = TOUCH_SIGNAL.try_take() {
                dispatch_touch_state(&ui, &mut last_touch_pos, touch_state);
            }

            if let Some(measurements) = I2C_MEASUREMENTS_SIGNAL.try_take() {
                ui.set_battery_percent(sstr!("{}", measurements.soc));
                ui.set_battery_voltage(sstr!("{}", measurements.battery_mv));
                ui.set_vbus_voltage(sstr!("{}", measurements.vbus_mv));
                ui.set_vsys_voltage(sstr!("{}", measurements.vsys_mv));
                ui.set_temperature(sstr!("{:.1}", measurements.temp_c));

                ui.set_ina_ch1_voltage(sstr!("{:.0} mV", measurements.ina_voltage_mv[0]));
                ui.set_ina_ch1_current(sstr!("{:.1} mA", measurements.ina_current_ma[0]));
                ui.set_ina_ch2_voltage(sstr!("{:.0} mV", measurements.ina_voltage_mv[1]));
                ui.set_ina_ch2_current(sstr!("{:.1} mA", measurements.ina_current_ma[1]));
                ui.set_ina_ch3_voltage(sstr!("{:.0} mV", measurements.ina_voltage_mv[2]));
                ui.set_ina_ch3_current(sstr!("{:.1} mA", measurements.ina_current_ma[2]));

                if let Some(dt) = measurements.rtc_time {
                    ui.set_clock_text(sstr!(
                        "{:02}:{:02}:{:02}",
                        dt.hours,
                        dt.minutes,
                        dt.seconds
                    ));
                }
            }

            // Handle backlight changes from UI slider
            let new_bl = ui.get_backlight_value();
            if (new_bl - current_backlight).abs() > 0.01 {
                current_backlight = new_bl;
                let brightness = (current_backlight.max(0.0).min(1.0) * 100.0) as u8;
                BACKLIGHT_SIGNAL.signal(brightness);
            }

            // Handle external 5V output toggle (BLDO2 -> AXP_BoostEN)
            let new_ext5v = ui.get_ext5v_enabled();
            if new_ext5v != current_ext5v {
                current_ext5v = new_ext5v;
                EXT5V_SIGNAL.signal(current_ext5v);
            }

            if let Ok(buf) = ble::TIME_CHANNEL.try_receive() {
                RTC_SET_SIGNAL.signal(DateTime {
                    year: buf[0],
                    month: buf[1],
                    day: buf[2],
                    weekday: buf[3],
                    hours: buf[4],
                    minutes: buf[5],
                    seconds: buf[6],
                });
            }

            if stats_timer.elapsed() >= Duration::from_secs(1) {
                stats_timer = Instant::now();

                // Update free heap stats
                let free_sram =
                    esp_alloc::HEAP.free_caps(esp_alloc::MemoryCapability::Internal.into());
                let free_psram =
                    esp_alloc::HEAP.free_caps(esp_alloc::MemoryCapability::External.into());
                ui.set_free_sram(sstr!("{} KB", free_sram / 1024));
                ui.set_free_psram(sstr!("{} KB", free_psram / 1024));

                // Update uptime
                let up_secs = Instant::now().as_millis() / 1000;
                let mins = up_secs / 60;
                let secs = up_secs % 60;
                ui.set_uptime(sstr!("{}m {}s", mins, secs));
            }

            // --- Render ---
            let render_start = Instant::now();
            slint_window.draw_if_needed(|renderer| {
                renderer.render_by_line(&mut display);
            });

            if let Err(e) = display.finish_frame() {
                log::warn!("Display write failed: {:?}", e);
                slint_window.request_redraw();
            }
            render_ms_accum += render_start.elapsed().as_millis();

            // --- FPS calculation ---
            fps_frame_count += 1;
            if fps_timer.elapsed() >= Duration::from_secs(1) {
                let elapsed_ms = fps_timer.elapsed().as_millis().max(1);
                let fps = (fps_frame_count as u64 * 1000) / elapsed_ms;
                ui.set_fps_text(sstr!("{}", fps));

                let frame_ms = elapsed_ms / fps_frame_count.max(1) as u64;
                let avg_render_ms = render_ms_accum / fps_frame_count.max(1) as u64;
                ui.set_frame_time(sstr!("{} ms", frame_ms));
                ui.set_render_time(sstr!("{} ms", avg_render_ms));
                render_ms_accum = 0;

                fps_frame_count = 0;
                fps_timer = Instant::now();
            }

            // Cap at ~60fps, but always yield so other tasks (BLE, I2C) get
            // a chance to run even when a frame takes longer than 16ms
            // (e.g. animation screens render ~67ms).
            let elapsed = frame_start.elapsed();
            if elapsed < Duration::from_millis(16) {
                Timer::after(Duration::from_millis(16) - elapsed).await;
            } else {
                embassy_futures::yield_now().await;
            }
        }
    }; // end app_future

    app_future.await;
}
