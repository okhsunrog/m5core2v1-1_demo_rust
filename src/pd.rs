use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use esp_hal::i2c::master::I2c;
use fusb302b::Fusb302b;
use log::{error, info};
use usbpd::protocol_layer::message::data::request::{self, PowerSource};
use usbpd::protocol_layer::message::data::source_capabilities::SourceCapabilities;
use usbpd::sink::device_policy_manager::{DevicePolicyManager, Event};
use usbpd::sink::policy_engine::Sink;
use usbpd::timers::Timer as PdTimer;

pub static CAPS_CHANNEL: Channel<CriticalSectionRawMutex, SourceCapabilities, 3> = Channel::new();

/// Signals that the FUSB302B was successfully initialized.
pub static INIT_OK_CHANNEL: Channel<CriticalSectionRawMutex, bool, 1> = Channel::new();

struct AppTimer;

impl PdTimer for AppTimer {
    async fn after_millis(milliseconds: u64) {
        Timer::after(Duration::from_millis(milliseconds)).await;
    }
}

struct MyDevicePolicyManager;

impl DevicePolicyManager for MyDevicePolicyManager {
    async fn request(&mut self, source_capabilities: &SourceCapabilities) -> PowerSource {
        info!(
            "Source capabilities received: {} PDOs",
            source_capabilities.pdos().len()
        );

        CAPS_CHANNEL.try_send(source_capabilities.clone()).ok();

        request::PowerSource::new_fixed(
            request::CurrentRequest::Highest,
            request::VoltageRequest::Safe5V,
            source_capabilities,
        )
        .unwrap()
    }

    async fn transition_power(&mut self, _accepted: &PowerSource) {
        info!("PD power transition accepted");
    }

    async fn get_event(&mut self, _source_capabilities: &SourceCapabilities) -> Event {
        core::future::pending().await
    }
}

#[embassy_executor::task]
pub async fn pd_task(i2c: I2c<'static, esp_hal::Async>) {
    let fusb_driver = match Fusb302b::init(i2c).await {
        Ok(driver) => driver,
        Err(e) => {
            error!("Failed to initialize FUSB302B: {:?}", e);
            INIT_OK_CHANNEL.try_send(false).ok();
            return;
        }
    };
    info!("FUSB302B initialized!");
    INIT_OK_CHANNEL.try_send(true).ok();

    let dpm = MyDevicePolicyManager;
    let mut policy_engine = Sink::<_, AppTimer, _>::new(fusb_driver, dpm);

    info!("Starting USB-PD policy engine...");
    if let Err(e) = policy_engine.run().await {
        error!("PD policy engine exited with error: {:?}", e);
    }
}
