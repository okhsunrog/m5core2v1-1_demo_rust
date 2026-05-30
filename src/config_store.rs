//! Persistent config storage in the `nvs` partition via `sequential-storage`.
//!
//! Stores user-tweakable settings (backlight brightness, external 5V output
//! state) across reboots. The writer task on core 0 owns the flash and
//! batches writes — in particular `PERSIST_BACKLIGHT` is debounced so that
//! dragging the brightness slider doesn't hammer flash.

use core::future::pending;

use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_storage::FlashStorage;
use log::{info, warn};
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage, PostcardValue};
use serde::{Deserialize, Serialize};

/// `nvs` partition from the default ESP-IDF partition table (24 KB,
/// 0x9000..0xF000). Unused by this firmware, so we claim it as a wear-leveled
/// key/value store.
const NVS_START: u32 = 0x9000;
const NVS_END: u32 = 0xF000;

const KEY_BACKLIGHT: u8 = 0;
const KEY_EXT5V: u8 = 1;

/// Time to wait after the last `PERSIST_BACKLIGHT` signal before actually
/// writing to flash. Long enough that slider drags collapse into one write.
const BACKLIGHT_DEBOUNCE: Duration = Duration::from_secs(3);

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
struct Backlight(u8);
impl<'a> PostcardValue<'a> for Backlight {}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
struct Ext5v(bool);
impl<'a> PostcardValue<'a> for Ext5v {}

/// Values loaded from flash at boot. `None` for keys that haven't been
/// written yet (first boot or after a flash erase).
#[derive(Clone, Copy, Debug, Default)]
pub struct LoadedConfig {
    pub backlight: Option<u8>,
    pub ext5v: Option<bool>,
}

/// Signaled by the UI when the backlight slider changes. Resets the debounce
/// timer; only the latest value is written.
pub static PERSIST_BACKLIGHT: Signal<CriticalSectionRawMutex, u8> = Signal::new();

/// Signaled by the UI when the external 5V toggle changes. Written
/// immediately (no debounce — only changes on tap).
pub static PERSIST_EXT5V: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// Carries the loaded config to the render task (consumed by its single
/// `wait()`).
pub static CONFIG_LOADED: Signal<CriticalSectionRawMutex, LoadedConfig> = Signal::new();

/// Unit gate raised the instant the initial flash read finishes. `main` awaits
/// this before starting the second core, so the first (cache-disabling) flash
/// access never races the second core booting from flash (XIP). Distinct from
/// `CONFIG_LOADED` because `Signal::wait` consumes the value, and the render
/// task is the consumer of `CONFIG_LOADED`.
pub static INITIAL_READ_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task]
pub async fn task(flash: esp_hal::peripherals::FLASH<'static>) {
    let flash = BlockingAsync::new(FlashStorage::new(flash).multicore_auto_park());
    let config = const { MapConfig::new(NVS_START..NVS_END) };
    let mut storage = MapStorage::<u8, _, _>::new(flash, config, NoCache::new());
    let mut buf = [0u8; 128];

    // --- Initial read ---
    let backlight = storage
        .fetch_item::<Backlight>(&mut buf, &KEY_BACKLIGHT)
        .await
        .ok()
        .flatten()
        .map(|b| b.0);

    let ext5v = storage
        .fetch_item::<Ext5v>(&mut buf, &KEY_EXT5V)
        .await
        .ok()
        .flatten()
        .map(|e| e.0);

    info!("[cfg] loaded: backlight={:?}, ext5v={:?}", backlight, ext5v);
    CONFIG_LOADED.signal(LoadedConfig { backlight, ext5v });
    // Release `main` to start the second core now that the cache-disabling
    // initial read is done (no XIP-from-flash race on the other core).
    INITIAL_READ_DONE.signal(());

    // --- Writer loop ---
    let mut pending_backlight: Option<u8> = None;

    loop {
        // If we have a pending backlight value, race the debounce timer
        // against any new event. A new event resets the timer because the
        // entire select restarts.
        let debounce = async {
            if pending_backlight.is_some() {
                Timer::after(BACKLIGHT_DEBOUNCE).await;
            } else {
                pending::<()>().await;
            }
        };

        match select(
            select(PERSIST_BACKLIGHT.wait(), PERSIST_EXT5V.wait()),
            debounce,
        )
        .await
        {
            Either::First(Either::First(bl)) => {
                pending_backlight = Some(bl);
            }
            Either::First(Either::Second(ext)) => {
                match storage.store_item(&mut buf, &KEY_EXT5V, &Ext5v(ext)).await {
                    Ok(()) => info!("[cfg] ext5v persisted: {}", ext),
                    Err(e) => warn!("[cfg] failed to store ext5v: {:?}", e),
                }
            }
            Either::Second(()) => {
                if let Some(bl) = pending_backlight.take() {
                    match storage
                        .store_item(&mut buf, &KEY_BACKLIGHT, &Backlight(bl))
                        .await
                    {
                        Ok(()) => info!("[cfg] backlight persisted: {}", bl),
                        Err(e) => warn!("[cfg] failed to store backlight: {:?}", e),
                    }
                }
            }
        }
    }
}
