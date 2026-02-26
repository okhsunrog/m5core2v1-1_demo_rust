//! BLE GATT server for time synchronization.
//!
//! Exposes a writable characteristic that accepts 7 bytes:
//! `[year-2000, month, day, weekday, hours, minutes, seconds]`
//!
//! When written, the data is sent via an embassy channel to the main loop,
//! which sets the RTC.

#![allow(clippy::needless_borrows_for_generic_args)] // macro-generated code

use bt_hci::controller::ExternalController;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_radio::ble::controller::BleConnector;
use log::{info, warn};
use trouble_host::prelude::*;

/// Channel for passing received time data from BLE to main loop.
pub static TIME_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 7], 1> = Channel::new();

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + ATT

// GATT Server definition
#[gatt_server]
struct Server {
    time_service: TimeService,
}

// Custom time sync service
#[allow(clippy::needless_borrows_for_generic_args)]
#[gatt_service(uuid = "12345678-1234-5678-1234-56789abc0010")]
struct TimeService {
    /// Time data: [year-2000, month, day, weekday, hours, minutes, seconds]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abc0011", write)]
    time_data: [u8; 7],
}

/// Run the BLE stack: advertise, accept connections, handle GATT writes.
///
/// This function never returns under normal operation.
pub async fn run(controller: ExternalController<BleConnector<'_>, 1>) {
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("[ble] address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "M5Core2",
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    info!("[ble] GATT server created, starting BLE stack");

    let _ = embassy_futures::join::join(
        // BLE HCI runner — must run forever alongside everything else
        async {
            loop {
                if let Err(e) = runner.run().await {
                    warn!("[ble] runner error: {:?}", e);
                }
            }
        },
        // Advertising + connection handling loop
        async {
            loop {
                match advertise(&mut peripheral, &server).await {
                    Ok(conn) => {
                        info!("[ble] connected");
                        handle_connection(&server, &conn).await;
                    }
                    Err(e) => {
                        warn!("[ble] advertising error: {:?}", e);
                    }
                }
            }
        },
    )
    .await;
}

async fn advertise<'values, 'server, C: Controller>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(b"M5Core2"),
        ],
        &mut adv_data[..],
    )?;

    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &[],
            },
        )
        .await?;

    info!("[ble] advertising...");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

async fn handle_connection<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    let time_handle = server.time_service.time_data.handle;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("[ble] disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::Gatt { event } => {
                if let GattEvent::Write(ref write_event) = event
                    && write_event.handle() == time_handle
                {
                    let data = write_event.data();
                    if data.len() == 7 {
                        let mut buf = [0u8; 7];
                        buf.copy_from_slice(data);
                        info!(
                            "[ble] time received: 20{:02}-{:02}-{:02} {:02}:{:02}:{:02}",
                            buf[0], buf[1], buf[2], buf[4], buf[5], buf[6]
                        );
                        // Non-blocking send — drop if channel full
                        let _ = TIME_CHANNEL.try_send(buf);
                    } else {
                        warn!("[ble] invalid time data length: {}", data.len());
                    }
                }
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[ble] error sending response: {:?}", e),
                }
            }
            _ => {}
        }
    }
}
