//! PMIC (AXP2101) configuration for M5Stack Core2 v1.1
//!
//! This module provides async initialization and configuration for the AXP2101
//! power management IC on the M5Stack Core2 v1.1 board.

use axp2101_dd::{AdcChannel, Axp2101Async, AxpError, FastChargeCurrentLimit, LdoId};
use log::info;

/// Initialize the AXP2101 PMIC following M5Stack Core2 v1.1 initialization sequence
///
/// This follows the exact initialization sequence from M5Unified for Core2 v1.1:
/// - PowerKey timing: Hold=1sec, PowerOff=4sec
/// - Internal off-discharge enabled for DCDC/LDO/SWITCH
/// - BATFET disabled
/// - Battery detection enabled
/// - CHGLED configured
/// - Vibration motor (DLDO1) disabled
/// - ADC channels enabled for monitoring
pub async fn init_pmic<I2C, E>(
    i2c: I2C,
) -> Result<Axp2101Async<axp2101_dd::AxpInterface<I2C>, E>, AxpError<E>>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: core::fmt::Debug,
{
    let mut axp = Axp2101Async::new(i2c);

    info!("Initializing AXP2101 PMIC for M5Stack Core2 v1.1");

    // Verify chip ID
    let chip_id = axp.get_chip_id().await?;
    info!("AXP2101 Chip ID: 0x{:02X}", chip_id);

    // M5Stack uses 0x4A (Version A, chip_id_low=1010)
    // Datasheet specifies 0x47 (Version A, chip_id_low=0111)
    if chip_id != 0x4A && chip_id != 0x47 {
        log::warn!(
            "Unexpected chip ID: 0x{:02X} (expected 0x4A or 0x47)",
            chip_id
        );
    }

    // === M5Stack Core2 v1.1 Initialization Sequence ===
    // Following M5Unified Power_Class.cpp:458-468

    // 1. Configure PowerKey timing (0x27 = 0x00)
    // PowerKey Hold=1sec, PowerOff=4sec
    axp.ll.power_on_level().write_async(|_| {}).await?;

    // 2. PMU common config (0x10 = 0x30)
    // 0x30 = 0b00110000: discharge_off_enable=1, reserved_bit4=1, pwrok_restart_enable=0
    // Enable internal off-discharge, but DISABLE pwrok restart to prevent
    // AXP_ESP from being cut when reset button (S1) is pressed
    // Note: M5Unified sets bit 4 (reserved) to 1, but we leave it as 0 - works fine
    axp.ll
        .common_config()
        .write_async(|w| {
            w.set_discharge_off_enable(true); // Bit 5: Enable discharge
            w.set_pwrok_restart_enable(false); // Bit 3: DISABLE PWROK restart!
            // Bit 4 (reserved): M5Unified sets to 1, we default to 0
        })
        .await?;

    // 3. BATFET disable (0x12 = 0x00)
    axp.ll.batfet_control().write_async(|_| {}).await?;

    // 4. Battery detection enabled (0x68 = 0x01)
    axp.ll
        .battery_detection_control()
        .write_async(|w| {
            w.set_bat_det_en(true);
        })
        .await?;

    // 5. CHGLED setting (0x69 = 0x13)
    // 0x13 = 0b00010011: chgled_out_ctrl=001, chgled_func=01, chgled_en=1
    axp.ll
        .chg_led_control()
        .write_async(|w| {
            w.set_chgled_en(true); // Bit 0: Enable CHGLED
            w.set_chgled_func(1); // Bits 1-2: Function mode 1
            w.set_chgled_out_ctrl(1); // Bits 4-6: Output control mode 1
        })
        .await?;

    // 6. DLDO1 set 0.5V - vibration motor OFF (0x99 = 0x00)
    axp.set_ldo_voltage_mv(LdoId::Dldo1, 500).await?;

    // 7. Set fast charge current to 500mA (battery is 500mAh, 1C rate)
    axp.set_battery_charge_current(FastChargeCurrentLimit::Ma500)
        .await?;

    // 8. Enable ADC channels for monitoring
    axp.set_adc_channel_enable(AdcChannel::BatteryVoltage, true)
        .await?;
    axp.set_adc_channel_enable(AdcChannel::VbusVoltage, true)
        .await?;
    axp.set_adc_channel_enable(AdcChannel::VsysVoltage, true)
        .await?;
    axp.set_adc_channel_enable(AdcChannel::DieTemperature, true)
        .await?;
    // GPADC not enabled - pin not used on M5Stack Core2 v1.1

    info!("M5Stack initialization sequence completed");

    // Read and display initial status
    let battery_mv = axp.get_battery_voltage_mv().await?;
    let vbus_mv = axp.get_vbus_voltage_mv().await?;
    let vsys_mv = axp.get_vsys_voltage_mv().await?;
    let temp_c = axp.get_die_temperature_c().await?;

    // Read battery percentage (State of Charge)
    let soc = axp.ll.battery_percentage().read_async().await?;
    let soc_percent = soc.percentage();

    info!("Power Status:");
    info!("  Battery: {} mV ({}%)", battery_mv, soc_percent);
    info!("  VBUS:    {} mV", vbus_mv);
    info!("  VSYS:    {} mV", vsys_mv);
    info!("  Temp:    {:.1} C", temp_c);

    Ok(axp)
}

/// Configure all M5Stack Core2 v1.1 power rails
///
/// Based on Core2 v1.1 schematic (Sch_Core2_v1.1_2023-07-20.pdf):
/// - ALDO2: LCD & Touch Panel reset
/// - ALDO3: Speaker enable (NS4168)
/// - ALDO4: LCD & microSD power
/// - BLDO1: LCD backlight
/// - BLDO2: (Not used in schematic)
/// - DLDO1: Vibration motor
/// - CHGLED: Blue power indicator LED (controlled separately, not an LDO)
pub async fn configure_all_rails<I2C, E>(
    axp: &mut Axp2101Async<axp2101_dd::AxpInterface<I2C>, E>,
) -> Result<(), AxpError<E>>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: core::fmt::Debug,
{
    info!("Configuring M5Stack Core2 v1.1 power rails per schematic...");

    // ALDO4: LCD & microSD power (ILI9342C PWR)
    info!("  ALDO4: LCD & microSD 3.3V");
    axp.set_ldo_enable(LdoId::Aldo4, true).await?;
    axp.set_ldo_voltage_mv(LdoId::Aldo4, 3300).await?;

    // ALDO2: LCD & Touch Panel reset (ILI9342C RST, FT6336U RST)
    info!("  ALDO2: LCD & Touch Reset 3.3V");
    axp.set_ldo_enable(LdoId::Aldo2, true).await?;
    axp.set_ldo_voltage_mv(LdoId::Aldo2, 3300).await?;

    // ALDO3: Speaker enable (NS4168 SPK_EN) - disabled to save power
    info!("  ALDO3: Speaker (disabled)");
    axp.set_ldo_enable(LdoId::Aldo3, false).await?;

    // BLDO1: LCD backlight (ILI9342C BL)
    info!("  BLDO1: LCD Backlight 3.3V");
    axp.set_ldo_enable(LdoId::Bldo1, true).await?;
    axp.set_ldo_voltage_mv(LdoId::Bldo1, 3300).await?;

    // DLDO1: Vibration motor - disabled by default
    // M5Unified sets this to 0.5V (0x00) to disable the motor
    info!("  DLDO1: Vibration Motor (disabled)");
    axp.set_ldo_enable(LdoId::Dldo1, false).await?;

    // Note: Blue LED is connected to CHGLED pin (controlled via register 0x69)
    // ALDO1, BLDO2, and DLDO2 are not used in the Core2 v1.1 schematic

    info!("All power rails configured per Core2 v1.1 schematic");
    Ok(())
}

/// LCD backlight state.
pub enum Backlight {
    /// Backlight on at given brightness (0-100%).
    /// 0% = minimum visible brightness, 100% = maximum.
    On(u8),
    /// Backlight completely off (BLDO1 disabled).
    Off,
}

/// Set LCD backlight for M5Stack Core2 v1.1.
///
/// On Core2 v1.1, the LCD backlight is controlled by the BLDO1 output of the AXP2101 PMIC.
/// Brightness is adjusted by varying the BLDO1 voltage (2588-3300mV usable range).
pub async fn set_backlight<I2C, E>(
    axp: &mut Axp2101Async<axp2101_dd::AxpInterface<I2C>, E>,
    backlight: Backlight,
) -> Result<(), AxpError<E>>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: core::fmt::Debug,
{
    match backlight {
        Backlight::Off => {
            axp.set_ldo_enable(LdoId::Bldo1, false).await?;
            info!("Backlight disabled");
        }
        Backlight::On(percent) => {
            let percent = percent.min(100);
            // Map 0-100 to voltage 2588-3300mV
            let voltage_mv = 2588 + (percent as u32 * 712 / 100);

            axp.set_ldo_voltage_mv(LdoId::Bldo1, voltage_mv as u16)
                .await?;
            axp.set_ldo_enable(LdoId::Bldo1, true).await?;

            info!("Backlight set to {}% ({}mV)", percent, voltage_mv);
        }
    }

    Ok(())
}
