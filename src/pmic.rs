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

    // 1. Configure PowerKey timing (0x27)
    //   - off_level = S10: holding the key for 10 s hard-powers-off the board
    //                 (done entirely by the PMIC, independent of firmware).
    //   - irq_level = S2_5: the long-press IRQ asserts after 2.5 s of holding,
    //                 which firmware uses to pop the power menu.
    //   - on_level left at reset default (power-on press time).
    axp.ll
        .power_on_level()
        .write_async(|w| {
            w.set_off_level(axp2101_dd::OffLevel::S10);
            w.set_irq_level(axp2101_dd::IrqLevel::S25);
        })
        .await?;

    // 1b. Enable the power-key short- and long-press IRQs so firmware can
    // distinguish a quick tap (toggle display) from a hold (power menu).
    // Short-press is enabled at reset; explicitly enable long-press too.
    axp.ll
        .irq_enable_1()
        .modify_async(|w| {
            w.set_power_key_short_press_irq_enable(true);
            w.set_power_key_long_press_irq_enable(true);
        })
        .await?;

    // 1c. Clear any power-key IRQ latched during power-on (the button press
    // that turned the board on sets these). Otherwise the first poll would
    // fire a phantom gesture and pop the power menu right after boot.
    axp.ll
        .irq_status_1()
        .write_async(|w| {
            w.set_pons_irq(true);
            w.set_ponl_irq(true);
            w.set_ponn_irq(true);
            w.set_ponp_irq(true);
        })
        .await?;

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

    // 5. CHGLED — blue LED near power button
    // TypeB: off on battery, blinks 1Hz while charging, solid on when VBUS present + full/no-bat
    // Alternatives: TypeA (different charge indication), Manual (control via chgled_out_ctrl)
    axp.ll
        .chg_led_control()
        .write_async(|w| {
            w.set_chgled_en(true);
            w.set_chgled_func(axp2101_dd::ChgledFunction::TypeB);
            w.set_chgled_out_ctrl(axp2101_dd::ChgledOutputControl::Blink1Hz);
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

/// A power-key gesture reported by the AXP2101, as classified in hardware.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerKey {
    /// Quick tap (pressed and released before the long-press threshold).
    Short,
    /// Held past `irq_level` (2.5 s) — the long-press IRQ fired while holding.
    Long,
}

/// Poll the power-key interrupt status (REG49) and return the latched gesture,
/// if any. The IRQ bits are write-1-to-clear and latched, so periodic polling
/// never misses an event. `Long` takes priority when both bits are set.
pub async fn poll_power_key<I2C, E>(
    axp: &mut Axp2101Async<axp2101_dd::AxpInterface<I2C>, E>,
) -> Result<Option<PowerKey>, AxpError<E>>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: core::fmt::Debug,
{
    let status = axp.ll.irq_status_1().read_async().await?;
    let long = status.ponl_irq();
    let short = status.pons_irq();
    if !long && !short {
        return Ok(None);
    }

    // Clear only the bits we consume (writing 1 clears, writing 0 is a no-op).
    axp.ll
        .irq_status_1()
        .write_async(|w| {
            if long {
                w.set_ponl_irq(true);
            }
            if short {
                w.set_pons_irq(true);
            }
        })
        .await?;

    Ok(Some(if long { PowerKey::Long } else { PowerKey::Short }))
}

/// Configure all M5Stack Core2 v1.1 power rails
///
/// Based on Core2 v1.1 schematic (Sch_Core2_v1.1_2023-07-20.pdf):
/// - ALDO2: LCD & Touch Panel reset
/// - ALDO3: Speaker enable (NS4168)
/// - ALDO4: LCD & microSD power
/// - BLDO1: LCD backlight
/// - BLDO2: External 5V output enable (AXP_BoostEN)
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

    // ALDO3: Speaker (NS4168 SPK_EN). OFF at boot; pmic_task powers it only
    // during audio playback (driven by audio::SPEAKER_POWER) to cut idle draw.
    info!("  ALDO3: Speaker 3.3V (off until playback)");
    axp.set_ldo_voltage_mv(LdoId::Aldo3, 3300).await?;
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
#[derive(Clone, Copy)]
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
