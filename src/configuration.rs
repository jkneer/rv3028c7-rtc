//! Typed configuration of Backup (0x37) and CLKOUT (0x35) RAM mirrors.
//! Register definitions follow Application Manual rev. 1.4, sections 3.15.4,
//! 3.15.6 and 4.2–4.4. EEPROM persistence is deliberately separate.

use crate::{Error, RV3028, TrickleChargeCurrentLimiter};
use embedded_hal::i2c::I2c;

// Backup: `EEOffset[0]` BSIE TCE FEDE BSM[1:0] TCR[1:0].
pub(crate) const BACKUP_REGISTER: u8 = 0x37;
const OFFSET_LSB: u8 = 0b1000_0000;
const BSIE: u8 = 0b0100_0000;
const TCE: u8 = 0b0010_0000;
const FEDE: u8 = 0b0001_0000;
const BSM: u8 = 0b0000_1100;
const TCR: u8 = 0b0000_0011;
// CLKOUT: CLKOE CLKSY - - PORIE FD[2:0].
pub(crate) const CLKOUT_REGISTER: u8 = 0x35;
const CLKOE: u8 = 0b1000_0000;
const CLKSY: u8 = 0b0100_0000;
const PORIE: u8 = 0b0000_1000;
const FD: u8 = 0b0000_0111;

/// Automatic VDD-to-VBACKUP switchover; values are the unshifted BSM field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BackupSwitchoverMode {
  /// No switchover (BSM=00). Reading BSM=10 also returns this variant;
  /// encoding normalizes either disabled setting to 00.
  Disabled = 0b00,
  /// Direct Switching Mode (BSM=01): switch when VDD < VBACKUP.
  Direct = 0b01,
  /// Level Switching Mode (BSM=11): switch when VDD < 2.0 V and VBACKUP > 2.0 V.
  Level = 0b11,
}

/// Backup settings, excluding the oscillator calibration bit `EEOffset[0]`.
///
/// No board-specific defaults are selected. Charging and switchover suitability
/// depend on the attached backup supply; the driver cannot check those voltages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BackupConfig {
  /// BSM: automatic backup switchover mode.
  pub switchover_mode: BackupSwitchoverMode,
  /// BSIE: enable INT output for a backup switchover; does not clear BSF.
  pub switchover_interrupt_enabled: bool,
  /// TCE: enable the internal trickle charger (disabled on backup power).
  pub trickle_charge_enabled: bool,
  /// TCR: series resistance of the charger, also retained when charging is off.
  pub trickle_charge_resistance: TrickleChargeCurrentLimiter,
  /// FEDE: fast-edge detection. The manufacturer specifies keeping this true.
  /// The raw hardware setting remains representable for inspection and testing.
  pub fast_edge_detection_enabled: bool,
}

impl BackupConfig {
  /// Decode register 0x37, ignoring the calibration bit.
  /// Both disabled BSM encodings (00 and 10) decode to Disabled.
  pub const fn from_register(register: u8) -> Self {
    Self {
      switchover_mode: match register & BSM {
        0b0000_0100 => BackupSwitchoverMode::Direct,
        0b0000_1100 => BackupSwitchoverMode::Level,
        _ => BackupSwitchoverMode::Disabled,
      },
      switchover_interrupt_enabled: register & BSIE != 0,
      trickle_charge_enabled: register & TCE != 0,
      trickle_charge_resistance: match register & TCR {
        0b00 => TrickleChargeCurrentLimiter::Ohms3k,
        0b01 => TrickleChargeCurrentLimiter::Ohms5k,
        0b10 => TrickleChargeCurrentLimiter::Ohms9k,
        _ => TrickleChargeCurrentLimiter::Ohms15k,
      },
      fast_edge_detection_enabled: register & FEDE != 0,
    }
  }

  /// Encode settings into register 0x37, preserving `EEOffset[0]` from `current`.
  /// No bus access or EEPROM operation is performed.
  pub const fn to_register(self, current: u8) -> u8 {
    (current & OFFSET_LSB)
      | ((self.switchover_mode as u8) << 2)
      | if self.switchover_interrupt_enabled {
        BSIE
      } else {
        0
      }
      | if self.trickle_charge_enabled { TCE } else { 0 }
      | if self.fast_edge_detection_enabled {
        FEDE
      } else {
        0
      }
      | self.trickle_charge_resistance as u8
  }
}

/// CLKOUT signal selection; values are the FD field, not register addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ClkoutSource {
  /// 32768 Hz directly from the crystal; not digitally offset-compensated.
  Hertz32768 = 0b000,
  /// 8192 Hz, digitally offset-compensated.
  Hertz8192 = 0b001,
  /// 1024 Hz, digitally offset-compensated.
  Hertz1024 = 0b010,
  /// 64 Hz, digitally offset-compensated.
  Hertz64 = 0b011,
  /// 32 Hz, digitally offset-compensated.
  Hertz32 = 0b100,
  /// 1 Hz, digitally offset-compensated.
  Hertz1 = 0b101,
  /// Periodic countdown timer interrupt signal; CLKSY has no effect.
  CountdownTimerInterrupt = 0b110,
  /// Force CLKOUT LOW regardless of CLKOE and the transient CLKF flag.
  ForcedLow = 0b111,
}

/// Settings of the CLKOUT register 0x35, including its POR interrupt setting.
///
/// CLKIE (Control 2), interrupt source masks (0x12) and CLKF (Status) are
/// independent; this configuration does not change or acknowledge them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClkoutConfig {
  /// FD: output frequency, timer interrupt signal, or forced LOW.
  pub source: ClkoutSource,
  /// CLKOE: normal clock output enable. CLKF can enable output even if false;
  /// select ForcedLow to hold the pin LOW independently of that flag.
  pub enabled: bool,
  /// CLKSY: synchronize output enabling/disabling to the selected clock.
  pub synchronized: bool,
  /// PORIE: enable power-on-reset INT output; does not acknowledge PORF.
  pub power_on_reset_interrupt_enabled: bool,
}

impl ClkoutConfig {
  /// Decode register 0x35; unimplemented bits 5:4 are ignored.
  pub const fn from_register(register: u8) -> Self {
    Self {
      source: match register & FD {
        0b000 => ClkoutSource::Hertz32768,
        0b001 => ClkoutSource::Hertz8192,
        0b010 => ClkoutSource::Hertz1024,
        0b011 => ClkoutSource::Hertz64,
        0b100 => ClkoutSource::Hertz32,
        0b101 => ClkoutSource::Hertz1,
        0b110 => ClkoutSource::CountdownTimerInterrupt,
        _ => ClkoutSource::ForcedLow,
      },
      enabled: register & CLKOE != 0,
      synchronized: register & CLKSY != 0,
      power_on_reset_interrupt_enabled: register & PORIE != 0,
    }
  }

  /// Encode register 0x35; unimplemented bits 5:4 are always zero.
  /// No bus access or EEPROM operation is performed.
  pub const fn to_register(self) -> u8 {
    self.source as u8
      | if self.enabled { CLKOE } else { 0 }
      | if self.synchronized { CLKSY } else { 0 }
      | if self.power_on_reset_interrupt_enabled {
        PORIE
      } else {
        0
      }
  }
}

impl<I: I2c> RV3028<I> {
  /// Read active Backup settings from RAM mirror 0x37, not persistent EEPROM.
  /// No flags are acknowledged.
  ///
  /// # Errors
  /// [`Error::I2c`] for bus or mux failures.
  pub fn get_backup_config(&mut self) -> Result<BackupConfig, Error<I::Error>> {
    self.select_mux_channel()?;
    self
      .read_register_raw(BACKUP_REGISTER)
      .map(BackupConfig::from_register)
  }

  /// Apply Backup settings to RAM mirror 0x37, preserving `EEOffset[0]`.
  ///
  /// This reads the current register then writes the merged settings. It does
  /// not persist EEPROM, change EERD or acknowledge flags. A later automatic,
  /// software or power-on refresh replaces these settings with EEPROM contents.
  /// Exclude concurrent refresh/calibration changes during this read/write pair.
  /// Keep fast-edge detection enabled as specified by the manufacturer.
  /// LSM activation may take up to 15.625 ms; this method does not wait.
  ///
  /// # Errors
  /// [`Error::I2c`] for bus or mux failures. No retry or readback is performed.
  pub fn set_backup_config(&mut self, config: &BackupConfig) -> Result<(), Error<I::Error>> {
    self.select_mux_channel()?;
    let current = self.read_register_raw(BACKUP_REGISTER)?;
    self.write_register_raw(BACKUP_REGISTER, config.to_register(current))
  }

  /// Read active CLKOUT settings from RAM mirror 0x35, not persistent EEPROM.
  /// No flags are acknowledged.
  ///
  /// # Errors
  /// [`Error::I2c`] for bus or mux failures.
  pub fn get_clkout_config(&mut self) -> Result<ClkoutConfig, Error<I::Error>> {
    self.select_mux_channel()?;
    self
      .read_register_raw(CLKOUT_REGISTER)
      .map(ClkoutConfig::from_register)
  }

  /// Apply CLKOUT settings to RAM mirror 0x35.
  ///
  /// EEPROM, EERD, Control 2, interrupt masks and status flags are unchanged.
  /// A subsequent refresh can replace the active settings from EEPROM.
  /// This is a direct register write, not a glitch-free frequency-change
  /// sequence: disable both CLKOE and CLKF before selecting a new frequency
  /// when glitch-free switching is required (Application Manual 4.4.4).
  ///
  /// # Errors
  /// [`Error::I2c`] for bus or mux failures. No retry or readback is performed.
  pub fn set_clkout_config(&mut self, config: &ClkoutConfig) -> Result<(), Error<I::Error>> {
    self.select_mux_channel()?;
    self.write_register_raw(CLKOUT_REGISTER, config.to_register())
  }
}
