//! Typed EVI detection and its existing interrupt/CLKOUT routing controls.
use crate::{Error, REG_CONTROL2, RV3028};
use embedded_hal::i2c::I2c;

/// EHL field, unshifted. ET selects edge or level detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EviPolarity {
  /// Falling edge with ET=00; low level with filtering.
  FallingOrLow = 0b0,
  /// Rising edge with ET=00; high level with filtering.
  RisingOrHigh = 0b1,
}

/// Unshifted ET field. Names use the sampling periods rounded in the manual.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EviFilter {
  /// ET=00: edge detection without level filtering.
  EdgeDetection = 0b00,
  /// ET=01: level sampling at 256 Hz, approximately 3.9 ms.
  Millis3Point9 = 0b01,
  /// ET=10: level sampling at 64 Hz, approximately 15.6 ms.
  Millis15Point6 = 0b10,
  /// ET=11: level sampling at 8 Hz, 125 ms.
  Millis125 = 0b11,
}

impl EviFilter {
  /// Validate an unshifted ET value without bus access.
  pub const fn from_bits(bits: u8) -> Option<Self> {
    match bits {
      0b00 => Some(Self::EdgeDetection),
      0b01 => Some(Self::Millis3Point9),
      0b10 => Some(Self::Millis15Point6),
      0b11 => Some(Self::Millis125),
      _ => None,
    }
  }
}

/// EVI detection settings and shared event-output enables.
///
/// EIE/CEIE route the event source selected by TSS, which can also be Backup
/// Switchover. This configuration does not select TSS or enable timestamp capture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EviConfig {
  /// EHL: selected edge/level polarity.
  pub polarity: EviPolarity,
  /// ET: edge detection or level-filter sampling period.
  pub filter: EviFilter,
  /// EIE: route the selected event source to INT.
  pub interrupt_enabled: bool,
  /// CEIE: select the event source for interrupt-controlled CLKOUT.
  /// CLKIE and CLKOUT frequency/enable settings remain independent.
  pub clockout_enabled: bool,
}

impl<I: I2c> RV3028<I> {
  /// Read EVI settings and event-output enables in one burst from 0x10–0x13.
  ///
  /// Read-only: no flags or timestamp logs are acknowledged/reset. The burst
  /// includes GP Bits but does not modify them. Exclude concurrent configuration
  /// changes; this is not an atomic snapshot of asynchronous hardware events.
  ///
  /// # Errors
  /// [Error::I2c] on mux or bus failure.
  pub fn get_evi_config(&mut self) -> Result<EviConfig, Error<I::Error>> {
    self.select_mux_channel()?;
    let mut registers = [0; 4];
    self.read_multi_registers_raw(REG_CONTROL2, &mut registers)?;
    Ok(EviConfig {
      polarity: if registers[3] & 0b0100_0000 != 0 {
        EviPolarity::RisingOrHigh
      } else {
        EviPolarity::FallingOrLow
      },
      filter: EviFilter::from_bits((registers[3] >> 4) & 0b11).unwrap(),
      interrupt_enabled: registers[0] & 0b0000_0100 != 0,
      clockout_enabled: registers[2] & 0b0000_1000 != 0,
    })
  }

  /// Apply EVI settings, pausing EIE and acknowledging only EVF.
  ///
  /// Preserves timestamp source, overwrite, enable and retained log, unrelated
  /// interrupt enables, hour mode and CLKOUT sources. Does not enable CLKIE.
  /// Capture remains active if TSE is set: changing EVI detection can affect
  /// which events are captured. The owner must pause capture when necessary.
  /// Exclude concurrent configuration changes throughout the operation.
  /// Bus failure stops immediately without rollback; settings may be partial
  /// and EIE may remain disabled. Acknowledged EVF cannot be recovered.
  ///
  /// # Errors
  /// [Error::I2c] on mux or bus failure.
  pub fn set_evi_config(&mut self, config: &EviConfig) -> Result<(), Error<I::Error>> {
    self.select_mux_channel()?;
    self.config_evi_raw(config)
  }
}
