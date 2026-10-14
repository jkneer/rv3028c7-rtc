//! Typed countdown presets and read-only preset inspection.
use crate::{Error, REG_CONTROL1, REG_TIMER_VALUE0, RV3028, RegControl1Bits};
use core::time::Duration;
use embedded_hal::i2c::I2c;

/// Unshifted TD field: countdown source clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TimerClockFreq {
  /// TD=00: 4096 Hz, nominal tick 244.140625 µs.
  Hertz4096 = 0b00,
  /// TD=01: 64 Hz, nominal tick 15.625 ms.
  Hertz64 = 0b01,
  /// TD=10: 1 Hz, nominal tick 1 s.
  Hertz1 = 0b10,
  /// TD=11: 1/60 Hz, nominal tick 60 s.
  HertzSixtieth = 0b11,
}

impl TimerClockFreq {
  /// Decode only TD from Control 1 without bus access.
  pub const fn from_register(control1: u8) -> Self {
    match control1 & 0b11 {
      0b00 => Self::Hertz4096,
      0b01 => Self::Hertz64,
      0b10 => Self::Hertz1,
      _ => Self::HertzSixtieth,
    }
  }

  const fn ratio(self) -> (u128, u128) {
    match self {
      Self::Hertz4096 => (4096, 1),
      Self::Hertz64 => (64, 1),
      Self::Hertz1 => (1, 1),
      Self::HertzSixtieth => (1, 60),
    }
  }
}

/// Maximum 12-bit countdown preset (0x0FFF ticks).
pub const COUNTDOWN_MAX_TICKS: u16 = 0x0FFF;

/// Timer preset and control state, separate from its current counter and TIE.
///
/// TE can clear automatically when a one-shot expires. Reading this configuration
/// returns the preset at 0x0A–0x0B, not the running count at 0x0C–0x0D.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CountdownTimerConfig {
  /// TD: exact source clock.
  pub clock: TimerClockFreq,
  /// Binary 12-bit preset, 0–4095. Zero does not count down.
  /// Setters reject larger values before any bus access.
  pub ticks: u16,
  /// TRPT: automatically reload the preset after expiry.
  pub repeat: bool,
  /// TE: start/stop state, not the timer interrupt enable.
  pub enabled: bool,
}

impl CountdownTimerConfig {
  /// Quantize a duration without I/O, using the finest clock whose full 12-bit
  /// range includes it, and round down to whole ticks. Positive sub-tick requests
  /// clamp to one 4096-Hz tick, requests above 4095 minutes clamp to the maximum,
  /// and zero produces zero ticks. Neither repeat nor enabled is inferred.
  pub fn from_duration(duration: &Duration, repeat: bool, enabled: bool) -> Self {
    let nanos = duration.as_nanos();
    for clock in [
      TimerClockFreq::Hertz4096,
      TimerClockFreq::Hertz64,
      TimerClockFreq::Hertz1,
      TimerClockFreq::HertzSixtieth,
    ] {
      let (numerator, denominator) = clock.ratio();
      let scaled = nanos * numerator;
      let unit = 1_000_000_000 * denominator;
      if scaled <= u128::from(COUNTDOWN_MAX_TICKS) * unit {
        let ticks = if nanos == 0 {
          0
        } else {
          (scaled / unit).max(1)
        };
        return Self {
          clock,
          ticks: ticks as u16,
          repeat,
          enabled,
        };
      }
    }
    Self {
      clock: TimerClockFreq::HertzSixtieth,
      ticks: COUNTDOWN_MAX_TICKS,
      repeat,
      enabled,
    }
  }

  /// Nominal preset period, truncating fractional nanoseconds.
  /// Returns None for a preset exceeding 12 bits. Start phase affects the first
  /// expiry; this is not a first-expiry deadline (Application Manual 4.8.3).
  pub fn nominal_period(&self) -> Option<Duration> {
    if self.ticks > COUNTDOWN_MAX_TICKS {
      return None;
    }
    let (numerator, denominator) = self.clock.ratio();
    Some(Duration::from_nanos(
      (u128::from(self.ticks) * 1_000_000_000 * denominator / numerator) as u64,
    ))
  }
}

impl<I: I2c> RV3028<I> {
  /// Read Control 1 and the two preset bytes, without writes or TF acknowledgement.
  ///
  /// Unimplemented preset bits 7:4 are ignored. The running counter is read by
  /// [Self::get_countdown_value]. Control and preset reads are separate; exclude
  /// concurrent configuration changes. TE may change after it is sampled.
  ///
  /// # Errors
  /// [Error::I2c] on mux or bus failure.
  pub fn get_countdown_timer_config(&mut self) -> Result<CountdownTimerConfig, Error<I::Error>> {
    self.select_mux_channel()?;
    let control = self.read_register_raw(REG_CONTROL1)?;
    let mut preset = [0; 2];
    self.read_multi_registers_raw(REG_TIMER_VALUE0, &mut preset)?;
    Ok(CountdownTimerConfig {
      clock: TimerClockFreq::from_register(control),
      ticks: u16::from(preset[0]) | (u16::from(preset[1] & 0b0000_1111) << 8),
      repeat: control & 0b1000_0000 != 0,
      enabled: control & 0b0000_0100 != 0,
    })
  }

  /// Stop the timer, apply the preset/repeat/clock, acknowledge only TF, and
  /// optionally start a fresh countdown according to enabled.
  ///
  /// Preserves WADA, USEL, EERD, interrupt enables and CLKOUT routing. Even
  /// reapplying the same enabled configuration restarts from its preset.
  /// Exclude concurrent timer/control changes. Bus failures stop without retry
  /// or rollback; the timer can remain stopped or partially configured.
  ///
  /// # Errors
  /// [Error::InvalidData] for ticks above 0x0FFF, before any bus/mux access.
  /// [Error::I2c] on mux or bus failure.
  pub fn set_countdown_timer_config(
    &mut self,
    config: &CountdownTimerConfig,
  ) -> Result<(), Error<I::Error>> {
    if config.ticks > COUNTDOWN_MAX_TICKS {
      return Err(Error::InvalidData);
    }
    self.select_mux_channel()?;
    self.config_pct_raw(config.ticks, config.clock, config.repeat)?;
    if config.enabled {
      self.set_reg_bits_raw(REG_CONTROL1, RegControl1Bits::TimerEnableBit as u8)?;
    }
    Ok(())
  }
}
