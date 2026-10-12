//! Timestamp capture configuration and readout (Application Manual 3.8–3.9, 4.14).
use crate::{
  DateTimeParts, Error, REG_CONTROL2, REG_COUNT_EVENTS_TS, REG_EVENT_CONTROL, REG_STATUS, RV3028,
};
use embedded_hal::i2c::I2c;

const TSE: u8 = 0b1000_0000;
const EIE: u8 = 0b0000_0100;
const EHL_ET: u8 = 0b0111_0000;
const TSR: u8 = 0b0000_0100;
const TSOW: u8 = 0b0000_0010;
const TSS: u8 = 0b0000_0001;
const EVF_BSF: u8 = 0b0010_0010;

/// Source of timestamp capture (TSS field).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TimestampSource {
  /// External event on EVI; detection polarity/filtering are configured separately.
  ExternalEvent = 0b0,
  /// Automatic switchover to VBACKUP; backup mode is configured separately.
  BackupSwitchover = 0b1,
}

/// Which event's date/time is retained (TSOW field).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TimestampCapture {
  /// Retain the first event after EVF was cleared.
  FirstEvent = 0b0,
  /// Replace the timestamp on each event, retaining the most recent.
  LastEvent = 0b1,
}

/// Timestamp settings only; EVI detection, backup settings and INT/CLKOUT routing
/// are independent. The event counter runs for both capture policies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimestampConfig {
  /// TSS: EVI or automatic backup switchover.
  pub source: TimestampSource,
  /// TSOW: retain the first or latest event.
  pub capture: TimestampCapture,
  /// TSE: enable event counting and timestamp capture.
  pub enabled: bool,
}

/// Snapshot of the seven Count TS through Year TS registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimestampSnapshot {
  /// Binary event count modulo 256, not a saturating or unbounded count.
  /// Zero can also mean that 256 events occurred since reset.
  pub count: u8,
  /// Recorded calendar time, normalized to hours 0–23. None denotes the
  /// all-zero timestamp register contents after reset, with count zero.
  /// A valid timestamp is retained even when the counter has wrapped to zero.
  /// This alone does not establish calendar accuracy or validity before capture.
  pub datetime: Option<DateTimeParts>,
}

impl<I: I2c> RV3028<I> {
  /// Read TSE, TSS and TSOW without writes or flag acknowledgements.
  /// Control 2 and Event Control are separate accesses; exclude concurrent edits.
  ///
  /// # Errors
  /// [`Error::I2c`] for bus or mux failures.
  pub fn get_timestamp_config(&mut self) -> Result<TimestampConfig, Error<I::Error>> {
    self.select_mux_channel()?;
    let control = self.read_register_raw(REG_CONTROL2)?;
    let event = self.read_register_raw(REG_EVENT_CONTROL)?;
    Ok(TimestampConfig {
      source: if event & TSS != 0 {
        TimestampSource::BackupSwitchover
      } else {
        TimestampSource::ExternalEvent
      },
      capture: if event & TSOW != 0 {
        TimestampCapture::LastEvent
      } else {
        TimestampCapture::FirstEvent
      },
      enabled: control & TSE != 0,
    })
  }

  /// Apply timestamp settings without resetting the log or clearing flags.
  ///
  /// Pauses TSE and EIE before changing TSS, preserving EVI polarity/filtering.
  /// Restores EIE and applies the requested TSE state afterwards. Other Control 2
  /// bits, backup configuration and CLKOUT routing remain unchanged. Events during
  /// the pause are not captured. Existing EVF can prevent FirstEvent capture;
  /// use [Self::configure_timestamp] to explicitly restart with an empty log.
  /// Exclude concurrent register changes across this sequence.
  ///
  /// # Errors
  /// [`Error::I2c`] for bus or mux failures. No retry or rollback; a partial
  /// operation may leave capture/interrupts disabled or configuration changed.
  pub fn set_timestamp_config(&mut self, config: &TimestampConfig) -> Result<(), Error<I::Error>> {
    self.select_mux_channel()?;
    self.timestamp_config_raw(config, false)
  }

  /// Restart timestamp logging using the manufacturer's initialization sequence.
  ///
  /// Disables TSE/EIE, sets TSOW, clears only EVF/BSF, pulses TSR to clear all
  /// seven log registers, selects TSS and applies the requested TSE state.
  /// EIE remains disabled; enable external-event INT output separately if needed.
  /// EVI polarity/filtering, other Control 2 bits, BSIE and CLKOUT routing are
  /// preserved. Configure the chosen EVI/backup source separately.
  ///
  /// This deliberately discards the old log and acknowledges EVF/BSF. Read any
  /// required timestamp first. Events during reconfiguration may be missed.
  ///
  /// # Errors
  /// [`Error::I2c`] for bus or mux failures. No retry or rollback is performed.
  pub fn configure_timestamp(&mut self, config: &TimestampConfig) -> Result<(), Error<I::Error>> {
    self.select_mux_channel()?;
    self.timestamp_config_raw(config, true)
  }

  fn timestamp_config_raw(
    &mut self,
    config: &TimestampConfig,
    restart: bool,
  ) -> Result<(), Error<I::Error>> {
    let control = self.read_register_raw(REG_CONTROL2)?;
    self.write_register_raw(REG_CONTROL2, control & !(TSE | EIE))?;
    let event = self.read_register_raw(REG_EVENT_CONTROL)? & EHL_ET;
    let overwrite = (config.capture as u8) << 1;
    if restart {
      self.write_register_raw(REG_EVENT_CONTROL, event | overwrite)?;
      self.clear_reg_bits_raw(REG_STATUS, EVF_BSF)?;
      self.write_register_raw(REG_EVENT_CONTROL, event | overwrite | TSR)?;
    }
    self.write_register_raw(REG_EVENT_CONTROL, event | overwrite | config.source as u8)?;
    let final_control = if restart {
      control & !(TSE | EIE)
    } else {
      control & !TSE
    };
    self.write_register_raw(
      REG_CONTROL2,
      final_control | if config.enabled { TSE } else { 0 },
    )
  }

  /// Reset only the seven timestamp registers by pulsing TSR.
  /// TSE, source, capture policy and status flags remain unchanged.
  /// This does not re-arm FirstEvent capture when EVF is still set.
  ///
  /// # Errors
  /// [`Error::I2c`] for bus or mux failures.
  pub fn reset_timestamp(&mut self) -> Result<(), Error<I::Error>> {
    self.select_mux_channel()?;
    let event = self.read_register_raw(REG_EVENT_CONTROL)? & (EHL_ET | TSOW | TSS);
    self.write_register_raw(REG_EVENT_CONTROL, event | TSR)
  }

  /// Read the complete timestamp register block in one burst, without writes.
  ///
  /// Capture is blocked by the chip during this read. Control 2 is read first
  /// for hour decoding; do not change the hour mode after capture or during readout.
  /// Count TS wraps modulo 256; zero count does not imply an empty timestamp.
  /// Only count=0 with all six calendar bytes zero returns datetime=None.
  /// PORF and flags are not checked or acknowledged. The captured calendar may
  /// have been inaccurate; trust and elapsed-time calculation belong to the owner.
  ///
  /// # Errors
  /// [`Error::InvalidData`] for malformed BCD/date/time, including a nonzero
  /// count with an empty log. [`Error::I2c`] for bus or mux failures.
  pub fn get_timestamp(&mut self) -> Result<TimestampSnapshot, Error<I::Error>> {
    self.select_mux_channel()?;
    let control = self.read_register_raw(REG_CONTROL2)?;
    let mut bytes = [0; 7];
    self.read_multi_registers_raw(REG_COUNT_EVENTS_TS, &mut bytes)?;
    let datetime = if bytes == [0; 7] {
      None
    } else {
      let parts = DateTimeParts {
        second: Self::checked_bcd(bytes[1])?,
        minute: Self::checked_bcd(bytes[2])?,
        hour: Self::calendar_hour(bytes[3], control)?,
        day: Self::checked_bcd(bytes[4])?,
        month: Self::checked_bcd(bytes[5])?,
        year: 2000 + i32::from(Self::checked_bcd(bytes[6])?),
      };
      parts.validate()?;
      Some(parts)
    };
    Ok(TimestampSnapshot {
      count: bytes[0],
      datetime,
    })
  }
}
