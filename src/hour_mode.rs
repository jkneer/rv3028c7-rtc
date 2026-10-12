//! Hour encoding and guarded switching between the chip's 12-/24-hour modes.
use crate::{
  ALARM_NO_WATCH_FLAG, Error, REG_CONTROL2, REG_COUNT_EVENTS_TS, REG_HOURS_ALARM, RV3028,
};
use embedded_hal::i2c::I2c;

/// Hardware hour encoding. Public calendar and alarm hours remain 0–23 in both modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HourMode {
  /// 24-hour BCD encoding (Control 2 12_24=0).
  Hours24 = 0b0000_0000,
  /// 12-hour BCD encoding with AM/PM (Control 2 12_24=1).
  Hours12 = 0b0000_0010,
}

impl HourMode {
  /// Decode only 12_24 from Control 2, without bus access.
  pub const fn from_register(register: u8) -> Self {
    if register & 0b0000_0010 != 0 {
      Self::Hours12
    } else {
      Self::Hours24
    }
  }
}

impl<I: I2c> RV3028<I> {
  /// Read the current 12-/24-hour encoding without modifying the chip.
  ///
  /// # Errors
  /// [`Error::I2c`] on mux or bus failure.
  pub fn get_hour_mode(&mut self) -> Result<HourMode, Error<I::Error>> {
    self.select_mux_channel()?;
    self
      .read_register_raw(REG_CONTROL2)
      .map(HourMode::from_register)
  }

  /// Change hour encoding and rewrite the Hours Alarm value in the new format.
  ///
  /// The chip automatically converts the running Hours register. This method
  /// preserves the alarm's matching enable and civil hour. Invalid disabled alarm
  /// hours are normalized to midnight; invalid enabled hours fail before writes.
  /// AIE is paused during the mode/alarm writes, then restored. Other Control 2
  /// bits, Unix time, alarm minute/day and all status flags are preserved.
  /// AF may still latch during the transition; this method never acknowledges it.
  ///
  /// An unchanged mode is a read-only no-op. A real change requires TSE disabled
  /// and the complete timestamp log at 0x14–0x1A reset to zero, since captured
  /// hours cannot be rewritten. Disable capture and explicitly reset the log
  /// first; consuming or discarding timestamps belongs to the owner.
  ///
  /// Exclude concurrent mode/alarm/capture changes for the entire operation.
  /// No retry or rollback occurs. A bus failure can leave AIE disabled, the
  /// new mode applied, or an alarm awaiting conversion; inspect and recover.
  ///
  /// # Errors
  /// [`Error::InvalidData`] for active capture, a nonempty timestamp log or an
  /// invalid enabled alarm hour, before any writes.
  /// [`Error::I2c`] on mux or bus failure.
  pub fn set_hour_mode(&mut self, mode: HourMode) -> Result<(), Error<I::Error>> {
    self.select_mux_channel()?;
    let control2 = self.read_register_raw(REG_CONTROL2)?;
    if HourMode::from_register(control2) == mode {
      return Ok(());
    }
    if control2 & 0b1000_0000 != 0 {
      return Err(Error::InvalidData);
    }
    let mut timestamp = [0; 7];
    self.read_multi_registers_raw(REG_COUNT_EVENTS_TS, &mut timestamp)?;
    if timestamp != [0; 7] {
      return Err(Error::InvalidData);
    }
    let alarm = self.read_register_raw(REG_HOURS_ALARM)?;
    let hour = match Self::calendar_hour(alarm & !ALARM_NO_WATCH_FLAG, control2) {
      Ok(hour) => hour,
      Err(_) if alarm & ALARM_NO_WATCH_FLAG != 0 => 0,
      Err(error) => return Err(error),
    };
    // RESET is a self-clearing command: never pulse it while changing the mode.
    let paused = control2 & !0b0000_1001;
    let changed = (paused & !0b0000_0010) | mode as u8;
    self.write_register_raw(REG_CONTROL2, paused)?;
    self.write_register_raw(REG_CONTROL2, changed)?;
    self.write_register_raw(
      REG_HOURS_ALARM,
      Self::encode_calendar_hour(hour, changed) | (alarm & ALARM_NO_WATCH_FLAG),
    )?;
    self.write_register_raw(REG_CONTROL2, changed | (control2 & 0b0000_1000))
  }
}
