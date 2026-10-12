//! EEPROM command protocol from the RV-3028-C7 Application Manual rev. 1.4,
//! sections 4.6.3–4.6.9. No allocation or platform-specific sleeping is used.

use crate::{Error, REG_CONTROL1, REG_STATUS, RV3028};
use core::fmt;
use embedded_hal::{delay::DelayNs, i2c::I2c};

/// Number of non-volatile User EEPROM bytes (addresses 0x00 through 0x2A).
pub const USER_EEPROM_LEN: usize = 0x2B;
/// First Configuration EEPROM address; its RAM mirror has the same address.
pub const CONFIGURATION_EEPROM_START: u8 = 0x30;
/// Number of Configuration EEPROM bytes (addresses 0x30 through 0x37).
pub const CONFIGURATION_EEPROM_LEN: usize = 8;

// Control 1: EEPROM Memory Refresh Disable; Status: EEPROM busy.
const EERD: u8 = 0b0000_1000;
const EEBUSY: u8 = 0b1000_0000;
// EEPROM interface registers.
const EEADDR: u8 = 0x25;
const EEDATA: u8 = 0x26;
const EECMD: u8 = 0x27;
// A command must be preceded by 0x00 (Application Manual 4.6.3–4.6.6).
const UPDATE: u8 = 0x11;
const REFRESH: u8 = 0x12;
const WRITE: u8 = 0x21;
const READ: u8 = 0x22;
// Manual 4.6.7: minimum delay before checking EEbusy.
const READ_SETTLE_MS: u32 = 1;
const WRITE_SETTLE_MS: u32 = 10;
// Driver policy: at most 200 one-millisecond intervals per idle wait.
// This is a polling limit, not a wall-clock deadline including I2C transactions.
const BUSY_WAIT_MS: u32 = 200;

/// Cause of an EEPROM operation failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EepromErrorKind<E> {
  /// Original driver error, including an unmodified HAL bus error.
  Driver(Error<E>),
  /// EEbusy did not clear within 200 one-millisecond polling intervals.
  BusyTimeout,
  /// A byte read back from EEPROM did not match the requested value.
  VerificationFailed {
    /// Absolute EEPROM address.
    address: u8,
    /// Requested byte.
    expected: u8,
    /// Byte read back.
    actual: u8,
  },
}

/// EEPROM failure, including a possible failure to restore automatic refresh.
///
/// After a restore failure, the EERD state is unknown. After a timeout or bus
/// error, an EEPROM command may still be in progress. No automatic retry or
/// rollback is performed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EepromError<E> {
  /// Operation failed; restoration succeeded or no change was needed.
  Operation(EepromErrorKind<E>),
  /// Restoration failed, possibly in addition to the operation itself.
  Restore {
    /// Original failure, or None if the operation succeeded.
    operation: Option<EepromErrorKind<E>>,
    /// Failure while reading/modifying Control 1 to restore EERD.
    error: Error<E>,
  },
}

impl<E: fmt::Debug> fmt::Display for EepromError<E> {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::Operation(error) => write!(f, "EEPROM operation failed: {error:?}"),
      Self::Restore { operation, error } => write!(
        f,
        "EEPROM refresh restoration failed: {error}; operation: {operation:?}"
      ),
    }
  }
}
impl<E: fmt::Debug> core::error::Error for EepromError<E> {}

type Failure<E> = EepromErrorKind<E>;
type Result<T, E> = core::result::Result<T, EepromError<E>>;
type OperationResult<T, E> = core::result::Result<T, Failure<E>>;

impl<I2C: I2c> RV3028<I2C> {
  /// Read User EEPROM bytes, with an offset relative to address 0x00.
  ///
  /// The requested range must fit within 43 bytes (0x00–0x2A). Empty slices
  /// at offsets 0 through 43 perform no bus or delay calls. Validation occurs
  /// before selecting a mux. Bytes are read individually in ascending order;
  /// on failure, the successfully read prefix remains in the supplied buffer.
  ///
  /// Uses the command protocol documented in the
  /// [Application Manual](https://www.microcrystal.com/fileadmin/Media/Products/RTC/App.Manual/RV-3028-C7_App-Manual.pdf),
  /// sections 4.6.6–4.6.8. EERD is restored to its original state, including
  /// when an operation fails; restoration failures are reported separately.
  ///
  /// # Errors
  /// Returns an invalid-range, bus, busy-timeout or refresh-restoration error.
  pub fn read_user_eeprom<D: DelayNs>(
    &mut self,
    offset: u8,
    bytes: &mut [u8],
    delay: &mut D,
  ) -> Result<(), I2C::Error> {
    Self::eeprom_range(offset, bytes.len(), 0, USER_EEPROM_LEN)?;
    if bytes.is_empty() {
      return Ok(());
    }
    self.with_eeprom(delay, |rtc, delay| {
      for (index, byte) in bytes.iter_mut().enumerate() {
        *byte = rtc.eeprom_read_raw(offset + index as u8, delay)?;
      }
      Ok(())
    })
  }

  /// Write User EEPROM bytes only where stored values differ, then verify
  /// every requested byte with a separate readback pass.
  ///
  /// Offset and empty-slice rules match [Self::read_user_eeprom]. Writes occur
  /// in ascending address order. This is not an atomic record update: a failed
  /// operation can leave a partially written range. Record layout, validity
  /// markers and commit ordering belong to the caller.
  ///
  /// VDD must remain at least 1.5 V throughout programming, including until
  /// EEbusy clears; the chip must use VDD rather than backup power. Unlock
  /// password-protected writes first. Neither condition can be guaranteed by
  /// this driver. The caller supplies an embedded-hal delay implementation.
  ///
  /// ```no_run
  /// use embedded_hal::{delay::DelayNs, i2c::I2c};
  /// use rv3028c7_rtc::{EepromError, RV3028};
  ///
  /// fn store<I: I2c, D: DelayNs>(
  ///     rtc: &mut RV3028<I>, delay: &mut D,
  /// ) -> Result<(), EepromError<I::Error>> {
  ///     let mut bytes = [0; 2];
  ///     rtc.read_user_eeprom(0, &mut bytes, delay)?;
  ///     bytes[1] = 0xA5;
  ///     rtc.write_user_eeprom(1, &bytes[1..], delay)
  /// }
  /// ```
  ///
  /// # Errors
  /// Returns an invalid-range, bus, busy-timeout, verification or restoration
  /// error. No writes are retried or rolled back.
  pub fn write_user_eeprom<D: DelayNs>(
    &mut self,
    offset: u8,
    bytes: &[u8],
    delay: &mut D,
  ) -> Result<(), I2C::Error> {
    Self::eeprom_range(offset, bytes.len(), 0, USER_EEPROM_LEN)?;
    if bytes.is_empty() {
      return Ok(());
    }
    self.with_eeprom(delay, |rtc, delay| {
      for (index, &byte) in bytes.iter().enumerate() {
        let address = offset + index as u8;
        if rtc.eeprom_read_raw(address, delay)? != byte {
          rtc.eeprom_write_raw(address, byte, delay)?;
        }
      }
      rtc.eeprom_verify_raw(offset, bytes, delay)
    })
  }

  /// Read one byte at an absolute EEPROM address, not an I2C register address.
  ///
  /// Accepts User EEPROM 0x00–0x2A and Configuration EEPROM 0x30–0x37.
  /// Reserved addresses are rejected before I/O. Configuration reads access
  /// persistent storage, not the active RAM mirrors. Password EEPROM reads
  /// require the device to be unlocked.
  ///
  /// # Errors
  /// Returns an invalid-address, bus, busy-timeout or restoration error.
  pub fn read_eeprom<D: DelayNs>(&mut self, address: u8, delay: &mut D) -> Result<u8, I2C::Error> {
    Self::eeprom_address(address)?;
    self.with_eeprom(delay, |rtc, delay| rtc.eeprom_read_raw(address, delay))
  }

  /// Write and verify one byte at an absolute EEPROM address.
  ///
  /// Accepts the same addresses as [Self::read_eeprom]. Does not write an
  /// unchanged byte. A Configuration EEPROM write does not modify its active
  /// RAM mirror: use [Self::refresh_configuration] to activate it, or use
  /// [Self::write_configuration_eeprom] to edit and persist mirrors together.
  /// Supply, password and partial-write conditions match [Self::write_user_eeprom].
  ///
  /// # Errors
  /// Returns an invalid-address, bus, busy-timeout, verification or restoration error.
  pub fn write_eeprom<D: DelayNs>(
    &mut self,
    address: u8,
    value: u8,
    delay: &mut D,
  ) -> Result<(), I2C::Error> {
    Self::eeprom_address(address)?;
    self.with_eeprom(delay, |rtc, delay| {
      if rtc.eeprom_read_raw(address, delay)? != value {
        rtc.eeprom_write_raw(address, value, delay)?;
      }
      rtc.eeprom_verify_raw(address, &[value], delay)
    })
  }

  /// Edit Configuration RAM mirrors and persist all eight configuration bytes.
  ///
  /// `address` is an absolute mirror address in 0x30–0x37; the whole supplied
  /// slice must fit in that region. Empty slices at 0x30–0x38 are no-ops.
  /// EERD is disabled before editing mirrors. Command 0x11 then copies ALL
  /// mirrors 0x30–0x37, including password settings, into EEPROM. Unspecified
  /// bytes retain their current mirror values, which may differ from EEPROM.
  ///
  /// The active configuration changes when the mirrors are written, before
  /// persistence completes. This method always issues an update for a nonempty
  /// request; it verifies only the supplied bytes in persistent EEPROM.
  /// It does not choose backup, charger or CLKOUT settings for an application.
  ///
  /// Password changes can alter read/write protection and make verification
  /// fail even after programming succeeds; manage unlocking explicitly.
  /// Supply and partial-write conditions match [Self::write_user_eeprom].
  ///
  /// # Errors
  /// Returns an invalid-range, bus, busy-timeout, verification or restoration error.
  pub fn write_configuration_eeprom<D: DelayNs>(
    &mut self,
    address: u8,
    bytes: &[u8],
    delay: &mut D,
  ) -> Result<(), I2C::Error> {
    Self::eeprom_range(
      address,
      bytes.len(),
      CONFIGURATION_EEPROM_START,
      CONFIGURATION_EEPROM_LEN,
    )?;
    if bytes.is_empty() {
      return Ok(());
    }
    self.with_eeprom(delay, |rtc, delay| {
      rtc.eeprom_idle(delay)?;
      for (index, &byte) in bytes.iter().enumerate() {
        rtc
          .write_register_raw(address + index as u8, byte)
          .map_err(Failure::Driver)?;
      }
      rtc.eeprom_command(UPDATE, WRITE_SETTLE_MS, delay)?;
      rtc.eeprom_verify_raw(address, bytes, delay)
    })
  }

  /// Persist all Configuration RAM mirrors (0x30–0x37) with command 0x11.
  ///
  /// This stores all eight bytes, including password settings. It does not
  /// compare values or verify readback. Prefer [Self::write_configuration_eeprom]
  /// when editing mirrors: it disables auto-refresh before the edits.
  /// Supply and password conditions match [Self::write_user_eeprom].
  ///
  /// # Errors
  /// Returns a bus, busy-timeout or restoration error.
  pub fn update_configuration_eeprom<D: DelayNs>(
    &mut self,
    delay: &mut D,
  ) -> Result<(), I2C::Error> {
    self.with_eeprom(delay, |rtc, delay| {
      rtc.eeprom_command(UPDATE, WRITE_SETTLE_MS, delay)
    })
  }

  /// Reload all Configuration RAM mirrors from EEPROM with command 0x12.
  ///
  /// This replaces active settings, including unpersisted changes. The chip
  /// applies the settings as the mirrors are loaded. EERD's original state
  /// is restored afterwards; no readback verification is performed.
  ///
  /// # Errors
  /// Returns a bus, busy-timeout or restoration error.
  pub fn refresh_configuration<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), I2C::Error> {
    self.with_eeprom(delay, |rtc, delay| {
      rtc.eeprom_command(REFRESH, READ_SETTLE_MS, delay)
    })
  }

  fn eeprom_range(address: u8, len: usize, start: u8, size: usize) -> Result<(), I2C::Error> {
    let offset = address.checked_sub(start).map(usize::from);
    if !offset.is_some_and(|offset| offset <= size && len <= size - offset) {
      return Err(EepromError::Operation(Failure::Driver(Error::InvalidData)));
    }
    Ok(())
  }

  fn eeprom_address(address: u8) -> Result<(), I2C::Error> {
    if usize::from(address) < USER_EEPROM_LEN || (0x30..=0x37).contains(&address) {
      Ok(())
    } else {
      Err(EepromError::Operation(Failure::Driver(Error::InvalidData)))
    }
  }

  fn with_eeprom<D: DelayNs, T>(
    &mut self,
    delay: &mut D,
    body: impl FnOnce(&mut Self, &mut D) -> OperationResult<T, I2C::Error>,
  ) -> Result<T, I2C::Error> {
    self
      .select_mux_channel()
      .map_err(|e| EepromError::Operation(Failure::Driver(e)))?;
    let original = self
      .read_register_raw(REG_CONTROL1)
      .map_err(|e| EepromError::Operation(Failure::Driver(e)))?;
    // Even a failed I2C write may have reached the chip; restore afterwards.
    let result = if original & EERD == 0 {
      self
        .write_register_raw(REG_CONTROL1, original | EERD)
        .map_err(Failure::Driver)
        .and_then(|()| body(self, delay))
    } else {
      body(self, delay)
    };
    let restored = if original & EERD == 0 {
      // Preserve other Control 1 bits, including timer changes during waits.
      self
        .read_register_raw(REG_CONTROL1)
        .and_then(|current| self.write_register_raw(REG_CONTROL1, current & !EERD))
    } else {
      Ok(())
    };
    match (result, restored) {
      (Ok(value), Ok(())) => Ok(value),
      (Err(error), Ok(())) => Err(EepromError::Operation(error)),
      (result, Err(error)) => Err(EepromError::Restore {
        operation: result.err(),
        error,
      }),
    }
  }

  fn eeprom_idle<D: DelayNs>(&mut self, delay: &mut D) -> OperationResult<(), I2C::Error> {
    for elapsed in 0..=BUSY_WAIT_MS {
      if self
        .read_register_raw(REG_STATUS)
        .map_err(Failure::Driver)?
        & EEBUSY
        == 0
      {
        return Ok(());
      }
      if elapsed < BUSY_WAIT_MS {
        delay.delay_ms(1);
      }
    }
    Err(Failure::BusyTimeout)
  }

  fn eeprom_command<D: DelayNs>(
    &mut self,
    command: u8,
    settle_ms: u32,
    delay: &mut D,
  ) -> OperationResult<(), I2C::Error> {
    self.eeprom_idle(delay)?;
    self
      .write_register_raw(EECMD, 0x00)
      .map_err(Failure::Driver)?;
    // A failed bus transaction may still have started the command.
    let started = self.write_register_raw(EECMD, command);
    delay.delay_ms(settle_ms);
    started.map_err(Failure::Driver)?;
    self.eeprom_idle(delay)
  }

  fn eeprom_read_raw<D: DelayNs>(
    &mut self,
    address: u8,
    delay: &mut D,
  ) -> OperationResult<u8, I2C::Error> {
    // Address/data registers must not be changed while a command is busy.
    self.eeprom_idle(delay)?;
    self
      .write_register_raw(EEADDR, address)
      .map_err(Failure::Driver)?;
    self.eeprom_command(READ, READ_SETTLE_MS, delay)?;
    self.read_register_raw(EEDATA).map_err(Failure::Driver)
  }

  fn eeprom_write_raw<D: DelayNs>(
    &mut self,
    address: u8,
    value: u8,
    delay: &mut D,
  ) -> OperationResult<(), I2C::Error> {
    self.eeprom_idle(delay)?;
    self
      .write_register_raw(EEADDR, address)
      .map_err(Failure::Driver)?;
    self
      .write_register_raw(EEDATA, value)
      .map_err(Failure::Driver)?;
    self.eeprom_command(WRITE, WRITE_SETTLE_MS, delay)
  }

  fn eeprom_verify_raw<D: DelayNs>(
    &mut self,
    address: u8,
    bytes: &[u8],
    delay: &mut D,
  ) -> OperationResult<(), I2C::Error> {
    for (index, &expected) in bytes.iter().enumerate() {
      let address = address + index as u8;
      let actual = self.eeprom_read_raw(address, delay)?;
      if actual != expected {
        return Err(Failure::VerificationFailed {
          address,
          expected,
          actual,
        });
      }
    }
    Ok(())
  }
}

#[cfg(test)]
#[path = "eeprom_tests.rs"]
mod tests;
