//! Read-only technical status and raw register access.
use crate::{Error, RV3028};
use embedded_hal::i2c::I2c;

/// Raw register address space size (0x00–0x3F), including reserved addresses.
pub const REGISTER_COUNT: usize = 0x40;

/// Status register flags, including read-only EEPROM busy.
/// Reading or decoding these flags never acknowledges them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusFlags(u8);

impl StatusFlags {
  /// Decode all eight bits of register 0x0E without bus access.
  pub const fn from_register(register: u8) -> Self {
    Self(register)
  }
  /// Original register byte, including EEbusy.
  pub const fn bits(self) -> u8 {
    self.0
  }
  /// PORF: power-on reset has occurred; calendar trust belongs to the owner.
  pub const fn power_on_reset(self) -> bool {
    self.0 & 0b0000_0001 != 0
  }
  /// EVF: external event or timestamp-related backup event is latched.
  pub const fn external_event(self) -> bool {
    self.0 & 0b0000_0010 != 0
  }
  /// AF: alarm event is latched.
  pub const fn alarm(self) -> bool {
    self.0 & 0b0000_0100 != 0
  }
  /// TF: periodic countdown timer event is latched.
  pub const fn countdown_timer(self) -> bool {
    self.0 & 0b0000_1000 != 0
  }
  /// UF: periodic time-update event is latched.
  pub const fn time_update(self) -> bool {
    self.0 & 0b0001_0000 != 0
  }
  /// BSF: backup switchover event is latched.
  pub const fn backup_switchover(self) -> bool {
    self.0 & 0b0010_0000 != 0
  }
  /// CLKF: interrupt-controlled clock output flag is latched.
  pub const fn clock_output(self) -> bool {
    self.0 & 0b0100_0000 != 0
  }
  /// EEbusy: an EEPROM operation is in progress (read-only).
  pub const fn eeprom_busy(self) -> bool {
    self.0 & 0b1000_0000 != 0
  }
}

/// Technical registers 0x0E–0x13 read together in one transfer.
///
/// This exposes actual register values without interpreting application policy.
/// A burst does not make asynchronously changing registers an atomic snapshot.
/// Later events or register changes are not represented; the separate calendar,
/// Unix counter and configuration mirrors are not part of this snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusSnapshot {
  /// Status (0x0E); no flags are acknowledged by the reader.
  pub flags: StatusFlags,
  /// Control 1 (0x0F), including timer control and EERD.
  pub control1: u8,
  /// Control 2 (0x10), including TSE, hour mode and interrupt enables.
  pub control2: u8,
  /// General-purpose bits (0x11).
  pub gp_bits: u8,
  /// Clock interrupt mask (0x12), selecting CLKOUT interrupt sources.
  pub clock_interrupt_mask: u8,
  /// Event Control (0x13), including EHL, ET, TSOW and TSS; TSR reads zero.
  pub event_control: u8,
}

impl<I: I2c> RV3028<I> {
  /// Read one raw RAM register from 0x00–0x3F.
  ///
  /// Reserved addresses are intentionally accepted for diagnostics; their
  /// returned values are unspecified. Write-only registers may read as zero.
  /// Configuration addresses read active RAM mirrors, not persistent EEPROM.
  /// No writes, status acknowledgements or EEPROM commands are issued.
  ///
  /// # Errors
  /// [`Error::InvalidData`] outside the address space, before bus/mux access.
  /// [`Error::I2c`] for bus or mux failures.
  pub fn read_register(&mut self, register: u8) -> Result<u8, Error<I::Error>> {
    let mut byte = [0];
    self.read_registers(register, &mut byte)?;
    Ok(byte[0])
  }

  /// Read consecutive raw RAM registers into a borrowed slice in one burst.
  ///
  /// The entire range must fit in 0x00–0x3F; no address wrapping is allowed.
  /// Empty slices at starts 0x00–0x40 perform no I/O, including mux selection.
  /// Reserved/write-only/configuration address semantics match
  /// [Self::read_register]. Reads never acknowledge flags or command EEPROM.
  /// Calendar and timestamp blocking applies when the relevant registers are
  /// accessed; HAL transaction deadlines remain the caller's responsibility.
  /// On a HAL failure the buffer may contain a partially received prefix.
  ///
  /// # Errors
  /// [`Error::InvalidData`] for an invalid range, before I/O or buffer changes.
  /// [`Error::I2c`] for bus or mux failures. No retries are performed.
  pub fn read_registers(&mut self, register: u8, bytes: &mut [u8]) -> Result<(), Error<I::Error>> {
    let start = usize::from(register);
    if start > REGISTER_COUNT || bytes.len() > REGISTER_COUNT - start {
      return Err(Error::InvalidData);
    }
    if bytes.is_empty() {
      return Ok(());
    }
    self.select_mux_channel()?;
    self.read_multi_registers_raw(register, bytes)
  }

  /// Read Status, Control 1/2, GP Bits, Clock Interrupt Mask and Event Control
  /// in one burst. This is read-only, including when event flags are set.
  ///
  /// # Errors
  /// [`Error::I2c`] for bus or mux failures. No retries are performed.
  pub fn get_status(&mut self) -> Result<StatusSnapshot, Error<I::Error>> {
    let mut bytes = [0; 6];
    self.read_registers(0x0E, &mut bytes)?;
    Ok(StatusSnapshot {
      flags: StatusFlags::from_register(bytes[0]),
      control1: bytes[1],
      control2: bytes[2],
      gp_bits: bytes[3],
      clock_interrupt_mask: bytes[4],
      event_control: bytes[5],
    })
  }
}
