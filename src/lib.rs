//! `no_std` driver for the Micro Crystal RV-3028-C7 real time clock (RTC),
//! for any I2C bus implementing the [`embedded_hal`] 1.0 [`I2c`] trait.
//!
//! The driver is [`RV3028`]. Dates and times can be handled with one of two optional
//! date/time libraries, or without any:
//!
//! | Feature | Default | Provides |
//! |---|---|---|
//! | `chrono` | yes | `DateTimeAccess` (from `rtcc`) and `ChronoAccess`, using `chrono::NaiveDateTime` |
//! | `jiff` | no | `JiffDateTimeAccess`, using `jiff::civil::DateTime` |
//! | (none) | | [`DateTimeParts`] and [`AlarmParts`], with [`RV3028::set_datetime_parts`] and friends |
//!
//! Both features can be enabled at the same time. Everything that doesn't involve a date or time,
//! such as the countdown timer, interrupts, trickle charging and the unix time counter,
//! is always available.
//!
//! # Remaining API coverage
//!
//! The following chip functions do not yet have dedicated driver APIs. Raw register
//! and EEPROM access is available, but does not implement their field validation,
//! read-modify-write rules or complete operating sequences:
//!
//! - **Frequency offset correction (EEOffset):** no signed 9-bit correction type,
//!   getter/setter or conversion between correction steps and ppm. A future setter
//!   must merge registers 0x36 and 0x37 without changing Backup settings. Existing
//!   Backup setters preserve the shared calibration bit; EEPROM persistence is
//!   already available separately. Preserve the factory calibration unless an
//!   explicit replacement has been determined.
//! - **Password/write protection:** no dedicated unlock, password-change or
//!   protection-enable/disable sequence. Generic EEPROM methods do not unlock
//!   protected accesses automatically.
//! - **Explicit prescaler RESET command:** no dedicated method for Control 2 RESET
//!   and its documented timing effects. Calendar setters already reset the
//!   prescaler by writing Seconds; timestamp-log reset is implemented separately.
//! - **Combined interrupt/CLKOUT routing configuration:** no typed configuration
//!   spanning Control 2 interrupt enables and Clock Interrupt Mask. Individual
//!   interrupt toggles, CLKIE toggle and mask clearing exist, and technical readout
//!   exposes the raw control/mask bytes.
//!
//! Additional API gaps concern functions that are already implemented: EVI filtering
//! uses a validated raw two-bit ET argument rather than a typed enum, and countdown
//! configuration accepts a duration rather than an explicit clock/tick preset.
//! There are no dedicated configuration getters for those two functions.
//!
//! Hour-mode switching, 12-hour alarm encoding, timestamp logging, Backup/CLKOUT
//! configuration, calendar access and User RAM/EEPROM access are implemented.
//! Reference: Application Manual rev. 1.4, especially sections 4.15, 4.17 and 4.18.
//! Drift measurement, calibration policy and board-specific defaults belong to the
//! owner, rather than being missing hardware-driver features.
//!
//! # Example
//!
//! ```
//! use embedded_hal::i2c::I2c;
//! use rv3028c7_rtc::{DateTimeParts, Error, RV3028};
//!
//! fn set_clock<I2C: I2c>(i2c: I2C) -> Result<u32, Error<I2C::Error>> {
//!     let mut rtc = RV3028::new(i2c);
//!     rtc.set_datetime_parts(&DateTimeParts {
//!         year: 2024, month: 3, day: 15, hour: 13, minute: 45, second: 30,
//!     })?;
//!     rtc.get_unix_time()
//! }
//! ```
//!
//! With the `chrono` feature, the same through `DateTimeAccess`:
//!
//! ```
//! # #[cfg(feature = "chrono")] {
//! use embedded_hal::i2c::I2c;
//! use rv3028c7_rtc::{DateTimeAccess, Error, NaiveDateTime, RV3028};
//!
//! fn read_clock<I2C: I2c>(i2c: I2C) -> Result<NaiveDateTime, Error<I2C::Error>> {
//!     let mut rtc = RV3028::new(i2c);
//!     rtc.datetime()
//! }
//! # }
//! ```

#![cfg_attr(not(test), no_std)]


#[cfg(feature = "chrono")]
mod chrono_impl;
#[cfg(feature = "chrono")]
pub use chrono::{Datelike, NaiveDate, NaiveDateTime, NaiveTime, Timelike, Weekday};
#[cfg(feature = "chrono")]
pub use rtcc::{  DateTimeAccess };
#[cfg(feature = "chrono")]
pub use chrono_impl::ChronoAccess;
#[cfg(feature = "jiff")]
mod jiff_impl;
#[cfg(feature = "jiff")]
pub use jiff;
#[cfg(feature = "jiff")]
pub use jiff_impl::JiffDateTimeAccess;

mod hour_mode;
pub use hour_mode::HourMode;
#[cfg(test)]
mod hour_mode_tests;
#[cfg(test)]
mod event_timer_tests;

mod registers;
pub use registers::{REGISTER_COUNT, StatusFlags, StatusSnapshot};
#[cfg(test)]
mod register_tests;

mod timestamp;
pub use timestamp::{TimestampCapture, TimestampConfig, TimestampSnapshot, TimestampSource};
#[cfg(test)]
mod timestamp_tests;

mod configuration;
#[cfg(test)]
mod configuration_tests;
pub use configuration::{BackupConfig, BackupSwitchoverMode, ClkoutConfig, ClkoutSource};

mod eeprom;
pub use eeprom::{
  EepromError, EepromErrorKind, USER_EEPROM_LEN,
  CONFIGURATION_EEPROM_START, CONFIGURATION_EEPROM_LEN,
};

use core::time::Duration;
use embedded_hal::i2c::I2c;

/// Errors reported by the RTC driver. No allocation is required.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
  /// The underlying I2C transaction failed.
  I2c(E),
  /// An input/register value is invalid, or a memory access exceeds its range.
  InvalidData,
  /// No two consecutive Unix counter reads agreed within four reads.
  InconsistentRead,
}

impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
  fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
    match self {
      Self::I2c(error) => write!(f, "I2C error: {error:?}"),
      Self::InvalidData => f.write_str("invalid RTC data"),
      Self::InconsistentRead => f.write_str("inconsistent RTC counter reads"),
    }
  }
}

impl<E: core::fmt::Debug> core::error::Error for Error<E> {}

// Fixed i2c bus address of the device (7-bit)
const RV3028_ADDRESS: u8 = 0xA4 >> 1;

// Main time register addresses
const REG_SECONDS: u8 = 0x00;
const REG_MINUTES: u8 = 0x01;
const REG_HOURS: u8 = 0x02;


// Holds the current day of the week.
// Each value represents one weekday that is assigned by the user.
// Values will range from 0 to 6.
// The weekday counter is simply a 3-bit counter which counts up to 6 and then resets to 0.
const REG_WEEKDAY: u8 = 0x03;

// Holds the current day of the month, in two binary coded decimal (BCD) digits.
// Values will range from 01 to 31.
// Leap years are correctly handled from 2000 to 2099.
const REG_DATE: u8 = 0x04;

// Holds the current month, in two binary coded decimal (BCD) digits.
// Values will range from 01 to 12.
const REG_MONTH: u8 = 0x05;
const REG_YEAR: u8 = 0x06;

// Holds the Minutes Alarm Enable bit AE_M,
// and the alarm value for minutes,
// in two binary coded decimal (BCD) digits.
// Values will range from 00 to 59.
const REG_MINUTES_ALARM: u8 = 0x07;

// Holds the Hours Alarm Enable bit AE_H and the alarm value for hours,
// in two binary coded decimal (BCD) digits.
// - If the 12_24 bit is cleared (default value) (see Control 2 register),
// the values will range from 0 to 23.
// - If the 12_24 bit is set, the hour values will be from 1 to 12
// and the AMPM bit will be 0 for AM hours and 1 for PM hours.
// - If the 12_24 hour mode bit is changed then the value in the Hours Alarm register must be re-initialized.
const REG_HOURS_ALARM: u8 = 0x08;

// Holds the Weekday/Date Alarm (WADA) Enable bit AE_WD.
// - If the WADA bit is 0 (Bit 5 in Register REG_CONTROL1),
// it holds the alarm value for the weekday (weekdays assigned by the user),
// in two binary coded decimal (BCD) digits.
// Values will range from 0 to 6.
// - If the WADA bit is 1, it holds the alarm value for the date, in two binary coded decimal (BCD)
// digits. Values will range from 01 to 31.
const REG_WEEKDAY_DATE_ALARM: u8 = 0x09;

// This register is used to set the lower 8 bits of the 12 bit Timer Value (preset value)
// for the Periodic Countdown Timer.
// This value will be automatically reloaded into the Countdown Timer when it reaches zero
// If the TRPT bit is 1, this value will be automatically reloaded into the Countdown Timer
// when it reaches zero: this allows for periodic timer interrupts
const REG_TIMER_VALUE0: u8 = 0x0A;

// This register is used to set the upper 4 bits of the 12 bit Timer Value (preset value)
// for the Periodic Countdown Timer.
// If the TRPT bit is 1, this value will be automatically reloaded into the Countdown Timer
// when it reaches zero: this allows for periodic timer interrupts
// const REG_TIMER_VALUE1: u8 = 0x0B;

const REG_TIMER_STATUS0: u8 = 0x0C; // Read-only lower 8 bits of Periodic Countdown Timer
// const REG_TIMER_STATUS1: u8 = 0x0D; // Read-only upper 4 bits of Periodic Countdown Timer


// This register is used to detect the occurrence of various interrupt events
// and reliability problems in internal data.
const REG_STATUS: u8 = 0x0E;

// This register is used to configure
// - the Alarm Interrupt function
// - the Periodic Time Update Interrupt function
// - and to select or set operations for the Periodic Countdown Timer.
const REG_CONTROL1:u8  = 0x0F;

// This register is used to control:
// - interrupt event output for the INT̅ pin
// - stop/start status of clock and calendar operations
// - interrupt controlled clock output on CLKOUT pin
// - hour mode and time stamp enable
const REG_CONTROL2:u8 = 0x10;

// Clock Interrupt Mask
// This register is used to select a predefined interrupt for automatic clock output.
// Setting a bit to 1 selects the corresponding interrupt.
// Multiple interrupts can be selected.
// After power on, no interrupt is selected (see CLOCK OUTPUT SCHEME).
const REG_CLOCK_INTERRUPT_MASK:u8 = 0x12;

// Event Control register: EHL, ET,TSR, TSOW, TSS
const REG_EVENT_CONTROL: u8 = 0x13;

// Time Stamp function registers (Event Logging)
const REG_COUNT_EVENTS_TS: u8 = 0x14; // Count TS
// const REG_SECONDS_TS: u8 = 0x15; // Seconds TS
// const REG_MINUTES_TS: u8 = 0x16; // Minutes TS
// const REG_HOURS_TS: u8 = 0x17; // Hours TS
// const REG_DATE_TS: u8 = 0x18; // Date TS
// const REG_MONTH_TS: u8 = 0x19; // Month TS
// const REG_YEAR_TS: u8 = 0x1A; // Month TS


/// Number of general-purpose User RAM bytes at registers 0x1F and 0x20.
/// Alarm/timer registers reused by an application are not included.
pub const USER_RAM_LEN: usize = 2;
// First register of the two-byte User RAM region; distinct from an offset.
const REG_USER_RAM_START: u8 = 0x1F;

// First address of "Unix Time Counter"
const REG_UNIX_TIME_0: u8 = 0x1B;
// const REG_UNIX_TIME_1: u8 = 0x1C;
// const REG_UNIX_TIME_2: u8 = 0x1D;
// const REG_UNIX_TIME_3: u8 = 0x1E;

// REG_CONTROL1 "Control 1" register bits:
#[repr(u8)]
enum RegControl1Bits {
  // TRPT / Timer Repeat bit. Single or Repeat countdown timer
  TimerRepeatBit =  1 << 7,
  // WADA / Weekday Alarm / Date Alarm selection bit
  WadaBit = 1 << 5,
  //  USEL / Update Interrupt Select bit. Seconds or minutes.
  UselBit = 1 << 4,
  //  EERD_BIT: u8 = 1 << 3,
  // TE / Periodic Countdown Timer Enable bit.
  TimerEnableBit = 1 << 2,
  // TD / Timer Clock Frequency selection bits
  TimerClockFreqBits = 0b11,
}

/// Countown timer clock frequency selector
#[derive(Clone, Copy, Debug, PartialEq)]
enum TimerClockFreq {
  Hertz4096 = 0b00, // 4096 Hz, 244.14 μs period
  Hertz64 = 0b01, // 64 Hz, 15.625 ms period
  Hertz1 = 0b10, // 1 Hz, One second period
  HertzSixtieth = 0b11, // 1/60 Hz, One minute period
}

// Status bits 6–0 are write-zero-to-clear; EEbusy (bit 7) is read-only.
// Writing ones preserves unrelated flags, including events arriving after a read.
const STATUS_WRITABLE_FLAGS: u8 = 0b0111_1111;

// REG_STATUS Status register bits:
#[repr(u8)]
enum RegStatusBits {
  //EEBUSY_BIT = 1 << 7,
  // CLKF  / Clock Output Interrupt Flag
  ClockIntFlagBit = 1 << 6,
  // BSF bit
  BackupSwitchFlag = 1 << 5,
  // UF / Periodic Time Update Flag
  TimeUpdateFlag = 1 << 4,
  // TF bit / Periodic Countdown Timer Flag
  PeriodicTimerFlag = 1 << 3,
  // AF / Alarm Flag
  AlarmFlagBit = 1 << 2,
  // EVF / Event Flag (external event interrupt)
  EventFlagBit = 1 << 1,
  // PORF / Power On Reset Flag
  PowerOnResetFlagBit = 1 << 0,
}

// EEPROM register addresses and commands
const EEPROM_MIRROR_ADDRESS: u8 = 0x37;// RAM mirror of EEPROM config values
// const EEPROM_CMD_READ: u8 = 0x00;
// const EEPROM_CMD_WRITE: u8 = 0x01;


// REG_EVENT_CONTROL Event Control register bits:   EHL, ET, TSR, TSOW, TSS
#[repr(u8)]
enum RegEventControlBits {
  // EHL bit / Event High/Low Level (Rising/Falling Edge) selection for detection
  EventHighLowBit = 1 << 6,
  // ET bits / Event Filtering Time
  EventFilteringTimeBits = 0b11 << 4,
  // TSOW bit
  TimeStampOverwriteBit = 1 << 1,
}

/// Event log source is external interrupt EVI (default)
pub const TS_EVENT_SOURCE_EVI: u8 = 0;
/// Event log source is backup power switchover
pub const TS_EVENT_SOURCE_BSF: u8 = 1;

// REG_CLOCK_INTERRUPT_MASK bits
#[repr(u8)]
enum RegClockIntMaskBits {
  // CEIE / Clock output on Event Interrupt bit
  ClockoutOnExtEvtBit = 1 << 3,
  // CAIE / Clock output on Alarm Interrupt bit
  ClockoutOnAlarmBit = 1 << 2,
  // CTIE / Clock output on Periodic Countdown Timer Interrupt bit
  ClockoutOnPctBit = 1 << 1,
  // CUIE / Clock output on Periodic Time Update Interrupt bit
  ClockoutOnUpdateBit = 1 << 0,
}

// REG_CONTROL2 "Control 2" register bits: TSE CLKIE UIE TIE AIE EIE 12_24 RESET
#[repr(u8)]
enum RegControl2Bits {
  // TSE / Time Stamp Enable bit
  TimeStampEnableBit = 1 << 7,
  // CLKIE / Clock Output enabled by Interrupt source. (see also CLKOE)
  ClockoutIntEnableBit = 1 << 6,
  // UIE / Time Update Interrupt Enable
  TimeUpdateIntEnableBit = 1 << 5,
  // TIE Countdown Timer Interrupt Enable bit
  TimerIntEnableBit = 1 << 4,
  // AIE / Alarm Interrupt Enable bit
  AlarmIntEnableBit = 1 << 3,
  // EIE / Event Interrupt Enable bit
  EventIntEnableBit = 1 << 2,
}

// EEPROM_MIRROR_ADDRESS / EEPROM mirror register bits:
#[repr(u8)]
enum RegEepromMirrorBits {
  // BCIE / Backup Switchover Interrupt Enable bit bit
  BackupSwitchIntEnableBit = 1 << 6,
  // TCE bit
  TrickleChargeEnableBit = 1 << 5,
  TrickleChargeResistanceBits = 0b11, // TCR bits
}

/// Current limiting resistor for trickle charging the backup supply,
/// see [`RV3028::toggle_trickle_charge`]. A higher resistance means less charging current.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TrickleChargeCurrentLimiter {
  /// 3 kΩ (factory default)
  Ohms3k = 0b00,
  /// 5 kΩ
  Ohms5k = 0b01,
  /// 9 kΩ
  Ohms9k = 0b10,
  /// 15 kΩ
  Ohms15k = 0b11,
}

// Special alarm register value
const ALARM_NO_WATCH_FLAG: u8 = 1 <<  7;

/// Calendar date and time as stored in the RTC's date and time registers,
/// independent of any date/time library.
///
/// The RTC's date registers only cover the years 2000 to 2099.
/// Setters reject invalid dates and times before accessing the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DateTimeParts {
  /// Full year, 2000 to 2099
  pub year: i32,
  /// Month, 1 to 12
  pub month: u8,
  /// Day of month, 1 to 31
  pub day: u8,
  /// Hour, 0 to 23
  pub hour: u8,
  /// Minute, 0 to 59
  pub minute: u8,
  /// Second, 0 to 59
  pub second: u8,
}

impl DateTimeParts {
  fn validate<E>(&self) -> Result<(), Error<E>> {
    let days = match self.month {
      1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
      4 | 6 | 9 | 11 => 30,
      2 if self.year % 4 == 0 => 29,
      2 => 28,
      _ => return Err(Error::InvalidData),
    };
    if !(2000..=2099).contains(&self.year) || self.day == 0 || self.day > days
      || self.hour > 23 || self.minute > 59 || self.second > 59 {
      return Err(Error::InvalidData);
    }
    Ok(())
  }

  // Days since 1970-01-01 in the proleptic Gregorian calendar
  // (Howard Hinnant's `days_from_civil` algorithm)
  fn days_since_unix_epoch(&self) -> i64 {
    let month = self.month as i64;
    let year = self.year as i64 - if month <= 2 { 1 } else { 0 };
    let era = if year >= 0 { year } else { year - 399 } / 400;
    let year_of_era = year - era * 400;
    let day_of_year = (153 * ((month + 9) % 12) + 2) / 5 + self.day as i64 - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
  }

  // Seconds since the unix epoch, as used by the RTC's unix time counter
  fn unix_timestamp(&self) -> i64 {
    self.days_since_unix_epoch() * 86_400
      + self.hour as i64 * 3_600 + self.minute as i64 * 60 + self.second as i64
  }

  // Weekday as stored by this driver: 0 = Monday through 6 = Sunday
  fn weekday(&self) -> u8 {
    // 1970-01-01 was a Thursday
    (self.days_since_unix_epoch() + 3).rem_euclid(7) as u8
  }
}

/// Alarm settings as stored in the RTC's alarm registers,
/// independent of any date/time library.
///
/// The alarm triggers when all matched fields equal the current time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AlarmParts {
  /// Day of month (1 to 31) for a date alarm,
  /// or weekday (0 = Monday through 6 = Sunday) for a weekday alarm
  pub day: u8,
  /// Hour, 0 to 23
  pub hour: u8,
  /// Minute, 0 to 59
  pub minute: u8,
  /// `true` for a weekday alarm, `false` for a date alarm
  pub weekday_alarm: bool,
  /// Whether the day (or weekday) must match
  pub match_day: bool,
  /// Whether the hour must match
  pub match_hour: bool,
  /// Whether the minute must match
  pub match_minute: bool,
}

impl AlarmParts {
  fn validate<E>(&self) -> Result<(), Error<E>> {
    if self.hour > 23 || self.minute > 59
      || (self.weekday_alarm && self.day > 6)
      || (!self.weekday_alarm && !(1..=31).contains(&self.day)) {
      return Err(Error::InvalidData);
    }
    Ok(())
  }
}


/// RV-3028-C7
/// Extreme Low Power Real-Time Clock (RTC) Module with I2C-Bus Interface
/// rust no_std driver (utilizes the embedded_hal i2c interface)
pub struct RV3028<I2C> {
  i2c: I2C,
  mux_addr: u8,
  mux_chan: u8,
}

impl<I2C> RV3028<I2C>
  where
    I2C: I2c,
{

  /// New driver instance, assumes that there is no i2c mux
  /// sitting between the RTC and the host.
  pub fn new(i2c: I2C) -> Self {
    RV3028 {
      i2c,
      mux_addr: 0u8,
      mux_chan: 0u8
    }
  }

  /// Allows the caller to create a new driver instance with
  /// an i2c mux between the RTC and the host.
  /// - `mux_addr` : the i2c address of the mux itself
  /// - `mux_chan` : the mux channel assigned to the RTC
  pub fn new_with_mux(i2c: I2C, mux_addr: u8, mux_chan: u8) -> Self {
    RV3028 {
      i2c,
      mux_addr,
      mux_chan
    }
  }

  // Converts a binary value to BCD format
  fn checked_bcd(value: u8) -> Result<u8, Error<I2C::Error>> {
    if value & 0x0F > 9 || value >> 4 > 9 {
      return Err(Error::InvalidData);
    }
    Ok(Self::bcd_to_bin(value))
  }

  fn bin_to_bcd(value: u8) -> u8 {
    ((value / 10) << 4) | (value % 10)
  }

  // Converts a BCD value to binary format
  fn bcd_to_bin(value: u8) -> u8 {
    ((value & 0xF0) >> 4) * 10 + (value & 0x0F)
  }

  // If using an i2c mux, tell the mux to select our channel
  fn select_mux_channel(&mut self) -> Result<(), Error<I2C::Error>> {
    if self.mux_addr != 0u8 {
      self.i2c.write(self.mux_addr, &[self.mux_chan]).map_err(Error::I2c)
    }
    else {
      Ok(())
    }
  }

  // fn write_register(&mut self, reg: u8, data: u8) -> Result<(), Error<I2C::Error>> {
  //   self.select_mux_channel()?;
  //   self.write_register_raw(reg, data)
  // }

  fn write_register_raw(&mut self, reg: u8, data: u8) -> Result<(), Error<I2C::Error>> {
    self.i2c.write(RV3028_ADDRESS, &[reg, data]).map_err(Error::I2c)
  }

  fn read_register_raw(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
    let mut buf = [0];
    self.i2c.write_read(RV3028_ADDRESS, &[reg], &mut buf).map_err(Error::I2c)?;
    Ok(buf[0])
  }


  /// Read bytes from the two general-purpose User RAM registers.
  ///
  /// `offset` is a byte offset from User RAM 1: 0 selects register 0x1F,
  /// 1 selects register 0x20. The entire buffer must fit within
  /// [`USER_RAM_LEN`] bytes. An empty buffer at offset 0, 1 or 2 succeeds
  /// without bus access, including mux selection.
  ///
  /// These bytes are retained while the chip remains powered, including from
  /// VBACKUP, and reset to zero on power-on reset. No flags are cleared.
  /// Alarm and timer registers are not part of this memory region.
  /// See [Application Manual rev. 1.4, section 3.11, p. 31](https://www.microcrystal.com/fileadmin/Media/Products/RTC/App.Manual/RV-3028-C7_App-Manual.pdf#page=31).
  ///
  /// # Errors
  ///
  /// [`Error::InvalidData`] if the range does not fit, before any bus access.
  /// [`Error::I2c`] if mux selection or reading fails. After a failed read,
  /// the buffer may be partially modified by the HAL.
  pub fn read_user_ram(
    &mut self,
    offset: u8,
    bytes: &mut [u8],
  ) -> Result<(), Error<I2C::Error>> {
    Self::check_user_ram_range(offset, bytes.len())?;
    if bytes.is_empty() {
      return Ok(());
    }
    self.read_multi_registers(REG_USER_RAM_START + offset, bytes)
  }

  /// Write bytes to the two general-purpose User RAM registers.
  ///
  /// `offset` is a byte offset from User RAM 1: 0 selects register 0x1F,
  /// 1 selects register 0x20. The entire slice must fit within [`USER_RAM_LEN`]
  /// bytes. An empty slice at offset 0, 1 or 2 succeeds without bus access.
  ///
  /// Only the selected bytes are written; other registers are untouched.
  /// Two bytes use one I2C write, with the first slice byte written first.
  /// There is no readback, retry or rollback; a failed transfer may have written
  /// part of the data. The caller owns any marker or commit ordering.
  /// If password write protection is enabled, the chip must already be unlocked;
  /// successful bus acknowledgment alone does not verify the stored value.
  /// See [Application Manual rev. 1.4, section 3.11, p. 31](https://www.microcrystal.com/fileadmin/Media/Products/RTC/App.Manual/RV-3028-C7_App-Manual.pdf#page=31).
  ///
  /// # Examples
  ///
  /// ```no_run
  /// use embedded_hal::i2c::I2c;
  /// use rv3028c7_rtc::{Error, RV3028};
  ///
  /// fn update<I2C: I2c>(rtc: &mut RV3028<I2C>) -> Result<(), Error<I2C::Error>> {
  ///     let mut bytes = [0; 2];
  ///     rtc.read_user_ram(0, &mut bytes)?;
  ///     rtc.write_user_ram(0, &bytes)?;      // Both bytes.
  ///     rtc.write_user_ram(0, &bytes[..1])?; // First byte only.
  ///     bytes[1] = 0xA5;
  ///     rtc.write_user_ram(1, &bytes[1..])?; // Second byte only.
  ///     Ok(())
  /// }
  /// ```
  ///
  /// # Errors
  ///
  /// [`Error::InvalidData`] if the range does not fit, before any bus access.
  /// [`Error::I2c`] if mux selection or writing fails.
  pub fn write_user_ram(
    &mut self,
    offset: u8,
    bytes: &[u8],
  ) -> Result<(), Error<I2C::Error>> {
    Self::check_user_ram_range(offset, bytes.len())?;
    if bytes.is_empty() {
      return Ok(());
    }
    self.select_mux_channel()?;
    let mut packet = [0; 1 + USER_RAM_LEN];
    packet[0] = REG_USER_RAM_START + offset;
    packet[1..1 + bytes.len()].copy_from_slice(bytes);
    self.i2c.write(RV3028_ADDRESS, &packet[..1 + bytes.len()]).map_err(Error::I2c)
  }

  // Check before any I/O so a request cannot reach the adjacent password registers.
  fn check_user_ram_range(offset: u8, len: usize) -> Result<(), Error<I2C::Error>> {
    let offset = usize::from(offset);
    if offset > USER_RAM_LEN || len > USER_RAM_LEN - offset {
      return Err(Error::InvalidData);
    }
    Ok(())
  }

  /// Check whether the Power On Reset flag is set.
  /// If this flag is cleared (set to zero) beforehand,
  /// indicates a voltage drop below VPOR.
  /// If this flag is set, the data in the device RAM registers are no longer valid
  /// and all registers must be (re)initialized.
  /// The flag value 1 is retained until a 0 is written by the user.
  /// At power up (POR) the value is set to 1, the user has to write 0 to the flag to use it.
  ///
  /// Reports the flag observed in the initial read and, if set, acknowledges
  /// only that flag. Unrelated flags, including newly arriving events, are
  /// preserved. Multiple events on the same one-bit flag cannot be distinguished.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn check_and_clear_power_on_reset(&mut self) -> Result<bool, Error<I2C::Error>>  {
    let flag_set = 0 != self.check_and_clear_bits(
      REG_STATUS, RegStatusBits::PowerOnResetFlagBit as u8)?;
    Ok(flag_set)
  }

  /// Check whether an external event has been detected
  /// (an appropriate input signal on the EVI pin)
  ///
  /// Reports the flag observed in the initial read and, if set, acknowledges
  /// only that flag. Unrelated flags, including newly arriving events, are
  /// preserved. Multiple events on the same one-bit flag cannot be distinguished.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn check_and_clear_ext_event(&mut self)-> Result<bool, Error<I2C::Error>>  {
    let flag_set = 0 != self.check_and_clear_bits(
      REG_STATUS, RegStatusBits::EventFlagBit as u8)?;
    Ok(flag_set)
  }

  /// Check whether a switchover to the backup power supply has occurred
  /// (the Backup Switch Flag, BSF)
  ///
  /// Reports the flag observed in the initial read and, if set, acknowledges
  /// only that flag. Unrelated flags, including newly arriving events, are
  /// preserved. Multiple events on the same one-bit flag cannot be distinguished.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn check_and_clear_backup_event(&mut self) -> Result<bool, Error<I2C::Error>> {
    let flag_set = 0 != self.check_and_clear_bits(
      REG_STATUS, RegStatusBits::BackupSwitchFlag as u8)?;
    Ok(flag_set)
  }

  // // set specific bits in a register:
  // // all bits must be high that you wish to set
  // fn set_reg_bits(&mut self, reg: u8, bits: u8) -> Result<(), Error<I2C::Error>> {
  //   self.select_mux_channel()?;
  //   self.set_reg_bits_raw(reg, bits)
  // }

  // Set specific bits in a register: "raw" means it skips the mux
  // all bits must be high that you wish to set
  fn set_reg_bits_raw(&mut self, reg: u8, bits: u8) -> Result<(), Error<I2C::Error>> {
    let mut reg_val = self.read_register_raw(reg)?;
    reg_val |= bits; // Set bits that are high
    self.write_register_raw(reg, reg_val)
  }

  // clear specific bits in a register:
  // all bits must be high that you wish to be cleared
  // fn clear_reg_bits(&mut self, reg: u8, bits: u8) -> Result<(), Error<I2C::Error>> {
  //   self.select_mux_channel()?;
  //   self.clear_reg_bits_raw(reg,bits)
  // }

  // Clears selected bits without selecting the mux. Status flags require a
  // direct write-zero-to-clear acknowledgement, not read–modify–write.
  fn clear_reg_bits_raw(&mut self, reg: u8, bits: u8) -> Result<(), Error<I2C::Error>> {
    if reg == REG_STATUS {
      return self.write_register_raw(REG_STATUS, STATUS_WRITABLE_FLAGS & !bits);
    }
    let mut reg_val = self.read_register_raw(reg)?;
    reg_val &= !(bits); // Clear  bits that are high
    self.write_register_raw(reg, reg_val)
  }

  /// Enable or disable trickle charging
  /// - `enable` enables trickle charging if true, disables if false
  /// - `limit_resistance` Sets the current limiting resistor value: higher means less current
  /// Disabling also resets the `limit_resistance` to 3 kΩ, the factory default.
  /// Returns the status of trickle charging (true for enabled, false for disabled)
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn toggle_trickle_charge(&mut self, enable: bool,
                               limit_resistance: TrickleChargeCurrentLimiter) -> Result<bool, Error<I2C::Error>>  {
    self.select_mux_channel()?;

    // First disable charging before changing settings
    self.clear_reg_bits_raw(
      EEPROM_MIRROR_ADDRESS,  RegEepromMirrorBits::TrickleChargeEnableBit as u8)?;
    // Reset TCR to 3 kΩ, the factory default, by clearing the TCR bits
    self.clear_reg_bits_raw(
      EEPROM_MIRROR_ADDRESS,  RegEepromMirrorBits::TrickleChargeResistanceBits as u8 )?;

    if enable {
      self.set_reg_bits_raw(EEPROM_MIRROR_ADDRESS, limit_resistance as u8)?; //TODO
      self.set_reg_bits_raw(
        EEPROM_MIRROR_ADDRESS, RegEepromMirrorBits::TrickleChargeEnableBit as u8)?;
    }

    // confirm the value set
    let conf_val =
      0 != self.read_register_raw(EEPROM_MIRROR_ADDRESS)?
        & RegEepromMirrorBits::TrickleChargeEnableBit as u8;
    Ok(conf_val)
  }

  /// Toggle whether the Vbackup power source should be used
  /// when Vdd supply level drops below useful level.
  /// - `enable` enables switching to Vbackup, disables if false
  /// Selects DSM when enabled and BSM=00 when disabled; replaces the whole BSM
  /// field without changing calibration or charger settings. Use
  /// [Self::set_backup_config] for explicit DSM/LSM selection. Edits only RAM;
  /// refresh can replace it. Returns whether readback reports DSM or LSM.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn toggle_backup_switchover(&mut self, enable: bool) -> Result<bool, Error<I2C::Error>> {
    self.select_mux_channel()?;
    let current = self.read_register_raw(EEPROM_MIRROR_ADDRESS)?;
    // Replace both BSM bits, so enabling always selects DSM, even from BSM=10.
    let target = (current & !0b0000_1100) | if enable { 0b0000_0100 } else { 0 };
    self.write_register_raw(EEPROM_MIRROR_ADDRESS, target)?;
    let mode = self.read_register_raw(EEPROM_MIRROR_ADDRESS)? & 0b0000_1100;
    Ok(mode == 0b0000_0100 || mode == 0b0000_1100)
  }

  /// Disable all clock outputs triggered by interrupts
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn clear_all_int_clockout_bits(&mut self) -> Result<(), Error<I2C::Error>> {
    self.select_mux_channel()?;
    self.clear_reg_bits_raw(REG_CLOCK_INTERRUPT_MASK,
                            RegClockIntMaskBits::ClockoutOnExtEvtBit as u8 |
                              RegClockIntMaskBits::ClockoutOnAlarmBit as u8 |
                            RegClockIntMaskBits::ClockoutOnPctBit as u8 |
                              RegClockIntMaskBits::ClockoutOnUpdateBit as u8)
  }


  /// Get the current value of the EEPROM mirror from RAM
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn get_eeprom_mirror_value(&mut self) -> Result<u8, Error<I2C::Error>> {
    self.select_mux_channel()?;
    let reg_val = self.read_register_raw(EEPROM_MIRROR_ADDRESS)?;
    Ok(reg_val)
  }

  // Write Unix time first, then one coherent calendar burst. Writing seconds
  // resets the prescaler; the seven calendar registers share one I2C access.
  fn set_datetime_raw(&mut self, unix_timestamp: u32, dt: &DateTimeParts, weekday: u8) -> Result<(), Error<I2C::Error>> {
    self.select_mux_channel()?;
    let control2 = self.read_register_raw(REG_CONTROL2)?;
    let hour = Self::encode_calendar_hour(dt.hour, control2);
    let packet = [
      REG_SECONDS,
      Self::bin_to_bcd(dt.second),
      Self::bin_to_bcd(dt.minute),
      hour,
      weekday,
      Self::bin_to_bcd(dt.day),
      Self::bin_to_bcd(dt.month),
      Self::bin_to_bcd((dt.year - 2000) as u8),
    ];
    self.set_unix_time_raw(unix_timestamp)?;
    self.i2c.write(RV3028_ADDRESS, &packet).map_err(Error::I2c)
  }

  /// Set the date and time: both the unix time counter and all internal BCD date and time
  /// registers, so that they agree. The unix timestamp and weekday are calculated from `datetime`.
  ///
  /// The RTC's date registers only cover the years 2000 to 2099, and
  /// the unix time counter wraps after 2106. Invalid values return [`Error::InvalidData`].
  ///
  /// This method resets the internal prescaler pipeline, which means that
  /// subsecond counters are zeroed, when it writes to the Seconds register.
  /// This assists with clock synchronization with external clocks.
  ///
  /// Uses the existing 12-/24-hour setting without changing Control 2 or alarms.
  /// Unix time is written first, then all seven calendar bytes in one burst.
  /// These transfers are not atomic together: a bus/power failure may leave
  /// counter and calendar inconsistent. No readback or flag clearing is done.
  /// The calendar transfer must complete in less than 950 ms (the HAL's task).
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  ///
  /// [`Error::InvalidData`] if the date is invalid, the year is outside
  /// 2000–2099, or the time is outside 00:00:00–23:59:59. Validation happens
  /// before any bus access.
  pub fn set_datetime_parts(&mut self, datetime: &DateTimeParts) -> Result<(), Error<I2C::Error>> {
    datetime.validate()?;
    let unix_timestamp = datetime.unix_timestamp() as u32;
    self.set_datetime_raw(unix_timestamp, datetime, datetime.weekday())
  }

  /// Set only the calendar seconds register (0..=59).
  ///
  /// Writing Seconds resets the prescaler pipeline and zeroes subseconds.
  /// Other calendar fields and the independent Unix counter remain unchanged.
  /// Use [Self::set_datetime_parts] to set a consistent complete date/time.
  ///
  /// # Errors
  /// [`Error::InvalidData`] for values outside 0..=59, before any bus access.
  /// [`Error::I2c`] for bus or mux failures. No retry or readback is performed.
  pub fn set_seconds(&mut self, seconds: u8) -> Result<(), Error<I2C::Error>> {
    if !(0..=59).contains(&seconds) { return Err(Error::InvalidData); }
    self.select_mux_channel()?;
    self.write_register_raw(REG_SECONDS, Self::bin_to_bcd(seconds))
  }

  /// Set only the calendar minutes register (0..=59).
  ///
  /// Seconds and subseconds are not reset.
  /// Other calendar fields and the independent Unix counter remain unchanged.
  /// Use [Self::set_datetime_parts] to set a consistent complete date/time.
  ///
  /// # Errors
  /// [`Error::InvalidData`] for values outside 0..=59, before any bus access.
  /// [`Error::I2c`] for bus or mux failures. No retry or readback is performed.
  pub fn set_minutes(&mut self, minutes: u8) -> Result<(), Error<I2C::Error>> {
    if !(0..=59).contains(&minutes) { return Err(Error::InvalidData); }
    self.select_mux_channel()?;
    self.write_register_raw(REG_MINUTES, Self::bin_to_bcd(minutes))
  }

  /// Set only the calendar weekday register (0..=6).
  ///
  /// Uses Monday=0 through Sunday=6. No date/weekday consistency check is made.
  /// Other calendar fields and the independent Unix counter remain unchanged.
  /// Use [Self::set_datetime_parts] to set a consistent complete date/time.
  ///
  /// # Errors
  /// [`Error::InvalidData`] for values outside 0..=6, before any bus access.
  /// [`Error::I2c`] for bus or mux failures. No retry or readback is performed.
  pub fn set_weekday(&mut self, weekday: u8) -> Result<(), Error<I2C::Error>> {
    if !(0..=6).contains(&weekday) { return Err(Error::InvalidData); }
    self.select_mux_channel()?;
    self.write_register_raw(REG_WEEKDAY, weekday)
  }

  /// Set only the calendar day register (1..=31).
  ///
  /// The existing month/year are not read; the resulting date may be invalid.
  /// Other calendar fields and the independent Unix counter remain unchanged.
  /// Use [Self::set_datetime_parts] to set a consistent complete date/time.
  ///
  /// # Errors
  /// [`Error::InvalidData`] for values outside 1..=31, before any bus access.
  /// [`Error::I2c`] for bus or mux failures. No retry or readback is performed.
  pub fn set_day(&mut self, day: u8) -> Result<(), Error<I2C::Error>> {
    if !(1..=31).contains(&day) { return Err(Error::InvalidData); }
    self.select_mux_channel()?;
    self.write_register_raw(REG_DATE, Self::bin_to_bcd(day))
  }

  /// Set only the calendar month register (1..=12).
  ///
  /// The existing day/year are not read; the resulting date may be invalid.
  /// Other calendar fields and the independent Unix counter remain unchanged.
  /// Use [Self::set_datetime_parts] to set a consistent complete date/time.
  ///
  /// # Errors
  /// [`Error::InvalidData`] for values outside 1..=12, before any bus access.
  /// [`Error::I2c`] for bus or mux failures. No retry or readback is performed.
  pub fn set_month(&mut self, month: u8) -> Result<(), Error<I2C::Error>> {
    if !(1..=12).contains(&month) { return Err(Error::InvalidData); }
    self.select_mux_channel()?;
    self.write_register_raw(REG_MONTH, Self::bin_to_bcd(month))
  }

  /// Set only the calendar year register (2000..=2099).
  ///
  /// The existing day/month are not read; the resulting date may be invalid.
  /// Other calendar fields and the independent Unix counter remain unchanged.
  /// Use [Self::set_datetime_parts] to set a consistent complete date/time.
  ///
  /// # Errors
  /// [`Error::InvalidData`] for values outside 2000..=2099, before any bus access.
  /// [`Error::I2c`] for bus or mux failures. No retry or readback is performed.
  pub fn set_year(&mut self, year: i32) -> Result<(), Error<I2C::Error>> {
    if !(2000..=2099).contains(&year) { return Err(Error::InvalidData); }
    self.select_mux_channel()?;
    self.write_register_raw(REG_YEAR, Self::bin_to_bcd((year - 2000) as u8))
  }

  /// Set only the calendar Hours register, accepting hours 0–23.
  ///
  /// Reads Control 2 and encodes the hour using the existing 12-/24-hour mode.
  /// Callers must exclude concurrent hour-mode changes between these accesses.
  /// Other calendar fields, subseconds and the independent Unix counter remain
  /// unchanged. Use [Self::set_datetime_parts] for a complete date/time update.
  ///
  /// # Errors
  /// [`Error::InvalidData`] for hours above 23, before any bus access.
  /// [`Error::I2c`] for bus or mux failures. No retry or readback is performed.
  pub fn set_hours(&mut self, hours: u8) -> Result<(), Error<I2C::Error>> {
    if hours > 23 { return Err(Error::InvalidData); }
    self.select_mux_channel()?;
    let control2 = self.read_register_raw(REG_CONTROL2)?;
    self.write_register_raw(REG_HOURS, Self::encode_calendar_hour(hours, control2))
  }

  // Input is a validated hour in 0..=23.
  fn encode_calendar_hour(hour: u8, control2: u8) -> u8 {
    if control2 & 0b0000_0010 != 0 {
      let hour12 = match hour % 12 { 0 => 12, hour => hour };
      Self::bin_to_bcd(hour12) | if hour >= 12 { 0b0010_0000 } else { 0 }
    } else {
      Self::bin_to_bcd(hour)
    }
  }

  // Calendar hour encoding: Control 2 bit 1 selects 12 h; Hours bit 5 is PM.
  fn calendar_hour(raw: u8, control2: u8) -> Result<u8, Error<I2C::Error>> {
    if control2 & 0b0000_0010 != 0 {
      if raw & 0b1100_0000 != 0 { return Err(Error::InvalidData); }
      let hour12 = Self::checked_bcd(raw & 0b0001_1111)?;
      if !(1..=12).contains(&hour12) { return Err(Error::InvalidData); }
      Ok(hour12 % 12 + if raw & 0b0010_0000 != 0 { 12 } else { 0 })
    } else {
      let hour = Self::checked_bcd(raw)?;
      if hour > 23 { return Err(Error::InvalidData); }
      Ok(hour)
    }
  }

  /// Read the complete calendar in one burst from registers 0x00 through 0x06.
  ///
  /// Control 2 is read first to determine 12-/24-hour encoding. Returned hours
  /// are always 0–23. BCD, date, time and the Monday=0 weekday are validated.
  /// This reads the calendar, unlike backend `datetime()` methods which read
  /// the independent Unix counter. PORF is not checked or cleared.
  ///
  /// The calendar's single transfer must finish in less than 950 ms to use the
  /// chip's register blocking (Application Manual rev. 1.4, section 4.5).
  /// The HAL owns transaction timeouts. Control 2 and the calendar are separate
  /// accesses: callers must exclude concurrent changes of the hour mode.
  ///
  /// # Errors
  /// Returns [`Error::InvalidData`] for invalid calendar fields or a mismatched
  /// weekday, and [`Error::I2c`] for bus or mux failures. No retry is performed.
  pub fn get_datetime_parts(&mut self) -> Result<DateTimeParts, Error<I2C::Error>> {
    self.select_mux_channel()?;
    let control2 = self.read_register_raw(REG_CONTROL2)?;
    let mut bytes = [0; 7];
    self.read_multi_registers_raw(REG_SECONDS, &mut bytes)?;
    let hour = Self::calendar_hour(bytes[2], control2)?;
    let datetime = DateTimeParts {
      year: 2000 + i32::from(Self::checked_bcd(bytes[6])?),
      month: Self::checked_bcd(bytes[5])?,
      day: Self::checked_bcd(bytes[4])?,
      hour,
      minute: Self::checked_bcd(bytes[1])?,
      second: Self::checked_bcd(bytes[0])?,
    };
    datetime.validate()?;
    let weekday = bytes[(REG_WEEKDAY - REG_SECONDS) as usize];
    if weekday != datetime.weekday() { return Err(Error::InvalidData); }
    Ok(datetime)
  }

  /// Get the year, month, day from the internal BCD registers
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  ///
  /// [`Error::InvalidData`] if the registers contain invalid BCD or an
  /// invalid calendar date.
  pub fn get_ymd(&mut self) -> Result<(i32, u8, u8), Error<I2C::Error>> {
    let mut read_buf = [0u8;3];
    self.read_multi_registers(REG_DATE, &mut read_buf)?;
    let day = Self::checked_bcd(read_buf[0])?;
    let month = Self::checked_bcd(read_buf[1])?;
    let year:i32 = Self::checked_bcd(read_buf[2])? as i32 + 2000;

    DateTimeParts { year, month, day, hour: 0, minute: 0, second: 0 }.validate()?;
    Ok((year, month, day))
  }

  /// Get hour, minute, second in one burst from the internal BCD registers.
  /// Uses the current 12-/24-hour mode and always returns hours 0–23.
  /// Use [Self::get_datetime_parts] for a coherent snapshot including the date.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  ///
  /// [`Error::InvalidData`] if the registers contain invalid BCD or a
  /// time outside 00:00:00–23:59:59.
  pub fn get_hms(&mut self) -> Result<(u8, u8, u8), Error<I2C::Error>> {
    let mut read_buf = [0u8;3];
    self.select_mux_channel()?;
    let control2 = self.read_register_raw(REG_CONTROL2)?;
    self.read_multi_registers_raw(REG_SECONDS, &mut read_buf)?;
    let seconds = Self::checked_bcd(read_buf[0])?;
    let minutes = Self::checked_bcd(read_buf[1])?;
    let hours = Self::calendar_hour(read_buf[2], control2)?;
    if hours > 23 || minutes > 59 || seconds > 59 {
      return Err(Error::InvalidData);
    }
    Ok( (hours, minutes, seconds) )
  }

  // read a block of registers all at once
  fn read_multi_registers(&mut self, reg: u8, read_buf: &mut [u8] )  -> Result<(), Error<I2C::Error>> {
    self.select_mux_channel()?;
    self.read_multi_registers_raw(reg, read_buf)
  }

  // read a block of registers all at once: skip mux
  fn read_multi_registers_raw(&mut self, reg: u8, read_buf: &mut [u8] )  -> Result<(), Error<I2C::Error>> {
    self.i2c.write_read(RV3028_ADDRESS, &[reg], read_buf).map_err(Error::I2c)
  }

  /// Set just the Unix time counter.
  /// Prefer [`Self::set_datetime_parts`] (or the `set_datetime` method of a date/time backend)
  /// to properly set all internal BCD registers.
  /// Note:
  /// - This does NOT set other internal BCD registers
  /// such as Year or Hour, which the alarm and event timestamp functions use.
  /// - This does not reset the prescaler pipeline,
  /// which means subseconds are not reset to zero.
  ///
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn set_unix_time(&mut self, unix_time: u32) -> Result<(), Error<I2C::Error>> {
    self.select_mux_channel()?;
    self.set_unix_time_raw(unix_time)
  }

  // sets the unix time counter but skips the mux
  fn set_unix_time_raw(&mut self, unix_time: u32) -> Result<(), Error<I2C::Error>> {
    let bytes = unix_time.to_le_bytes(); // Convert to little-endian byte array
    self.i2c.write(RV3028_ADDRESS, &[REG_UNIX_TIME_0, bytes[0], bytes[1], bytes[2], bytes[3]]).map_err(Error::I2c)
  }

  /// Reads the value of the RTC's unix time counter, notionally seconds elapsed since the
  /// common "unix epoch" in the year 1970. It cannot represent datetimes from prior to 1970.
  /// - Note that this is an unsigned u32 value, with different characteristics from the
  /// widely used rust / chrono i64 system timestamp.
  /// - The RTC will continue to increment this counter until it wraps at 0xFFFFFFFF
  /// which defers the "Year 2038 problem" until about the year 2106.
  /// - Note that the RTC's automatic leap year correction is only valid until 2099
  /// See the App Manual section "3.10. UNIX TIME REGISTERS"
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn get_unix_time(&mut self) -> Result<u32, Error<I2C::Error>> {
    let mut read_buf = [0u8; 4];
    self.read_multi_registers(REG_UNIX_TIME_0, &mut read_buf)?;
    let val = u32::from_le_bytes(read_buf);
    Ok(val)
  }

  /// The vendor application manual suggest we read the unix time twice,
  /// in case an internal increment or timestamp set is interspersed between the multi-byte read.
  /// Compares consecutive reads, with at most four reads total. Returns
  /// [`Error::InconsistentRead`] if none agree. Transaction timeouts remain the HAL's responsibility.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  ///
  /// [`Error::InconsistentRead`] if no consecutive counter reads agree
  /// within four reads total. Individual transaction timeouts remain the
  /// HAL's responsibility.
  pub fn get_unix_time_blocking(&mut self) -> Result<u32, Error<I2C::Error>> {
    let mut previous = self.get_unix_time()?;
    for _ in 0..3 {
      let current = self.get_unix_time()?;
      if current == previous {
        return Ok(current);
      }
      previous = current;
    }
    Err(Error::InconsistentRead)
  }

  /// Toggle whether EVI events trigger on high/rising or low/falling edges
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn toggle_event_high_low(&mut self, high: bool) -> Result<(), Error<I2C::Error>> {
    self.set_or_clear_reg_bits(REG_EVENT_CONTROL, RegEventControlBits::EventHighLowBit as u8, high)
  }

  /// Enable INT pin output when alarm occurs
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn toggle_alarm_int_enable(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
    self.set_or_clear_reg_bits(REG_CONTROL2, RegControl2Bits::AlarmIntEnableBit as u8, enable)
  }

  /// Toggle whether the RTC outputs a pulse (active low) on INT pin,
  /// when the countdown timer expires.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn toggle_countdown_int_enable(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
    self.set_or_clear_reg_bits(REG_CONTROL2, RegControl2Bits::TimerIntEnableBit as u8, enable)
  }

  /// Toggle whether interrupt signal is generated on the INT pin:
  /// - when an External Event on EVI pin occurs and TSS = 0
  /// - or when an Automatic Backup Switchover occurs and TSS = 1.
  /// The signal on the INT pin is retained until the EVF flag is cleared
  /// to 0 (no automatic cancellation)
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn toggle_ext_event_int_enable(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
    self.set_or_clear_reg_bits(REG_CONTROL2, RegControl2Bits::EventIntEnableBit as u8, enable)
  }

  /// Toggles whether an interrupt signal is generated on the INT pin:
  /// - when the time updates at either 1 second or 1 minute intervals
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn toggle_time_up_int_enable(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
    self.set_or_clear_reg_bits(REG_CONTROL2, RegControl2Bits::TimeUpdateIntEnableBit as u8, enable)
  }

  /// Disable all INT pin output selector bits in RAM, excludes PORIE
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn clear_all_int_out_bits(&mut self) -> Result<(), Error<I2C::Error>> {
    self.select_mux_channel()?;
    // UIE, TIE, AIE,  EIE
    self.clear_reg_bits_raw(REG_CONTROL2,
                        RegControl2Bits::TimeUpdateIntEnableBit as u8 |
                          RegControl2Bits::TimerIntEnableBit as u8 |
                          RegControl2Bits::AlarmIntEnableBit as u8 |
                          RegControl2Bits::EventIntEnableBit as u8  )?;
    // BSIE
    self.clear_reg_bits_raw(
      EEPROM_MIRROR_ADDRESS, RegEepromMirrorBits::BackupSwitchIntEnableBit as u8)?;

    // PORIE -- must be set in EEPROM -- don't bother to set?
    Ok(())
  }


  /// Clear all seven writable Status flags with one write; EEbusy is read-only.
  /// This deliberately acknowledges all events, including ones arriving before
  /// the write. Use the individual check-and-clear methods to preserve unrelated flags.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn clear_all_status_flags(&mut self) -> Result<(), Error<I2C::Error>> {
    self.select_mux_channel()?;
    self.clear_reg_bits_raw(REG_STATUS,
                            RegStatusBits::ClockIntFlagBit  as u8 |
                              RegStatusBits::BackupSwitchFlag  as u8 |
                              RegStatusBits::TimeUpdateFlag  as u8 |
                              RegStatusBits::PeriodicTimerFlag  as u8 |
                              RegStatusBits::AlarmFlagBit  as u8 |
                              RegStatusBits::EventFlagBit  as u8 |
                              RegStatusBits::PowerOnResetFlagBit as u8
    )

  }

  /// - `int_enable` enables INT output on the periodic time updates
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn configure_periodic_time_update(&mut self, minutes: bool, int_enable: bool) -> Result<(), Error<I2C::Error>> {
    self.select_mux_channel()?;

    // 1. Initialize bits UIE and UF to 0.
    // 2. Choose the timer source clock and write the corresponding value in the USEL bit.
    // 3. Set the UIE bit to 1 if you want to get a hardware interrupt on INT̅ ̅ ̅ ̅ ̅
    // pin.
    // 4. Set CUIE bit to 1 to enable clock output when a time update interrupt occurs. See also CLOCK OUTPUT
    // SCHEME.
    // 5. The first interrupt will occur after the next event, either second or minute change.

    // UIE clear
    self.clear_reg_bits_raw(
      REG_CONTROL2, RegControl2Bits::TimeUpdateIntEnableBit as u8)?;
    // UF clear
    self.clear_reg_bits_raw(REG_STATUS, RegStatusBits::TimeUpdateFlag as u8)?;
    // USEL set/clear
    self.set_or_clear_reg_bits_raw(
      REG_CONTROL1, RegControl1Bits::UselBit as u8, minutes)?;
    // UIE re-set
    self.set_or_clear_reg_bits_raw(
      REG_CONTROL2, RegControl2Bits::TimeUpdateIntEnableBit as u8, int_enable)?;

    Ok(())
  }


  /// Get event count -- the number of events that have been logged since enabling logging.
  /// Returns the count of events since last reset, and the date and time of one event
  /// (see [`EventTimeStampLogger`] for which event is recorded).
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  ///
  /// [`Error::InvalidData`] for malformed BCD/date/time. Zero count with a valid
  /// timestamp is retained after counter wraparound; only an entirely empty
  /// register block returns None. Hour decoding follows [Self::get_timestamp].
  pub fn get_event_count_and_parts(&mut self) -> Result<(u32, Option<DateTimeParts>), Error<I2C::Error>> {
    let snapshot = self.get_timestamp()?;
    Ok((u32::from(snapshot.count), snapshot.datetime))
  }

  /// Check the alarm status, and if it's triggered, clear it
  /// return bool indicating whether the alarm triggered
  ///
  /// Reports the flag observed in the initial read and, if set, acknowledges
  /// only that flag. Unrelated flags, including newly arriving events, are
  /// preserved. Multiple events on the same one-bit flag cannot be distinguished.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn check_and_clear_alarm(&mut self) -> Result<bool, Error<I2C::Error>> {
    // Check if the AF flag is set
    let alarm_flag_set = 0 != self.check_and_clear_bits(
      REG_STATUS, RegStatusBits::AlarmFlagBit as u8)?;
    Ok(alarm_flag_set)
  }

  /// All-in-one method to set an alarm:
  /// See the App Note section "Procedure to use the Alarm Interrupt".
  /// Only day (or weekday), hour and minute are supported; see [`AlarmParts`].
  /// Hours are 0–23, encoded according to the current 12-/24-hour mode.
  /// All fields, including disabled fields, must be valid; checked before bus access.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  ///
  /// [`Error::InvalidData`] if the day is outside 1–31 for a date alarm
  /// or 0–6 for a weekday alarm, the hour exceeds 23, or the minute exceeds
  /// 59. All fields are validated, including disabled fields, before bus access.
  pub fn set_alarm_parts(&mut self, alarm: &AlarmParts) -> Result<(), Error<I2C::Error>> {

    alarm.validate()?;
    self.select_mux_channel()?;
    let control2 = self.read_register_raw(REG_CONTROL2)?;
    // Initialize AF to 0; AIE/AlarmIntEnableBit is managed independently
    self.clear_reg_bits_raw(REG_STATUS, RegStatusBits::AlarmFlagBit as u8)?;

    // Procedure suggested by App Notes:
    // 1. Initialize bits AIE and AF to 0.
    // 2. Choose weekday alarm or date alarm (weekday/date) by setting the WADA bit.
    // WADA = 0 for weekday alarm or WADA = 1 for date alarm.
    // 3. Write the desired alarm settings in registers 07h to 09h. The three alarm enable bits, AE_M, AE_H and
    // AE_WD, are used to select the corresponding register that has to be taken into account for match or not.
    // See the following table.

    // Clear WADA for weekday alarm, or set for date alarm
    self.set_or_clear_reg_bits_raw(
      REG_CONTROL1, RegControl1Bits::WadaBit as u8, !alarm.weekday_alarm)?;

    let bcd_minute = Self::bin_to_bcd(alarm.minute);
    self.write_register_raw(REG_MINUTES_ALARM,
                        if alarm.match_minute { bcd_minute }
                        else { ALARM_NO_WATCH_FLAG | bcd_minute })?;

    let bcd_hour = Self::encode_calendar_hour(alarm.hour, control2);
    self.write_register_raw(REG_HOURS_ALARM,
                        if alarm.match_hour { bcd_hour  }
                        else { ALARM_NO_WATCH_FLAG | bcd_hour })?;

    // either the weekday (for a weekday alarm) or the day of month (for a date alarm)
    let bcd_day = Self::bin_to_bcd(alarm.day);
    self.write_register_raw(REG_WEEKDAY_DATE_ALARM,
                        if alarm.match_day { bcd_day }
                        else { ALARM_NO_WATCH_FLAG | bcd_day })?;

    // Clear AF again in case the above setting process immediately triggered the alarm
    self.clear_reg_bits_raw(REG_STATUS, RegStatusBits::AlarmFlagBit as u8)?;

    Ok(())
  }

  /// Read the alarm settings, normalizing hours to 0–23 in both hour modes.
  /// Invalid enabled fields return [`Error::InvalidData`].
  /// Invalid disabled fields become zero (or day 1 for a date alarm).
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  ///
  /// [`Error::InvalidData`] if an enabled alarm field contains invalid
  /// BCD or an out-of-range value. Invalid disabled fields are replaced
  /// with zero (or day 1 for a date alarm).
  pub fn get_alarm_parts(&mut self) -> Result<AlarmParts, Error<I2C::Error>> {

    self.select_mux_channel()?;
    let control2 = self.read_register_raw(REG_CONTROL2)?;

    let raw_day = self.read_register_raw(REG_WEEKDAY_DATE_ALARM)?;
    let match_day = 0 == (raw_day & ALARM_NO_WATCH_FLAG);
    let day = Self::checked_bcd(0x7F & raw_day);

    let raw_hour = self.read_register_raw(REG_HOURS_ALARM)?;
    let match_hour = 0 == (raw_hour & ALARM_NO_WATCH_FLAG);
    let hour = Self::calendar_hour(0x7F & raw_hour, control2);

    let raw_minutes = self.read_register_raw(REG_MINUTES_ALARM)?;
    let match_minutes = 0 == (raw_minutes & ALARM_NO_WATCH_FLAG);
    let minutes = Self::checked_bcd(0x7F & raw_minutes);

    let wada_state = self.read_register_raw(REG_CONTROL1)? & RegControl1Bits::WadaBit as u8;

    // Disabled fields may contain unusable calendar values. Preserve valid
    // values, but substitute a valid placeholder for an invalid ignored field.
    let weekday_alarm = wada_state == 0;
    let day = day.and_then(|v| {
      if (weekday_alarm && v <= 6) || (!weekday_alarm && (1..=31).contains(&v)) {
        Ok(v)
      } else { Err(Error::InvalidData) }
    });
    let hour = hour.and_then(|v| if v <= 23 { Ok(v) } else { Err(Error::InvalidData) });
    let minutes = minutes.and_then(|v| if v <= 59 { Ok(v) } else { Err(Error::InvalidData) });
    Ok(AlarmParts {
      day: if match_day { day? } else { day.unwrap_or(if weekday_alarm { 0 } else { 1 }) },
      hour: if match_hour { hour? } else { hour.unwrap_or(0) },
      minute: if match_minutes { minutes? } else { minutes.unwrap_or(0) },
      weekday_alarm: 0 == wada_state,
      match_day,
      match_hour,
      match_minute: match_minutes,
    })
  }



  // If `set` is true, set the high bits given in `bits`, otherwise clear those bits
  fn set_or_clear_reg_bits(&mut self, reg: u8, bits: u8, set: bool) -> Result<(), Error<I2C::Error>> {
    self.select_mux_channel()?;
    self.set_or_clear_reg_bits_raw(reg, bits, set)
  }

  fn set_or_clear_reg_bits_raw(&mut self, reg: u8, bits: u8, set: bool) -> Result<(), Error<I2C::Error>> {
    if set {
      self.set_reg_bits_raw(reg, bits)
    }
    else {
      self.clear_reg_bits_raw(reg, bits)
    }
  }


  /// Set CLKOE in the CLKOUT RAM mirror 0x35 (EEPROM is unchanged).
  /// CLKF can still enable output when CLKOE is clear. Use
  /// [Self::set_clkout_config] with [ClkoutSource::ForcedLow] to force LOW.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn toggle_plain_clockout(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
    self.select_mux_channel()?;
    self.set_or_clear_reg_bits_raw(configuration::CLKOUT_REGISTER, 0b1000_0000, enable)
  }

  /// Enables or disables interrupt-controlled CLKOUT
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn toggle_int_clockout(&mut self, enable: bool)  -> Result<(), Error<I2C::Error>> {
    self.select_mux_channel()?;
    self.set_or_clear_reg_bits_raw(
    REG_CONTROL2,  RegControl2Bits::ClockoutIntEnableBit as u8, enable)
  }

  // Configure the Periodic Countdown Timer prior to the next countdown.
  fn config_pct_raw(&mut self, value: u16, freq: TimerClockFreq, repeat: bool ) -> Result<(), Error<I2C::Error>> {
    let value_high: u8 = ((value >> 8) as u8) & 0x0F;
    let value_low: u8 = (value & 0xFF) as u8;

    // configure the timer clock source / period
    self.clear_reg_bits_raw(REG_CONTROL1, RegControl1Bits::TimerEnableBit as u8)?;
    self.set_or_clear_reg_bits_raw(REG_CONTROL1, RegControl1Bits::TimerRepeatBit as u8, repeat)?;

    self.clear_reg_bits_raw(REG_CONTROL1, RegControl1Bits::TimerClockFreqBits as u8)?;
    self.set_reg_bits_raw(REG_CONTROL1, freq as u8)?; // Frequency selector is the unshifted TD field.

    // write to REG_TIMER_VALUE0 and REG_TIMER_VALUE1
    let write_buf = [ REG_TIMER_VALUE0, value_low, value_high];
    self.i2c.write(RV3028_ADDRESS, &write_buf).map_err(Error::I2c)?;

    self.clear_reg_bits_raw(REG_STATUS, RegStatusBits::PeriodicTimerFlag as u8)?;
    Ok(())
  }

  const MAX_PCT_TICKS: u16 = 0x0FFF; // 4095
  // Select the finest source whose full 12-bit range includes the request.
  // Use rational frequency arithmetic; Duration truncates fractional nanoseconds.
  fn pct_ticks_and_rate_for_duration(duration: &Duration) -> (u16, TimerClockFreq, Duration) {
    let nanos = duration.as_nanos();
    for (freq, numerator, denominator) in [
      (TimerClockFreq::Hertz4096, 4096u128, 1u128),
      (TimerClockFreq::Hertz64, 64, 1),
      (TimerClockFreq::Hertz1, 1, 1),
      (TimerClockFreq::HertzSixtieth, 1, 60),
    ] {
      let scaled = nanos * numerator;
      let unit = 1_000_000_000 * denominator;
      if scaled <= u128::from(Self::MAX_PCT_TICKS) * unit {
        let ticks = if nanos == 0 { 0 } else { (scaled / unit).max(1) };
        let period_ns = ticks * unit / numerator;
        return (ticks as u16, freq, Duration::from_nanos(period_ns as u64));
      }
    }
    (Self::MAX_PCT_TICKS, TimerClockFreq::HertzSixtieth,
      Duration::from_secs(u64::from(Self::MAX_PCT_TICKS) * 60))
  }

  /// Prepare the Periodic Countdown Timer for a countdown,
  /// and optionally start the countdown.
  ///
  /// - `repeat`: If true, the countdown timer will repeat as a periodic timer.
  ///   If false, the countdown timer will only run once ("one-shot" mode).
  /// - `start`: If true, start the countdown.
  ///
  /// Selects the finest clock whose full 12-bit range includes the request and
  /// rounds down to whole ticks. Positive requests below one 4096-Hz tick are
  /// clamped to one tick; durations above 4095 minutes are clamped to that maximum.
  /// Zero programs zero ticks (the chip does not count down).
  /// Returns the nominal period, truncating fractional nanoseconds only.
  /// This is not the first-period latency: starting the timer has a phase-dependent
  /// additional delay, as specified in Application Manual section 4.8.3.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn config_countdown_timer(&mut self, duration: &Duration,
                                repeat: bool, start: bool
  ) -> Result<Duration, Error<I2C::Error>> {
    let (ticks, freq, estimated) =
      Self::pct_ticks_and_rate_for_duration(duration);

    self.select_mux_channel()?;
    self.config_pct_raw(ticks, freq, repeat)?;
    if start {
      self.set_reg_bits_raw(REG_CONTROL1, RegControl1Bits::TimerEnableBit as u8)?;
    }

    Ok(estimated)
  }

  /// Set whether the Periodic Countdown Timer mode is repeating (periodic) or one-shot.
  /// - `enable`: If true, starts the timer countdown. If false, stops the timer.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn toggle_countdown_timer(&mut self, enable: bool)  -> Result<(), Error<I2C::Error>> {
    self.set_or_clear_reg_bits(
      REG_CONTROL1, RegControl1Bits::TimerEnableBit as u8, enable)
  }

  /// Check whether countdown timer has finished counting down, and clear it
  ///
  /// Reports the flag observed in the initial read and, if set, acknowledges
  /// only that flag. Unrelated flags, including newly arriving events, are
  /// preserved. Multiple events on the same one-bit flag cannot be distinguished.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn check_and_clear_countdown(&mut self) -> Result<bool, Error<I2C::Error>> {
    let flag_set = 0 != self.check_and_clear_bits(
      REG_STATUS, RegStatusBits::PeriodicTimerFlag as u8)?;
    Ok(flag_set)
  }

  /// Read the current value of the Periodic Countdown Timer,
  /// which is only valid after the timer has been enabled.
  /// The meaning of the value depends on the configured TimerClockFreq
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn get_countdown_value(&mut self) -> Result<u16, Error<I2C::Error>> {
    let mut read_buf = [0u8;2];
    self.read_multi_registers(REG_TIMER_STATUS0, &mut read_buf)?;
    let value = ((read_buf[1] as u16) << 8) | (read_buf[0] as u16);
    Ok(value)
  }

  // Returns only the flags observed in the initial read. A later event on an
  // unrelated flag is preserved; a second event on the acknowledged flag cannot
  // be distinguished from the first by the chip's single-bit latch.
  fn check_and_clear_bits(&mut self, reg: u8, bits: u8) -> Result<u8, Error<I2C::Error>> {
    self.select_mux_channel()?;
    let reg_val = self.read_register_raw(reg)?;
    let bits_val =  reg_val & bits;
    if 0 != bits_val {
      self.clear_reg_bits_raw(reg, bits)?;
    }
    Ok(bits_val)
  }


  /// Configure event detection on the EVI pin
  /// - `rising` whether edge detection is on rising edge / high level
  /// - `int_enable` whether events detected on EVI pin should generate an interrupt on INT pin
  /// - `filtering`: unshifted ET field, 0b00 (edge detection), 0b01 (3.9 ms),
  ///   0b10 (15.6 ms), or 0b11 (125 ms) level filtering.
  /// - `clockout_enable`: enable clock output following an EVI event.
  ///
  /// # Errors
  ///
  /// [`Error::InvalidData`] if `filtering` exceeds 0b11, before any bus access.
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  pub fn config_ext_event_detection(
    &mut self, rising: bool, int_enable: bool, filtering: u8, clockout_enable: bool) -> Result<(), Error<I2C::Error>>
  {
    if filtering > 0b11 { return Err(Error::InvalidData); }
    self.select_mux_channel()?;

    // Timestamp capture is managed separately; this method only pauses EIE.
    // Clear EVF and update the EVI detection and routing fields.
    // 4. Set EHL bit to 1 or 0 to choose high or low level (or rising or falling edge) detection on pin EVI.
    // 5. Select EDGE DETECTION (ET = 00) or LEVEL DETECTION WITH FILTERING (ET ≠ 00).
    // 8. Set CEIE bit to 1 to enable clock output when external event occurs. See also CLOCK OUTPUT SCHEME.
    // 10. Set EIE bit to 1 if you want to get a hardware interrupt on INT̅ ̅ ̅ ̅ ̅
    // pin.

    // Pause external-event interrupt output; timestamp capture is unchanged.
    // 1. Initialize EIE to 0.
    self.clear_reg_bits_raw(REG_CONTROL2,
                              RegControl2Bits::EventIntEnableBit  as u8)?;
    // 2. Clear flag EVF to 0.
    self.clear_reg_bits_raw(
      REG_STATUS, RegStatusBits::EventFlagBit as u8)?;

    // 4. Set EHL bit to 1 or 0 to choose high or low level
    // (or rising or falling edge) detection on pin EVI.
    self.set_or_clear_reg_bits_raw(
      REG_EVENT_CONTROL, RegEventControlBits::EventHighLowBit as u8, rising)?;

    // 5. Select EDGE DETECTION (ET = 00) or LEVEL DETECTION WITH FILTERING (ET ≠ 00).
    self.clear_reg_bits_raw(REG_EVENT_CONTROL,RegEventControlBits::EventFilteringTimeBits as u8)?;
    if 0 != filtering {
      // Validated ET field, shifted into bits 5:4.
      self.set_reg_bits_raw(REG_EVENT_CONTROL, filtering << 4)?;
    }

    // 8. Set CEIE bit to 1 to enable clock output when external event occurs.
    // See also CLOCK OUTPUT SCHEME.
    self.set_or_clear_reg_bits_raw(
      REG_CLOCK_INTERRUPT_MASK, RegClockIntMaskBits::ClockoutOnExtEvtBit as u8, clockout_enable)?;

    // 10. Set EIE bit to 1 if you want to get a hardware interrupt on INT pin
    self.set_or_clear_reg_bits_raw(
      REG_CONTROL2, RegControl2Bits::EventIntEnableBit as u8, int_enable)?;

    Ok(())
  }

}


/// Time stamp logging of events: external events on the EVI pin,
/// or switchovers to the backup power supply.
///
/// Reading the logged time stamp needs a date/time representation, so it is provided by
/// [`RV3028::get_event_count_and_parts`] and by the date/time backend traits.
pub trait EventTimeStampLogger {
  /// Error type
  type Error;

  /// Enable or disable the Time Stamp Function for event logging
  /// This logs external interrupts or other events
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  fn toggle_timestamp_logging(&mut self, enable: bool) -> Result<(), Self::Error>;

  /// clear out any existing logged event timestamps
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  fn reset_timestamp_log(&mut self) -> Result<(), Self::Error>;

  /// Setup time stamp logging for events
  /// - `evt_source` source for timestamp events, eg TS_EVENT_SOURCE_BSF
  /// - `overwrite` Save the most recent event timestamp?
  /// - `start` Should event timestamp logging immediately start?
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  fn config_timestamp_logging(
    &mut self, evt_source: u8, overwrite: bool,   start: bool)
    -> Result<(), Self::Error>;

  /// Enable or disable event time stamp overwriting
  /// If this is disabled (default), the first event time stamp is saved.
  /// If this is enabled, the most recent event time stamp is saved.
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  fn toggle_time_stamp_overwrite(&mut self, enable: bool) -> Result<(), Self::Error>;

  /// Select a source for events to be logged, device-specific
  ///
  /// # Errors
  ///
  /// [`Error::I2c`] if an I2C transaction fails, including mux selection.
  fn set_event_timestamp_source(&mut self, source: u8) -> Result<(), Self::Error>;
}

impl<I2C> EventTimeStampLogger for  RV3028<I2C>
  where
    I2C: I2c,
{
  type Error = Error<I2C::Error>;


  fn toggle_timestamp_logging(&mut self, enable: bool) -> Result<(), Self::Error> {
    self.select_mux_channel()?;
    self.set_or_clear_reg_bits_raw(REG_CONTROL2, RegControl2Bits::TimeStampEnableBit as u8, enable)
  }

  fn reset_timestamp_log(&mut self) -> Result<(), Self::Error> {
    self.reset_timestamp()
  }

  fn config_timestamp_logging(
    &mut self, evt_source: u8, overwrite: bool,  start:bool) -> Result<(), Self::Error>
  {
    let source = match evt_source {
      TS_EVENT_SOURCE_EVI => TimestampSource::ExternalEvent,
      TS_EVENT_SOURCE_BSF => TimestampSource::BackupSwitchover,
      _ => return Err(Error::InvalidData),
    };
    self.configure_timestamp(&TimestampConfig {
      source,
      capture: if overwrite { TimestampCapture::LastEvent } else { TimestampCapture::FirstEvent },
      enabled: start,
    })
  }

  fn toggle_time_stamp_overwrite(&mut self, enable: bool) -> Result<(), Self::Error> {
    self.set_or_clear_reg_bits(
      REG_EVENT_CONTROL, RegEventControlBits::TimeStampOverwriteBit as u8, enable)
  }

  fn set_event_timestamp_source(&mut self, source: u8) -> Result<(), Self::Error> {
    let enable = match source {
      TS_EVENT_SOURCE_EVI => false,
      TS_EVENT_SOURCE_BSF => true,
      _ => return Err(Error::InvalidData),
    };
    let mut config = self.get_timestamp_config()?;
    config.source = if enable { TimestampSource::BackupSwitchover } else { TimestampSource::ExternalEvent };
    self.set_timestamp_config(&config)
  }

}

#[cfg(test)]
mod tests {
  use super::*;
  use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTrans};
  use std::vec;




  #[test]
  fn test_set_unix_time() {
    let unix_time: u32 = 1_614_456_789; // Example Unix time
    let bytes = unix_time.to_le_bytes(); // Convert to little-endian byte array
    let expectations = [
      I2cTrans::write(
        RV3028_ADDRESS,
        vec![
          REG_UNIX_TIME_0,
          bytes[0],
          bytes[1],
          bytes[2],
          bytes[3],
        ],
      ),
    ];
    let mock = I2cMock::new(&expectations);
    let mut rv3028 = RV3028::new(mock);
    rv3028.set_unix_time(unix_time).unwrap();
    rv3028.i2c.done();
  }

  #[test]
  fn test_get_unix_time() {
    let unix_time: u32 = 1_614_456_789; // Example Unix time
    let bytes = unix_time.to_le_bytes(); // Convert to little-endian byte array
    let expectations = [
      I2cTrans::write_read(
        RV3028_ADDRESS,
        vec![REG_UNIX_TIME_0],
        bytes.to_vec(),
      ),
      I2cTrans::write_read(
        RV3028_ADDRESS,
        vec![REG_UNIX_TIME_0],
        bytes.to_vec(),
      ),
    ];
    let mock = I2cMock::new(&expectations);
    let mut rv3028 = RV3028::new(mock);
    assert_eq!(rv3028.get_unix_time_blocking().unwrap(), unix_time);
    rv3028.i2c.done();
  }

  // 2024-03-15 13:45:30 UTC, a Friday
  const TEST_PARTS: DateTimeParts =
    DateTimeParts { year: 2024, month: 3, day: 15, hour: 13, minute: 45, second: 30 };
  const TEST_UNIX_TIME: u32 = 1_710_510_330;

  #[test]
  fn test_set_datetime_parts() {
    let unix_bytes = TEST_UNIX_TIME.to_le_bytes();
    let expectations = [
      I2cTrans::write_read(RV3028_ADDRESS, vec![REG_CONTROL2], vec![0]),
      I2cTrans::write(RV3028_ADDRESS, vec![
        REG_UNIX_TIME_0, unix_bytes[0], unix_bytes[1], unix_bytes[2], unix_bytes[3]]),
      // One coherent calendar burst: seconds through year.
      I2cTrans::write(RV3028_ADDRESS, vec![REG_SECONDS, 0x30, 0x45, 0x13, 0x04, 0x15, 0x03, 0x24]),
    ];
    let mut rv3028 = RV3028::new(I2cMock::new(&expectations));
    rv3028.set_datetime_parts(&TEST_PARTS).unwrap();
    rv3028.i2c.done();
  }

  // Check the unix timestamp and weekday calculation against chrono
  // for every day the RTC's date registers support
  #[test]
  fn test_parts_unix_timestamp_and_weekday() {
    use chrono::{Datelike, NaiveDate};
    let mut date = NaiveDate::from_ymd_opt(2000, 1, 1).unwrap();
    let last = NaiveDate::from_ymd_opt(2099, 12, 31).unwrap();
    while date <= last {
      let parts = DateTimeParts {
        year: date.year(), month: date.month() as u8, day: date.day() as u8,
        hour: 23, minute: 59, second: 58,
      };
      let expected = date.and_hms_opt(23, 59, 58).unwrap().and_utc().timestamp();
      assert_eq!(parts.unix_timestamp(), expected, "{date}");
      assert_eq!(parts.weekday(), date.weekday().num_days_from_monday() as u8, "{date}");
      date = date.succ_opt().unwrap();
    }
  }

  #[test]
  fn test_get_alarm_parts() {
    let expectations = [
      I2cTrans::write_read(RV3028_ADDRESS, vec![REG_CONTROL2], vec![0]),
      I2cTrans::write_read(RV3028_ADDRESS, vec![REG_WEEKDAY_DATE_ALARM], vec![0x82]),
      I2cTrans::write_read(RV3028_ADDRESS, vec![REG_HOURS_ALARM], vec![0x13]),
      I2cTrans::write_read(RV3028_ADDRESS, vec![REG_MINUTES_ALARM], vec![0x45]),
      I2cTrans::write_read(RV3028_ADDRESS, vec![REG_CONTROL1], vec![0x00]),
    ];
    let mut rv3028 = RV3028::new(I2cMock::new(&expectations));
    assert_eq!(rv3028.get_alarm_parts().unwrap(), AlarmParts {
      day: 2, hour: 13, minute: 45, weekday_alarm: true,
      match_day: false, match_hour: true, match_minute: true,
    });
    rv3028.i2c.done();
  }

  #[test]
  fn test_get_event_count_and_parts() {
    let expectations = [
      // count, seconds, minutes, hours, date, month, year
      I2cTrans::write_read(RV3028_ADDRESS, vec![REG_CONTROL2], vec![0]),
      I2cTrans::write_read(RV3028_ADDRESS, vec![REG_COUNT_EVENTS_TS],
                           vec![3, 0x30, 0x45, 0x13, 0x15, 0x03, 0x24]),
    ];
    let mut rv3028 = RV3028::new(I2cMock::new(&expectations));
    assert_eq!(rv3028.get_event_count_and_parts().unwrap(), (3, Some(TEST_PARTS)));
    rv3028.i2c.done();
  }

  #[test]
  fn counter_retry_limit_and_last_read_success() {
    for values in [[1u32, 2, 3, 4], [1, 2, 3, 3]] {
      let expectations = values.map(|v|
        I2cTrans::write_read(RV3028_ADDRESS, vec![REG_UNIX_TIME_0], v.to_le_bytes().to_vec()));
      let mut rtc = RV3028::new(I2cMock::new(&expectations));
      let expected = if values[3] == 3 { Ok(3) } else { Err(Error::InconsistentRead) };
      assert_eq!(rtc.get_unix_time_blocking(), expected);
      rtc.i2c.done();
    }
  }

  #[test]
  fn bus_errors_are_preserved() {
    use embedded_hal::i2c::ErrorKind;
    let expectations = [I2cTrans::write_read(RV3028_ADDRESS, vec![REG_UNIX_TIME_0], vec![0; 4])
      .with_error(ErrorKind::Bus)];
    let mut rtc = RV3028::new(I2cMock::new(&expectations));
    assert_eq!(rtc.get_unix_time_blocking(), Err(Error::I2c(ErrorKind::Bus)));
    rtc.i2c.done();
    let expectations = [I2cTrans::write(0x70, vec![1]).with_error(ErrorKind::ArbitrationLoss)];
    let mut rtc = RV3028::new_with_mux(I2cMock::new(&expectations), 0x70, 1);
    assert_eq!(rtc.set_datetime_parts(&TEST_PARTS), Err(Error::I2c(ErrorKind::ArbitrationLoss)));
    rtc.i2c.done();
  }

  #[test]
  fn invalid_inputs_do_not_access_bus() {
    let mut rtc = RV3028::new_with_mux(I2cMock::new(&[]), 0x70, 1);
    for dt in [
      DateTimeParts { year: 1999, ..TEST_PARTS },
      DateTimeParts { year: 2100, ..TEST_PARTS },
      DateTimeParts { month: 0, ..TEST_PARTS },
      DateTimeParts { month: 4, day: 31, ..TEST_PARTS },
      DateTimeParts { year: 2023, month: 2, day: 29, ..TEST_PARTS },
      DateTimeParts { day: 0, ..TEST_PARTS },
      DateTimeParts { hour: 24, ..TEST_PARTS },
      DateTimeParts { minute: 60, ..TEST_PARTS },
      DateTimeParts { second: 60, ..TEST_PARTS },
    ] {
      assert_eq!(rtc.set_datetime_parts(&dt), Err(Error::InvalidData));
    }
    let alarm = AlarmParts { day: 1, hour: 0, minute: 0, weekday_alarm: false,
      match_day: true, match_hour: true, match_minute: true };
    for invalid in [AlarmParts { day: 0, ..alarm }, AlarmParts { hour: 24, ..alarm },
      AlarmParts { minute: 60, ..alarm }, AlarmParts { day: 7, weekday_alarm: true, ..alarm }] {
      assert_eq!(rtc.set_alarm_parts(&invalid), Err(Error::InvalidData));
    }
    rtc.i2c.done();
  }

  #[test]
  fn invalid_event_data_returns_error() {
    // Invalid BCD which would otherwise decode to a valid 10 seconds,
    // and a correctly encoded but impossible February 30.
    for bytes in [vec![1, 0x0A, 0, 0, 1, 1, 0x24], vec![1, 0, 0, 0, 0x30, 2, 0x24]] {
      let expectations = [
        I2cTrans::write_read(RV3028_ADDRESS, vec![REG_CONTROL2], vec![0]),
        I2cTrans::write_read(RV3028_ADDRESS, vec![REG_COUNT_EVENTS_TS], bytes),
      ];
      let mut rtc = RV3028::new(I2cMock::new(&expectations));
      assert_eq!(rtc.get_event_count_and_parts(), Err(Error::InvalidData));
      rtc.i2c.done();
    }
  }

  #[test]
  fn invalid_alarm_fields_are_only_ignored_when_disabled() {
    for disabled in [false, true] {
      let flag = if disabled { 0x80 } else { 0 };
      let expectations = [
        I2cTrans::write_read(RV3028_ADDRESS, vec![REG_CONTROL2], vec![0]),
        I2cTrans::write_read(RV3028_ADDRESS, vec![REG_WEEKDAY_DATE_ALARM], vec![flag]),
        I2cTrans::write_read(RV3028_ADDRESS, vec![REG_HOURS_ALARM], vec![flag | 0x24]),
        I2cTrans::write_read(RV3028_ADDRESS, vec![REG_MINUTES_ALARM], vec![flag | 0x6A]),
        I2cTrans::write_read(RV3028_ADDRESS, vec![REG_CONTROL1], vec![0x20]),
      ];
      let mut rtc = RV3028::new(I2cMock::new(&expectations));
      let result = rtc.get_alarm_parts();
      if disabled {
        assert_eq!(result, Ok(AlarmParts { day: 1, hour: 0, minute: 0, weekday_alarm: false,
          match_day: false, match_hour: false, match_minute: false }));
      } else { assert_eq!(result, Err(Error::InvalidData)); }
      rtc.i2c.done();
    }
  }


  #[test]
  fn user_ram_reads_exact_requested_register_range() {
    for (offset, values) in [(0, vec![0xA1]), (1, vec![0xB2]), (0, vec![0xA1, 0xB2])] {
      let expectations = [
        I2cTrans::write_read(RV3028_ADDRESS, vec![0x1F + offset], values.clone()),
      ];
      let mut rtc = RV3028::new(I2cMock::new(&expectations));
      let mut buffer = [0xCC; 2];
      rtc.read_user_ram(offset, &mut buffer[..values.len()]).unwrap();
      assert_eq!(&buffer[..values.len()], &values);
      if values.len() == 1 {
        assert_eq!(buffer[1], 0xCC);
      }
      rtc.i2c.done();
    }
  }

  #[test]
  fn user_ram_writes_exact_requested_bytes_in_one_transfer() {
    for (offset, data, packet) in [
      (0, vec![0xA1], vec![0x1F, 0xA1]),
      (1, vec![0xB2], vec![0x20, 0xB2]),
      (0, vec![0xA1, 0xB2], vec![0x1F, 0xA1, 0xB2]),
    ] {
      let expectations = [I2cTrans::write(RV3028_ADDRESS, packet)];
      let mut rtc = RV3028::new(I2cMock::new(&expectations));
      rtc.write_user_ram(offset, &data).unwrap();
      rtc.i2c.done();
    }
  }

  #[test]
  fn user_ram_reuses_the_same_buffer_for_read_and_partial_write() {
    let expectations = [
      I2cTrans::write_read(RV3028_ADDRESS, vec![0x1F], vec![0xA1, 0xB2]),
      I2cTrans::write(RV3028_ADDRESS, vec![0x20, 0xC3]),
    ];
    let mut rtc = RV3028::new(I2cMock::new(&expectations));
    let mut bytes = [0; 2];
    rtc.read_user_ram(0, &mut bytes).unwrap();
    bytes[1] = 0xC3;
    rtc.write_user_ram(1, &bytes[1..]).unwrap();
    assert_eq!(bytes, [0xA1, 0xC3]);
    rtc.i2c.done();
  }

  #[test]
  fn user_ram_empty_ranges_do_not_access_bus_or_mux() {
    let mut rtc = RV3028::new_with_mux(I2cMock::new(&[]), 0x70, 1);
    for offset in 0..=2 {
      rtc.read_user_ram(offset, &mut []).unwrap();
      rtc.write_user_ram(offset, &[]).unwrap();
    }
    rtc.i2c.done();
  }

  #[test]
  fn user_ram_invalid_ranges_do_not_access_bus_or_modify_buffer() {
    let mut rtc = RV3028::new_with_mux(I2cMock::new(&[]), 0x70, 1);
    for (offset, len) in [(0, 3), (1, 2), (2, 1), (3, 0), (u8::MAX, 0), (u8::MAX, 1)] {
      let mut bytes = [0xA5; 3];
      assert_eq!(rtc.read_user_ram(offset, &mut bytes[..len]), Err(Error::InvalidData));
      assert_eq!(bytes, [0xA5; 3]);
      assert_eq!(rtc.write_user_ram(offset, &bytes[..len]), Err(Error::InvalidData));
    }
    rtc.i2c.done();
  }

  #[test]
  fn user_ram_selects_mux_before_each_nonempty_access() {
    let expectations = [
      I2cTrans::write(0x70, vec![4]),
      I2cTrans::write_read(RV3028_ADDRESS, vec![0x20], vec![0xA5]),
      I2cTrans::write(0x70, vec![4]),
      I2cTrans::write(RV3028_ADDRESS, vec![0x1F, 0xB6]),
    ];
    let mut rtc = RV3028::new_with_mux(I2cMock::new(&expectations), 0x70, 4);
    let mut byte = [0];
    rtc.read_user_ram(1, &mut byte).unwrap();
    assert_eq!(byte, [0xA5]);
    rtc.write_user_ram(0, &[0xB6]).unwrap();
    rtc.i2c.done();
  }

  #[test]
  fn user_ram_preserves_read_write_and_mux_errors_without_retries() {
    use embedded_hal::i2c::ErrorKind;
    let expectations = [
      I2cTrans::write_read(RV3028_ADDRESS, vec![0x1F], vec![0])
        .with_error(ErrorKind::Bus),
      I2cTrans::write(RV3028_ADDRESS, vec![0x20, 0xA5])
        .with_error(ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Data)),
    ];
    let mut rtc = RV3028::new(I2cMock::new(&expectations));
    assert_eq!(rtc.read_user_ram(0, &mut [0]), Err(Error::I2c(ErrorKind::Bus)));
    assert_eq!(rtc.write_user_ram(1, &[0xA5]),
      Err(Error::I2c(ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Data))));
    rtc.i2c.done();

    let expectations = [
      I2cTrans::write(0x70, vec![1]).with_error(ErrorKind::ArbitrationLoss),
      I2cTrans::write(0x70, vec![1]).with_error(ErrorKind::ArbitrationLoss),
    ];
    let mut rtc = RV3028::new_with_mux(I2cMock::new(&expectations), 0x70, 1);
    assert_eq!(rtc.read_user_ram(0, &mut [0]), Err(Error::I2c(ErrorKind::ArbitrationLoss)));
    assert_eq!(rtc.write_user_ram(0, &[0xA5]), Err(Error::I2c(ErrorKind::ArbitrationLoss)));
    rtc.i2c.done();
  }
}

#[cfg(test)]
mod status_tests;

#[cfg(test)]
mod calendar_tests;
