//! Regression models for asynchronous hardware behavior between I2C transfers.
use super::*;
use embedded_hal::i2c::{ErrorKind, ErrorType, Operation};

struct CalendarBus {
  registers: [u8; 64],
  prescaler_reset: bool,
  pending_tick: bool,
}

impl ErrorType for CalendarBus {
  type Error = ErrorKind;
}

impl I2c for CalendarBus {
  fn transaction(
    &mut self,
    address: u8,
    operations: &mut [Operation<'_>],
  ) -> Result<(), ErrorKind> {
    assert_eq!(address, 0x52);
    // Model an old prescaler edge between the Unix write and the next access.
    // Writing Seconds beforehand resets this phase, preventing that edge.
    if self.pending_tick {
      let unix = u32::from_le_bytes(self.registers[0x1B..0x1F].try_into().unwrap());
      self.registers[0x1B..0x1F].copy_from_slice(&unix.wrapping_add(1).to_le_bytes());
      self.pending_tick = false;
    }
    let mut pointer = 0;
    for operation in operations {
      match operation {
        Operation::Write(bytes) => {
          pointer = usize::from(bytes[0]);
          if bytes.len() > 1 {
            let count = bytes.len() - 1;
            self.registers[pointer..pointer + count].copy_from_slice(&bytes[1..]);
            if pointer == 0 {
              self.prescaler_reset = true;
            }
            if pointer == 0x1B && !self.prescaler_reset {
              self.pending_tick = true;
            }
            pointer += count;
          }
        }
        Operation::Read(bytes) => {
          bytes.copy_from_slice(&self.registers[pointer..pointer + bytes.len()]);
          pointer += bytes.len();
        }
      }
    }
    Ok(())
  }
}

#[test]
fn datetime_set_resets_old_tick_phase_before_programming_unix_counter() {
  let mut rtc = RV3028::new(CalendarBus {
    registers: [0; 64],
    prescaler_reset: false,
    pending_tick: false,
  });
  let parts = DateTimeParts {
    year: 2024,
    month: 3,
    day: 15,
    hour: 13,
    minute: 45,
    second: 30,
  };
  rtc.set_datetime_parts(&parts).unwrap();
  assert_eq!(rtc.get_datetime_parts(), Ok(parts));
  assert_eq!(rtc.get_unix_time(), Ok(1_710_510_330));
}

struct AlarmBus {
  registers: [u8; 64],
  interrupt_seen: bool,
}

impl ErrorType for AlarmBus {
  type Error = ErrorKind;
}

impl I2c for AlarmBus {
  fn transaction(
    &mut self,
    address: u8,
    operations: &mut [Operation<'_>],
  ) -> Result<(), ErrorKind> {
    assert_eq!(address, 0x52);
    let mut pointer = 0;
    for operation in operations {
      match operation {
        Operation::Write(bytes) => {
          pointer = usize::from(bytes[0]);
          for value in &bytes[1..] {
            if pointer == 0x0E {
              self.registers[pointer] &= *value | 0x80; // W0C, EEbusy read-only
            } else {
              self.registers[pointer] = *value;
            }
            pointer += 1;
          }
          if matches!(bytes[0], 0x07..=0x09) && bytes.len() > 1 {
            let matched = [
              (self.registers[7], self.registers[1]),
              (self.registers[8], self.registers[2]),
              (self.registers[9], self.registers[4]),
            ]
            .iter()
            .all(|(alarm, current)| alarm & 0x80 != 0 || alarm == current);
            if matched {
              self.registers[0x0E] |= 4;
              if self.registers[0x10] & 8 != 0 {
                self.interrupt_seen = true;
                if self.registers[0x10] & 0x40 != 0 && self.registers[0x12] & 4 != 0 {
                  self.registers[0x0E] |= 0x40;
                }
              }
            }
          }
        }
        Operation::Read(bytes) => {
          bytes.copy_from_slice(&self.registers[pointer..pointer + bytes.len()]);
          pointer += bytes.len();
        }
      }
    }
    Ok(())
  }
}

#[test]
fn alarm_rewrite_cannot_emit_an_interrupt_or_latch_clockout_for_a_partial_match() {
  let mut registers = [0; 64];
  // Current 12:45, old alarm 12:30, new alarm 13:45 on the same date.
  registers[1] = 0x45;
  registers[2] = 0x12;
  registers[4] = 0x15;
  registers[7] = 0x30;
  registers[8] = 0x12;
  registers[9] = 0x15;
  registers[0x0F] = 0x20;
  registers[0x10] = 0x48; // AIE and CLKIE
  registers[0x12] = 4; // CAIE
  let mut rtc = RV3028::new(AlarmBus {
    registers,
    interrupt_seen: false,
  });
  rtc
    .set_alarm_parts(&AlarmParts {
      day: 15,
      hour: 13,
      minute: 45,
      weekday_alarm: false,
      match_day: true,
      match_hour: true,
      match_minute: true,
    })
    .unwrap();
  assert!(!rtc.i2c.interrupt_seen);
  let status = rtc.get_status().unwrap();
  assert!(!status.flags.alarm());
  assert!(!status.flags.clock_output());
  assert_eq!(status.control2, 0x48);
}

struct EepromBus {
  registers: [u8; 64],
  fail_completion_read: bool,
  refresh_enabled_while_busy: bool,
}

impl ErrorType for EepromBus {
  type Error = ErrorKind;
}

impl I2c for EepromBus {
  fn transaction(
    &mut self,
    address: u8,
    operations: &mut [Operation<'_>],
  ) -> Result<(), ErrorKind> {
    assert_eq!(address, 0x52);
    let mut pointer = 0;
    for operation in operations {
      match operation {
        Operation::Write(bytes) => {
          pointer = usize::from(bytes[0]);
          for value in &bytes[1..] {
            self.registers[pointer] = *value;
            if pointer == 0x27 && *value == 0x11 {
              self.registers[0x0E] = 0x80;
              self.fail_completion_read = true;
            }
            if pointer == 0x0F && value & 8 == 0 && self.registers[0x0E] & 0x80 != 0 {
              self.refresh_enabled_while_busy = true;
            }
            pointer += 1;
          }
        }
        Operation::Read(bytes) => {
          if pointer == 0x0E && self.fail_completion_read {
            self.fail_completion_read = false;
            return Err(ErrorKind::ArbitrationLoss);
          }
          bytes.copy_from_slice(&self.registers[pointer..pointer + bytes.len()]);
          pointer += bytes.len();
        }
      }
    }
    Ok(())
  }
}

#[test]
fn failed_eeprom_completion_read_cannot_enable_refresh_during_programming() {
  struct Delay;
  impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, _: u32) {}
  }
  let mut rtc = RV3028::new(EepromBus {
    registers: [0; 64],
    fail_completion_read: false,
    refresh_enabled_while_busy: false,
  });
  assert_eq!(
    rtc.update_configuration_eeprom(&mut Delay),
    Err(EepromError::RestoreBusy {
      operation: Some(EepromErrorKind::Driver(Error::I2c(
        ErrorKind::ArbitrationLoss
      ))),
    }),
  );
  assert!(!rtc.i2c.refresh_enabled_while_busy);
  assert_eq!(rtc.i2c.registers[0x0F] & 8, 8);
}
