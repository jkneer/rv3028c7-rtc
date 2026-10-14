use super::*;
use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
use std::vec;
use std::vec::Vec;

#[derive(Default)]
struct Delay {
  calls: Vec<u32>,
}
impl DelayNs for Delay {
  fn delay_ns(&mut self, ns: u32) {
    self.calls.push(ns);
  }
}

fn read(reg: u8, value: u8) -> T {
  T::write_read(0x52, vec![reg], vec![value])
}
fn write(reg: u8, value: u8) -> T {
  T::write(0x52, vec![reg, value])
}
fn begin(control: u8) -> Vec<T> {
  let mut result = vec![read(0x0F, control)];
  if control & 0x08 == 0 {
    result.push(write(0x0F, control | 0x08));
  }
  result
}
fn finish(expected: &mut Vec<T>, current: u8, restored: u8) {
  expected.extend([read(0x0E, 0), read(0x0F, current), write(0x0F, restored)]);
}
fn byte_read(expected: &mut Vec<T>, address: u8, value: u8) {
  // Literal bus sequence from Application Manual 4.6.6, including idle checks.
  expected.extend([
    read(0x0E, 0),
    write(0x25, address),
    read(0x0E, 0),
    write(0x27, 0),
    write(0x27, 0x22),
    read(0x0E, 0),
    read(0x26, value),
  ]);
}
fn byte_write(expected: &mut Vec<T>, address: u8, value: u8) {
  expected.extend([
    read(0x0E, 0),
    write(0x25, address),
    write(0x26, value),
    read(0x0E, 0),
    write(0x27, 0),
    write(0x27, 0x21),
    read(0x0E, 0),
  ]);
}
fn command(expected: &mut Vec<T>, cmd: u8) {
  expected.extend([
    read(0x0E, 0),
    write(0x27, 0),
    write(0x27, cmd),
    read(0x0E, 0),
  ]);
}
fn invalid() -> EepromError<ErrorKind> {
  EepromError::Operation(Failure::Driver(Error::InvalidData))
}

#[test]
fn user_reads_addresses_in_order_and_restores_only_eerd() {
  let mut expected = begin(0x21);
  byte_read(&mut expected, 0x29, 7);
  byte_read(&mut expected, 0x2A, 8);
  // The timer changed a different Control 1 bit during the operation.
  finish(&mut expected, 0x29 | 0x10, 0x21 | 0x10);
  let mut rtc = RV3028::new(Mock::new(&expected));
  let mut delay = Delay::default();
  let mut bytes = [0xFF; 3];
  rtc
    .read_user_eeprom(0x29, &mut bytes[..2], &mut delay)
    .unwrap();
  assert_eq!(bytes, [7, 8, 0xFF]);
  assert_eq!(delay.calls, [1_000_000, 1_000_000]);
  rtc.i2c.done();
}

#[test]
fn user_writes_only_changed_bytes_then_verifies_entire_range() {
  let mut expected = begin(0);
  byte_read(&mut expected, 0, 7); // unchanged
  byte_read(&mut expected, 1, 8);
  byte_write(&mut expected, 1, 9);
  byte_read(&mut expected, 0, 7);
  byte_read(&mut expected, 1, 9);
  finish(&mut expected, 8, 0);
  let mut rtc = RV3028::new(Mock::new(&expected));
  let mut delay = Delay::default();
  rtc.write_user_eeprom(0, &[7, 9], &mut delay).unwrap();
  assert_eq!(
    delay.calls,
    [1_000_000, 1_000_000, 10_000_000, 1_000_000, 1_000_000]
  );
  rtc.i2c.done();
}

#[test]
fn unchanged_single_byte_is_not_programmed() {
  let mut expected = begin(8); // EERD was already set: leave it set.
  byte_read(&mut expected, 0x37, 0x3C);
  byte_read(&mut expected, 0x37, 0x3C);
  let mut rtc = RV3028::new(Mock::new(&expected));
  let mut delay = Delay::default();
  rtc.write_eeprom(0x37, 0x3C, &mut delay).unwrap();
  assert_eq!(delay.calls, [1_000_000, 1_000_000]);
  rtc.i2c.done();
}

#[test]
fn ranges_are_checked_before_mux_or_delay_and_empty_ranges_are_noops() {
  let mut rtc = RV3028::new_with_mux(Mock::new(&[]), 0x70, 1);
  let mut delay = Delay::default();
  for offset in 0..=0x2B {
    rtc.read_user_eeprom(offset, &mut [], &mut delay).unwrap();
    rtc.write_user_eeprom(offset, &[], &mut delay).unwrap();
  }
  for address in 0x30..=0x38 {
    rtc
      .write_configuration_eeprom(address, &[], &mut delay)
      .unwrap();
  }
  for (offset, len) in [(0x2A, 2), (0x2B, 1), (0x2C, 0), (255, 0), (0, 44)] {
    let mut bytes = [0xAA; 44];
    assert_eq!(
      rtc.read_user_eeprom(offset, &mut bytes[..len], &mut delay),
      Err(invalid())
    );
    assert_eq!(bytes, [0xAA; 44]);
    assert_eq!(
      rtc.write_user_eeprom(offset, &bytes[..len], &mut delay),
      Err(invalid())
    );
  }
  for (address, len) in [(0x2F, 0), (0x37, 2), (0x38, 1), (0x39, 0), (255, 0)] {
    assert_eq!(
      rtc.write_configuration_eeprom(address, &[0; 2][..len], &mut delay),
      Err(invalid())
    );
  }
  for address in 0..=255 {
    if address <= 0x2A || (0x30..=0x37).contains(&address) {
      continue;
    }
    assert_eq!(rtc.read_eeprom(address, &mut delay), Err(invalid()));
    assert_eq!(rtc.write_eeprom(address, 1, &mut delay), Err(invalid()));
  }
  assert!(delay.calls.is_empty());
  rtc.i2c.done();
}

#[test]
fn full_user_region_and_configuration_endpoints_are_accessible() {
  let mut expected = begin(8);
  for address in 0..=0x2A {
    byte_read(&mut expected, address, address);
  }
  byte_read(&mut expected, 0x30, 0xAA);
  byte_read(&mut expected, 0x37, 0xBB);
  // Each call reads Control 1; full-region read has only the first begin.
  expected.insert(1 + 43 * 7, read(0x0F, 8));
  expected.insert(1 + 43 * 7 + 8, read(0x0F, 8));
  let mut rtc = RV3028::new(Mock::new(&expected));
  let mut delay = Delay::default();
  let mut bytes = [0; 43];
  rtc.read_user_eeprom(0, &mut bytes, &mut delay).unwrap();
  assert_eq!(bytes, core::array::from_fn(|i| i as u8));
  assert_eq!(rtc.read_eeprom(0x30, &mut delay), Ok(0xAA));
  assert_eq!(rtc.read_eeprom(0x37, &mut delay), Ok(0xBB));
  rtc.i2c.done();
}

#[test]
fn configuration_edits_are_under_eerd_and_update_precedes_readback() {
  let mut expected = begin(0x20);
  expected.extend([
    read(0x0E, 0),
    write(0x35, 0x40),
    write(0x36, 0xAA),
    write(0x37, 0x3C),
  ]);
  command(&mut expected, 0x11);
  byte_read(&mut expected, 0x35, 0x40);
  byte_read(&mut expected, 0x36, 0xAA);
  byte_read(&mut expected, 0x37, 0x3C);
  finish(&mut expected, 0x28, 0x20);
  let mut rtc = RV3028::new(Mock::new(&expected));
  let mut delay = Delay::default();
  rtc
    .write_configuration_eeprom(0x35, &[0x40, 0xAA, 0x3C], &mut delay)
    .unwrap();
  assert_eq!(delay.calls, [10_000_000, 1_000_000, 1_000_000, 1_000_000]);
  rtc.i2c.done();
}

#[test]
fn update_and_refresh_use_distinct_commands_and_required_delays() {
  for (cmd, settle) in [(0x11, 10_000_000), (0x12, 1_000_000)] {
    let mut expected = begin(0);
    command(&mut expected, cmd);
    finish(&mut expected, 8, 0);
    let mut rtc = RV3028::new(Mock::new(&expected));
    let mut delay = Delay::default();
    if cmd == 0x11 {
      rtc.update_configuration_eeprom(&mut delay).unwrap();
    } else {
      rtc.refresh_configuration(&mut delay).unwrap();
    }
    assert_eq!(delay.calls, [settle]);
    rtc.i2c.done();
  }
}

#[test]
fn busy_waits_precede_address_changes_and_completion_read() {
  let mut expected = begin(0);
  expected.extend([
    read(0x0E, 0x80),
    read(0x0E, 0x80),
    read(0x0E, 0),
    write(0x25, 0),
    read(0x0E, 0),
    write(0x27, 0),
    write(0x27, 0x22),
    read(0x0E, 0x80),
    read(0x0E, 0),
    read(0x26, 0xAA),
  ]);
  finish(&mut expected, 8, 0);
  let mut rtc = RV3028::new(Mock::new(&expected));
  let mut delay = Delay::default();
  assert_eq!(rtc.read_eeprom(0, &mut delay), Ok(0xAA));
  assert_eq!(delay.calls, [1_000_000; 4]);
  rtc.i2c.done();
}

#[test]
fn busy_timeout_is_bounded_before_and_after_commands_and_keeps_refresh_disabled() {
  for after_command in [false, true] {
    let mut expected = begin(0);
    if after_command {
      expected.extend([read(0x0E, 0), write(0x27, 0), write(0x27, 0x11)]);
    }
    expected.extend((0..201).map(|_| read(0x0E, 0x80)));
    expected.push(read(0x0E, 0x80)); // no Control 1 restore while still busy
    let mut rtc = RV3028::new(Mock::new(&expected));
    let mut delay = Delay::default();
    assert_eq!(
      rtc.update_configuration_eeprom(&mut delay),
      Err(EepromError::RestoreBusy { operation: Some(Failure::BusyTimeout) })
    );
    assert_eq!(delay.calls.len(), 200 + usize::from(after_command));
    assert_eq!(
      delay.calls.iter().sum::<u32>(),
      200_000_000 + if after_command { 10_000_000 } else { 0 }
    );
    rtc.i2c.done();
  }
}

#[test]
fn verification_error_reports_address_values_and_still_restores() {
  let mut expected = begin(0);
  byte_read(&mut expected, 0x2A, 0);
  byte_write(&mut expected, 0x2A, 0xA5);
  byte_read(&mut expected, 0x2A, 0xFF);
  finish(&mut expected, 8, 0);
  let mut rtc = RV3028::new(Mock::new(&expected));
  let mut delay = Delay::default();
  assert_eq!(
    rtc.write_user_eeprom(0x2A, &[0xA5], &mut delay),
    Err(EepromError::Operation(Failure::VerificationFailed {
      address: 0x2A,
      expected: 0xA5,
      actual: 0xFF,
    }))
  );
  rtc.i2c.done();
}

#[test]
fn each_read_transaction_failure_preserves_error_and_attempts_restore() {
  let mut baseline = begin(0);
  byte_read(&mut baseline, 0, 0xAA);
  let restore_index = baseline.len();
  finish(&mut baseline, 8, 0);
  for index in 0..baseline.len() {
    let mut expected = baseline[..=index].to_vec();
    expected[index] = expected[index].clone().with_error(ErrorKind::Bus);
    if index > 0 && index < restore_index {
      finish(&mut expected, 8, 0);
    }
    let mut rtc = RV3028::new(Mock::new(&expected));
    let mut delay = Delay::default();
    let result = rtc.read_eeprom(0, &mut delay);
    let bus = Error::I2c(ErrorKind::Bus);
    let error = if index >= restore_index {
      EepromError::Restore {
        operation: None,
        error: bus,
      }
    } else {
      EepromError::Operation(Failure::Driver(bus))
    };
    assert_eq!(result, Err(error), "transaction {index}");
    rtc.i2c.done();
  }
}

#[test]
fn operation_and_restore_failures_are_both_retained() {
  let mut expected = begin(0);
  expected.extend([
    read(0x0E, 0).with_error(ErrorKind::Bus),
    read(0x0E, 0),
    read(0x0F, 8).with_error(ErrorKind::ArbitrationLoss),
  ]);
  let mut rtc = RV3028::new(Mock::new(&expected));
  let mut delay = Delay::default();
  assert_eq!(
    rtc.read_eeprom(0, &mut delay),
    Err(EepromError::Restore {
      operation: Some(Failure::Driver(Error::I2c(ErrorKind::Bus))),
      error: Error::I2c(ErrorKind::ArbitrationLoss),
    })
  );
  rtc.i2c.done();
}

#[test]
fn failed_idle_confirmation_during_restore_never_writes_control_and_retains_both_errors() {
  let mut expected = begin(0);
  expected.extend([
    read(0x0E, 0).with_error(ErrorKind::Bus),
    read(0x0E, 0).with_error(ErrorKind::ArbitrationLoss),
  ]);
  let mut rtc = RV3028::new(Mock::new(&expected));
  assert_eq!(
    rtc.read_eeprom(0, &mut Delay::default()),
    Err(EepromError::Restore {
      operation: Some(Failure::Driver(Error::I2c(ErrorKind::Bus))),
      error: Error::I2c(ErrorKind::ArbitrationLoss),
    }),
  );
  rtc.i2c.done();
}

#[test]
fn partial_reads_leave_only_successful_prefix_and_mux_is_selected_once() {
  let mut expected = vec![T::write(0x70, vec![4])];
  expected.extend(begin(0));
  byte_read(&mut expected, 0, 7);
  expected.push(read(0x0E, 0).with_error(ErrorKind::Bus));
  finish(&mut expected, 8, 0);
  let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 4);
  let mut delay = Delay::default();
  let mut bytes = [0xAA; 2];
  assert_eq!(
    rtc.read_user_eeprom(0, &mut bytes, &mut delay),
    Err(EepromError::Operation(Failure::Driver(Error::I2c(
      ErrorKind::Bus
    ))))
  );
  assert_eq!(bytes, [7, 0xAA]);
  rtc.i2c.done();
}

#[test]
fn mux_failure_has_no_other_bus_or_delay_calls() {
  let expected =
    [T::write(0x70, vec![4]).with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))];
  let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 4);
  let mut delay = Delay::default();
  assert_eq!(
    rtc.write_user_eeprom(0, &[1], &mut delay),
    Err(EepromError::Operation(Failure::Driver(Error::I2c(
      ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
    ))))
  );
  assert!(delay.calls.is_empty());
  rtc.i2c.done();
}

#[test]
fn each_write_transaction_failure_stops_without_retry_and_restores() {
  let mut baseline = begin(0);
  byte_read(&mut baseline, 0, 0);
  byte_write(&mut baseline, 0, 0xAA);
  byte_read(&mut baseline, 0, 0xAA);
  let restore_index = baseline.len();
  finish(&mut baseline, 8, 0);
  for index in 0..baseline.len() {
    let mut expected = baseline[..=index].to_vec();
    expected[index] = expected[index]
      .clone()
      .with_error(ErrorKind::ArbitrationLoss);
    if index > 0 && index < restore_index {
      finish(&mut expected, 8, 0);
    }
    let mut rtc = RV3028::new(Mock::new(&expected));
    let mut delay = Delay::default();
    let result = rtc.write_user_eeprom(0, &[0xAA], &mut delay);
    let bus = Error::I2c(ErrorKind::ArbitrationLoss);
    let error = if index >= restore_index {
      EepromError::Restore {
        operation: None,
        error: bus,
      }
    } else {
      EepromError::Operation(Failure::Driver(bus))
    };
    assert_eq!(result, Err(error), "transaction {index}");
    // The write command may have reached the chip despite the HAL error.
    if index == 14 {
      assert_eq!(delay.calls, [1_000_000, 10_000_000]);
    }
    rtc.i2c.done();
  }
}

#[test]
fn required_delays_occur_after_commands_and_before_status_reads() {
  use embedded_hal::i2c::{ErrorType, Operation, SevenBitAddress};
  use std::{cell::Cell, rc::Rc};
  struct Bus {
    mock: Mock,
    count: Rc<Cell<usize>>,
  }
  impl ErrorType for Bus {
    type Error = ErrorKind;
  }
  impl I2c<SevenBitAddress> for Bus {
    fn write(&mut self, address: u8, bytes: &[u8]) -> core::result::Result<(), ErrorKind> {
      self.count.set(self.count.get() + 1);
      self.mock.write(address, bytes)
    }
    fn write_read(
      &mut self,
      address: u8,
      bytes: &[u8],
      output: &mut [u8],
    ) -> core::result::Result<(), ErrorKind> {
      self.count.set(self.count.get() + 1);
      self.mock.write_read(address, bytes, output)
    }
    fn transaction(
      &mut self,
      address: u8,
      operations: &mut [Operation<'_>],
    ) -> core::result::Result<(), ErrorKind> {
      self.count.set(self.count.get() + 1);
      self.mock.transaction(address, operations)
    }
  }
  struct GuardDelay {
    count: Rc<Cell<usize>>,
    next: usize,
  }
  impl DelayNs for GuardDelay {
    fn delay_ns(&mut self, ns: u32) {
      let golden = [(7, 1_000_000), (15, 10_000_000), (21, 1_000_000)];
      assert_eq!((self.count.get(), ns), golden[self.next]);
      self.next += 1;
    }
  }
  let mut expected = begin(0);
  byte_read(&mut expected, 0, 0);
  byte_write(&mut expected, 0, 0xAA);
  byte_read(&mut expected, 0, 0xAA);
  finish(&mut expected, 8, 0);
  let count = Rc::new(Cell::new(0));
  let mut rtc = RV3028::new(Bus {
    mock: Mock::new(&expected),
    count: count.clone(),
  });
  let mut delay = GuardDelay { count, next: 0 };
  rtc.write_user_eeprom(0, &[0xAA], &mut delay).unwrap();
  assert_eq!(delay.next, 3);
  rtc.i2c.mock.done();
}

#[test]
fn partial_configuration_edit_failure_does_not_issue_update() {
  let mut expected = begin(0);
  expected.extend([
    read(0x0E, 0),
    write(0x35, 0x40),
    write(0x36, 0xAA).with_error(ErrorKind::Bus),
  ]);
  finish(&mut expected, 8, 0);
  let mut rtc = RV3028::new(Mock::new(&expected));
  let mut delay = Delay::default();
  assert_eq!(
    rtc.write_configuration_eeprom(0x35, &[0x40, 0xAA, 0x3C], &mut delay),
    Err(EepromError::Operation(Failure::Driver(Error::I2c(
      ErrorKind::Bus
    ))))
  );
  assert!(delay.calls.is_empty());
  rtc.i2c.done();
}
