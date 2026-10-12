use super::*;
use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource, Operation};
use std::vec::Vec;

/// Model Status's write-zero-to-clear flags and read-only EEbusy. Inject events
/// just before an acknowledgement, so a stale register value loses those flags.
struct StatusBus {
  registers: [u8; 64],
  arriving: u8,
  status_reads: usize,
  acknowledgements: Vec<u8>,
  mux_selections: usize,
  fail_read: bool,
  fail_write: bool,
}

impl StatusBus {
  fn new(status: u8, arriving: u8) -> Self {
    let mut registers = [0; 64];
    registers[0x0E] = status;
    Self {
      registers,
      arriving,
      status_reads: 0,
      acknowledgements: Vec::new(),
      mux_selections: 0,
      fail_read: false,
      fail_write: false,
    }
  }
}

impl embedded_hal::i2c::ErrorType for StatusBus {
  type Error = ErrorKind;
}

impl I2c for StatusBus {
  fn write_read(&mut self, address: u8, prefix: &[u8], bytes: &mut [u8]) -> Result<(), ErrorKind> {
    assert_eq!(address, 0x52);
    assert_eq!(prefix.len(), 1);
    if prefix[0] == 0x0E {
      self.status_reads += 1;
      if self.fail_read {
        return Err(ErrorKind::Bus);
      }
    }
    let first = usize::from(prefix[0]);
    bytes.copy_from_slice(&self.registers[first..first + bytes.len()]);
    Ok(())
  }

  fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), ErrorKind> {
    if address == 0x70 {
      assert_eq!(bytes, [1]);
      self.mux_selections += 1;
      return Ok(());
    }
    assert_eq!(address, 0x52);
    if bytes[0] == 0x0E {
      assert_eq!(bytes.len(), 2);
      self.acknowledgements.push(bytes[1]);
      if self.fail_write {
        return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
      }
      self.registers[0x0E] |= self.arriving & 0x7F;
      let busy = self.registers[0x0E] & 0x80;
      self.registers[0x0E] = busy | (self.registers[0x0E] & bytes[1] & 0x7F);
    } else {
      let first = usize::from(bytes[0]);
      self.registers[first..first + bytes.len() - 1].copy_from_slice(&bytes[1..]);
    }
    Ok(())
  }

  fn transaction(&mut self, _: u8, _: &mut [Operation<'_>]) -> Result<(), ErrorKind> {
    panic!("unexpected transaction: the driver uses write and write_read");
  }
}

type TestRtc = RV3028<StatusBus>;
type Check = fn(&mut TestRtc) -> Result<bool, Error<ErrorKind>>;

#[test]
fn individual_checks_preserve_every_other_flag_arriving_before_acknowledgement() {
  let checks: [(u8, Check); 5] = [
    (0x01, TestRtc::check_and_clear_power_on_reset),
    (0x02, TestRtc::check_and_clear_ext_event),
    (0x04, TestRtc::check_and_clear_alarm),
    (0x08, TestRtc::check_and_clear_countdown),
    (0x20, TestRtc::check_and_clear_backup_event),
  ];
  for (flag, check) in checks {
    for arriving in 0..=0x7F {
      let mut rtc = RV3028::new(StatusBus::new(flag | 0x80, arriving));
      assert_eq!(check(&mut rtc), Ok(true));
      assert_eq!(rtc.i2c.registers[0x0E], 0x80 | (arriving & !flag));
      assert_eq!(rtc.i2c.acknowledgements, [0x7F & !flag]);
      assert_eq!(rtc.i2c.status_reads, 1);
    }
    let mut rtc = RV3028::new_with_mux(StatusBus::new(0x80, 0), 0x70, 1);
    assert_eq!(check(&mut rtc), Ok(false));
    assert_eq!(rtc.i2c.mux_selections, 1);
    assert_eq!(rtc.i2c.status_reads, 1);
    assert!(rtc.i2c.acknowledgements.is_empty());
  }
}

#[test]
fn all_flag_masks_preserve_unselected_flags_and_never_write_eebusy() {
  for mask in 0..=255 {
    let mut rtc = RV3028::new(StatusBus::new(0x80, 0x7F));
    rtc.clear_reg_bits_raw(REG_STATUS, mask).unwrap();
    assert_eq!(rtc.i2c.status_reads, 0);
    assert_eq!(rtc.i2c.acknowledgements, [0x7F & !mask]);
    assert_eq!(rtc.i2c.registers[0x0E], 0x80 | (0x7F & !mask));
  }
}

#[test]
fn clear_all_acknowledges_all_flags_in_one_write_and_leaves_busy_unchanged() {
  let mut rtc = RV3028::new_with_mux(StatusBus::new(0xFF, 0x7F), 0x70, 1);
  rtc.clear_all_status_flags().unwrap();
  assert_eq!(rtc.i2c.registers[0x0E], 0x80);
  assert_eq!(rtc.i2c.status_reads, 0);
  assert_eq!(rtc.i2c.acknowledgements, [0]);
  assert_eq!(rtc.i2c.mux_selections, 1);
}

#[test]
fn configuration_paths_use_targeted_acknowledgements() {
  let run = |expected: &[u8], operation: fn(&mut TestRtc)| {
    let mut rtc = RV3028::new(StatusBus::new(0x80, 0x7F));
    operation(&mut rtc);
    assert_eq!(rtc.i2c.status_reads, 0);
    assert_eq!(rtc.i2c.acknowledgements, expected);
    assert_eq!(rtc.i2c.registers[0x0E], 0x80 | *expected.last().unwrap());
  };
  run(&[0x6F], |rtc| {
    rtc.configure_periodic_time_update(false, false).unwrap()
  });
  run(&[0x7B, 0x7B], |rtc| {
    rtc
      .set_alarm_parts(&AlarmParts {
        day: 1,
        hour: 2,
        minute: 3,
        weekday_alarm: false,
        match_day: true,
        match_hour: true,
        match_minute: true,
      })
      .unwrap()
  });
  run(&[0x77], |rtc| {
    rtc
      .config_countdown_timer(&Duration::from_secs(1), false, false)
      .unwrap();
  });
  run(&[0x7D], |rtc| {
    rtc
      .config_ext_event_detection(false, false, 0, false)
      .unwrap()
  });
  run(&[0x5D], |rtc| {
    rtc
      .config_timestamp_logging(TS_EVENT_SOURCE_BSF, true, true)
      .unwrap()
  });
}

#[test]
fn normal_control_registers_still_preserve_unselected_bits() {
  let mut rtc = RV3028::new(StatusBus::new(0, 0));
  rtc.i2c.registers[0x0F] = 0xB5;
  rtc.clear_reg_bits_raw(REG_CONTROL1, 0x0C).unwrap();
  assert_eq!(rtc.i2c.registers[0x0F], 0xB1);
  assert!(rtc.i2c.acknowledgements.is_empty());
}

#[test]
fn read_and_acknowledgement_errors_are_preserved_without_retry() {
  let mut rtc = RV3028::new(StatusBus::new(0x20, 0));
  rtc.i2c.fail_read = true;
  assert_eq!(
    rtc.check_and_clear_backup_event(),
    Err(Error::I2c(ErrorKind::Bus))
  );
  assert_eq!(rtc.i2c.status_reads, 1);
  assert!(rtc.i2c.acknowledgements.is_empty());

  let mut rtc = RV3028::new(StatusBus::new(0x20, 0));
  rtc.i2c.fail_write = true;
  assert_eq!(
    rtc.check_and_clear_backup_event(),
    Err(Error::I2c(ErrorKind::NoAcknowledge(
      NoAcknowledgeSource::Data
    )))
  );
  assert_eq!(rtc.i2c.status_reads, 1);
  assert_eq!(rtc.i2c.acknowledgements, [0x5F]);
}
