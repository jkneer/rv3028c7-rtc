use super::*;
use embedded_hal::i2c::ErrorKind;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
use std::vec;

#[test]
fn status_snapshot_reads_one_block_without_flag_acknowledgements() {
  let expected = [
    T::write(0x70, vec![1]),
    T::write_read(0x52, vec![0x0E], vec![0xFF, 0xA5, 0x82, 0x7C, 0x0F, 0x73]),
  ];
  let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
  assert_eq!(
    rtc.get_status(),
    Ok(StatusSnapshot {
      flags: StatusFlags::from_register(0xFF),
      control1: 0xA5,
      control2: 0x82,
      gp_bits: 0x7C,
      clock_interrupt_mask: 0x0F,
      event_control: 0x73,
    })
  );
  rtc.i2c.done();
}

#[test]
fn all_status_bytes_decode_without_discarding_busy_or_any_event_flag() {
  for raw in 0..=255 {
    let flags = StatusFlags::from_register(raw);
    assert_eq!(flags.bits(), raw);
    let decoded = [
      flags.power_on_reset(),
      flags.external_event(),
      flags.alarm(),
      flags.countdown_timer(),
      flags.time_update(),
      flags.backup_switchover(),
      flags.clock_output(),
      flags.eeprom_busy(),
    ];
    for (bit, set) in decoded.into_iter().enumerate() {
      assert_eq!(set, raw & (1 << bit) != 0);
    }
  }
}

#[test]
fn register_reads_include_boundaries_and_reserved_addresses_in_one_transfer() {
  let expected = [
    T::write_read(0x52, vec![0], vec![0x59]),
    T::write_read(0x52, vec![0x3F], vec![0xA5]),
    T::write_read(0x52, vec![0x29], vec![0xCC, 0xDD]),
    T::write_read(0x52, vec![0], (0..64).collect()),
  ];
  let mut rtc = RV3028::new(Mock::new(&expected));
  assert_eq!(rtc.read_register(0), Ok(0x59));
  assert_eq!(rtc.read_register(0x3F), Ok(0xA5));
  let mut reserved = [0; 2];
  rtc.read_registers(0x29, &mut reserved).unwrap();
  assert_eq!(reserved, [0xCC, 0xDD]);
  let mut all = [0; 64];
  rtc.read_registers(0, &mut all).unwrap();
  assert_eq!(all, core::array::from_fn(|i| i as u8));
  rtc.i2c.done();
}

#[test]
fn empty_reads_and_invalid_ranges_never_select_mux_or_modify_buffer() {
  let mut rtc = RV3028::new_with_mux(Mock::new(&[]), 0x70, 1);
  for start in 0..=64 {
    rtc.read_registers(start, &mut []).unwrap();
  }
  for (start, length) in [(0, 65), (63, 2), (64, 1), (65, 0), (255, 0)] {
    let mut bytes = [0xA5; 65];
    assert_eq!(
      rtc.read_registers(start, &mut bytes[..length]),
      Err(Error::InvalidData)
    );
    assert_eq!(bytes, [0xA5; 65]);
  }
  for register in 64..=255 {
    assert_eq!(rtc.read_register(register), Err(Error::InvalidData));
  }
  rtc.i2c.done();
}

#[test]
fn status_and_raw_reads_preserve_mux_and_bus_errors_without_retry() {
  let expected = [T::write(0x70, vec![1]).with_error(ErrorKind::ArbitrationLoss)];
  let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
  assert_eq!(
    rtc.get_status(),
    Err(Error::I2c(ErrorKind::ArbitrationLoss))
  );
  rtc.i2c.done();
  let expected = [T::write_read(0x52, vec![0x0E], vec![0; 6]).with_error(ErrorKind::Bus)];
  let mut rtc = RV3028::new(Mock::new(&expected));
  assert_eq!(rtc.get_status(), Err(Error::I2c(ErrorKind::Bus)));
  rtc.i2c.done();
}
