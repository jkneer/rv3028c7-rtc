use super::*;
use embedded_hal::i2c::ErrorKind;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
use std::{vec, vec::Vec};

// Explicit datasheet hour tables: midnight through 23:00, including noon.
const HOURS24: [u8; 24] = [
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
  0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23,
];
const HOURS12: [u8; 24] = [
  0x12, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x32, 0x21, 0x22, 0x23,
  0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31,
];

#[test]
fn mode_getter_decodes_only_12_24_and_never_writes() {
  for control in 0..=255 {
    let expected = if control & 0x02 != 0 {
      HourMode::Hours12
    } else {
      HourMode::Hours24
    };
    assert_eq!(HourMode::from_register(control), expected);
    let mut rtc = RV3028::new_with_mux(
      Mock::new(&[
        T::write(0x70, vec![1]),
        T::write_read(0x52, vec![0x10], vec![control]),
      ]),
      0x70,
      1,
    );
    assert_eq!(rtc.get_hour_mode(), Ok(expected));
    rtc.i2c.done();
  }
}

#[test]
fn alarm_read_and_write_use_all_civil_hours_in_both_modes() {
  for (control, hours) in [(0, HOURS24), (0x02, HOURS12)] {
    for (hour, encoded) in hours.into_iter().enumerate() {
      for disabled in [0, 0x80] {
        for weekday in [false, true] {
          let day = if weekday { 2 } else { 15 };
          let encoded_day = if weekday { 0x02 } else { 0x15 };
          let wada = if weekday { 0 } else { 0x20 };
          let alarm = AlarmParts {
            day,
            hour: hour as u8,
            minute: 45,
            weekday_alarm: weekday,
            match_day: true,
            match_hour: disabled == 0,
            match_minute: true,
          };
          let mut rtc = RV3028::new(Mock::new(&[
            T::write_read(0x52, vec![0x10], vec![control]),
            T::write(0x52, vec![0x0E, 0x7B]),
            T::write_read(0x52, vec![0x0F], vec![wada]),
            T::write(0x52, vec![0x0F, wada]),
            T::write(0x52, vec![0x07, 0x45]),
            T::write(0x52, vec![0x08, encoded | disabled]),
            T::write(0x52, vec![0x09, encoded_day]),
            T::write(0x52, vec![0x0E, 0x7B]),
            T::write_read(0x52, vec![0x10], vec![control]),
            T::write_read(0x52, vec![0x09], vec![encoded_day]),
            T::write_read(0x52, vec![0x08], vec![encoded | disabled]),
            T::write_read(0x52, vec![0x07], vec![0x45]),
            T::write_read(0x52, vec![0x0F], vec![wada]),
          ]));
          rtc.set_alarm_parts(&alarm).unwrap();
          assert_eq!(rtc.get_alarm_parts(), Ok(alarm));
          rtc.i2c.done();
        }
      }
    }
  }
}

fn change_transactions(control: u8, old_hour: u8, new_control: u8, new_hour: u8) -> Vec<T> {
  vec![
    T::write(0x70, vec![1]),
    T::write_read(0x52, vec![0x10], vec![control]),
    T::write_read(0x52, vec![0x14], vec![0; 7]),
    T::write_read(0x52, vec![0x08], vec![old_hour]),
    T::write(0x52, vec![0x10, control & !0x09]),
    T::write(0x52, vec![0x10, new_control & !0x09]),
    T::write(0x52, vec![0x08, new_hour]),
    T::write(0x52, vec![0x10, new_control & !0x01]),
  ]
}

#[test]
fn switching_modes_preserves_each_alarm_hour_and_ae_aie_and_other_control_bits() {
  for other in (0..=0x7F).filter(|control| control & 0x02 == 0) {
    for (old_mode, mode, old_hours, new_hours) in [
      (0, HourMode::Hours12, HOURS24, HOURS12),
      (0x02, HourMode::Hours24, HOURS12, HOURS24),
    ] {
      for hour in 0..24 {
        for disabled in [0, 0x80] {
          let control = other | old_mode;
          let changed = (other & !0x02) | mode as u8;
          let expectations = change_transactions(
            control,
            old_hours[hour] | disabled,
            changed,
            new_hours[hour] | disabled,
          );
          let mut rtc = RV3028::new_with_mux(Mock::new(&expectations), 0x70, 1);
          rtc.set_hour_mode(mode).unwrap();
          rtc.i2c.done();
        }
      }
    }
  }
}

#[test]
fn unchanged_mode_is_read_only_even_with_capture_enabled() {
  for (mode, control) in [(HourMode::Hours24, 0xFD), (HourMode::Hours12, 0xFF)] {
    let mut rtc = RV3028::new(Mock::new(&[T::write_read(0x52, vec![0x10], vec![control])]));
    rtc.set_hour_mode(mode).unwrap();
    rtc.i2c.done();
  }
}

#[test]
fn switching_rejects_enabled_capture_and_every_nonempty_timestamp_byte_before_writes() {
  let mut rtc = RV3028::new(Mock::new(&[T::write_read(0x52, vec![0x10], vec![0x80])]));
  assert_eq!(
    rtc.set_hour_mode(HourMode::Hours12),
    Err(Error::InvalidData)
  );
  rtc.i2c.done();
  // A zero Count TS does not imply an empty log: the counter may have wrapped.
  for index in 0..7 {
    let mut bytes = vec![0; 7];
    bytes[index] = 1;
    let mut rtc = RV3028::new(Mock::new(&[
      T::write_read(0x52, vec![0x10], vec![0]),
      T::write_read(0x52, vec![0x14], bytes),
    ]));
    assert_eq!(
      rtc.set_hour_mode(HourMode::Hours12),
      Err(Error::InvalidData)
    );
    rtc.i2c.done();
  }
}

#[test]
fn invalid_enabled_alarm_blocks_switch_but_disabled_alarm_becomes_midnight() {
  for (control, mode, raw, midnight) in [
    (0x00, HourMode::Hours12, 0x24, 0x12),
    (0x00, HourMode::Hours12, 0x0A, 0x12),
    (0x02, HourMode::Hours24, 0x00, 0x00),
    (0x02, HourMode::Hours24, 0x13, 0x00),
    (0x02, HourMode::Hours24, 0x40, 0x00),
  ] {
    let expected = change_transactions(control, raw, mode as u8, midnight);
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected[..4]), 0x70, 1);
    assert_eq!(rtc.set_hour_mode(mode), Err(Error::InvalidData));
    rtc.i2c.done();
    let expected = change_transactions(control, raw | 0x80, mode as u8, midnight | 0x80);
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    rtc.set_hour_mode(mode).unwrap();
    rtc.i2c.done();
  }
}

#[test]
fn invalid_12_hour_alarm_is_rejected_only_when_matching_is_enabled() {
  for raw in [0x00, 0x13, 0x1A, 0x33, 0x40, 0x60] {
    for disabled in [0, 0x80] {
      let mut rtc = RV3028::new(Mock::new(&[
        T::write_read(0x52, vec![0x10], vec![0x02]),
        T::write_read(0x52, vec![0x09], vec![1]),
        T::write_read(0x52, vec![0x08], vec![raw | disabled]),
        T::write_read(0x52, vec![0x07], vec![0]),
        T::write_read(0x52, vec![0x0F], vec![0x20]),
      ]));
      let result = rtc.get_alarm_parts();
      if disabled == 0 {
        assert_eq!(result, Err(Error::InvalidData));
      } else {
        assert_eq!(result.unwrap().hour, 0);
      }
      rtc.i2c.done();
    }
  }
}

#[test]
fn switching_propagates_failure_at_every_transfer_without_retry_or_rollback() {
  let successful = change_transactions(0x7C, 0x23, 0x7E, 0x31);
  for index in 0..successful.len() {
    let mut expected = successful[..=index].to_vec();
    expected[index] = expected[index].clone().with_error(ErrorKind::Bus);
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    assert_eq!(
      rtc.set_hour_mode(HourMode::Hours12),
      Err(Error::I2c(ErrorKind::Bus))
    );
    rtc.i2c.done();
  }
}
