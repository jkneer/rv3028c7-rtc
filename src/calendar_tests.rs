use super::*;
use embedded_hal::i2c::ErrorKind;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
use std::vec;

const PARTS: DateTimeParts = DateTimeParts {
  year: 2024,
  month: 3,
  day: 15,
  hour: 13,
  minute: 45,
  second: 30,
};

fn read_calendar(mode: u8, bytes: [u8; 7]) -> RV3028<Mock> {
  RV3028::new(Mock::new(&[
    T::write_read(0x52, vec![0x10], vec![mode]),
    T::write_read(0x52, vec![0x00], bytes.to_vec()),
  ]))
}

#[test]
fn complete_calendar_is_read_in_one_transfer_after_mode_and_one_mux_selection() {
  let expected = [
    T::write(0x70, vec![1]),
    T::write_read(0x52, vec![0x10], vec![0x80]),
    T::write_read(
      0x52,
      vec![0x00],
      vec![0x30, 0x45, 0x13, 4, 0x15, 0x03, 0x24],
    ),
  ];
  let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
  assert_eq!(rtc.get_datetime_parts(), Ok(PARTS));
  rtc.i2c.done();
}

#[test]
fn valid_leap_day_and_calendar_endpoints_are_decoded_with_weekday_validation() {
  for (bytes, parts) in [
    (
      [0x59, 0x59, 0x23, 3, 0x29, 0x02, 0x24],
      DateTimeParts {
        year: 2024,
        month: 2,
        day: 29,
        hour: 23,
        minute: 59,
        second: 59,
      },
    ),
    (
      [0, 0, 0, 5, 1, 1, 0],
      DateTimeParts {
        year: 2000,
        month: 1,
        day: 1,
        hour: 0,
        minute: 0,
        second: 0,
      },
    ),
    (
      [0x59, 0x59, 0x23, 3, 0x31, 0x12, 0x99],
      DateTimeParts {
        year: 2099,
        month: 12,
        day: 31,
        hour: 23,
        minute: 59,
        second: 59,
      },
    ),
  ] {
    let mut rtc = read_calendar(0, bytes);
    assert_eq!(rtc.get_datetime_parts(), Ok(parts));
    rtc.i2c.done();
  }
}

#[test]
fn invalid_bcd_date_time_and_weekday_are_rejected_without_extra_io() {
  let good = [0x30, 0x45, 0x13, 4, 0x15, 0x03, 0x24];
  for (index, value) in [
    (0, 0x60),
    (0, 0x3A),
    (0, 0x80),
    (1, 0x60),
    (2, 0x24),
    (3, 7),
    (3, 0),
    (4, 0),
    (4, 0x32),
    (5, 0),
    (5, 0x13),
    (6, 0xFA),
  ] {
    let mut bytes = good;
    bytes[index] = value;
    let mut rtc = read_calendar(0, bytes);
    assert_eq!(rtc.get_datetime_parts(), Err(Error::InvalidData));
    rtc.i2c.done();
  }
  let mut rtc = read_calendar(0, [0, 0, 0, 0, 0x29, 0x02, 0x23]); // 2023-02-29
  assert_eq!(rtc.get_datetime_parts(), Err(Error::InvalidData));
  rtc.i2c.done();
}

#[test]
fn every_hour_round_trips_in_12_and_24_hour_mode_without_mode_writes() {
  // Literal hardware hour codes, including midnight and noon (AM/PM bit 5).
  let codes12 = [
    0x12, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x32, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31,
  ];
  for mode in [0xA0, 0xA2] {
    for hour in 0..24 {
      let code = if mode & 2 != 0 {
        codes12[hour]
      } else {
        (hour / 10 * 16 + hour % 10) as u8
      };
      let parts = DateTimeParts {
        hour: hour as u8,
        ..PARTS
      };
      let unix = (1_710_510_330u32 - 13 * 3600 + hour as u32 * 3600).to_le_bytes();
      let expected = [
        T::write_read(0x52, vec![0x10], vec![mode]),
        T::write(0x52, vec![0x1B, unix[0], unix[1], unix[2], unix[3]]),
        T::write(0x52, vec![0, 0x30, 0x45, code, 4, 0x15, 0x03, 0x24]),
        T::write_read(0x52, vec![0x10], vec![mode]),
        T::write_read(0x52, vec![0], vec![0x30, 0x45, code, 4, 0x15, 0x03, 0x24]),
        T::write_read(0x52, vec![0x10], vec![mode]),
        T::write_read(0x52, vec![0], vec![0x30, 0x45, code]),
      ];
      let mut rtc = RV3028::new(Mock::new(&expected));
      rtc.set_datetime_parts(&parts).unwrap();
      assert_eq!(rtc.get_datetime_parts(), Ok(parts));
      assert_eq!(rtc.get_hms(), Ok((hour as u8, 45, 30)));
      rtc.i2c.done();
    }
  }
}

#[test]
fn malformed_12_hour_codes_are_rejected() {
  for hour in [0x00, 0x20, 0x13, 0x33, 0x1A, 0x40, 0x80] {
    let mut rtc = read_calendar(2, [0x30, 0x45, hour, 4, 0x15, 0x03, 0x24]);
    assert_eq!(rtc.get_datetime_parts(), Err(Error::InvalidData));
    rtc.i2c.done();
  }
}

#[test]
fn write_failures_stop_before_later_transfers_without_retry_or_status_clear() {
  let unix = 1_710_510_330u32.to_le_bytes();
  let baseline = [
    T::write_read(0x52, vec![0x10], vec![0]),
    T::write(0x52, vec![0x1B, unix[0], unix[1], unix[2], unix[3]]),
    T::write(0x52, vec![0, 0x30, 0x45, 0x13, 4, 0x15, 0x03, 0x24]),
  ];
  for failure in 0..3 {
    let mut expected = baseline[..=failure].to_vec();
    expected[failure] = expected[failure].clone().with_error(ErrorKind::Bus);
    let mut rtc = RV3028::new(Mock::new(&expected));
    assert_eq!(
      rtc.set_datetime_parts(&PARTS),
      Err(Error::I2c(ErrorKind::Bus))
    );
    rtc.i2c.done();
  }
}

#[test]
fn read_failures_and_mux_failure_are_preserved_without_retry() {
  let baseline = [
    T::write_read(0x52, vec![0x10], vec![0]),
    T::write_read(0x52, vec![0], vec![0x30, 0x45, 0x13, 4, 0x15, 0x03, 0x24]),
  ];
  for failure in 0..2 {
    let mut expected = baseline[..=failure].to_vec();
    expected[failure] = expected[failure].clone().with_error(ErrorKind::Bus);
    let mut rtc = RV3028::new(Mock::new(&expected));
    assert_eq!(rtc.get_datetime_parts(), Err(Error::I2c(ErrorKind::Bus)));
    rtc.i2c.done();
  }
  let expected = [T::write(0x70, vec![1]).with_error(ErrorKind::ArbitrationLoss)];
  let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
  assert_eq!(
    rtc.get_datetime_parts(),
    Err(Error::I2c(ErrorKind::ArbitrationLoss))
  );
  rtc.i2c.done();
}

#[test]
fn individual_fields_write_only_the_selected_register_after_one_mux_selection() {
  // Literal register packets: no Unix, status, or other calendar access.
  let expected = [
    T::write(0x70, vec![1]),
    T::write(0x52, vec![0x00, 0x59]),
    T::write(0x70, vec![1]),
    T::write(0x52, vec![0x01, 0x45]),
    T::write(0x70, vec![1]),
    T::write(0x52, vec![0x03, 6]),
    T::write(0x70, vec![1]),
    T::write(0x52, vec![0x04, 0x31]),
    T::write(0x70, vec![1]),
    T::write(0x52, vec![0x05, 0x12]),
    T::write(0x70, vec![1]),
    T::write(0x52, vec![0x06, 0x99]),
  ];
  let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
  rtc.set_seconds(59).unwrap();
  rtc.set_minutes(45).unwrap();
  rtc.set_weekday(6).unwrap();
  rtc.set_day(31).unwrap();
  rtc.set_month(12).unwrap();
  rtc.set_year(2099).unwrap();
  rtc.i2c.done();
}

#[test]
fn individual_field_ranges_are_checked_before_mux_or_bus_access() {
  type Setter = fn(&mut RV3028<Mock>, u8) -> Result<(), Error<ErrorKind>>;
  let setters: [(Setter, u8, u8, u8); 6] = [
    (RV3028::set_seconds, 0x00, 0, 59),
    (RV3028::set_minutes, 0x01, 0, 59),
    (RV3028::set_hours, 0x02, 0, 23),
    (RV3028::set_weekday, 0x03, 0, 6),
    (RV3028::set_day, 0x04, 1, 31),
    (RV3028::set_month, 0x05, 1, 12),
  ];
  for (setter, reg, min, max) in setters {
    for value in 0..=u8::MAX {
      if value < min || value > max {
        let mut rtc = RV3028::new_with_mux(Mock::new(&[]), 0x70, 1);
        assert_eq!(setter(&mut rtc, value), Err(Error::InvalidData));
        rtc.i2c.done();
      } else {
        let code = if reg == 0x03 {
          value
        } else {
          value / 10 * 16 + value % 10
        };
        let mut expected = vec![];
        if reg == 0x02 {
          expected.push(T::write_read(0x52, vec![0x10], vec![0]));
        }
        expected.push(T::write(0x52, vec![reg, code]));
        let mut rtc = RV3028::new(Mock::new(&expected));
        setter(&mut rtc, value).unwrap();
        rtc.i2c.done();
      }
    }
  }
  let mut rtc = RV3028::new_with_mux(Mock::new(&[]), 0x70, 1);
  for year in [i32::MIN, -1, 1999, 2100, i32::MAX] {
    assert_eq!(rtc.set_year(year), Err(Error::InvalidData));
  }
  rtc.i2c.done();
  for year in 2000..=2099 {
    let value = (year - 2000) as u8;
    let expected = [T::write(0x52, vec![0x06, value / 10 * 16 + value % 10])];
    let mut rtc = RV3028::new(Mock::new(&expected));
    rtc.set_year(year).unwrap();
    rtc.i2c.done();
  }
}

#[test]
fn individual_hours_encode_every_am_pm_value_without_changing_mode() {
  let codes = [
    0x12, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x32, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31,
  ];
  for (hour, code) in codes.into_iter().enumerate() {
    let expected = [
      T::write(0x70, vec![1]),
      T::write_read(0x52, vec![0x10], vec![0xA2]),
      T::write(0x52, vec![0x02, code]),
    ];
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    rtc.set_hours(hour as u8).unwrap();
    rtc.i2c.done();
  }
}

#[test]
fn individual_setter_errors_are_preserved_without_retry_or_later_access() {
  let baseline = [
    T::write(0x70, vec![1]),
    T::write_read(0x52, vec![0x10], vec![2]),
    T::write(0x52, vec![0x02, 0x32]),
  ];
  for failure in 0..3 {
    let mut expected = baseline[..=failure].to_vec();
    expected[failure] = expected[failure].clone().with_error(ErrorKind::Bus);
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    assert_eq!(rtc.set_hours(12), Err(Error::I2c(ErrorKind::Bus)));
    rtc.i2c.done();
  }
  let expected = [T::write(0x52, vec![0x06, 0x24]).with_error(ErrorKind::Bus)];
  let mut rtc = RV3028::new(Mock::new(&expected));
  assert_eq!(rtc.set_year(2024), Err(Error::I2c(ErrorKind::Bus)));
  rtc.i2c.done();
}
