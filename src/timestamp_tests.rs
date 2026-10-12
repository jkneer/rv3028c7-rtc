use super::*;
use embedded_hal::i2c::ErrorKind;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
use std::vec;

const CONFIG: TimestampConfig = TimestampConfig {
  source: TimestampSource::BackupSwitchover,
  capture: TimestampCapture::LastEvent,
  enabled: true,
};

fn restart() -> std::vec::Vec<T> {
  vec![
    T::write_read(0x52, vec![0x10], vec![0xBF]),
    T::write(0x52, vec![0x10, 0x3B]),
    T::write_read(0x52, vec![0x13], vec![0x71]),
    T::write(0x52, vec![0x13, 0x72]),
    T::write(0x52, vec![0x0E, 0x5D]),
    T::write(0x52, vec![0x13, 0x76]),
    T::write(0x52, vec![0x13, 0x73]),
    T::write(0x52, vec![0x10, 0xBB]),
  ]
}

#[test]
fn restart_follows_vendor_order_and_preserves_unrelated_configuration() {
  let mut expected = vec![T::write(0x70, vec![1])];
  expected.extend(restart());
  let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
  rtc.configure_timestamp(&CONFIG).unwrap();
  rtc.i2c.done();
}

#[test]
fn config_change_pauses_capture_before_source_without_reset_or_acknowledgement() {
  let expected = [
    T::write_read(0x52, vec![0x10], vec![0xBF]),
    T::write(0x52, vec![0x10, 0x3B]),
    T::write_read(0x52, vec![0x13], vec![0x73]),
    T::write(0x52, vec![0x13, 0x70]),
    T::write(0x52, vec![0x10, 0x3F]),
  ];
  let mut rtc = RV3028::new(Mock::new(&expected));
  rtc
    .set_timestamp_config(&TimestampConfig {
      source: TimestampSource::ExternalEvent,
      capture: TimestampCapture::FirstEvent,
      enabled: false,
    })
    .unwrap();
  rtc.i2c.done();
}

#[test]
fn config_reader_decodes_all_source_capture_enable_combinations_without_writes() {
  for enabled in [false, true] {
    for source in [
      TimestampSource::ExternalEvent,
      TimestampSource::BackupSwitchover,
    ] {
      for capture in [TimestampCapture::FirstEvent, TimestampCapture::LastEvent] {
        let expected = [
          T::write(0x70, vec![1]),
          T::write_read(
            0x52,
            vec![0x10],
            vec![0x3F | if enabled { 0x80 } else { 0 }],
          ),
          T::write_read(
            0x52,
            vec![0x13],
            vec![0x70 | source as u8 | ((capture as u8) << 1)],
          ),
        ];
        let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
        assert_eq!(
          rtc.get_timestamp_config(),
          Ok(TimestampConfig {
            enabled,
            source,
            capture
          })
        );
        rtc.i2c.done();
      }
    }
  }
}

#[test]
fn reset_pulses_tsr_without_acknowledging_flags_or_changing_capture_settings() {
  let expected = [
    T::write_read(0x52, vec![0x13], vec![0x73]),
    T::write(0x52, vec![0x13, 0x77]),
  ];
  let mut rtc = RV3028::new(Mock::new(&expected));
  rtc.reset_timestamp().unwrap();
  rtc.i2c.done();
}

#[test]
fn readout_preserves_valid_timestamp_after_count_wrap_and_distinguishes_empty_reset() {
  for count in [0, 1, 255] {
    let expected = [
      T::write(0x70, vec![1]),
      T::write_read(0x52, vec![0x10], vec![0]),
      T::write_read(
        0x52,
        vec![0x14],
        vec![count, 0x30, 0x45, 0x13, 0x15, 0x03, 0x24],
      ),
    ];
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    assert_eq!(
      rtc.get_timestamp(),
      Ok(TimestampSnapshot {
        count,
        datetime: Some(DateTimeParts {
          year: 2024,
          month: 3,
          day: 15,
          hour: 13,
          minute: 45,
          second: 30
        }),
      })
    );
    rtc.i2c.done();
  }
  let expected = [
    T::write_read(0x52, vec![0x10], vec![2]),
    T::write_read(0x52, vec![0x14], vec![0; 7]),
  ];
  let mut rtc = RV3028::new(Mock::new(&expected));
  assert_eq!(
    rtc.get_timestamp(),
    Ok(TimestampSnapshot {
      count: 0,
      datetime: None
    })
  );
  rtc.i2c.done();
}

#[test]
fn every_12_hour_code_is_normalized_and_invalid_timestamp_data_is_rejected() {
  let codes = [
    0x12, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0x10, 0x11, 0x32, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    0x28, 0x29, 0x30, 0x31,
  ];
  for (hour, code) in codes.into_iter().enumerate() {
    let expected = [
      T::write_read(0x52, vec![0x10], vec![2]),
      T::write_read(0x52, vec![0x14], vec![1, 0, 0, code, 1, 1, 0]),
    ];
    let mut rtc = RV3028::new(Mock::new(&expected));
    assert_eq!(
      rtc.get_timestamp().unwrap().datetime.unwrap().hour,
      hour as u8
    );
    rtc.i2c.done();
  }
  for bytes in [
    vec![1, 0, 0, 0, 0, 0, 0],
    vec![0, 0, 0, 0, 0, 0, 0x24],
    vec![1, 0x0A, 0, 0, 1, 1, 0x24],
    vec![1, 0, 0, 0x24, 1, 1, 0x24],
    vec![1, 0, 0, 0, 0x29, 2, 0x23],
  ] {
    let expected = [
      T::write_read(0x52, vec![0x10], vec![0]),
      T::write_read(0x52, vec![0x14], bytes),
    ];
    let mut rtc = RV3028::new(Mock::new(&expected));
    assert_eq!(rtc.get_timestamp(), Err(Error::InvalidData));
    rtc.i2c.done();
  }
}

#[test]
fn configuration_and_readout_failures_stop_without_retry() {
  let baseline = restart();
  for failure in 0..baseline.len() {
    let mut expected = baseline[..=failure].to_vec();
    expected[failure] = expected[failure].clone().with_error(ErrorKind::Bus);
    let mut rtc = RV3028::new(Mock::new(&expected));
    assert_eq!(
      rtc.configure_timestamp(&CONFIG),
      Err(Error::I2c(ErrorKind::Bus))
    );
    rtc.i2c.done();
  }
  let baseline = [
    T::write_read(0x52, vec![0x10], vec![0]),
    T::write_read(0x52, vec![0x14], vec![0; 7]),
  ];
  for failure in 0..2 {
    let mut expected = baseline[..=failure].to_vec();
    expected[failure] = expected[failure].clone().with_error(ErrorKind::Bus);
    let mut rtc = RV3028::new(Mock::new(&expected));
    assert_eq!(rtc.get_timestamp(), Err(Error::I2c(ErrorKind::Bus)));
    rtc.i2c.done();
  }
}

#[test]
fn legacy_source_arguments_are_validated_before_bus_access() {
  let mut rtc = RV3028::new_with_mux(Mock::new(&[]), 0x70, 1);
  for source in 2..=255 {
    assert_eq!(
      rtc.config_timestamp_logging(source, true, true),
      Err(Error::InvalidData)
    );
    assert_eq!(
      rtc.set_event_timestamp_source(source),
      Err(Error::InvalidData)
    );
  }
  rtc.i2c.done();
}
