use super::*;
use embedded_hal::i2c::ErrorKind;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
use std::{vec, vec::Vec};

#[test]
fn every_invalid_evi_filter_is_rejected_before_mux_or_register_access() {
  let mut rtc = RV3028::new_with_mux(Mock::new(&[]), 0x70, 1);
  for filter in 0b100..=255 {
    assert_eq!(
      rtc.config_ext_event_detection(true, true, filter, true),
      Err(Error::InvalidData)
    );
  }
  rtc.i2c.done();
}

pub(super) fn evi_transactions(
  filter: u8,
  rising: bool,
  interrupt: bool,
  clockout: bool,
) -> Vec<T> {
  // Preserve TSE, hour mode, other interrupts, TSOW/TSS and other CLKOUT sources.
  let edge = if rising { 0xC3 } else { 0x83 };
  let event = edge | (filter << 4);
  let clock_mask = if clockout { 0xBF } else { 0xB7 };
  let control = if interrupt { 0xFE } else { 0xFA };
  let mut transactions = vec![
    T::write(0x70, vec![1]),
    T::write_read(0x52, vec![0x10], vec![0xFE]),
    T::write(0x52, vec![0x10, 0xFA]),
    T::write(0x52, vec![0x0E, 0x7D]),
    T::write_read(0x52, vec![0x13], vec![0xF3]),
    T::write(0x52, vec![0x13, edge | 0x30]),
    T::write_read(0x52, vec![0x13], vec![edge | 0x30]),
    T::write(0x52, vec![0x13, edge]),
  ];
  if filter != 0 {
    transactions.extend([
      T::write_read(0x52, vec![0x13], vec![edge]),
      T::write(0x52, vec![0x13, event]),
    ]);
  }
  transactions.extend([
    T::write_read(0x52, vec![0x12], vec![0xBF]),
    T::write(0x52, vec![0x12, clock_mask]),
    T::write_read(0x52, vec![0x10], vec![0xFA]),
    T::write(0x52, vec![0x10, control]),
  ]);
  transactions
}

#[test]
fn valid_evi_filters_update_only_selected_fields_and_acknowledge_only_evf() {
  for filter in 0..=0b11 {
    for rising in [false, true] {
      for interrupt in [false, true] {
        for clockout in [false, true] {
          let expected = evi_transactions(filter, rising, interrupt, clockout);
          let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
          rtc
            .config_ext_event_detection(rising, interrupt, filter, clockout)
            .unwrap();
          rtc.i2c.done();
        }
      }
    }
  }
}

#[test]
fn evi_configuration_preserves_bus_errors_at_every_transfer() {
  let successful = evi_transactions(0b11, true, true, true);
  for index in 0..successful.len() {
    let mut expected = successful[..=index].to_vec();
    expected[index] = expected[index].clone().with_error(ErrorKind::Bus);
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    assert_eq!(
      rtc.config_ext_event_detection(true, true, 0b11, true),
      Err(Error::I2c(ErrorKind::Bus))
    );
    rtc.i2c.done();
  }
}

type TimerCase = (Duration, TimerClockFreq, u16, Duration);
fn timer_cases() -> Vec<TimerCase> {
  use TimerClockFreq::*;
  vec![
    (Duration::ZERO, Hertz4096, 0, Duration::ZERO),
    (
      Duration::from_nanos(1),
      Hertz4096,
      1,
      Duration::from_nanos(244_140),
    ),
    (
      Duration::from_nanos(244_140),
      Hertz4096,
      1,
      Duration::from_nanos(244_140),
    ),
    (
      Duration::from_nanos(244_141),
      Hertz4096,
      1,
      Duration::from_nanos(244_140),
    ),
    (
      Duration::from_micros(655),
      Hertz4096,
      2,
      Duration::from_nanos(488_281),
    ),
    (
      Duration::from_millis(125),
      Hertz4096,
      512,
      Duration::from_millis(125),
    ),
    (
      Duration::from_millis(500),
      Hertz4096,
      2048,
      Duration::from_millis(500),
    ),
    (
      Duration::from_nanos(999_755_859),
      Hertz4096,
      4094,
      Duration::from_nanos(999_511_718),
    ),
    (
      Duration::from_nanos(999_755_860),
      Hertz64,
      63,
      Duration::from_nanos(984_375_000),
    ),
    (Duration::from_secs(1), Hertz64, 64, Duration::from_secs(1)),
    (
      Duration::from_millis(2047),
      Hertz64,
      131,
      Duration::from_nanos(2_046_875_000),
    ),
    (
      Duration::from_secs(60),
      Hertz64,
      3840,
      Duration::from_secs(60),
    ),
    (
      Duration::from_secs(61),
      Hertz64,
      3904,
      Duration::from_secs(61),
    ),
    (
      Duration::from_nanos(63_984_375_000),
      Hertz64,
      4095,
      Duration::from_nanos(63_984_375_000),
    ),
    (
      Duration::from_nanos(63_984_375_001),
      Hertz1,
      63,
      Duration::from_secs(63),
    ),
    (Duration::from_secs(64), Hertz1, 64, Duration::from_secs(64)),
    (
      Duration::from_secs(4095),
      Hertz1,
      4095,
      Duration::from_secs(4095),
    ),
    (
      Duration::from_secs(4095) + Duration::from_nanos(1),
      HertzSixtieth,
      68,
      Duration::from_secs(4080),
    ),
    (
      Duration::from_secs(4096),
      HertzSixtieth,
      68,
      Duration::from_secs(4080),
    ),
    (
      Duration::from_secs(4140),
      HertzSixtieth,
      69,
      Duration::from_secs(4140),
    ),
    (
      Duration::from_secs(245_700),
      HertzSixtieth,
      4095,
      Duration::from_secs(245_700),
    ),
    (
      Duration::from_secs(245_701),
      HertzSixtieth,
      4095,
      Duration::from_secs(245_700),
    ),
    (
      Duration::MAX,
      HertzSixtieth,
      4095,
      Duration::from_secs(245_700),
    ),
  ]
}

#[test]
fn timer_quantization_uses_exact_frequencies_and_defined_range_limits() {
  for (requested, clock, ticks, nominal) in timer_cases() {
    assert_eq!(
      {
        let config = CountdownTimerConfig::from_duration(&requested, false, false);
        (config.ticks, config.clock, config.nominal_period().unwrap())
      },
      (ticks, clock, nominal),
      "{requested:?}"
    );
  }
}

pub(super) fn timer_transactions(
  clock: TimerClockFreq,
  ticks: u16,
  repeat: bool,
  start: bool,
) -> Vec<T> {
  let repeating = if repeat { 0xFB } else { 0x7B };
  let configured = (repeating & !0b11) | clock as u8;
  let mut transactions = vec![
    T::write(0x70, vec![1]),
    T::write_read(0x52, vec![0x0F], vec![0xFF]),
    T::write(0x52, vec![0x0F, 0xFB]), // stop TE
    T::write_read(0x52, vec![0x0F], vec![0xFB]),
    T::write(0x52, vec![0x0F, repeating]),
    T::write_read(0x52, vec![0x0F], vec![repeating]),
    T::write(0x52, vec![0x0F, repeating & !0b11]),
    T::write_read(0x52, vec![0x0F], vec![repeating & !0b11]),
    T::write(0x52, vec![0x0F, configured]),
    T::write(0x52, vec![0x0A, ticks as u8, (ticks >> 8) as u8]),
    T::write(0x52, vec![0x0E, 0x77]), // only TF acknowledged
  ];
  if start {
    transactions.extend([
      T::write_read(0x52, vec![0x0F], vec![configured]),
      T::write(0x52, vec![0x0F, configured | 0x04]),
    ]);
  }
  transactions
}

#[test]
fn returned_timer_period_matches_programmed_ticks_for_all_modes_and_boundaries() {
  for (requested, clock, ticks, nominal) in timer_cases() {
    for repeat in [false, true] {
      for start in [false, true] {
        let expected = timer_transactions(clock, ticks, repeat, start);
        let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
        assert_eq!(
          rtc.config_countdown_timer(&requested, repeat, start),
          Ok(nominal)
        );
        rtc.i2c.done();
      }
    }
  }
}

#[test]
fn timer_configuration_stops_on_first_bus_error_without_retry() {
  let successful = timer_transactions(TimerClockFreq::Hertz64, 64, true, true);
  for index in 0..successful.len() {
    let mut expected = successful[..=index].to_vec();
    expected[index] = expected[index].clone().with_error(ErrorKind::Bus);
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    assert_eq!(
      rtc.config_countdown_timer(&Duration::from_secs(1), true, true),
      Err(Error::I2c(ErrorKind::Bus))
    );
    rtc.i2c.done();
  }
}
