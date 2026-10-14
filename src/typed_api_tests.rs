use super::*;
use embedded_hal::i2c::ErrorKind;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
use std::{vec, vec::Vec};

#[test]
fn evi_filter_conversion_accepts_only_unshifted_et_bits() {
  for bits in 0..=255 {
    let expected = match bits {
      0 => Some(EviFilter::EdgeDetection),
      1 => Some(EviFilter::Millis3Point9),
      2 => Some(EviFilter::Millis15Point6),
      3 => Some(EviFilter::Millis125),
      _ => None,
    };
    assert_eq!(EviFilter::from_bits(bits), expected);
  }
}

#[test]
fn evi_getter_reads_settings_once_without_acknowledging_or_resetting_anything() {
  for event in 0..=255 {
    let control = event ^ 0xA5;
    let mask = event ^ 0x5A;
    let expected = [
      T::write(0x70, vec![1]),
      T::write_read(0x52, vec![0x10], vec![control, 0x7F, mask, event]),
    ];
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    let config = rtc.get_evi_config().unwrap();
    assert_eq!(
      config.polarity == EviPolarity::RisingOrHigh,
      event & 0x40 != 0
    );
    assert_eq!(config.filter as u8, (event >> 4) & 0b11);
    assert_eq!(config.interrupt_enabled, control & 0x04 != 0);
    assert_eq!(config.clockout_enabled, mask & 0x08 != 0);
    rtc.i2c.done();
  }
}

#[test]
fn typed_evi_setter_shares_legacy_sequence_for_all_settings() {
  for filter in [
    EviFilter::EdgeDetection,
    EviFilter::Millis3Point9,
    EviFilter::Millis15Point6,
    EviFilter::Millis125,
  ] {
    for polarity in [EviPolarity::FallingOrLow, EviPolarity::RisingOrHigh] {
      for interrupt_enabled in [false, true] {
        for clockout_enabled in [false, true] {
          let config = EviConfig {
            filter,
            polarity,
            interrupt_enabled,
            clockout_enabled,
          };
          let expected = event_timer_tests::evi_transactions(
            filter as u8,
            polarity == EviPolarity::RisingOrHigh,
            interrupt_enabled,
            clockout_enabled,
          );
          let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
          rtc.set_evi_config(&config).unwrap();
          rtc.i2c.done();
        }
      }
    }
  }
}

#[test]
fn typed_timer_programs_exact_presets_for_every_clock_repeat_and_enable_setting() {
  for clock in [
    TimerClockFreq::Hertz4096,
    TimerClockFreq::Hertz64,
    TimerClockFreq::Hertz1,
    TimerClockFreq::HertzSixtieth,
  ] {
    for ticks in [0, 1, 0x0100, 0x0FFF] {
      for repeat in [false, true] {
        for enabled in [false, true] {
          let config = CountdownTimerConfig {
            clock,
            ticks,
            repeat,
            enabled,
          };
          let expected = event_timer_tests::timer_transactions(clock, ticks, repeat, enabled);
          let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
          rtc.set_countdown_timer_config(&config).unwrap();
          rtc.i2c.done();
        }
      }
    }
  }
}

#[test]
fn invalid_timer_preset_is_rejected_before_mux_selection_and_cannot_report_a_period() {
  let mut rtc = RV3028::new_with_mux(Mock::new(&[]), 0x70, 1);
  for ticks in [0x1000, 0x8000, 0xFFFF] {
    let config = CountdownTimerConfig {
      clock: TimerClockFreq::Hertz1,
      ticks,
      repeat: true,
      enabled: true,
    };
    assert_eq!(
      rtc.set_countdown_timer_config(&config),
      Err(Error::InvalidData)
    );
    assert_eq!(config.nominal_period(), None);
  }
  rtc.i2c.done();
}

#[test]
fn timer_getter_reads_preset_not_running_counter_and_ignores_unimplemented_high_bits() {
  for control in 0..=255 {
    for ticks in [0u16, 1, 0x0100, 0x0FFF] {
      let expected = [
        T::write(0x70, vec![1]),
        T::write_read(0x52, vec![0x0F], vec![control]),
        T::write_read(
          0x52,
          vec![0x0A],
          vec![ticks as u8, ((ticks >> 8) as u8) | 0xF0],
        ),
      ];
      let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
      let config = rtc.get_countdown_timer_config().unwrap();
      assert_eq!(config.clock as u8, control & 0b11);
      assert_eq!(config.ticks, ticks);
      assert_eq!(config.repeat, control & 0x80 != 0);
      assert_eq!(config.enabled, control & 0x04 != 0);
      rtc.i2c.done();
    }
  }
}

#[test]
fn timer_duration_constructor_preserves_requested_control_state() {
  for repeat in [false, true] {
    for enabled in [false, true] {
      let config = CountdownTimerConfig::from_duration(&Duration::from_secs(1), repeat, enabled);
      assert_eq!(
        config,
        CountdownTimerConfig {
          clock: TimerClockFreq::Hertz64,
          ticks: 64,
          repeat,
          enabled
        }
      );
      assert_eq!(config.nominal_period(), Some(Duration::from_secs(1)));
    }
  }
}

#[test]
fn running_timer_counter_masks_reserved_bits_and_uses_shadow_register_burst() {
  let mut rtc = RV3028::new(Mock::new(&[T::write_read(
    0x52,
    vec![0x0C],
    vec![0x34, 0xF2],
  )]));
  assert_eq!(rtc.get_countdown_value(), Ok(0x0234));
  rtc.i2c.done();
}

#[test]
fn routing_encoding_covers_all_sources_preserves_tse_and_hour_mode_and_never_issues_reset() {
  for control in 0..=255 {
    for mask in 0..=255 {
      let config = InterruptConfig::from_registers(control, mask, 0xFF, 0xFF);
      assert_eq!(config.clockout.to_mask(), mask & 0x0F);
      assert_eq!(config.clockout.time_update, mask & 0x01 != 0);
      assert_eq!(config.clockout.countdown_timer, mask & 0x02 != 0);
      assert_eq!(config.clockout.alarm, mask & 0x04 != 0);
      assert_eq!(config.clockout.event, mask & 0x08 != 0);
      assert_eq!(config.to_control2(control), control & !0x01);
      assert!(config.interrupts.power_on_reset);
      assert!(config.interrupts.backup_switchover);
      assert_eq!(config.interrupts.time_update, control & 0x20 != 0);
      assert_eq!(config.interrupts.countdown_timer, control & 0x10 != 0);
      assert_eq!(config.interrupts.alarm, control & 0x08 != 0);
      assert_eq!(config.interrupts.event, control & 0x04 != 0);
      assert_eq!(config.clockout.enabled, control & 0x40 != 0);
      for current in [0, 0x02, 0x80, 0x82, 0xFF] {
        assert_eq!(
          config.to_control2(current),
          (current & 0x82) | (control & 0x7C)
        );
      }
    }
  }
}

#[test]
fn routing_decodes_only_por_and_backup_enable_bits_from_shared_mirrors() {
  for mirror in 0..=255 {
    let config = InterruptConfig::from_registers(0, 0, mirror, mirror);
    assert_eq!(config.interrupts.power_on_reset, mirror & 0x08 != 0);
    assert_eq!(config.interrupts.backup_switchover, mirror & 0x40 != 0);
  }
}

fn routing_get_transactions(control: u8, mask: u8, clkout: u8, backup: u8) -> Vec<T> {
  vec![
    T::write(0x70, vec![1]),
    T::write_read(0x52, vec![0x10], vec![control]),
    T::write_read(0x52, vec![0x12], vec![mask]),
    T::write_read(0x52, vec![0x35], vec![clkout]),
    T::write_read(0x52, vec![0x37], vec![backup]),
  ]
}

#[test]
fn routing_getter_reads_all_six_enables_and_four_sources_without_writes() {
  for (control, mask, clkout, backup) in [
    (0xFF, 0xFF, 0xFF, 0xFF),
    (0, 0, 0, 0),
    (0xA6, 0x05, 0xC8, 0xBF),
    (0xDA, 0x0A, 0xC0, 0xFF),
  ] {
    let expected = routing_get_transactions(control, mask, clkout, backup);
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    assert_eq!(
      rtc.get_interrupt_config(),
      Ok(InterruptConfig::from_registers(
        control, mask, clkout, backup
      ))
    );
    rtc.i2c.done();
  }
}

fn routing_set_transactions(
  control: u8,
  clkout: u8,
  backup: u8,
  config: InterruptConfig,
) -> Vec<T> {
  vec![
    T::write(0x70, vec![1]),
    T::write_read(0x52, vec![0x10], vec![control]),
    T::write_read(0x52, vec![0x35], vec![clkout]),
    T::write_read(0x52, vec![0x37], vec![backup]),
    T::write(0x52, vec![0x10, control & 0x82]),
    T::write(
      0x52,
      vec![
        0x35,
        (clkout & !0x08)
          | if config.interrupts.power_on_reset {
            0x08
          } else {
            0
          },
      ],
    ),
    T::write(
      0x52,
      vec![
        0x37,
        (backup & !0x40)
          | if config.interrupts.backup_switchover {
            0x40
          } else {
            0
          },
      ],
    ),
    T::write(0x52, vec![0x12, config.clockout.to_mask()]),
    T::write(0x52, vec![0x10, config.to_control2(control)]),
  ]
}

#[test]
fn routing_setter_merges_por_and_backup_enables_without_touching_fd_charger_or_calibration() {
  for enabled in [false, true] {
    let config = InterruptConfig {
      interrupts: InterruptEnables {
        time_update: enabled,
        countdown_timer: enabled,
        alarm: enabled,
        event: enabled,
        power_on_reset: enabled,
        backup_switchover: enabled,
      },
      clockout: ClkoutInterruptConfig {
        enabled,
        time_update: !enabled,
        countdown_timer: enabled,
        alarm: !enabled,
        event: enabled,
      },
    };
    // Exercise every byte of both shared mirrors, including the calibration bit.
    for byte in 0..=255 {
      let expected = routing_set_transactions(byte, byte, byte, config);
      let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
      rtc.set_interrupt_config(&config).unwrap();
      rtc.i2c.done();
    }
  }
}

#[test]
fn routing_setter_stops_on_each_bus_failure_without_acknowledgement_or_rollback() {
  let config = InterruptConfig::from_registers(0x7C, 0x0F, 0x08, 0x40);
  let successful = routing_set_transactions(0xFF, 0xC5, 0xB7, config);
  for index in 0..successful.len() {
    let mut expected = successful[..=index].to_vec();
    expected[index] = expected[index].clone().with_error(ErrorKind::Bus);
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    assert_eq!(
      rtc.set_interrupt_config(&config),
      Err(Error::I2c(ErrorKind::Bus))
    );
    rtc.i2c.done();
  }
}

#[test]
fn every_new_getter_preserves_errors_without_retry_or_write() {
  for successful in [
    vec![
      T::write(0x70, vec![1]),
      T::write_read(0x52, vec![0x10], vec![0; 4]),
    ],
    vec![
      T::write(0x70, vec![1]),
      T::write_read(0x52, vec![0x0F], vec![0]),
      T::write_read(0x52, vec![0x0A], vec![0; 2]),
    ],
    routing_get_transactions(0, 0, 0, 0),
  ] {
    for index in 0..successful.len() {
      let mut expected = successful[..=index].to_vec();
      expected[index] = expected[index].clone().with_error(ErrorKind::Bus);
      let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
      let result = match successful.len() {
        2 => rtc.get_evi_config().map(|_| ()),
        3 => rtc.get_countdown_timer_config().map(|_| ()),
        _ => rtc.get_interrupt_config().map(|_| ()),
      };
      assert_eq!(result, Err(Error::I2c(ErrorKind::Bus)));
      rtc.i2c.done();
    }
  }
}
