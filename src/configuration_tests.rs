use super::*;
use embedded_hal::i2c::ErrorKind;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
use std::vec;

#[test]
fn every_backup_byte_decodes_and_encodes_without_losing_calibration() {
  for raw in 0..=u8::MAX {
    let config = BackupConfig::from_register(raw);
    let expected = if raw & 0x0C == 0x08 { raw & !0x0C } else { raw };
    assert_eq!(config.to_register(raw), expected);
    assert_eq!(config.to_register(raw ^ 0x80), expected ^ 0x80);
    let mode = match raw & 0x0C {
      0x04 => BackupSwitchoverMode::Direct,
      0x0C => BackupSwitchoverMode::Level,
      _ => BackupSwitchoverMode::Disabled,
    };
    assert_eq!(config.switchover_mode, mode);
    assert_eq!(config.switchover_interrupt_enabled, raw & 0x40 != 0);
    assert_eq!(config.trickle_charge_enabled, raw & 0x20 != 0);
    assert_eq!(config.fast_edge_detection_enabled, raw & 0x10 != 0);
    assert_eq!(config.trickle_charge_resistance as u8, raw & 3);
  }
}

#[test]
fn every_clkout_byte_decodes_all_sources_and_zeros_unimplemented_bits() {
  let sources = [
    ClkoutSource::Hertz32768,
    ClkoutSource::Hertz8192,
    ClkoutSource::Hertz1024,
    ClkoutSource::Hertz64,
    ClkoutSource::Hertz32,
    ClkoutSource::Hertz1,
    ClkoutSource::CountdownTimerInterrupt,
    ClkoutSource::ForcedLow,
  ];
  for raw in 0..=u8::MAX {
    let config = ClkoutConfig::from_register(raw);
    assert_eq!(config.source, sources[(raw & 7) as usize]);
    assert_eq!(config.enabled, raw & 0x80 != 0);
    assert_eq!(config.synchronized, raw & 0x40 != 0);
    assert_eq!(config.power_on_reset_interrupt_enabled, raw & 8 != 0);
    assert_eq!(config.to_register(), raw & 0xCF);
  }
}

#[test]
fn configuration_reads_select_mux_once_and_access_only_active_mirrors() {
  let expected = [
    T::write(0x70, vec![1]),
    T::write_read(0x52, vec![0x37], vec![0xBD]),
    T::write(0x70, vec![1]),
    T::write_read(0x52, vec![0x35], vec![0xC0]),
  ];
  let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
  assert_eq!(
    rtc.get_backup_config(),
    Ok(BackupConfig::from_register(0xBD))
  );
  assert_eq!(
    rtc.get_clkout_config(),
    Ok(ClkoutConfig::from_register(0xC0))
  );
  rtc.i2c.done();
}

#[test]
fn configuration_writes_preserve_calibration_without_status_or_eeprom_commands() {
  let backup = BackupConfig {
    switchover_mode: BackupSwitchoverMode::Level,
    switchover_interrupt_enabled: false,
    trickle_charge_enabled: true,
    trickle_charge_resistance: TrickleChargeCurrentLimiter::Ohms3k,
    fast_edge_detection_enabled: true,
  };
  let clkout = ClkoutConfig {
    source: ClkoutSource::ForcedLow,
    enabled: false,
    synchronized: true,
    power_on_reset_interrupt_enabled: true,
  };
  for current in [0x00, 0x80, 0x7F, 0xFF] {
    let expected = [
      T::write(0x70, vec![1]),
      T::write_read(0x52, vec![0x37], vec![current]),
      T::write(0x52, vec![0x37, (current & 0x80) | 0x3C]),
      T::write(0x70, vec![1]),
      T::write(0x52, vec![0x35, 0x4F]),
    ];
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    rtc.set_backup_config(&backup).unwrap();
    rtc.set_clkout_config(&clkout).unwrap();
    rtc.i2c.done();
  }
}

#[test]
fn legacy_clockout_toggle_uses_35h_and_never_touches_calibration_in_37h() {
  for (enable, initial, target) in [(true, 0x4D, 0xCD), (false, 0xCD, 0x4D)] {
    let expected = [
      T::write_read(0x52, vec![0x35], vec![initial]),
      T::write(0x52, vec![0x35, target]),
    ];
    let mut rtc = RV3028::new(Mock::new(&expected));
    rtc.toggle_plain_clockout(enable).unwrap();
    rtc.i2c.done();
  }
}

#[test]
fn legacy_backup_toggle_replaces_both_mode_bits_and_preserves_every_other_bit() {
  for current in 0..=u8::MAX {
    for enable in [false, true] {
      let target = (current & !0x0C) | if enable { 0x04 } else { 0 };
      let expected = [
        T::write_read(0x52, vec![0x37], vec![current]),
        T::write(0x52, vec![0x37, target]),
        T::write_read(0x52, vec![0x37], vec![target]),
      ];
      let mut rtc = RV3028::new(Mock::new(&expected));
      assert_eq!(rtc.toggle_backup_switchover(enable), Ok(enable));
      rtc.i2c.done();
    }
  }
}

#[test]
fn configuration_bus_failures_stop_without_retry_or_other_register_access() {
  let backup = BackupConfig::from_register(0x3C);
  let baseline = [
    T::write(0x70, vec![1]),
    T::write_read(0x52, vec![0x37], vec![0x80]),
    T::write(0x52, vec![0x37, 0xBC]),
  ];
  for failure in 0..3 {
    let mut expected = baseline[..=failure].to_vec();
    expected[failure] = expected[failure].clone().with_error(ErrorKind::Bus);
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    assert_eq!(
      rtc.set_backup_config(&backup),
      Err(Error::I2c(ErrorKind::Bus))
    );
    rtc.i2c.done();
  }
  let clkout = ClkoutConfig::from_register(0x47);
  let baseline = [T::write(0x70, vec![1]), T::write(0x52, vec![0x35, 0x47])];
  for failure in 0..2 {
    let mut expected = baseline[..=failure].to_vec();
    expected[failure] = expected[failure].clone().with_error(ErrorKind::Bus);
    let mut rtc = RV3028::new_with_mux(Mock::new(&expected), 0x70, 1);
    assert_eq!(
      rtc.set_clkout_config(&clkout),
      Err(Error::I2c(ErrorKind::Bus))
    );
    rtc.i2c.done();
  }
  for reg in [0x35, 0x37] {
    let expected = [T::write_read(0x52, vec![reg], vec![0]).with_error(ErrorKind::Bus)];
    let mut rtc = RV3028::new(Mock::new(&expected));
    if reg == 0x35 {
      assert_eq!(rtc.get_clkout_config(), Err(Error::I2c(ErrorKind::Bus)));
    } else {
      assert_eq!(rtc.get_backup_config(), Err(Error::I2c(ErrorKind::Bus)));
    }
    rtc.i2c.done();
  }
}
