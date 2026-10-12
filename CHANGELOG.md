# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.6.1] - Unreleased

### Added
- Read-only `get_status()` with `StatusSnapshot` and decoded `StatusFlags`, plus
  `read_register` / `read_registers` for the raw 0x00–0x3F register map.
  Burst reads validate ranges before I/O, support no-op empty slices and never
  acknowledge flags or issue EEPROM commands. Reserved values remain unspecified.
- Typed timestamp source/capture/configuration and `TimestampSnapshot`, with
  separate settings update, explicit restart, log reset and read-only readout.
  Restart follows the vendor sequence, preserves EVI filtering and unrelated
  flags, and leaves EIE disabled. Compatibility methods share these implementations.
- Typed `BackupConfig`, `BackupSwitchoverMode`, `ClkoutConfig` and `ClkoutSource`,
  with active RAM-mirror getters/setters and pure register encoding/decoding.
  Backup updates preserve the EEOffset calibration bit; CLKOUT encoding clears
  unimplemented bits. EEPROM persistence remains explicit and separate.
- `read_user_ram(offset, &mut bytes)` and `write_user_ram(offset, &bytes)` for
  single-byte or two-byte accesses, with `USER_RAM_LEN = 2`. Requests are checked
  before any bus access; valid empty slices are no-ops. No alarm registers are
  repurposed. Tests cover exact transfers, boundaries, mux selection and bus errors.
- User EEPROM slice access with `read_user_eeprom(offset, bytes, delay)` and
  `write_user_eeprom(offset, bytes, delay)`, `USER_EEPROM_LEN = 0x2B`,
  pre-I/O range validation, no-op empty slices, write-only-changed programming
  and verification of the complete requested range.
- Single-byte `read_eeprom` / `write_eeprom` at absolute User or Configuration
  EEPROM addresses; reserved addresses are rejected.
- `write_configuration_eeprom`, `update_configuration_eeprom` and
  `refresh_configuration` for configuration mirrors and persistent settings.
- EEPROM protocol with caller-provided `embedded-hal::delay::DelayNs`, mandatory
  settling delays, bounded busy polling, preservation of the original EERD state,
  and `EepromError` / `EepromErrorKind` retaining operation and restoration failures.
  Existing non-EEPROM error types are unchanged.
- Individual calendar field setters: `set_seconds`, `set_minutes`, `set_hours`,
  `set_weekday`, `set_day`, `set_month` and `set_year`. Each validates its own
  range before I/O and leaves the independent Unix counter unchanged.
- `get_datetime_parts()` reads the complete calendar in one burst, validates
  BCD/date/time/weekday and normalizes the current 12-/24-hour encoding.

### Fixed
- Timestamp readout normalizes 12-hour AM/PM and retains valid captured time
  after Count TS wraps to zero. Invalid legacy timestamp sources are rejected
  before bus access; source changes pause capture before changing TSS.
- `toggle_plain_clockout` now addresses CLKOUT register 0x35 rather than Backup
  register 0x37; it no longer changes the oscillator calibration bit by mistake.
- `toggle_backup_switchover` replaces both BSM bits, selecting DSM when enabled
  and BSM=00 when disabled, including when starting from LSM or BSM=10.
- `get_hms()` uses the existing hour mode and normalizes 12-hour AM/PM values
  to hours 0–23, sharing the complete calendar reader's hour validation.
- All datetime setters write seconds through year in one coherent calendar
  transfer after writing the independent Unix counter. Both hour modes are
  encoded using the existing Control 2 setting, without modifying alarm setup.
- Status flag acknowledgements write zeros only to selected flags and ones to
  every unrelated writable flag, without read-modify-write. This prevents loss
  of unrelated events arriving between reads and acknowledgements in alarm,
  timer, event, timestamp and individual flag operations. EEbusy is read-only.

## [0.6.0]

### Changed
- **Breaking:** driver methods and backend traits now return `Error<I2C::Error>`:
  `I2c` preserves bus failures, `InvalidData` reports invalid input/register values,
  and `InconsistentRead` reports a counter that did not stabilize within four reads.
- Date/time and alarm setters validate inputs before bus access. Calendar setters
  require years 2000–2099. Invalid disabled alarm register fields are normalized
  to zero (or day 1 for date alarms) when read; invalid enabled fields return an error.
- **Breaking:** ported to `embedded-hal` 1.0. `RV3028<I2C>` now requires
  `I2C: embedded_hal::i2c::I2c` instead of the `embedded-hal` 0.2
  `Write + Read + WriteRead` traits. HALs that only implement `embedded-hal` 0.2
  are no longer supported.
- **Breaking:** updated `rtcc` to 0.4. The `DateTimeAccess` trait is unchanged, but it is
  a different trait from the one in `rtcc` 0.3. Import it via `rv3028c7_rtc::DateTimeAccess`
  or depend on `rtcc` 0.4.
- **Breaking:** moved to Rust edition 2024; the minimum supported Rust version is now 1.85.
- Removed the redundant error type parameter from the driver's `impl` blocks. Bus errors are
  preserved inside `Error::I2c`.
- Require `chrono` 0.4.45 and replace deprecated `chrono` APIs.
- Examples share the I2C bus with `embedded-hal-bus` instead of `shared-bus`, and use
  `linux-embedded-hal` 0.5, `ds323x` 0.7, `gpiocdev` 0.8 and `spin_sleep` 1.3.
- **Breaking:** `config_countdown_timer` takes and returns `core::time::Duration` instead of
  `chrono::Duration`, and `Duration` is no longer re-exported. Durations too long for the timer
  are clamped to its maximum instead of panicking.
- **Breaking:** `set_alarm`, `get_alarm_datetime_wday_matches` and `get_event_count_and_datetime`
  moved into the new `ChronoAccess` trait. `get_event_count_and_datetime` is no longer part of
  `EventTimeStampLogger`. Add `use rv3028c7_rtc::ChronoAccess;` to keep calling them.
- `chrono` and `rtcc` are optional dependencies, enabled by the new `chrono` feature (on by default).
  Without it, the driver depends only on `embedded-hal`.

### Added
- `check_and_clear_backup_event()` to check and clear the Backup Switch Flag (BSF).
- Optional `jiff` feature with the `JiffDateTimeAccess` trait: the clock, alarm and event timestamp
  using `jiff::civil` types. The `jiff` crate is re-exported as `rv3028c7_rtc::jiff`.
- `DateTimeParts` and `AlarmParts`, with `set_datetime_parts`, `get_event_count_and_parts`,
  `set_alarm_parts` and `get_alarm_parts`, for use without any date/time library.
- `jiff_alarm` example.
- Crate-level documentation, and docs for all public items.
- Tests that check the exact I2C traffic for setting the date and time, the alarm and the
  event timestamp, for each date/time backend.

### Fixed
- `datetime()` reads the unix time counter twice, as the application manual recommends, so a counter
  increment during the read can no longer return a torn value.
- The doc comments of `TS_EVENT_SOURCE_EVI` and `TS_EVENT_SOURCE_BSF` were attached to the wrong items.
- The `long_clock_setup` example now compiles.
- `test_get_unix_time` now tests the read-twice `get_unix_time_blocking()` it was written for.

## [0.5.2] and earlier

See the git history.
