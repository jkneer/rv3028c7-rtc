# rv3028c7-rtc

Rust `no_std` driver for RV-3028-C7 and similar Real Time Clocks (RTC)
manufactured by Micro Crystal AG, Switzerland. 
Based on the
[Application Manual](https://www.microcrystal.com/fileadmin/Media/Products/RTC/App.Manual/RV-3028-C7_App-Manual.pdf)
downloaded November 2023.

This driver provides many methods for reading and writing the i2c registers of the RTC,
but it does not fully exercise all the features of the RTC. 

The driver works with any I2C implementation of the [`embedded-hal`](https://crates.io/crates/embedded-hal)
1.0 `I2c` trait. The minimum supported Rust version is 1.85.

## Features

Dates and times can be handled with [`chrono`](https://crates.io/crates/chrono),
[`jiff`](https://crates.io/crates/jiff), or without any date/time library.

| Feature | Default | Provides |
|---|---|---|
| `chrono` | yes | `DateTimeAccess` (from [`rtcc`](https://crates.io/crates/rtcc)) and `ChronoAccess`, using `chrono::NaiveDateTime` |
| `jiff` | no | `JiffDateTimeAccess`, using `jiff::civil::DateTime` |
| (none) | | `DateTimeParts` and `AlarmParts` with `set_datetime_parts`, `set_alarm_parts` and friends |

Everything that doesn't involve a date or time, such as the countdown timer
(which takes a `core::time::Duration`), interrupts, trickle charging and the unix time counter,
is always available. Both features can be enabled at the same time.

## Usage

With `chrono` (the default):

```rust
use linux_embedded_hal::I2cdev;
use rv3028c7_rtc::{ChronoAccess, DateTimeAccess, RV3028};

let i2c = I2cdev::new("/dev/i2c-1").unwrap();
let mut rtc = RV3028::new(i2c);

let now = rtc.datetime().unwrap();
// alarm on the same day of the month, at the same hour and minute
rtc.set_alarm(&now, None, true, true, true).unwrap();
```

With `jiff` (`rv3028c7-rtc = { version = "0.6", default-features = false, features = ["jiff"] }`):

```rust
use rv3028c7_rtc::jiff::civil::date;
use rv3028c7_rtc::{JiffDateTimeAccess, RV3028};

let mut rtc = RV3028::new(i2c);
rtc.set_datetime(&date(2024, 3, 15).at(13, 45, 30, 0)).unwrap();
let now = rtc.datetime().unwrap();
```

On targets without atomic compare-and-swap, such as Cortex-M0 (`thumbv6m-none-eabi`), jiff relies on
[`portable-atomic`](https://crates.io/crates/portable-atomic), and your application has to choose how it
provides atomics, for example by enabling its `critical-section` feature.

Without a date/time library (`default-features = false`):

```rust
use rv3028c7_rtc::{DateTimeParts, RV3028};

let mut rtc = RV3028::new(i2c);
rtc.set_datetime_parts(&DateTimeParts {
    year: 2024, month: 3, day: 15, hour: 13, minute: 45, second: 30,
}).unwrap();
let unix_time = rtc.get_unix_time().unwrap();
```

To talk to several devices on one bus (for example several RTCs behind an i2c mux),
share the bus with [`embedded-hal-bus`](https://crates.io/crates/embedded-hal-bus):

```rust
use core::cell::RefCell;
use embedded_hal_bus::i2c::RefCellDevice;

let i2c_bus = RefCell::new(i2c);
let mut rtc1 = RV3028::new_with_mux(RefCellDevice::new(&i2c_bus), 0x70, 0b0000_0001);
let mut rtc2 = RV3028::new_with_mux(RefCellDevice::new(&i2c_bus), 0x70, 0b1000_0000);
```

Call these drivers sequentially. If tasks or threads can change the same mux's
channel, serialize entire driver methods, including EEPROM delays. A shared-bus
adapter's per-transaction lock does not protect mux selection and subsequent
RTC accesses together.

### Calendar access

`set_datetime_parts(&parts)` and both backend `set_datetime` methods write
all seven calendar registers (0x00–0x06) in one coherent burst, resetting the
prescaler through Seconds, then the Unix counter. They use the existing
12-/24-hour mode without changing Control 2 or alarm configuration.
Input hours are always 0–23.

`set_seconds`, `set_minutes`, `set_hours`, `set_weekday`, `set_day`,
`set_month` and `set_year` write individual calendar registers. Arguments are
ordinary binary values, with hours 0–23, years 2000–2099 and Monday=0 weekdays.
Only the supplied field is range-checked; other fields and the Unix counter
are unchanged. Date fields can temporarily form an invalid date, and changing
the date does not recalculate the weekday. Writing seconds resets subseconds;
the other field setters do not. Use the complete setter for clock synchronization.

`get_datetime_parts()` reads Control 2, then the complete calendar in one burst.
It normalizes 12-hour AM/PM to hours 0–23 and validates BCD, calendar date, time
and the Monday=0 weekday. It does not check or acknowledge PORF.

```rust
let calendar = rtc.get_datetime_parts()?;
rtc.set_datetime_parts(&calendar)?;
```

Backend `datetime()` methods still read the independent Unix counter, rather than
the calendar. Reading the date and time separately with `get_ymd()` and
`get_hms()` does not form one coherent full-calendar snapshot.

The [Application Manual](https://www.microcrystal.com/fileadmin/Media/Products/RTC/App.Manual/RV-3028-C7_App-Manual.pdf),
sections 4.5 and 4.16.1, requires the calendar transfer to finish in less than
950 ms for register blocking, and the Unix write's STOP to occur less than
950 ms after writing Seconds. The caller/HAL must enforce these deadlines,
including scheduling delays between the two writes.
Control 2 and the calendar are separate accesses; exclude concurrent hour-mode
changes. Unix counter and calendar writes are also separate: a failed operation
can leave them inconsistent. No rollback, retry, readback or flag clearing occurs.

### Hour mode and alarms

`get_hour_mode()` and `set_hour_mode(HourMode::Hours12 / Hours24)` expose
Control 2's 12_24 field. Calendar, timestamp and alarm APIs use civil hours 0–23
in either mode; the driver encodes and decodes AM/PM. Alarm setters/readers,
including the chrono and jiff backends, use the current mode and preserve each
alarm field's matching enable.

Alarm setters pause AIE before changing WADA or alarm fields, acknowledge AF,
then restore the original AIE state. Other Control 2 bits and CLKF are preserved.
Exclude concurrent mode/alarm/control changes. Bus failures stop without rollback
and can leave AIE disabled or the alarm partially configured.

The chip automatically converts its running Hours register on a mode change,
but requires Hours Alarm to be rewritten. The setter pauses AIE, changes the
mode, converts the stored alarm hour, then restores AIE. Other Control 2 bits,
alarm minute/day, Unix counter and status flags are preserved; RESET is never
issued. AF may still latch during the transition and is not acknowledged.

A real mode change requires timestamp capture disabled (`TSE=0`) and all seven
timestamp registers cleared; otherwise it returns `Error::InvalidData` before
writes. Captured timestamp hours are read-only and cannot be converted.
Consume the log, disable capture and explicitly reset the log before switching;
the owner decides whether to discard events and when to resume capture.
Requesting the existing mode only reads Control 2.

Invalid enabled alarm hours prevent a change; invalid disabled hours become
midnight with hour matching still disabled. Exclude concurrent mode, alarm and
capture changes for the entire operation. Bus failures stop immediately without
rollback and can leave AIE disabled, the new mode applied or the alarm awaiting
conversion; the owner must inspect and recover.

### Interrupt and CLKOUT routing

`get_interrupt_config()` and `set_interrupt_config(&config)` use
`InterruptConfig`, containing `InterruptEnables` and `ClkoutInterruptConfig`.
They cover UIE, TIE, AIE, EIE, PORIE, BSIE, CLKIE and all four Clock Interrupt
Mask bits CUIE, CTIE, CAIE, CEIE. The getter is read-only and never acknowledges
flags. Source-mask selection does not enable the corresponding internal interrupt;
configure its interrupt enable as required by the hardware interrupt scheme.

The setter pauses Control 2 output enables, merges PORIE/BSIE into their active
RAM mirrors, writes the source mask and applies the requested Control 2 enables.
It preserves TSE, hour mode, CLKOUT source/CLKOE/CLKSY, Backup/charger settings and
oscillator calibration; RESET is never issued. All flags are retained, so existing
flags may assert an output when enabled. It does not perform a glitch-free clock
transition, and retained CLKF can keep CLKOUT active.

PORIE/BSIE persistence requires the existing EEPROM API; automatic refresh may
replace their active values. Exclude concurrent shared-register changes and
EEPROM refresh throughout the operation. Failures stop without retry or rollback,
and may leave output enables paused or configuration partially applied.

### EVI filtering and countdown timer

`EviConfig` describes `EviPolarity::{FallingOrLow, RisingOrHigh}`,
`EviFilter::{EdgeDetection, Millis3Point9, Millis15Point6, Millis125}`, and the
shared EIE/CEIE output enables. `get_evi_config()` reads the settings in one
0x10–0x13 burst without writes or acknowledgements. `set_evi_config(&config)`
pauses EIE, acknowledges only EVF, applies detection/routing and restores the
requested EIE. Timestamp source, overwrite, capture enable and retained log
are preserved. EIE/CEIE use the source selected by TSS, including Backup
Switchover; they do not select TSS or enable CLKIE. The owner must pause
timestamp capture if changing detection while TSE is set is inappropriate.
Bus failures can leave partial settings and EIE paused; exclude concurrent
configuration changes. The raw-filter compatibility method delegates to this API.

`CountdownTimerConfig` contains `clock: TimerClockFreq`, `ticks: u16`,
`repeat` and `enabled`. `set_countdown_timer_config(&config)` accepts explicit
0–4095 tick presets; larger values are rejected before mux/I/O. It stops the
timer, writes clock/repeat/preset, acknowledges only TF, then optionally starts
a fresh countdown. It preserves other Control 1 fields, interrupt enables and
CLKOUT routing. Reapplying an enabled configuration restarts from its preset.
Failures can leave the timer stopped or partially configured.

`get_countdown_timer_config()` reads the preset and TE/TRPT/TD without writes
or flag acknowledgement. The separate `get_countdown_value()` reads the running
12-bit counter using the hardware shadow-register burst. TE can clear after a
one-shot expires, so readout is not an atomic snapshot of a running timer.
`CountdownTimerConfig::from_duration(&duration, repeat, enabled)` quantizes
without I/O; `nominal_period()` returns the preset period, or `None` for an
invalid preset. Duration-based configuration delegates to the typed setter.


`config_ext_event_detection(rising, int_enable, filtering, clockout_enable)`
takes an unshifted two-bit ET value: `0b00` for edge detection, or `0b01`,
`0b10`, `0b11` for level detection sampled at approximately 3.9 ms, 15.6 ms,
125 ms respectively. Values above `0b11` return `Error::InvalidData` before
mux selection or register access. EHL, EIE and CEIE are configured independently;
timestamp source, overwrite mode and enable are preserved. EVF is acknowledged.

`config_countdown_timer(&duration, repeat, start)` selects the finest clock whose
full 12-bit range includes the requested duration, then rounds down to whole
ticks. Its clocks are 4096 Hz, 64 Hz, 1 Hz and 1/60 Hz, with a maximum count of
4095. Positive durations shorter than one 4096-Hz tick are clamped to one tick;
durations above 4095 minutes are clamped to that maximum. Zero programs zero
ticks, for which the hardware does not count down.

The returned duration is the nominal period `ticks / frequency`, with fractional
nanoseconds truncated. For example, 500 ms programs 2048 ticks at 4096 Hz,
1 s programs 64 ticks at 64 Hz, and 2047 ms programs 131 ticks at 64 Hz and
returns 2046.875 ms. At clock-range boundaries, rounding to a coarser source can
shorten the nominal period. First-period latency also depends on start phase;
the return value is not a precise first-expiry deadline. See the
[Application Manual](https://www.microcrystal.com/fileadmin/Media/Products/RTC/App.Manual/RV-3028-C7_App-Manual.pdf),
sections 3.7–3.8 and 4.8.3.

### Backup and CLKOUT configuration

`BackupConfig` describes the BSM switchover mode (Disabled, Direct or Level),
BSIE interrupt enable, TCE charger enable, TCR series resistance and FEDE
fast-edge detection. `get_backup_config()` reads the active RAM mirror at 0x37;
`set_backup_config(&config)` reads it and preserves the oscillator calibration
bit EEOffset[0] when applying the supplied settings. Disabled BSM=10 is decoded
as Disabled and encoded as BSM=00. Keep FEDE enabled as specified by the vendor;
charging and switchover choices belong to the application and its backup supply.

`ClkoutConfig` covers register 0x35: CLKOE, CLKSY, PORIE and typed
`ClkoutSource`. Sources include all six frequencies (32768, 8192, 1024, 64, 32
and 1 Hz), the countdown timer interrupt and ForcedLow. CLKF can enable output
even with CLKOE disabled; ForcedLow holds the pin LOW independently of that flag.
Control 2 CLKIE and interrupt masks at 0x12 remain independent.

```rust
use rv3028c7_rtc::{BackupSwitchoverMode, ClkoutSource};

let mut backup = rtc.get_backup_config()?;
backup.switchover_mode = BackupSwitchoverMode::Disabled;
backup.trickle_charge_enabled = false;
backup.fast_edge_detection_enabled = true;
rtc.set_backup_config(&backup)?;

let mut clkout = rtc.get_clkout_config()?;
clkout.source = ClkoutSource::ForcedLow;
rtc.set_clkout_config(&clkout)?;
```

These setters change active RAM only: they do not persist EEPROM, alter EERD,
acknowledge flags, wait for LSM activation or guarantee glitch-free frequency
changes. Refresh can reload the persistent values. For persistence, use the
existing explicit EEPROM APIs; `BackupConfig::to_register(current)` preserves
the current calibration bit and `ClkoutConfig::to_register()` zeros unimplemented
bits 5:4. The owner decides which changes to persist and when. Exclude
concurrent refresh/calibration changes while updating Backup.

The compatibility helper `toggle_backup_switchover(true)` selects DSM, and
`false` selects BSM=00. Use the typed configuration to select LSM explicitly.
`toggle_plain_clockout` now correctly changes CLKOE at 0x35.

### Timestamp capture

`TimestampConfig` selects `TimestampSource::ExternalEvent` or
`BackupSwitchover`, `TimestampCapture::FirstEvent` or `LastEvent`, and TSE enable.
Configure EVI polarity/filtering or backup switchover separately; INT and CLKOUT
routing are independent.

- `get_timestamp_config()`: read settings without changing flags or log.
- `set_timestamp_config(&config)`: pause TSE/EIE, change source/policy, restore
  EIE and apply TSE; preserve the log and flags. FirstEvent still depends on EVF.
- `configure_timestamp(&config)`: explicitly restart with an empty log, following
  the vendor sequence: TSE/EIE off, TSOW, acknowledge only EVF/BSF, pulse TSR,
  select TSS, then TSE. EIE remains off.
- `reset_timestamp()`: pulse TSR only; does not acknowledge EVF or re-arm the
  FirstEvent latch.
- `get_timestamp()`: read Control 2 and then the complete 0x14–0x1A block in
  one burst, without flag acknowledgements or reset.

`TimestampSnapshot` contains the binary event count modulo 256 and an optional
validated `DateTimeParts`. Count zero can be wraparound; a valid calendar remains
available then. Only the entirely zero reset block means None. Hours are normalized
to 0–23 using the current hour mode; do not change that mode after capture.
The chip blocks capture while reading the timestamp block. An event after that
read may still be discarded by a subsequent explicit restart.

Configuration operations can miss events while paused. Bus failures are not
retried or rolled back; capture/interrupt state may remain changed. The driver
does not establish the accuracy or trust of the calendar at capture time.
Legacy event readers and backend converters delegate to the same reader, and
legacy configuration delegates to the typed restart implementation. Unknown
legacy source values return InvalidData before bus access.

### Technical status and raw registers

`get_status()` reads registers 0x0E–0x13 in one burst and returns
`StatusSnapshot`: decoded `StatusFlags`, raw Control 1/2, GP Bits,
Clock Interrupt Mask and Event Control. Flags expose PORF, EVF, AF, TF, UF,
BSF, CLKF and read-only EEbusy. The read acknowledges no flags, resets no log
and issues no EEPROM commands. It describes the registers at read time;
calendar validity and application decisions remain the owner's responsibility.
A burst does not make asynchronously changing registers an atomic snapshot.

`read_register(address)` reads one raw register.
`read_registers(address, &mut bytes)` reads a consecutive range in one burst.
Both accept the physical register map 0x00–0x3F, including reserved addresses
whose values are unspecified. Configuration addresses expose active RAM
mirrors, rather than persistent EEPROM. Write-only registers may read as zero.

```rust
let status = rtc.get_status()?;
let reset_seen = status.flags.power_on_reset();
let mut calendar_raw = [0; 7];
rtc.read_registers(0x00, &mut calendar_raw)?;
let backup_raw = rtc.read_register(0x37)?;
```

Ranges are checked before any bus access or buffer change, with no address
wrapping. Empty slices at starts 0x00–0x40 perform no I/O. HAL failures preserve
the original bus error and may leave a partially received buffer. Raw reads do
not perform BCD validation, counter stabilization or hour-mode conversion;
use the corresponding typed methods when those guarantees are needed.
The HAL must meet the chip's transfer deadlines for calendar/timestamp blocking.

`write_register(address, value)` is an explicit low-level escape hatch for
application-owned register uses. It sends the byte unchanged after checking
0x00–0x3F bounds. It performs no readback, field validation or EEPROM protocol.
The caller must choose a writable implemented register and obey its command,
write-zero-to-clear or RAM-mirror semantics; prefer typed methods otherwise.

### User RAM

The RTC has two general-purpose RAM bytes at registers 0x1F and 0x20.
They are retained on backup power and reset to zero on power-on reset.
The offset is relative to this two-byte region; alarm and timer registers are
not included.

```rust
let mut bytes = [0; 2];
rtc.read_user_ram(0, &mut bytes)?;
rtc.write_user_ram(0, &bytes)?;      // both bytes in one transfer
rtc.write_user_ram(0, &bytes[..1])?; // first byte only
bytes[1] = 0xA5;
rtc.write_user_ram(1, &bytes[1..])?; // second byte only
```

Out-of-range requests return `Error::InvalidData` before bus access.
Empty slices at offsets 0, 1 or 2 perform no I/O. Writes do not verify readback
or roll back partial transfers; record validity and commit ordering belong
to the application. Password-protected writes require the chip to be unlocked.

### EEPROM

Version 0.6.1 adds access to all 43 User EEPROM bytes (0x00–0x2A).
Like User RAM, the API takes an offset and a borrowed slice; a caller-provided
`embedded_hal::delay::DelayNs` implements the required waits.

```rust
let mut bytes = [0; 2];
rtc.read_user_eeprom(0, &mut bytes, &mut delay)?;
bytes[1] = 0xA5;
rtc.write_user_eeprom(1, &bytes[1..], &mut delay)?;
```

Writes compare stored values, program only changed bytes, then verify the entire
requested range. Empty slices at offsets 0 through 43 perform no I/O.
Out-of-range requests are rejected before mux selection or any other bus access.
Reads and writes proceed in ascending address order; they are not atomic.
Record layout, validity markers and commit ordering belong to the application.

`read_eeprom(address, delay)` and `write_eeprom(address, value, delay)` access
one byte at an absolute EEPROM address in 0x00–0x2A or 0x30–0x37.
Reserved addresses are rejected. Reading configuration this way reads persistent
EEPROM rather than its active RAM mirror. Writing it does not activate the change.

For configuration mirrors, the driver provides:

- `write_configuration_eeprom(address, bytes, delay)`: disable auto-refresh,
  edit the supplied RAM mirrors in 0x30–0x37, update all eight EEPROM configuration
  bytes, then verify the supplied bytes in EEPROM. Unspecified configuration
  bytes are persisted from their current mirrors, including password settings.
- `update_configuration_eeprom(delay)`: store all current configuration mirrors;
  this always programs and does not verify readback.
- `refresh_configuration(delay)`: reload all active configuration mirrors from
  EEPROM, replacing any unpersisted edits.

The chip's command protocol uses 1 ms settling for read/refresh and 10 ms for
write/update before polling EEbusy. Each idle wait allows at most 200 one-millisecond
intervals; HAL transaction timeouts remain the HAL's responsibility. The original
EERD state is restored only after confirming EEbusy=0, including after a failure.
Other Control 1 bits are preserved. EEPROM methods return `EepromError<I2C::Error>`:

- `Operation(kind)`: driver/bus failure, busy timeout, or readback mismatch.
- `RestoreBusy { operation }`: EEbusy is still set; restoring auto-refresh was
  skipped, retaining any original failure. EERD normally remains 1, but is
  unknown if the initial write to disable refresh failed. Confirm EEbusy=0
  before explicitly restoring EERD through register access.
- `Restore { operation, error }`: failure to restore EERD, retaining any original
  operation failure too. This includes a failed idle check; EERD is then unknown.

After a timeout or bus failure, a command may still be running and data may be
partially written. There is no automatic retry or rollback. VDD must stay at least
1.5 V throughout programming and the chip must be operating from VDD. Unlock
password protection before protected accesses; changing password configuration
may require unlocking again for verification. The driver cannot ensure supply
voltage or unlock the chip automatically.

See the [Application Manual](https://www.microcrystal.com/fileadmin/Media/Products/RTC/App.Manual/RV-3028-C7_App-Manual.pdf),
sections 4.6.3–4.6.9, for the hardware protocol and operating conditions.

### Errors

Non-EEPROM fallible driver methods return `rv3028c7_rtc::Error<I2C::Error>`:

- `Error::I2c(error)` preserves the underlying HAL error.
- `Error::InvalidData` reports an invalid input/register value or an out-of-range memory access.
- `Error::InconsistentRead` means no consecutive Unix counter reads agreed within
  four reads. The HAL remains responsible for individual transaction timeouts.

Date/time setters require valid dates in 2000–2099 and validate before any bus
access. Alarm setters validate all fields, even disabled ones. Alarm reads preserve
valid disabled values, replacing invalid disabled values with zero (or day 1 for
date alarms). Invalid enabled fields return an error. Subseconds are ignored.

### Upgrading from 0.5

- Explicit result types must use `Error<I2C::Error>` instead of `I2C::Error`;
  match `Error::I2c(inner)` to inspect bus failures.
- Version 0.6 moves from `embedded-hal` 0.2 to 1.0, so your HAL must provide an
  `embedded-hal` 1.0 I2C implementation. If you used `shared-bus`, switch to `embedded-hal-bus`
  as shown above.
- `set_alarm`, `get_alarm_datetime_wday_matches` and `get_event_count_and_datetime` are now
  methods of the `ChronoAccess` trait: add `use rv3028c7_rtc::ChronoAccess;`.
- `config_countdown_timer` takes and returns a `core::time::Duration` instead of a `chrono::Duration`,
  e.g. `Duration::from_millis(1500)` instead of `Duration::milliseconds(1500)`.
- If you import `DateTimeAccess` from `rtcc` directly, use `rtcc` 0.4.
- If you use `default-features = false`, add `features = ["chrono"]` to keep the chrono API.

See the [changelog](./CHANGELOG.md) for details.

## Remaining API coverage

The following chip functions do not yet have dedicated driver APIs. Raw register
and EEPROM access is available, but does not implement their field validation,
read-modify-write rules or complete operating sequences:

- **Frequency offset correction (EEOffset):** no signed 9-bit correction type,
  getter/setter or conversion between correction steps and ppm. A future setter
  must merge registers 0x36 and 0x37 without changing Backup settings. Existing
  Backup setters preserve the shared calibration bit; EEPROM persistence is
  already available separately. Preserve the factory calibration unless an
  explicit replacement has been determined.
- **Password/write protection:** no dedicated unlock, password-change or
  protection-enable/disable sequence. Generic EEPROM methods do not unlock
  protected accesses automatically.
- **Explicit prescaler RESET command:** no dedicated method for Control 2 RESET
  and its documented timing effects. Calendar setters already reset the
  prescaler by writing Seconds; timestamp-log reset is implemented separately.
`EviConfig` with typed polarity/filter, `CountdownTimerConfig` with explicit
clock/tick presets, and `InterruptConfig` with all six INT enables and all four
CLKOUT masks are implemented, including configuration getters. Duration-based
timer and raw-filter EVI compatibility methods use the same implementations.
Hour-mode switching, 12-hour alarm encoding, timestamp logging, Backup/CLKOUT
configuration, calendar access and User RAM/EEPROM access are also implemented.
Reference: Application Manual rev. 1.4, especially sections 4.15, 4.17 and 4.18.
Drift measurement, calibration policy and board-specific defaults belong to the
owner, rather than being missing hardware-driver features.

## Running examples

All of the examples have been run and tested on a raspberry pi running linux, 
for example with the command line:
```
cargo run --example rpil
 ```
from the linux command line.

### Single RTC examples
These examples all assume there is a single RTC connected directly to a raspberry pi-like linux host.
- [`rpil`](./examples/rpil.rs) demonstrates some basic interactions with the RTC attached to eg a Raspberry Pi host.
- [`event_log`](./examples/ext_evt.rs) shows how to configure and enable event logging (for example, 
detecting interrupt signals on the EVI pin)
- [`alarm_int`](./examples/alarm_int.rs) shows how to set and get the alarm, and check for alarm trigger
- [`trickle`](./examples/trickle.rs) shows how to enable and disable trickle charging, as well as enable
Vbackup switchover
- [`countdown`](./examples/countdown.rs) demonstrates setting the Periodic Countdown Timer for repeating and
one-shot modes.
- [`set_alarm`](./examples/set_alarm.rs) shows setting the alarm
- [`jiff_alarm`](./examples/jiff_alarm.rs) sets the clock and an alarm using `jiff` types instead of `chrono`
  (run with `--no-default-features --features jiff`)
- [`countdown_int`](./examples/countdown_int.rs) shows the Periodic Countdown Timer triggering the INT pin
- [`long_clock_setup`](./examples/long_clock_setup.rs) enables backup switchover and logs switchover events
while watching the clock over a long period


### Multiple RTC examples
These examples all assume that there are multiple RTCs connected to the host
via an i2c mux (something like the TCA9548A)
- [`muxit`](./examples/muxit.rs) demonstrates talking to two RTCs (with the same i2c address) via TCA9548A-like i2c mux.
  See the [high-level wiring diagram schematic](./res/dual-rtc-schematic.pdf).
- [`comp_mux`](./examples/comp_mux.rs) compares the output of four independent RTCs (two each of RV-3028-C7 and DS3231)
  and detects when they drift apart (usually after multiple days).
  See the associated [quad RTC wiring diagram](./res/comp-quad-rtc-mux.pdf).
- [`trickle_mux`](./examples/trickle_mux.rs) shows how to configure and confirm the backup voltage supply trickle charger.

## Testing

```
RUST_BACKTRACE=1 cargo test --tests
```
This will build and run _only_ the tests, on any host platform. 
These tests do not require a real hardware RTC to be connected to the host. 
Note that plain `cargo test` currently also builds all examples,
and if you're testing on a non-linux platform the `linux_embedded_hal`-based examples will fail to build. 

To check that the examples compile from a non-linux host, build them for a linux target:
```
rustup target add aarch64-unknown-linux-gnu
cargo check --examples --target aarch64-unknown-linux-gnu
```



## Funstuff

Here's a breadboard with four RTCs connected via an i2c mux (for the RTC [comparison example](./examples/comp_mux.rs))
![](./res/quad-rtc-drift.jpg)
