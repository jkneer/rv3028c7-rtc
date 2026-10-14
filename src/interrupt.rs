//! Typed INT enables and interrupt-controlled CLKOUT routing.
//! Application Manual 3.7, 3.15.4, 3.15.6 and 4.4–4.13.
use crate::{Error, REG_CLOCK_INTERRUPT_MASK, REG_CONTROL2, RV3028};
use embedded_hal::i2c::I2c;

const CLKOUT: u8 = 0x35;
const BACKUP: u8 = 0x37;
const CONTROL_OUTPUTS: u8 = 0b0111_1100;
const RESET: u8 = 0b0000_0001;
const PORIE: u8 = 0b0000_1000;
const BSIE: u8 = 0b0100_0000;

/// All six INT-output enables. Event refers to EI, selected by TSS.
/// These enables do not acknowledge flags or enable the underlying functions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterruptEnables {
  /// UIE (Control 2): periodic second/minute update.
  pub time_update: bool,
  /// TIE (Control 2): countdown expiry.
  pub countdown_timer: bool,
  /// AIE (Control 2): alarm match.
  pub alarm: bool,
  /// EIE (Control 2): event selected by TSS (EVI or Backup Switchover).
  pub event: bool,
  /// PORIE (CLKOUT mirror 0x35): power-on reset.
  pub power_on_reset: bool,
  /// BSIE (Backup mirror 0x37): backup switchover; independent of BSM.
  pub backup_switchover: bool,
}

/// Interrupt-controlled CLKOUT master enable and all four source masks.
///
/// These fields are distinct from INT enables, normal CLKOE, FD, CLKSY and
/// retained CLKF. Selecting a mask does not enable its internal interrupt source:
/// enable UIE/TIE/AIE/EIE as required by the interrupt scheme.
/// PORIE/BSIE do not have separate masks in register 0x12. The event source EI
/// follows TSS and can originate from EVI or Backup Switchover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClkoutInterruptConfig {
  /// CLKIE (Control 2): enable interrupt-controlled clock output.
  pub enabled: bool,
  /// CUIE (mask bit 0): periodic update source.
  pub time_update: bool,
  /// CTIE (mask bit 1): countdown source.
  pub countdown_timer: bool,
  /// CAIE (mask bit 2): alarm source.
  pub alarm: bool,
  /// CEIE (mask bit 3): selected event source.
  pub event: bool,
}

impl ClkoutInterruptConfig {
  /// Encode only implemented Clock Interrupt Mask bits. No bus access.
  pub const fn to_mask(self) -> u8 {
    (self.time_update as u8)
      | ((self.countdown_timer as u8) << 1)
      | ((self.alarm as u8) << 2)
      | ((self.event as u8) << 3)
  }

  /// Decode CLKIE from Control 2 and implemented mask bits.
  pub const fn from_registers(control2: u8, mask: u8) -> Self {
    Self {
      enabled: control2 & 0b0100_0000 != 0,
      time_update: mask & 0b0000_0001 != 0,
      countdown_timer: mask & 0b0000_0010 != 0,
      alarm: mask & 0b0000_0100 != 0,
      event: mask & 0b0000_1000 != 0,
    }
  }
}

/// INT-output enables and interrupt-controlled CLKOUT routing.
///
/// PORIE/BSIE reside in active configuration RAM mirrors. Persistence is separate
/// via EEPROM methods, and a refresh can replace those two settings. No board
/// defaults, source enabling, event acknowledgement or calendar trust is implied.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterruptConfig {
  /// INT-output enables.
  pub interrupts: InterruptEnables,
  /// CLKOUT interrupt sources and CLKIE.
  pub clockout: ClkoutInterruptConfig,
}

impl InterruptConfig {
  /// Decode raw Control 2, Clock Interrupt Mask, CLKOUT and Backup mirror bytes.
  pub const fn from_registers(control2: u8, mask: u8, clkout: u8, backup: u8) -> Self {
    Self {
      interrupts: InterruptEnables {
        time_update: control2 & 0b0010_0000 != 0,
        countdown_timer: control2 & 0b0001_0000 != 0,
        alarm: control2 & 0b0000_1000 != 0,
        event: control2 & 0b0000_0100 != 0,
        power_on_reset: clkout & PORIE != 0,
        backup_switchover: backup & BSIE != 0,
      },
      clockout: ClkoutInterruptConfig::from_registers(control2, mask),
    }
  }

  /// Merge output enables into Control 2, preserving TSE and hour mode.
  /// RESET is always zero: decoding/reapplying config must never issue a command.
  pub const fn to_control2(self, current: u8) -> u8 {
    (current & !(CONTROL_OUTPUTS | RESET))
      | ((self.clockout.enabled as u8) << 6)
      | ((self.interrupts.time_update as u8) << 5)
      | ((self.interrupts.countdown_timer as u8) << 4)
      | ((self.interrupts.alarm as u8) << 3)
      | ((self.interrupts.event as u8) << 2)
  }
}

impl<I: I2c> RV3028<I> {
  /// Read all routing/enables without writes, flag acknowledgement or EEPROM I/O.
  /// Separate reads are not atomic; exclude concurrent configuration changes.
  ///
  /// # Errors
  /// [Error::I2c] on mux or bus failure.
  pub fn get_interrupt_config(&mut self) -> Result<InterruptConfig, Error<I::Error>> {
    self.select_mux_channel()?;
    let control = self.read_register_raw(REG_CONTROL2)?;
    let mask = self.read_register_raw(REG_CLOCK_INTERRUPT_MASK)?;
    let clkout = self.read_register_raw(CLKOUT)?;
    let backup = self.read_register_raw(BACKUP)?;
    Ok(InterruptConfig::from_registers(
      control, mask, clkout, backup,
    ))
  }

  /// Apply INT enables and interrupt-controlled CLKOUT masks without clearing flags.
  ///
  /// Reads shared registers before writes. Pauses Control 2 output enables,
  /// merges PORIE/BSIE into their RAM mirrors, writes the mask, then applies
  /// requested Control 2 outputs. TSE, hour mode, CLKOUT frequency/CLKOE/CLKSY,
  /// Backup/charger/calibration settings and all status flags are preserved.
  /// RESET is never issued and EEPROM is not programmed.
  ///
  /// This is not atomic or a glitch-free CLKOUT sequence: existing CLKF can
  /// keep CLKOUT active. PORIE/BSIE are changed separately, and retained flags
  /// may immediately assert an output when enabled. Exclude concurrent routing,
  /// shared-register and EEPROM-refresh changes. Failures stop without rollback;
  /// output enables can remain paused and configuration partially applied.
  ///
  /// # Errors
  /// [Error::I2c] on mux or bus failure.
  pub fn set_interrupt_config(&mut self, config: &InterruptConfig) -> Result<(), Error<I::Error>> {
    self.select_mux_channel()?;
    let control = self.read_register_raw(REG_CONTROL2)?;
    let clkout = self.read_register_raw(CLKOUT)?;
    let backup = self.read_register_raw(BACKUP)?;
    self.write_register_raw(REG_CONTROL2, control & !(CONTROL_OUTPUTS | RESET))?;
    self.write_register_raw(
      CLKOUT,
      (clkout & !PORIE)
        | if config.interrupts.power_on_reset {
          PORIE
        } else {
          0
        },
    )?;
    self.write_register_raw(
      BACKUP,
      (backup & !BSIE)
        | if config.interrupts.backup_switchover {
          BSIE
        } else {
          0
        },
    )?;
    self.write_register_raw(REG_CLOCK_INTERRUPT_MASK, config.clockout.to_mask())?;
    self.write_register_raw(REG_CONTROL2, config.to_control2(control))
  }
}
