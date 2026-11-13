// SPDX-License-Identifier: GPL-2.0

//! Driver for the Lion Semiconductor LN8000 direct-charge pump (a switched-capacitor fast
//! charger that sits beside the main charger).
//!
//! Written from scratch in Rust; the register map and the ADC formulas come from the GPL v2
//! vendor driver (Lion Semiconductor and Xiaomi, 2021, `ln8000_charger.c` of the `rosemary-r-oss`
//! kernel). No datasheet is public, so everything below is only as good as that source.
//!
//! The chip has 8-bit register addresses and 8-bit values, so SMBus byte access is enough; no
//! regmap is needed.
//!
//! What it does: it identifies the chip, puts it into standby with its protections enabled,
//! applies optional limits from the device tree, and reports one mains supply with the standard
//! properties (input voltage and current, die temperature, health, charge type, status, and the
//! float voltage and input limits, which are writable).
//!
//! What it deliberately does not do yet: the interrupt line, the NTC monitors, the watchdog, the
//! second (secondary) chip of a dual setup, and the two undocumented sequences of the vendor
//! driver (a soft reset through a magic key, and a switching-frequency "trim" value).
//!
//! Device tree properties (all optional; absent means the chip's power-on value is kept):
//!
//! * `lionsemi,float-voltage-microvolt`: battery float voltage, 3725000 to 5000000, 5 mV steps.
//! * `lionsemi,input-current-limit-microamp`: input current limit, 500000 to 5000000, 50 mA steps.
//! * `lionsemi,input-overvoltage-microvolt`: input over-voltage threshold, one of the chip's
//!   levels (6.5, 11, 12 or 13 V); the largest level not above the value is used.

use kernel::{
    device::{
        Bound,
        Core, //
    },
    i2c::{
        self,
        I2cClient, //
    },
    io::Io,
    of,
    power_supply::{
        self,
        ChargeType,
        Health,
        Property,
        PropertyValue,
        Registration,
        Status,
        Type, //
    },
    prelude::*,
    sync::{
        new_mutex,
        Mutex, //
    },
};

/// Register offsets.
mod reg {
    pub(super) const DEVICE_ID: usize = 0x00;
    pub(super) const SYS_STS: usize = 0x03;
    pub(super) const SAFETY_STS: usize = 0x04;
    pub(super) const FAULT1_STS: usize = 0x05;
    pub(super) const FAULT2_STS: usize = 0x06;
    /// Input current ADC; the others follow at the next register pair.
    pub(super) const ADC_IIN: usize = 0x09;
    pub(super) const ADC_VIN: usize = 0x0B;
    pub(super) const ADC_DIE_TEMP: usize = 0x0F;
    pub(super) const IIN_CTRL: usize = 0x1B;
    pub(super) const REGULATION_CTRL: usize = 0x1C;
    pub(super) const SYS_CTRL: usize = 0x1E;
    pub(super) const GLITCH_CTRL: usize = 0x20;
    pub(super) const FAULT_CTRL: usize = 0x21;
    pub(super) const ADC_CTRL: usize = 0x23;
    pub(super) const ADC_CFG: usize = 0x24;
    pub(super) const TIMER_CTRL: usize = 0x26;
    pub(super) const V_FLOAT_CTRL: usize = 0x28;
}

const DEVICE_ID: u8 = 0x42;

// SYS_STS
const SYS_STS_BYPASS: u8 = 1 << 3;
const SYS_STS_SWITCHING: u8 = 1 << 2;
// SAFETY_STS
const SAFETY_TEMP_MAX: u8 = 1 << 6;
// FAULT1_STS
const FAULT1_WATCHDOG: u8 = 1 << 7;
const FAULT1_VBAT_OV: u8 = 1 << 6;
const FAULT1_VAC_UNPLUG: u8 = 1 << 4;
const FAULT1_VAC_OV: u8 = 1 << 3;
const FAULT1_VIN_OV: u8 = 1 << 1;
// FAULT2_STS
const FAULT2_IIN_OC: u8 = 1 << 7;
// SYS_CTRL
const SYS_CTRL_STANDBY_EN: u8 = 1 << 3;
const SYS_CTRL_REV_IIN_DET: u8 = 1 << 2;
const SYS_CTRL_EN_1TO1: u8 = 1 << 0;
// REGULATION_CTRL: the "disable" bits are 1 = off, the "enable" bits are 1 = on.
const REG_CTRL_DISABLE_VFLOAT_LOOP: u8 = 1 << 5;
const REG_CTRL_DISABLE_IIN_LOOP: u8 = 1 << 4;
const REG_CTRL_TEMP_MAX_EN: u8 = 1 << 2;
const REG_CTRL_TEMP_REG_EN: u8 = 1 << 1;
// FAULT_CTRL: 1 = protection disabled.
const FAULT_CTRL_DISABLE_IIN_OCP: u8 = 1 << 6;
const FAULT_CTRL_DISABLE_VBAT_OV: u8 = 1 << 5;
// TIMER_CTRL: freezes the ADC result registers so a two-byte value is read coherently.
const TIMER_CTRL_PAUSE_ADC: u8 = 1 << 1;
// ADC_CTRL: mode in bits 7:5, hibernate delay in bits 4:3 (bits 1:0 belong to the NTC alarm).
const ADC_CTRL_MODE_SHIFT: u8 = 5;
const ADC_CTRL_MODE_MASK: u8 = 0x7 << ADC_CTRL_MODE_SHIFT;
const ADC_CTRL_DELAY_MASK: u8 = 0x3 << 3;
const ADC_MODE_AUTO_HIBERNATE: u8 = 0;
const ADC_MODE_SHUTDOWN: u8 = 2;
const ADC_HIBERNATE_4S: u8 = 3 << 3;
// GLITCH_CTRL: input over-voltage level in bits 3:2.
const GLITCH_CTRL_VAC_OVP_MASK: u8 = 0x3 << 2;

// Electrical units, all in micro-units.
const VFLOAT_MIN: u32 = 3_725_000;
const VFLOAT_MAX: u32 = 5_000_000;
const VFLOAT_STEP: u32 = 5_000;
const IIN_STEP: u32 = 50_000;
const IIN_MIN: u32 = 500_000;
/// The input current ADC spans 0 to 5 A; a limit above that is refused (our choice: the vendor
/// code accepts anything the 7-bit field can hold).
const IIN_MAX: u32 = 5_000_000;
const ADC_VIN_STEP: i32 = 16_000;
const ADC_IIN_STEP: i32 = 4_890;
/// The input over-voltage levels the chip offers, indexed by the register field.
const VAC_OVP_LEVELS: [u32; 4] = [6_500_000, 11_000_000, 12_000_000, 13_000_000];

/// Input voltage in microvolts from the two ADC bytes.
fn vin_microvolt(lo: u8, hi: u8) -> i32 {
    ((i32::from(hi & 0x3f) << 4) | i32::from(lo >> 4)) * ADC_VIN_STEP
}

/// Input current in microamperes from the two ADC bytes.
fn iin_microamp(lo: u8, hi: u8) -> i32 {
    ((i32::from(hi & 0x03) << 8) | i32::from(lo)) * ADC_IIN_STEP
}

/// Die temperature in tenths of a degree Celsius from the two ADC bytes.
///
/// The vendor formula is `(935 - code) * 1000 / 2300`; its result is in degrees (0.435 degrees
/// per step, and the codes 567 to 992 then cover the chip's -25 to 160 degree range), although
/// the vendor code names it deci-degrees and clamps it as such. Degrees are assumed here: this
/// is inferred from the numbers, not from a datasheet, and to be checked on real hardware.
fn die_temp_decicelsius(lo: u8, hi: u8) -> i32 {
    let code = (i32::from(hi & 0x0f) << 6) | i32::from(lo >> 2);

    ((935 - code) * 10_000 / 2_300).clamp(-250, 1600)
}

/// Register access without locking: the caller serialises compound sequences.
#[derive(Clone, Copy)]
struct Chip<'a> {
    client: &'a I2cClient<Bound>,
}

impl Chip<'_> {
    fn read(&self, offset: usize) -> Result<u8> {
        self.client.smbus_io().try_read8(offset)
    }

    fn write(&self, offset: usize, value: u8) -> Result {
        self.client.smbus_io().try_write8(value, offset)
    }

    /// Replaces the bits of `mask` with those of `value`.
    fn update(&self, offset: usize, mask: u8, value: u8) -> Result {
        self.client
            .smbus_io()
            .try_update(offset, |v: u8| (v & !mask) | (value & mask))
    }

    /// Reads a two-byte ADC result coherently (the registers are frozen while reading).
    fn adc(&self, offset: usize) -> Result<(u8, u8)> {
        self.update(reg::TIMER_CTRL, TIMER_CTRL_PAUSE_ADC, TIMER_CTRL_PAUSE_ADC)?;
        let pair: Result<(u8, u8)> = (|| Ok((self.read(offset)?, self.read(offset + 1)?)))();
        // Always let the ADC run again, even when the read failed.
        let resume = self.update(reg::TIMER_CTRL, TIMER_CTRL_PAUSE_ADC, 0);

        let pair = pair?;
        resume?;
        Ok(pair)
    }

    fn set_standby(&self, standby: bool) -> Result {
        let value = if standby { SYS_CTRL_STANDBY_EN } else { 0 };

        self.update(reg::SYS_CTRL, SYS_CTRL_STANDBY_EN, value)
    }

    fn in_standby(&self) -> Result<bool> {
        Ok(self.read(reg::SYS_CTRL)? & SYS_CTRL_STANDBY_EN != 0)
    }

    fn float_voltage(&self) -> Result<u32> {
        Ok(VFLOAT_MIN + u32::from(self.read(reg::V_FLOAT_CTRL)?) * VFLOAT_STEP)
    }

    fn set_float_voltage(&self, microvolt: u32) -> Result {
        if !(VFLOAT_MIN..=VFLOAT_MAX).contains(&microvolt) {
            return Err(EINVAL);
        }
        let code = (microvolt - VFLOAT_MIN) / VFLOAT_STEP;

        self.write(reg::V_FLOAT_CTRL, u8::try_from(code).map_err(|_| EINVAL)?)
    }

    fn input_current_limit(&self) -> Result<u32> {
        let limit = u32::from(self.read(reg::IIN_CTRL)? & 0x7f) * IIN_STEP;

        Ok(limit.max(IIN_MIN))
    }

    fn set_input_current_limit(&self, microamp: u32) -> Result {
        if !(IIN_MIN..=IIN_MAX).contains(&microamp) {
            return Err(EINVAL);
        }

        // At most 100, so it fits the 7-bit field.
        self.update(reg::IIN_CTRL, 0x7f, (microamp / IIN_STEP) as u8)
    }

    fn input_overvoltage(&self) -> Result<u32> {
        let field = (self.read(reg::GLITCH_CTRL)? & GLITCH_CTRL_VAC_OVP_MASK) >> 2;

        Ok(VAC_OVP_LEVELS[usize::from(field)])
    }

    /// Selects the largest level that does not exceed `microvolt`.
    fn set_input_overvoltage(&self, microvolt: u32) -> Result {
        let field = VAC_OVP_LEVELS
            .iter()
            .rposition(|&level| level <= microvolt)
            .ok_or(EINVAL)?;

        self.update(
            reg::GLITCH_CTRL,
            GLITCH_CTRL_VAC_OVP_MASK,
            (field as u8) << 2,
        )
    }

    /// Brings the chip into a known safe state: standby, protections on, ADC running.
    fn init(&self) -> Result {
        self.set_standby(true)?;

        // Protections on: input over-current and battery over-voltage, both regulation loops,
        // die temperature protection and regulation, reverse input current detection.
        self.update(
            reg::FAULT_CTRL,
            FAULT_CTRL_DISABLE_IIN_OCP | FAULT_CTRL_DISABLE_VBAT_OV,
            0,
        )?;
        self.update(
            reg::REGULATION_CTRL,
            REG_CTRL_DISABLE_VFLOAT_LOOP
                | REG_CTRL_DISABLE_IIN_LOOP
                | REG_CTRL_TEMP_MAX_EN
                | REG_CTRL_TEMP_REG_EN,
            REG_CTRL_TEMP_MAX_EN | REG_CTRL_TEMP_REG_EN,
        )?;
        self.update(reg::SYS_CTRL, SYS_CTRL_REV_IIN_DET, SYS_CTRL_REV_IIN_DET)?;

        // ADC: the mode may only be changed while the ADC is shut down, so go through shutdown,
        // set the hibernate delay and the channels, then hand over to automatic mode.
        self.update(
            reg::ADC_CTRL,
            ADC_CTRL_MODE_MASK,
            ADC_MODE_SHUTDOWN << ADC_CTRL_MODE_SHIFT,
        )?;
        self.update(reg::ADC_CTRL, ADC_CTRL_DELAY_MASK, ADC_HIBERNATE_4S)?;
        self.write(reg::ADC_CFG, 0xff)?;
        self.update(
            reg::ADC_CTRL,
            ADC_CTRL_MODE_MASK,
            ADC_MODE_AUTO_HIBERNATE << ADC_CTRL_MODE_SHIFT,
        )
    }
}

/// The supply: answers queries by reading the chip.
#[pin_data]
struct Charger<'a> {
    chip: Chip<'a>,
    /// Serialises read-modify-write cycles and the pause/read/resume ADC sequence.
    #[pin]
    lock: Mutex<()>,
}

// SAFETY: the bound client stays valid for as long as the supply is registered, which the driver
// data guarantees (it owns the registration and is dropped at unbind); transfers on one client
// are serialised by the I2C core, and compound sequences by `lock`.
unsafe impl Send for Charger<'_> {}
// SAFETY: as above, shared access only starts SMBus transfers.
unsafe impl Sync for Charger<'_> {}

/// What the four status registers say.
struct State {
    sys: u8,
    safety: u8,
    fault1: u8,
    fault2: u8,
}

impl State {
    fn charging(&self) -> bool {
        self.sys & (SYS_STS_SWITCHING | SYS_STS_BYPASS) != 0
    }

    fn health(&self) -> Health {
        if self.safety & SAFETY_TEMP_MAX != 0 {
            Health::OVERHEAT
        } else if self.fault1 & (FAULT1_VBAT_OV | FAULT1_VAC_OV | FAULT1_VIN_OV) != 0 {
            Health::OVERVOLTAGE
        } else if self.fault2 & FAULT2_IIN_OC != 0 {
            Health::OVERCURRENT
        } else if self.fault1 & FAULT1_WATCHDOG != 0 {
            Health::WATCHDOG_TIMER_EXPIRE
        } else {
            Health::GOOD
        }
    }
}

impl Charger<'_> {
    fn state(&self) -> Result<State> {
        Ok(State {
            sys: self.chip.read(reg::SYS_STS)?,
            safety: self.chip.read(reg::SAFETY_STS)?,
            fault1: self.chip.read(reg::FAULT1_STS)?,
            fault2: self.chip.read(reg::FAULT2_STS)?,
        })
    }

    fn standby(&self) -> Result {
        let _guard = self.lock.lock();

        self.chip.set_standby(true)
    }
}

#[vtable]
impl power_supply::Driver for Charger<'_> {
    const NAME: &'static CStr = c"ln8000";
    const TYPE: Type = Type::MAINS;
    const PROPERTIES: &'static [Property] = &[
        Property::MANUFACTURER,
        Property::MODEL_NAME,
        Property::STATUS,
        Property::ONLINE,
        Property::HEALTH,
        Property::CHARGE_TYPE,
        Property::VOLTAGE_NOW,
        Property::CURRENT_NOW,
        Property::TEMP,
        Property::CONSTANT_CHARGE_VOLTAGE,
        Property::INPUT_CURRENT_LIMIT,
        Property::INPUT_VOLTAGE_LIMIT,
    ];

    fn get_property(&self, prop: Property, val: &PropertyValue) -> Result {
        match prop {
            Property::MANUFACTURER => val.set_str(c"Lion Semiconductor"),
            Property::MODEL_NAME => val.set_str(c"LN8000"),
            Property::STATUS => val.set_int(
                if self.state()?.charging() {
                    Status::CHARGING
                } else {
                    Status::NOT_CHARGING
                }
                .as_raw(),
            ),
            // Online means the input is present.
            Property::ONLINE => {
                val.set_int(i32::from(self.state()?.fault1 & FAULT1_VAC_UNPLUG == 0))
            }
            Property::HEALTH => val.set_int(self.state()?.health().as_raw()),
            Property::CHARGE_TYPE => {
                let state = self.state()?;
                val.set_int(
                    if !state.charging() {
                        ChargeType::NONE
                    } else if state.sys & SYS_STS_BYPASS != 0 {
                        ChargeType::BYPASS
                    } else {
                        ChargeType::STANDARD
                    }
                    .as_raw(),
                )
            }
            Property::VOLTAGE_NOW => {
                let _guard = self.lock.lock();
                let (lo, hi) = self.chip.adc(reg::ADC_VIN)?;
                val.set_int(vin_microvolt(lo, hi));
            }
            Property::CURRENT_NOW => {
                let _guard = self.lock.lock();
                let (lo, hi) = self.chip.adc(reg::ADC_IIN)?;
                val.set_int(iin_microamp(lo, hi));
            }
            Property::TEMP => {
                let _guard = self.lock.lock();
                let (lo, hi) = self.chip.adc(reg::ADC_DIE_TEMP)?;
                val.set_int(die_temp_decicelsius(lo, hi));
            }
            Property::CONSTANT_CHARGE_VOLTAGE => {
                val.set_int(i32::try_from(self.chip.float_voltage()?).map_err(|_| EINVAL)?)
            }
            Property::INPUT_CURRENT_LIMIT => {
                val.set_int(i32::try_from(self.chip.input_current_limit()?).map_err(|_| EINVAL)?)
            }
            Property::INPUT_VOLTAGE_LIMIT => {
                val.set_int(i32::try_from(self.chip.input_overvoltage()?).map_err(|_| EINVAL)?)
            }
            _ => return Err(EINVAL),
        }

        Ok(())
    }

    fn set_property(&self, prop: Property, val: &PropertyValue) -> Result {
        let _guard = self.lock.lock();
        let value = val.int();

        match prop {
            // Charging on or off. The mode (switching or bypass) is set with CHARGE_TYPE.
            Property::STATUS => {
                if value == Status::CHARGING.as_raw() {
                    self.chip.set_standby(false)
                } else if value == Status::NOT_CHARGING.as_raw()
                    || value == Status::DISCHARGING.as_raw()
                {
                    self.chip.set_standby(true)
                } else {
                    Err(EINVAL)
                }
            }
            // The mode is only changed in standby: without a datasheet, switching it under load
            // is not something to try.
            Property::CHARGE_TYPE => {
                if !self.chip.in_standby()? {
                    return Err(EBUSY);
                }
                if value == ChargeType::BYPASS.as_raw() {
                    self.chip
                        .update(reg::SYS_CTRL, SYS_CTRL_EN_1TO1, SYS_CTRL_EN_1TO1)
                } else if value == ChargeType::STANDARD.as_raw() {
                    self.chip.update(reg::SYS_CTRL, SYS_CTRL_EN_1TO1, 0)
                } else {
                    Err(EINVAL)
                }
            }
            Property::CONSTANT_CHARGE_VOLTAGE => self
                .chip
                .set_float_voltage(u32::try_from(value).map_err(|_| EINVAL)?),
            Property::INPUT_CURRENT_LIMIT => self
                .chip
                .set_input_current_limit(u32::try_from(value).map_err(|_| EINVAL)?),
            Property::INPUT_VOLTAGE_LIMIT => self
                .chip
                .set_input_overvoltage(u32::try_from(value).map_err(|_| EINVAL)?),
            _ => Err(EINVAL),
        }
    }

    fn property_is_writeable(&self, prop: Property) -> bool {
        matches!(
            prop,
            Property::STATUS
                | Property::CHARGE_TYPE
                | Property::CONSTANT_CHARGE_VOLTAGE
                | Property::INPUT_CURRENT_LIMIT
                | Property::INPUT_VOLTAGE_LIMIT
        )
    }
}

/// The bus device data: owning the registration ties the supply to the binding.
struct LnData<'bound> {
    psy: Registration<'bound, Charger<'bound>>,
}

struct Ln8000Driver;

kernel::i2c_device_table! {
    I2C_TABLE,
    <Ln8000Driver as i2c::Driver>::IdInfo,
    [(i2c::DeviceId::new(c"ln8000"), ())]
}

kernel::of_device_table! {
    OF_TABLE,
    <Ln8000Driver as i2c::Driver>::IdInfo,
    [(of::DeviceId::new(c"lionsemi,ln8000"), ())]
}

/// Applies the optional device tree limits; a value the chip cannot take fails the probe.
fn apply_device_tree(idev: &I2cClient<Core<'_>>, chip: &Chip<'_>) -> Result {
    let dev = idev.as_ref();
    let Some(node) = dev.fwnode() else {
        return Ok(());
    };

    if let Some(v) = node
        .property_read::<u32>(c"lionsemi,float-voltage-microvolt")
        .optional()
    {
        chip.set_float_voltage(v).inspect_err(|_| {
            dev_err!(dev, "lionsemi,float-voltage-microvolt {} is out of range\n", v)
        })?;
    }
    if let Some(v) = node
        .property_read::<u32>(c"lionsemi,input-current-limit-microamp")
        .optional()
    {
        chip.set_input_current_limit(v).inspect_err(|_| {
            dev_err!(dev, "lionsemi,input-current-limit-microamp {} is out of range\n", v)
        })?;
    }
    if let Some(v) = node
        .property_read::<u32>(c"lionsemi,input-overvoltage-microvolt")
        .optional()
    {
        chip.set_input_overvoltage(v).inspect_err(|_| {
            dev_err!(dev, "lionsemi,input-overvoltage-microvolt {} is out of range\n", v)
        })?;
    }

    Ok(())
}

fn setup<'bound>(idev: &'bound I2cClient<Core<'_>>) -> Result<LnData<'bound>> {
    // Deref coercion: `I2cClient<Core<'_>>` -> `I2cClient<Bound>`.
    let client: &'bound I2cClient<Bound> = idev;
    let chip = Chip { client };
    let dev = idev.as_ref();

    let id = chip.read(reg::DEVICE_ID)?;
    if id != DEVICE_ID {
        dev_err!(dev, "unexpected device id {:#04x}, expected {:#04x}\n", id, DEVICE_ID);
        return Err(ENODEV);
    }

    chip.init()?;
    apply_device_tree(idev, &chip)?;

    // SAFETY: the registration is stored in the bus device data, which is dropped when the
    // device is unbound, and nothing here forgets it.
    let psy = unsafe {
        Registration::new(
            dev,
            try_pin_init!(Charger {
                chip,
                lock <- new_mutex!(()),
            }),
        )
    }?;

    dev_info!(dev, "LN8000 charge pump found, in standby\n");

    Ok(LnData { psy })
}

impl i2c::Driver for Ln8000Driver {
    type IdInfo = ();
    type Data<'bound> = LnData<'bound>;

    const I2C_ID_TABLE: Option<i2c::IdTable<Self::IdInfo>> = Some(&I2C_TABLE);
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        idev: &'bound I2cClient<Core<'_>>,
        _info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        setup(idev)
    }

    // Leave the pump off when the system goes down and when the driver lets go.
    fn shutdown<'bound>(idev: &'bound I2cClient<Core<'_>>, this: Pin<&Self::Data<'bound>>) {
        if this.psy.data().standby().is_err() {
            dev_warn!(idev.as_ref(), "could not put the LN8000 into standby\n");
        }
    }

    fn unbind<'bound>(idev: &'bound I2cClient<Core<'_>>, this: Pin<&Self::Data<'bound>>) {
        if this.psy.data().standby().is_err() {
            dev_warn!(idev.as_ref(), "could not put the LN8000 into standby\n");
        }
    }
}

kernel::module_i2c_driver! {
    type: Ln8000Driver,
    name: "ln8000_charger",
    authors: ["Poco M5s Linux port"],
    description: "Lion Semiconductor LN8000 direct-charge pump",
    license: "GPL v2",
}
