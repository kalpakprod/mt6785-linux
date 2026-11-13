// SPDX-License-Identifier: GPL-2.0

//! Rust power supply driver sample on top of an I2C chip.
//!
//! The "chip" is a made-up battery gauge with byte registers, meant to be emulated by the
//! `i2c-stub` module (`i2cset` seeds the registers), so that the whole path
//! sysfs -> power supply core -> Rust driver -> SMBus -> i2c-stub can be exercised without
//! hardware:
//!
//! | register | meaning                                     | access     |
//! |----------|---------------------------------------------|------------|
//! | `0x00`   | status: 0 discharging, 1 charging, 2 full   | read       |
//! | `0x01`   | online: 0 or 1                              | read       |
//! | `0x02`   | capacity in percent                         | read       |
//! | `0x03`   | temperature in whole degrees Celsius        | read       |
//! | `0x04`   | charge end threshold in percent             | read/write |

use kernel::{
    acpi,
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
        Property,
        PropertyValue,
        Registration,
        Status,
        Type, //
    },
    prelude::*, //
};

const REG_STATUS: usize = 0x00;
const REG_ONLINE: usize = 0x01;
const REG_CAPACITY: usize = 0x02;
const REG_TEMP: usize = 0x03;
const REG_END_THRESHOLD: usize = 0x04;

/// The supply: answers queries by reading the chip.
struct Supply<'a> {
    client: &'a I2cClient<Bound>,
}

// SAFETY: the I2C core serialises transfers on the client, and the bound client stays valid for
// as long as the supply is registered, which the driver data guarantees (it owns the
// registration and is dropped at unbind).
unsafe impl Send for Supply<'_> {}
// SAFETY: as above, shared access only starts SMBus transfers, which the I2C core serialises.
unsafe impl Sync for Supply<'_> {}

#[vtable]
impl power_supply::Driver for Supply<'_> {
    const NAME: &'static CStr = c"rust-sample-psy";
    const TYPE: Type = Type::BATTERY;
    const PROPERTIES: &'static [Property] = &[
        Property::STATUS,
        Property::ONLINE,
        Property::CAPACITY,
        Property::TEMP,
        Property::CHARGE_CONTROL_END_THRESHOLD,
    ];

    fn get_property(&self, prop: Property, val: &PropertyValue) -> Result {
        let io = self.client.smbus_io();

        match prop {
            Property::STATUS => val.set_int(
                match io.try_read8(REG_STATUS)? {
                    0 => Status::DISCHARGING,
                    1 => Status::CHARGING,
                    2 => Status::FULL,
                    _ => Status::UNKNOWN,
                }
                .as_raw(),
            ),
            Property::ONLINE => val.set_int(i32::from(io.try_read8(REG_ONLINE)? != 0)),
            Property::CAPACITY => val.set_int(i32::from(io.try_read8(REG_CAPACITY)?)),
            // The core wants tenths of a degree.
            Property::TEMP => val.set_int(i32::from(io.try_read8(REG_TEMP)?) * 10),
            Property::CHARGE_CONTROL_END_THRESHOLD => {
                val.set_int(i32::from(io.try_read8(REG_END_THRESHOLD)?))
            }
            _ => return Err(EINVAL),
        }

        Ok(())
    }

    fn set_property(&self, prop: Property, val: &PropertyValue) -> Result {
        match prop {
            Property::CHARGE_CONTROL_END_THRESHOLD => {
                let percent = u8::try_from(val.int()).map_err(|_| EINVAL)?;
                if percent > 100 {
                    return Err(EINVAL);
                }
                self.client
                    .smbus_io()
                    .try_write8(percent, REG_END_THRESHOLD)
            }
            _ => Err(EINVAL),
        }
    }

    fn property_is_writeable(&self, prop: Property) -> bool {
        prop == Property::CHARGE_CONTROL_END_THRESHOLD
    }
}

/// The bus device data: owning the registration ties the supply to the binding.
struct SampleData<'bound> {
    _psy: Registration<'bound, Supply<'bound>>,
}

struct SampleDriver;

kernel::acpi_device_table! {
    ACPI_TABLE,
    <SampleDriver as i2c::Driver>::IdInfo,
    [(acpi::DeviceId::new(c"LNUXPSY0"), ())]
}

kernel::i2c_device_table! {
    I2C_TABLE,
    <SampleDriver as i2c::Driver>::IdInfo,
    [(i2c::DeviceId::new(c"rust_psy_sample"), ())]
}

kernel::of_device_table! {
    OF_TABLE,
    <SampleDriver as i2c::Driver>::IdInfo,
    [(of::DeviceId::new(c"test,rust_psy_sample"), ())]
}

impl i2c::Driver for SampleDriver {
    type IdInfo = ();
    type Data<'bound> = SampleData<'bound>;

    const ACPI_ID_TABLE: Option<acpi::IdTable<Self::IdInfo>> = Some(&ACPI_TABLE);
    const I2C_ID_TABLE: Option<i2c::IdTable<Self::IdInfo>> = Some(&I2C_TABLE);
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        idev: &'bound I2cClient<Core<'_>>,
        _info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        // Deref coercion: `I2cClient<Core<'_>>` -> `I2cClient<Bound>`.
        let client: &'bound I2cClient<Bound> = idev;

        // SAFETY: the registration is stored in the bus device data, which is dropped when the
        // device is unbound, and nothing here forgets it.
        let psy = unsafe {
            Registration::new(idev.as_ref(), Ok::<_, Error>(Supply { client }))
        };

        psy.map(|psy| SampleData { _psy: psy })
    }
}

kernel::module_i2c_driver! {
    type: SampleDriver,
    name: "rust_driver_power_supply",
    authors: ["Poco M5s Linux port"],
    description: "Rust power supply driver sample (i2c-stub test chip)",
    license: "GPL v2",
}
