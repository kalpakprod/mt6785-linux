// SPDX-License-Identifier: GPL-2.0

//! WL2866D camera LDO controller (Will Semiconductor), four outputs.
//!
//! The part sits on the camera I2C bus and feeds AVDD/DVDD of the phone's camera modules: two
//! outputs with a 600 mV + 6 mV * selector range and two with 1200 mV + 12.5 mV * selector, all
//! behind one enable GPIO. Register map and encodings come from the vendor driver of the same
//! SoC (`drivers/regulator/wl2866d.c` in the 4.19 drop): voltage selectors in `0x03..0x06`,
//! output enables in `0x0E` (bit n for output n), discharge control in `0x02`.
//!
//! There is **no identity check**, on purpose. The vendor's own is not one - it reads `0x03`,
//! which is output 1's voltage selector, so it passes by construction - and the parts'
//! datasheet-level description does not give a readable id register either: on the phone
//! `i2cdump` of `0-0028` reads `00 00 00 64 64 80 80 00`, i.e. `0x00` is zero while
//! `0x03..0x06` hold the voltage selectors. What the driver checks is what exists: the first
//! transfer must succeed (the I2C client only probes when the chip acknowledges its address),
//! and the registers it later programs are the ones the phone's dump shows as real.

use kernel::{
    device::Core,
    i2c,
    io::Io,
    of,
    prelude::*,
    regulator::driver::{
        create_ops, Kind, LinearRange, Ops as RegOps, OpsVTable, Params, Registration,
    },
};

/// Discharge control: `0x8F` enables the discharge FETs, `0x00` disables them.
const REG_DISCHARGE: u8 = 0x02;
/// Voltage selector of output 0; outputs 1..3 follow at `0x04..0x06`.
const REG_VSET0: u8 = 0x03;
/// Output enable, bit n for output n.
const REG_ENABLE: u8 = 0x0E;

/// Outputs 0-1 step 6 mV from 600 mV, outputs 2-3 step 12.5 mV from 1200 mV - the vendor's
/// `VSET_BASE_12`/`VSET_STEP_MV` and `VSET_BASE_34`/`VSET_STEP_MV_34`, in microvolts.
const DVDD_BASE_UV: i32 = 600_000;
const DVDD_STEP_UV: i32 = 6_000;
const AVDD_BASE_UV: i32 = 1_200_000;
const AVDD_STEP_UV: i32 = 12_500;
const N_VOLTAGES: u32 = 256;

/// One output of the chip.
struct Ldo<'a, const IDX: usize> {
    client: &'a i2c::I2cClient<kernel::device::Bound>,
}

// SAFETY: the I2C core serialises transfers on the client, and the bound client stays valid as
// long as the registration lives - the driver data owns the registrations and is dropped at
// unbind.
unsafe impl<const IDX: usize> Send for Ldo<'_, IDX> {}
// SAFETY: as above; shared access only starts SMBus transfers.
unsafe impl<const IDX: usize> Sync for Ldo<'_, IDX> {}

const fn base_uv(idx: usize) -> i32 {
    if idx < 2 {
        DVDD_BASE_UV
    } else {
        AVDD_BASE_UV
    }
}

const fn step_uv(idx: usize) -> i32 {
    if idx < 2 {
        DVDD_STEP_UV
    } else {
        AVDD_STEP_UV
    }
}

impl<'a, const IDX: usize> Ldo<'a, IDX> {
    /// The selector range this output covers, as the regulator core wants it.
    const RANGE: LinearRange = LinearRange::new(base_uv(IDX), step_uv(IDX), N_VOLTAGES);
    /// The range as a static slice for `Params`.
    const RANGES: &'static [LinearRange] = &[Self::RANGE];

    /// Microvolts of `selector`, or `None` past the last selector - the same mapping the
    /// core reaches through `list_voltage`, callable without a client so the tests can use it.
    const fn uv(selector: u32) -> Option<i32> {
        if selector >= N_VOLTAGES {
            return None;
        }

        Some(base_uv(IDX) + selector as i32 * step_uv(IDX))
    }

    /// The selector for `uv`, rounded up to the next supported step the way the regulator
    /// core's `regulator_map_voltage_linear_range` does, or `None` outside the range.
    const fn selector_for_uv(uv: i32) -> Option<u32> {
        let span = uv - base_uv(IDX);

        if span < 0 {
            return None;
        }

        let step = step_uv(IDX);
        let selector = (span + step - 1) / step;

        if selector as u32 >= N_VOLTAGES {
            return None;
        }

        Some(selector as u32)
    }

    fn read(&self, reg: u8) -> Result<u8> {
        self.client.smbus_io().try_read8(reg as usize)
    }

    fn write(&self, reg: u8, value: u8) -> Result {
        self.client.smbus_io().try_write8(value, reg as usize)
    }
}

impl<const IDX: usize> RegOps for Ldo<'_, IDX> {
    fn enable(data: &Self) -> Result {
        let mask = 1u8 << IDX;

        data.write(REG_ENABLE, data.read(REG_ENABLE)? | mask)
    }

    fn disable(data: &Self) -> Result {
        let mask = 1u8 << IDX;

        data.write(REG_ENABLE, data.read(REG_ENABLE)? & !mask)
    }

    fn is_enabled(data: &Self) -> Result<bool> {
        Ok((data.read(REG_ENABLE)? & (1u8 << IDX)) != 0)
    }

    fn list_voltage(_data: &Self, selector: u32) -> Result<i32> {
        Self::uv(selector).ok_or(EINVAL)
    }

    fn set_voltage_sel(data: &Self, selector: u32) -> Result {
        if selector >= N_VOLTAGES {
            return Err(EINVAL);
        }

        data.write(REG_VSET0 + IDX as u8, selector as u8)
    }

    fn get_voltage_sel(data: &Self) -> Result<u32> {
        Ok(u32::from(data.read(REG_VSET0 + IDX as u8)?))
    }
}

/// Four vtables, one per output: every registration needs its own.
static VTABLE_0: OpsVTable = create_ops::<Ldo<'static, 0>>();
static VTABLE_1: OpsVTable = create_ops::<Ldo<'static, 1>>();
static VTABLE_2: OpsVTable = create_ops::<Ldo<'static, 2>>();
static VTABLE_3: OpsVTable = create_ops::<Ldo<'static, 3>>();

kernel::of_device_table! {
    OF_TABLE,
    @none,
    [of::DeviceId::new(c"xiaomi,wl2866d")]
}

kernel::i2c_device_table! {
    I2C_TABLE,
    @none,
    [i2c::DeviceId::new(c"wl2866d")]
}

/// The driver data: the four registrations, owned here so they are released at unbind.
struct Wl2866d<'a> {
    _dvdd1: Registration<Ldo<'a, 0>>,
    _dvdd2: Registration<Ldo<'a, 1>>,
    _avdd1: Registration<Ldo<'a, 2>>,
    _avdd2: Registration<Ldo<'a, 3>>,
}

impl<'a> i2c::Driver for Wl2866d<'a> {
    type IdInfo = ();
    type Data<'bound> = Wl2866d<'bound>;

    const I2C_ID_TABLE: Option<i2c::IdTable<Self::IdInfo>> = Some(&I2C_TABLE);
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        client: &'bound i2c::I2cClient<Core<'_>>,
        _info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        // Deref coercion, as the in-tree power-supply sample does: the driver keeps the
        // client in its bound form for as long as the device is bound.
        let client: &'bound i2c::I2cClient<kernel::device::Bound> = client;
        let dev = client.as_ref();

        /*
         * No identity register exists to check: the vendor's "id" read is output 1's voltage
         * selector (0x03), which passes by construction, and the documented register map has no
         * id byte - the phone's i2cdump shows 0x00 reading as zero. The chip answering its
         * address at all is what the I2C probe already proved; the first write below is the
         * first real evidence that it is the part the board expects.
         */
        // Discharge off before anything is enabled, as the vendor's sequence does.
        client.smbus_io().try_write8(0x00, REG_DISCHARGE as usize)?;

        let params = |name, ops| Params::new(name, Kind::Voltage).ops(ops, N_VOLTAGES);

        // The module object the `module_i2c_driver!` macro generates for this driver; the
        // regulator core records it as the owner of these regulators.
        let module = kernel::module::this_module::<DriverModule>();

        /*
         * Each rail is a child node of the controller's node, and that child is what a board's
         * `<name>-supply` phandle names. Registering all four with the controller's node - which
         * is what the provider does when no node is given - makes every lookup miss, and the
         * sensors then defer for ever with "could not get regulators".
         */
        // The rails are the children of the controller's "regulators" node; naming the child in
        // of_match lets the regulator core find and install each node itself (of_regulator.c
        // :475-520). Handing the core a node pointer does not work - it records only what that
        // lookup returns.
        let rail = |params: Params, child: &'static kernel::str::CStr| {
            params.child_of(c"regulators", child)
        };

        Ok(Wl2866d {
            _dvdd1: Registration::new(
                dev,
                rail(
                    params(c"wl2866d_dvdd1", &VTABLE_0).linear_ranges(Ldo::<0>::RANGES),
                    c"dvdd1",
                ),
                module,
                Ldo::<0> { client },
            )?,
            _dvdd2: Registration::new(
                dev,
                rail(
                    params(c"wl2866d_dvdd2", &VTABLE_1).linear_ranges(Ldo::<1>::RANGES),
                    c"dvdd2",
                ),
                module,
                Ldo::<1> { client },
            )?,
            _avdd1: Registration::new(
                dev,
                rail(
                    params(c"wl2866d_avdd1", &VTABLE_2).linear_ranges(Ldo::<2>::RANGES),
                    c"avdd1",
                ),
                module,
                Ldo::<2> { client },
            )?,
            _avdd2: Registration::new(
                dev,
                rail(
                    params(c"wl2866d_avdd2", &VTABLE_3).linear_ranges(Ldo::<3>::RANGES),
                    c"avdd2",
                ),
                module,
                Ldo::<3> { client },
            )?,
        })
    }
}

kernel::module_i2c_driver! {
    type: Wl2866d<'static>,
    name: "wl2866d",
    authors: ["poco-display-worker"],
    description: "WL2866D camera LDO controller",
    license: "GPL v2",
}

#[kunit_tests(rust_wl2866d)]
mod tests {
    use super::*;

    /// Selector 0 is the bottom of each range, and the expected microvolts are written out
    /// from the vendor's model (`wl2866d.c`: `VSET_BASE_12` 600 mV + 6 mV * vset for outputs
    /// 1-2, `VSET_BASE_34` 1200 mV + 12.5 mV * vset for outputs 3-4), not computed here.
    #[test]
    fn bottom_of_each_range() {
        assert_eq!(Ldo::<0>::uv(0), Some(600_000));
        assert_eq!(Ldo::<1>::uv(0), Some(600_000));
        assert_eq!(Ldo::<2>::uv(0), Some(1_200_000));
        assert_eq!(Ldo::<3>::uv(0), Some(1_200_000));
    }

    /// Selector 255 is the top: 0.6 V + 255 * 6 mV = 2.13 V, and 1.2 V + 255 * 12.5 mV =
    /// 4.3875 V - the two upper bounds the binding states.
    #[test]
    fn top_of_each_range() {
        assert_eq!(Ldo::<0>::uv(255), Some(2_130_000));
        assert_eq!(Ldo::<1>::uv(255), Some(2_130_000));
        assert_eq!(Ldo::<2>::uv(255), Some(4_387_500));
        assert_eq!(Ldo::<3>::uv(255), Some(4_387_500));
    }

    /// The three voltages the phone's camera rails ask for, as the vendor's table writes
    /// them: dvdd1 1.2 V -> 0x64, dvdd2 1.05 V -> 0x4B, avdd1 2.8 V -> 0x80.
    #[test]
    fn vendor_table_values_round_trip() {
        assert_eq!(Ldo::<0>::selector_for_uv(1_200_000), Some(0x64));
        assert_eq!(Ldo::<0>::uv(0x64), Some(1_200_000));

        assert_eq!(Ldo::<1>::selector_for_uv(1_050_000), Some(0x4B));
        assert_eq!(Ldo::<1>::uv(0x4B), Some(1_050_000));

        assert_eq!(Ldo::<2>::selector_for_uv(2_800_000), Some(0x80));
        assert_eq!(Ldo::<2>::uv(0x80), Some(2_800_000));

        assert_eq!(Ldo::<3>::selector_for_uv(2_800_000), Some(0x80));
    }

    /// A request between two steps rounds up to the next one, as the core's linear-range
    /// mapping does: 1.203 V on a 6 mV grid above 0.6 V is selector 101, i.e. 1.206 V.
    #[test]
    fn between_steps_rounds_up() {
        assert_eq!(Ldo::<0>::selector_for_uv(1_203_000), Some(101));
        assert_eq!(Ldo::<0>::uv(101), Some(1_206_000));

        // 2.701 V on the 12.5 mV grid above 1.2 V is selector 121 (0x79), i.e. 2.7125 V.
        assert_eq!(Ldo::<2>::selector_for_uv(2_701_000), Some(0x79));
        assert_eq!(Ldo::<2>::uv(0x79), Some(2_712_500));
    }

    /// Outside the range there is no selector, and past the last one there is no voltage.
    #[test]
    fn out_of_range_is_rejected() {
        assert_eq!(Ldo::<0>::selector_for_uv(599_000), None);
        assert_eq!(Ldo::<0>::selector_for_uv(2_131_000), None);
        assert_eq!(Ldo::<2>::selector_for_uv(1_199_000), None);
        assert_eq!(Ldo::<2>::selector_for_uv(4_388_000), None);

        assert_eq!(Ldo::<0>::uv(256), None);
        assert_eq!(Ldo::<2>::uv(u32::MAX), None);
    }
}
