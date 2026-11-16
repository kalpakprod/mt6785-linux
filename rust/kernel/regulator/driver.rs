// SPDX-License-Identifier: GPL-2.0

//! Regulator provider (driver) abstractions.
//!
//! This is the other half of [`crate::regulator`]: that module lets a driver *consume* a
//! regulator, this one lets it *provide* one. The split mirrors the C headers -
//! `include/linux/regulator/consumer.h` and `include/linux/regulator/driver.h` - and the
//! shape follows what is already merged in this tree:
//!
//! * the vtable bridge is `rust/kernel/pwm.rs:492-509`: a trait describing the callbacks, an
//!   `Adapter<T>` whose `unsafe extern "C"` functions call into it, and a `const fn` that
//!   builds the C ops struct with `pin_init::zeroed()` and fills only the fields the trait
//!   supports (`struct regulator_ops` has some sixty optional callbacks, six are used here:
//!   `enable`, `disable`, `is_enabled`, `list_voltage`, `set_voltage_sel`, `get_voltage_sel`
//!   - `include/linux/regulator/driver.h:159-191`);
//! * registration is `rust/kernel/cpufreq.rs:927-1079`: a `Registration<T>` that owns the C
//!   object and the driver data, with the devres path as the normal one;
//! * names and types come from the consumer side, `rust/kernel/regulator.rs:386-398`.
//!
//! The descriptor and the linear ranges must not move once the regulator core has seen them
//! (`devm_regulator_register()` keeps the pointers), so everything is built inside the
//! `Registration` with pin-init and never handed out mutably.
//!
//! # Examples
//!
//! ```
//! use kernel::regulator::driver::{LinearRange, Ops, OpsVTable, Registration, Params};
//! use kernel::{device::Device, prelude::*, regulator::Voltage};
//!
//! struct MyLdo;
//!
//! impl Ops for MyLdo {
//!     fn enable(_data: &Self) -> Result { Ok(()) }
//!     fn disable(_data: &Self) -> Result { Ok(()) }
//!     fn is_enabled(_data: &Self) -> Result<bool> { Ok(true) }
//!     fn list_voltage(_data: &Self, selector: u32) -> Result<i32> {
//!         Ok(600_000 + selector as i32 * 6_000)
//!     }
//!     fn set_voltage_sel(_data: &Self, _selector: u32) -> Result { Ok(()) }
//!     fn get_voltage_sel(_data: &Self) -> Result<u32> { Ok(0) }
//! }
//!
//! static VTABLE: OpsVTable = kernel::regulator::driver::create_ops::<MyLdo>();
//! const RANGE: LinearRange = LinearRange::new(600_000, 6_000, 256);
//! static RANGES: [LinearRange; 1] = [RANGE];
//!
//! fn register(dev: &Device) -> Result<Registration<MyLdo>> {
//!     Registration::new(
//!         dev,
//!         Params::new(c"my-ldo", kernel::bindings::regulator_type_REGULATOR_VOLTAGE)
//!             .ops(&VTABLE, 256)
//!             .linear_ranges(&RANGES)
//!             .of_match(c"example,my-ldo"),
//!         MyLdo,
//!     )
//! }
//! ```

use crate::{
    bindings,
    device::Device,
    error::Result,
    prelude::*,
    str::CStr,
};

use core::marker::PhantomData;

/// One linear voltage range, mirroring `struct linear_range`: `count` values starting at
/// `min_uv`, `step_uv` apart.
#[repr(transparent)]
#[derive(Clone, Copy)]
pub struct LinearRange(bindings::linear_range);

impl PartialEq for LinearRange {
    fn eq(&self, other: &Self) -> bool {
        self.0.min == other.0.min
            && self.0.step == other.0.step
            && self.0.min_sel == other.0.min_sel
            && self.0.max_sel == other.0.max_sel
    }
}

impl Eq for LinearRange {}

impl core::fmt::Debug for LinearRange {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("LinearRange")
            .field("min", &self.0.min)
            .field("step", &self.0.step)
            .field("max_sel", &self.0.max_sel)
            .finish()
    }
}

impl LinearRange {
    /// A range of `count` values from `min_uv` in steps of `step_uv`, all microvolts.
    pub const fn new(min_uv: i32, step_uv: i32, count: u32) -> Self {
        Self(bindings::linear_range {
            min: min_uv as core::ffi::c_uint,
            step: step_uv as core::ffi::c_uint,
            min_sel: 0,
            max_sel: (count - 1) as core::ffi::c_uint,
        })
    }
}

// The wrapper has to be layout-identical to the C type: `regulator_desc::linear_ranges`
// points straight at an array of them.
const _: () = assert!(core::mem::size_of::<LinearRange>() == core::mem::size_of::<bindings::linear_range>());
const _: () = assert!(core::mem::align_of::<LinearRange>() == core::mem::align_of::<bindings::linear_range>());

/// What a regulator driver implements. Each callback mirrors the C one of the same name in
/// `struct regulator_ops` (`include/linux/regulator/driver.h:159-191`), with the driver's own
/// data in place of `struct regulator_dev *`.
pub trait Ops: Send + Sync {
    /// `->enable`.
    fn enable(data: &Self) -> Result;
    /// `->disable`.
    fn disable(data: &Self) -> Result;
    /// `->is_enabled`.
    fn is_enabled(data: &Self) -> Result<bool>;
    /// `->list_voltage`: microvolts for `selector`, or an error.
    fn list_voltage(data: &Self, selector: u32) -> Result<i32>;
    /// `->set_voltage_sel`.
    fn set_voltage_sel(data: &Self, selector: u32) -> Result;
    /// `->get_voltage_sel`.
    fn get_voltage_sel(data: &Self) -> Result<u32>;
}

/// The C `struct regulator_ops` built for one [`Ops`] implementation.
#[repr(transparent)]
pub struct OpsVTable(bindings::regulator_ops);

// SAFETY: the struct holds function pointers only, and it is never modified after
// `create_ops()` returns, so sharing it between threads is sound.
unsafe impl Send for OpsVTable {}
unsafe impl Sync for OpsVTable {}

impl OpsVTable {
    /// The C pointer to hand to `regulator_desc::ops`.
    pub const fn as_ptr(&self) -> *const bindings::regulator_ops {
        &self.0
    }
}

/// Builds the vtable for `T`, in the same way `create_pwm_ops()` does for PWM
/// (`rust/kernel/pwm.rs:496-509`).
pub const fn create_ops<T: Ops>() -> OpsVTable {
    let mut ops: bindings::regulator_ops = pin_init::zeroed();

    ops.enable = Some(Adapter::<T>::enable_cb);
    ops.disable = Some(Adapter::<T>::disable_cb);
    ops.is_enabled = Some(Adapter::<T>::is_enabled_cb);
    ops.list_voltage = Some(Adapter::<T>::list_voltage_cb);
    ops.set_voltage_sel = Some(Adapter::<T>::set_voltage_sel_cb);
    ops.get_voltage_sel = Some(Adapter::<T>::get_voltage_sel_cb);

    OpsVTable(ops)
}

/// Bridges [`Ops`] to the C callbacks.
struct Adapter<T>(PhantomData<T>);

impl<T: Ops> Adapter<T> {
    /// # Safety
    ///
    /// `rdev` must be a valid `struct regulator_dev *` whose driver data is the `T` this
    /// registration owns, and it must stay alive for the returned reference's lifetime.
    unsafe fn data<'a>(rdev: *mut bindings::regulator_dev) -> &'a T {
        // SAFETY: guaranteed by the caller.
        unsafe { &*(bindings::rdev_get_drvdata(rdev) as *const T) }
    }

    unsafe extern "C" fn enable_cb(rdev: *mut bindings::regulator_dev) -> core::ffi::c_int {
        // SAFETY: the regulator core passes a `regulator_dev` of this registration.
        match T::enable(unsafe { Self::data(rdev) }) {
            Ok(()) => 0,
            Err(e) => e.to_errno(),
        }
    }

    unsafe extern "C" fn disable_cb(rdev: *mut bindings::regulator_dev) -> core::ffi::c_int {
        // SAFETY: as above.
        match T::disable(unsafe { Self::data(rdev) }) {
            Ok(()) => 0,
            Err(e) => e.to_errno(),
        }
    }

    unsafe extern "C" fn is_enabled_cb(rdev: *mut bindings::regulator_dev) -> core::ffi::c_int {
        // SAFETY: as above.
        match T::is_enabled(unsafe { Self::data(rdev) }) {
            Ok(true) => 1,
            Ok(false) => 0,
            Err(e) => e.to_errno(),
        }
    }

    unsafe extern "C" fn list_voltage_cb(
        rdev: *mut bindings::regulator_dev,
        selector: core::ffi::c_uint,
    ) -> core::ffi::c_int {
        // SAFETY: as above.
        match T::list_voltage(unsafe { Self::data(rdev) }, selector) {
            Ok(uv) => uv,
            Err(e) => e.to_errno(),
        }
    }

    unsafe extern "C" fn set_voltage_sel_cb(
        rdev: *mut bindings::regulator_dev,
        selector: core::ffi::c_uint,
    ) -> core::ffi::c_int {
        // SAFETY: as above.
        match T::set_voltage_sel(unsafe { Self::data(rdev) }, selector) {
            Ok(()) => 0,
            Err(e) => e.to_errno(),
        }
    }

    unsafe extern "C" fn get_voltage_sel_cb(
        rdev: *mut bindings::regulator_dev,
    ) -> core::ffi::c_int {
        // SAFETY: as above.
        match T::get_voltage_sel(unsafe { Self::data(rdev) }) {
            Ok(sel) => sel as core::ffi::c_int,
            Err(e) => e.to_errno(),
        }
    }
}

/// The kind of regulator, mirroring `enum regulator_type`.
#[derive(Clone, Copy)]
pub enum Kind {
    /// A voltage regulator.
    Voltage,
    /// A current sink.
    Current,
}

impl Kind {
    const fn raw(self) -> bindings::regulator_type {
        match self {
            Kind::Voltage => bindings::regulator_type_REGULATOR_VOLTAGE,
            Kind::Current => bindings::regulator_type_REGULATOR_CURRENT,
        }
    }
}

/// Everything a [`Registration`] needs to describe the regulator it registers.
#[derive(Clone, Copy)]
pub struct Params {
    name: &'static CStr,
    kind: Kind,
    ops: Option<&'static OpsVTable>,
    n_voltages: u32,
    ranges: Option<&'static [LinearRange]>,
    of_match: Option<&'static CStr>,
}

impl Params {
    /// The name and the kind from `enum regulator_type`
    /// (`include/linux/regulator/driver.h:249`).
    pub const fn new(name: &'static CStr, kind: Kind) -> Self {
        Self {
            name,
            kind,
            ops: None,
            n_voltages: 0,
            ranges: None,
            of_match: None,
        }
    }

    /// The ops vtable and the number of voltage selectors it takes.
    pub const fn ops(mut self, ops: &'static OpsVTable, n_voltages: u32) -> Self {
        self.ops = Some(ops);
        self.n_voltages = n_voltages;
        self
    }

    /// Linear ranges; they must live at least as long as the registration, which a `static`
    /// array guarantees (the same way C drivers pass `static const struct linear_range[]`).
    pub const fn linear_ranges(mut self, ranges: &'static [LinearRange]) -> Self {
        self.ranges = Some(ranges);
        self
    }

    /// The `of_match` string of `struct regulator_desc`.
    pub const fn of_match(mut self, of_match: &'static CStr) -> Self {
        self.of_match = Some(of_match);
        self
    }
}

/// A registered regulator. It owns the descriptor and the driver data the C side points at;
/// the registration itself is released through devres when the device goes away.
pub struct Registration<T: Ops> {
    rdev: *mut bindings::regulator_dev,
    _desc: KBox<bindings::regulator_desc>,
    _data: KBox<T>,
}

// SAFETY: a registration owns the descriptor and the driver data; both are only reachable
// through the regulator core, which serialises access to them.
unsafe impl<T: Ops> Send for Registration<T> {}
// SAFETY: as above.
unsafe impl<T: Ops> Sync for Registration<T> {}

impl<T: Ops> Registration<T> {
    /// Registers one regulator of `dev`.
    pub fn new<Ctx: crate::device::DeviceContext>(
        dev: &Device<Ctx>,
        params: Params,
        data: T,
    ) -> Result<Self> {
        let ops = params.ops.ok_or(EINVAL)?;

        // SAFETY: `regulator_desc` is a plain data structure - integers, pointers and one
        // flag - so an all-zero value is a valid starting point; every field the core reads
        // is set below.
        let mut desc = KBox::new(unsafe { core::mem::zeroed::<bindings::regulator_desc>() },
                                 GFP_KERNEL)?;

        desc.name = params.name.as_char_ptr();
        desc.type_ = params.kind.raw();
        // SAFETY: the kernel's own module object, as every C driver does in this field.
        desc.owner = core::ptr::addr_of_mut!(bindings::__this_module);
        desc.ops = ops.as_ptr();
        desc.n_voltages = params.n_voltages;

        if let Some(of_match) = params.of_match {
            desc.of_match = of_match.as_char_ptr();
        }

        if let Some(ranges) = params.ranges {
            // `LinearRange` is a transparent wrapper, and a static array does not move.
            desc.linear_ranges = ranges.as_ptr().cast();
            desc.n_linear_ranges = ranges.len() as core::ffi::c_int;
        }

        let data = KBox::new(data, GFP_KERNEL)?;

        let mut config: bindings::regulator_config = pin_init::zeroed();

        config.dev = dev.as_raw();
        // SAFETY: `dev` is valid, so reading its `of_node` field is sound.
        config.of_node = unsafe { (*dev.as_raw()).of_node };
        config.driver_data = (&*data as *const T).cast_mut().cast();

        // SAFETY: `desc` and `data` live in `Self` below and never move (a `KBox` keeps its
        // allocation), `config` points at them, and `dev` is bound.
        let rdev = unsafe { bindings::devm_regulator_register(dev.as_raw(), &*desc, &config) };

        if rdev.is_null() {
            return Err(EINVAL);
        }

        Ok(Self {
            rdev,
            _desc: desc,
            _data: data,
        })
    }

    /// The C object.
    pub fn as_raw(&self) -> *mut bindings::regulator_dev {
        self.rdev
    }
}

#[kunit_tests(rust_regulator_driver)]
mod tests {
    use super::*;

    struct Dummy;

    impl Ops for Dummy {
        fn enable(_data: &Self) -> Result {
            Ok(())
        }
        fn disable(_data: &Self) -> Result {
            Ok(())
        }
        fn is_enabled(_data: &Self) -> Result<bool> {
            Ok(true)
        }
        fn list_voltage(_data: &Self, selector: u32) -> Result<i32> {
            Ok(600_000 + selector as i32 * 6_000)
        }
        fn set_voltage_sel(_data: &Self, _selector: u32) -> Result {
            Ok(())
        }
        fn get_voltage_sel(_data: &Self) -> Result<u32> {
            Ok(0)
        }
    }

    /// The vtable carries exactly the callbacks the trait has, and nothing else.
    #[test]
    fn vtable_sets_only_the_callbacks_we_implement() {
        let vt = create_ops::<Dummy>();
        // SAFETY: `as_ptr()` points at the vtable `create_ops` just built.
        let c = unsafe { &*vt.as_ptr() };

        assert!(c.enable.is_some());
        assert!(c.disable.is_some());
        assert!(c.is_enabled.is_some());
        assert!(c.list_voltage.is_some());
        assert!(c.set_voltage_sel.is_some());
        assert!(c.get_voltage_sel.is_some());

        // Callbacks a driver has not expressed must stay empty, or the core would call
        // into a NULL pointer.
        assert!(c.set_voltage.is_none());
        assert!(c.get_voltage.is_none());
        assert!(c.set_current_limit.is_none());
    }

    /// `LinearRange` lands in the C structure exactly as the regulator core reads it.
    #[test]
    fn linear_range_matches_the_c_struct() {
        let r = LinearRange::new(600_000, 6_000, 256);

        assert_eq!(r.0.min, 600_000);
        assert_eq!(r.0.step, 6_000);
        assert_eq!(r.0.min_sel, 0);
        assert_eq!(r.0.max_sel, 255);
    }

    /// The selector mapping the WL2866D will use: 600 mV + 6 mV * selector.
    #[test]
    fn list_voltage_walks_the_range() {
        assert_eq!(Dummy::list_voltage(&Dummy, 0), Ok(600_000));
        assert_eq!(Dummy::list_voltage(&Dummy, 100), Ok(1_200_000));
    }
}
