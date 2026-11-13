// SPDX-License-Identifier: GPL-2.0

//! Power supply class abstractions.
//!
//! C header: [`include/linux/power_supply.h`](srctree/include/linux/power_supply.h)
//!
//! A driver implements [`Driver`] for the type that answers property queries and registers it
//! with [`Registration::new`] from its probe function, keeping the [`Registration`] in its bus
//! device data so that it is dropped (and the supply unregistered) when the device is unbound.

use crate::{
    bindings,
    device::{self, Bound},
    error::{from_err_ptr, from_result},
    prelude::*,
    types::Opaque, //
};
use core::{
    ffi::{c_int, c_void},
    marker::PhantomData,
    pin::Pin,
    ptr::NonNull, //
};
use macros::vtable;

/// Defines a newtype over a C enum with named constants.
macro_rules! c_enum {
    (
        $(#[$meta:meta])*
        $name:ident($repr:ty) {
            $( $(#[$cmeta:meta])* $const:ident = $c:ident; )*
        }
    ) => {
        $(#[$meta])*
        #[repr(transparent)]
        #[derive(Clone, Copy, PartialEq, Eq, Debug)]
        pub struct $name($repr);

        impl $name {
            $(
                $(#[$cmeta])*
                pub const $const: Self = Self(bindings::$c as $repr);
            )*

            /// Returns the raw C value.
            #[inline]
            pub const fn as_raw(self) -> $repr {
                self.0
            }
        }
    };
}

c_enum! {
    /// A power supply property identifier (`enum power_supply_property`).
    ///
    /// Only the properties that drivers commonly report are named; any other can be added here
    /// when a driver needs it.
    Property(bindings::power_supply_property) {
        /// `POWER_SUPPLY_PROP_STATUS`: charging state, one of [`Status`].
        STATUS = power_supply_property_POWER_SUPPLY_PROP_STATUS;
        /// `POWER_SUPPLY_PROP_CHARGE_TYPE`: current charge type, one of [`ChargeType`].
        CHARGE_TYPE = power_supply_property_POWER_SUPPLY_PROP_CHARGE_TYPE;
        /// `POWER_SUPPLY_PROP_CHARGE_TYPES`: selectable charge types.
        CHARGE_TYPES = power_supply_property_POWER_SUPPLY_PROP_CHARGE_TYPES;
        /// `POWER_SUPPLY_PROP_HEALTH`: one of [`Health`].
        HEALTH = power_supply_property_POWER_SUPPLY_PROP_HEALTH;
        /// `POWER_SUPPLY_PROP_PRESENT`: 1 when the supply (battery) is present.
        PRESENT = power_supply_property_POWER_SUPPLY_PROP_PRESENT;
        /// `POWER_SUPPLY_PROP_ONLINE`: 1 when the supply is connected.
        ONLINE = power_supply_property_POWER_SUPPLY_PROP_ONLINE;
        /// `POWER_SUPPLY_PROP_VOLTAGE_NOW`: in microvolts.
        VOLTAGE_NOW = power_supply_property_POWER_SUPPLY_PROP_VOLTAGE_NOW;
        /// `POWER_SUPPLY_PROP_CURRENT_NOW`: in microamperes.
        CURRENT_NOW = power_supply_property_POWER_SUPPLY_PROP_CURRENT_NOW;
        /// `POWER_SUPPLY_PROP_CAPACITY`: in percent.
        CAPACITY = power_supply_property_POWER_SUPPLY_PROP_CAPACITY;
        /// `POWER_SUPPLY_PROP_TEMP`: in tenths of a degree Celsius.
        TEMP = power_supply_property_POWER_SUPPLY_PROP_TEMP;
        /// `POWER_SUPPLY_PROP_MODEL_NAME`: string.
        MODEL_NAME = power_supply_property_POWER_SUPPLY_PROP_MODEL_NAME;
        /// `POWER_SUPPLY_PROP_MANUFACTURER`: string.
        MANUFACTURER = power_supply_property_POWER_SUPPLY_PROP_MANUFACTURER;
        /// `POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD`: in percent.
        CHARGE_CONTROL_START_THRESHOLD =
            power_supply_property_POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD;
        /// `POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD`: in percent.
        CHARGE_CONTROL_END_THRESHOLD =
            power_supply_property_POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD;
        /// `POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT`: in microamperes.
        CONSTANT_CHARGE_CURRENT = power_supply_property_POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;
        /// `POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE`: in microvolts.
        CONSTANT_CHARGE_VOLTAGE = power_supply_property_POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE;
        /// `POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT`: in microamperes.
        INPUT_CURRENT_LIMIT = power_supply_property_POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;
    }
}

c_enum! {
    /// The kind of supply (`enum power_supply_type`).
    Type(bindings::power_supply_type) {
        /// `POWER_SUPPLY_TYPE_UNKNOWN`.
        UNKNOWN = power_supply_type_POWER_SUPPLY_TYPE_UNKNOWN;
        /// `POWER_SUPPLY_TYPE_BATTERY`.
        BATTERY = power_supply_type_POWER_SUPPLY_TYPE_BATTERY;
        /// `POWER_SUPPLY_TYPE_MAINS`.
        MAINS = power_supply_type_POWER_SUPPLY_TYPE_MAINS;
        /// `POWER_SUPPLY_TYPE_USB`.
        USB = power_supply_type_POWER_SUPPLY_TYPE_USB;
    }
}

c_enum! {
    /// Values of [`Property::STATUS`] (`POWER_SUPPLY_STATUS_*`).
    Status(i32) {
        /// Unknown.
        UNKNOWN = POWER_SUPPLY_STATUS_UNKNOWN;
        /// Charging.
        CHARGING = POWER_SUPPLY_STATUS_CHARGING;
        /// Discharging.
        DISCHARGING = POWER_SUPPLY_STATUS_DISCHARGING;
        /// Connected but not charging.
        NOT_CHARGING = POWER_SUPPLY_STATUS_NOT_CHARGING;
        /// Full.
        FULL = POWER_SUPPLY_STATUS_FULL;
    }
}

c_enum! {
    /// Values of [`Property::CHARGE_TYPE`] (`POWER_SUPPLY_CHARGE_TYPE_*`).
    ChargeType(i32) {
        /// Not charging.
        NONE = power_supply_charge_type_POWER_SUPPLY_CHARGE_TYPE_NONE;
        /// Trickle charge.
        TRICKLE = power_supply_charge_type_POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
        /// Fast charge.
        FAST = power_supply_charge_type_POWER_SUPPLY_CHARGE_TYPE_FAST;
    }
}

c_enum! {
    /// Values of [`Property::HEALTH`] (`POWER_SUPPLY_HEALTH_*`).
    Health(i32) {
        /// Unknown.
        UNKNOWN = POWER_SUPPLY_HEALTH_UNKNOWN;
        /// Good.
        GOOD = POWER_SUPPLY_HEALTH_GOOD;
        /// Overheated.
        OVERHEAT = POWER_SUPPLY_HEALTH_OVERHEAT;
        /// Too cold.
        COLD = POWER_SUPPLY_HEALTH_COLD;
    }
}

/// The value of a property query (`union power_supply_propval`).
///
/// For a read the driver stores the answer with [`set_int`](Self::set_int) or
/// [`set_str`](Self::set_str); for a write it reads the requested value with
/// [`int`](Self::int).
#[repr(transparent)]
pub struct PropertyValue(Opaque<bindings::power_supply_propval>);

impl PropertyValue {
    /// Creates a reference from a raw pointer.
    ///
    /// # Safety
    ///
    /// `ptr` must point to a valid `union power_supply_propval` that is not accessed through any
    /// other reference for the duration of `'a`.
    unsafe fn from_raw<'a>(ptr: *const bindings::power_supply_propval) -> &'a Self {
        // SAFETY: `PropertyValue` is `repr(transparent)` over the `Opaque` union; validity and
        // exclusivity are guaranteed by the caller.
        unsafe { &*ptr.cast() }
    }

    /// Stores an integer answer.
    #[inline]
    pub fn set_int(&self, value: i32) {
        // SAFETY: `self` wraps a valid union; writing the `intval` member is always allowed.
        unsafe { (*self.0.get()).intval = value };
    }

    /// Stores a string answer; the string must live as long as the program (`'static`).
    #[inline]
    pub fn set_str(&self, value: &'static CStr) {
        // SAFETY: as above, for the `strval` member; the pointer stays valid for `'static`.
        unsafe { (*self.0.get()).strval = value.as_char_ptr() };
    }

    /// Reads the integer value of a write request ([`Driver::set_property`]).
    #[inline]
    pub fn int(&self) -> i32 {
        // SAFETY: for a write request the power supply core passes an initialised `intval`.
        unsafe { (*self.0.get()).intval }
    }
}

/// A driver backing a power supply.
///
/// The implementing type is the driver's own state; it is owned by the [`Registration`] and
/// reachable from the callbacks, which may run concurrently from several threads, hence the
/// `Sync` bound. Methods take `&self`: use interior mutability for anything that changes.
#[vtable]
pub trait Driver: Send + Sync + Sized {
    /// The name under which the supply appears in `/sys/class/power_supply/`.
    const NAME: &'static CStr;

    /// The kind of supply.
    const TYPE: Type;

    /// The properties this supply can answer. The core only asks for those listed here.
    const PROPERTIES: &'static [Property];

    /// Answers a property query.
    ///
    /// Return `Err(EINVAL)` for a property that is listed but cannot be answered now.
    fn get_property(&self, prop: Property, val: &PropertyValue) -> Result;

    /// Applies a property write. Only called for properties that
    /// [`property_is_writeable`](Self::property_is_writeable) accepted.
    fn set_property(&self, _prop: Property, _val: &PropertyValue) -> Result {
        build_error!(VTABLE_DEFAULT_ERROR)
    }

    /// Tells whether `prop` can be written. Called while the supply is being registered, after
    /// the driver state is fully initialised.
    fn property_is_writeable(&self, _prop: Property) -> bool {
        false
    }
}

/// What the power supply core points to: the C descriptor it keeps a pointer to for the whole
/// life of the supply, and the driver state.
#[pin_data]
struct Inner<T: Driver> {
    desc: Opaque<bindings::power_supply_desc>,
    #[pin]
    data: T,
}

/// Recovers the driver state from the supply the core calls back with.
///
/// # Safety
///
/// `psy` must be a supply registered by [`Registration::new`] for the same `T`, and the call
/// must happen from a callback, i.e. before `power_supply_unregister()` returns.
unsafe fn inner_from_psy<'a, T: Driver>(psy: *mut bindings::power_supply) -> &'a Inner<T> {
    // SAFETY: `psy` is valid per the function contract.
    let ptr = unsafe { bindings::power_supply_get_drvdata(psy) };
    // SAFETY: `drv_data` was set to the `Inner<T>` that the registration keeps alive until after
    // `power_supply_unregister()` has returned and no callback can run any more.
    unsafe { &*ptr.cast::<Inner<T>>() }
}

unsafe extern "C" fn get_property_callback<T: Driver>(
    psy: *mut bindings::power_supply,
    psp: bindings::power_supply_property,
    val: *mut bindings::power_supply_propval,
) -> c_int {
    // SAFETY: the core calls this callback only for a supply registered by `Registration::new`.
    let inner = unsafe { inner_from_psy::<T>(psy) };
    // SAFETY: the core passes a valid `val` that nobody else uses during the call.
    let val = unsafe { PropertyValue::from_raw(val) };

    from_result(|| {
        inner.data.get_property(Property(psp), val)?;
        Ok(0)
    })
}

unsafe extern "C" fn set_property_callback<T: Driver>(
    psy: *mut bindings::power_supply,
    psp: bindings::power_supply_property,
    val: *const bindings::power_supply_propval,
) -> c_int {
    // SAFETY: as in `get_property_callback`.
    let inner = unsafe { inner_from_psy::<T>(psy) };
    // SAFETY: the core passes a valid `val` that nobody else uses during the call.
    let val = unsafe { PropertyValue::from_raw(val) };

    from_result(|| {
        inner.data.set_property(Property(psp), val)?;
        Ok(0)
    })
}

unsafe extern "C" fn property_is_writeable_callback<T: Driver>(
    psy: *mut bindings::power_supply,
    psp: bindings::power_supply_property,
) -> c_int {
    // SAFETY: as in `get_property_callback`.
    let inner = unsafe { inner_from_psy::<T>(psy) };

    c_int::from(inner.data.property_is_writeable(Property(psp)))
}

/// A registered power supply.
///
/// Owns the driver state (`T`), which the callbacks borrow. On drop the supply is unregistered,
/// which waits for running callbacks, and only then is the state dropped.
///
/// # Invariants
///
/// `psy` is a supply registered by [`Registration::new`] with `inner` as its driver data, and is
/// unregistered only in `drop`.
pub struct Registration<'a, T: Driver> {
    psy: NonNull<bindings::power_supply>,
    inner: Pin<KBox<Inner<T>>>,
    _parent: PhantomData<&'a device::Device<Bound>>,
}

// SAFETY: `psy` is only used to call thread-safe C functions (`power_supply_changed()`,
// `power_supply_unregister()`), and `T` is `Send`.
unsafe impl<T: Driver> Send for Registration<'_, T> {}

// SAFETY: sharing a `&Registration` gives `&T` (`T: Sync`) and `power_supply_changed()`, which
// may be called from any thread.
unsafe impl<T: Driver> Sync for Registration<'_, T> {}

impl<'a, T: Driver> Registration<'a, T> {
    /// Registers a power supply described by `T` as a child of `parent`, with `data` as its
    /// state.
    ///
    /// The state is initialised before the supply becomes visible, so a callback never sees a
    /// half-built `T`.
    ///
    /// # Safety
    ///
    /// The caller must not `mem::forget()` the returned [`Registration`] or otherwise prevent its
    /// [`Drop`] implementation from running: the supply must be unregistered before `parent` is
    /// unbound, because `T` may borrow data that only lives as long as the binding.
    pub unsafe fn new(
        parent: &'a device::Device<Bound>,
        data: impl PinInit<T, Error>,
    ) -> Result<Self> {
        let mut desc = bindings::power_supply_desc::default();
        desc.name = T::NAME.as_char_ptr();
        desc.type_ = T::TYPE.as_raw();
        desc.properties = T::PROPERTIES.as_ptr().cast();
        desc.num_properties = T::PROPERTIES.len();
        desc.get_property = Some(get_property_callback::<T>);
        if T::HAS_SET_PROPERTY {
            desc.set_property = Some(set_property_callback::<T>);
            desc.property_is_writeable = Some(property_is_writeable_callback::<T>);
        }

        let inner = KBox::pin_init(
            try_pin_init!(Inner::<T> {
                desc: Opaque::new(desc),
                data <- data,
            }? Error),
            GFP_KERNEL,
        )?;

        let cfg = bindings::power_supply_config {
            drv_data: core::ptr::from_ref::<Inner<T>>(&inner).cast_mut().cast::<c_void>(),
            ..Default::default()
        };

        // SAFETY: `parent` is a bound device; `inner.desc` and the driver data stay in `inner`,
        // which the returned registration keeps alive until `power_supply_unregister()` has
        // returned; the core copies `cfg` during the call.
        let psy = from_err_ptr(unsafe {
            bindings::power_supply_register(parent.as_raw(), inner.desc.get(), &cfg)
        })?;

        Ok(Self {
            // INVARIANT: `power_supply_register()` succeeded, so `psy` is a registered supply.
            psy: NonNull::new(psy).ok_or(EINVAL)?,
            inner,
            _parent: PhantomData,
        })
    }

    /// Returns the driver state.
    pub fn data(&self) -> &T {
        &self.inner.data
    }

    /// Tells the core and user space that a property of this supply changed.
    pub fn changed(&self) {
        // SAFETY: `psy` is a registered supply per the type invariant.
        unsafe { bindings::power_supply_changed(self.psy.as_ptr()) };
    }
}

impl<T: Driver> Drop for Registration<'_, T> {
    fn drop(&mut self) {
        // SAFETY: `psy` is registered and not yet unregistered per the type invariant.
        // `power_supply_unregister()` removes the sysfs files, which waits for running
        // callbacks; after it returns none can run, so `inner` may be dropped.
        unsafe { bindings::power_supply_unregister(self.psy.as_ptr()) };
    }
}
