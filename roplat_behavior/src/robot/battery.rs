//! Batteries: what a pack reports about its charge and health.

use std::ops::{BitOr, BitOrAssign};

use roplat_msgs::Stamped;

use crate::RobotResult;

/// What a battery is doing, as the device reports it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BatteryStatus {
    /// Taking charge from a charger.
    Charging,
    /// Delivering charge to the load.
    Discharging,
    /// Charged, the charger holding it there or disconnected.
    Full,
    /// Neither charging nor discharging, such as a pack on a charger that
    /// does not charge it, or a pack switched off from its load.
    Idle,
}

/// Faults a battery reports, as a set: any number of them at once.
///
/// ```
/// use roplat_behavior::BatteryFaults;
///
/// let faults = BatteryFaults::OVER_TEMPERATURE | BatteryFaults::OVER_CURRENT;
/// assert!(faults.contains(BatteryFaults::OVER_TEMPERATURE));
/// assert!(!faults.contains(BatteryFaults::UNDER_VOLTAGE));
/// assert!(BatteryFaults::NONE.is_empty());
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct BatteryFaults(u16);

impl BatteryFaults {
    /// No fault.
    pub const NONE: Self = Self(0);
    /// A cell or the pack above its highest voltage.
    pub const OVER_VOLTAGE: Self = Self(1 << 0);
    /// A cell or the pack below its lowest voltage.
    pub const UNDER_VOLTAGE: Self = Self(1 << 1);
    /// Charge or discharge current above the pack's limit.
    pub const OVER_CURRENT: Self = Self(1 << 2);
    /// Too hot to charge or discharge.
    pub const OVER_TEMPERATURE: Self = Self(1 << 3);
    /// Too cold to charge or discharge.
    pub const UNDER_TEMPERATURE: Self = Self(1 << 4);
    /// The cells' voltages too far apart.
    pub const CELL_IMBALANCE: Self = Self(1 << 5);
    /// A short circuit detected.
    pub const SHORT_CIRCUIT: Self = Self(1 << 6);
    /// A fault the device reports that none of the others names; the
    /// driver documents which device faults it maps here.
    pub const OTHER: Self = Self(1 << 7);

    /// Every fault this type names.
    pub const ALL: Self = Self(0xff);

    /// The set's bits, one per fault constant.
    pub const fn bits(self) -> u16 {
        self.0
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Whether every fault of `other` is in `self`.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }
}

impl BitOr for BatteryFaults {
    type Output = Self;

    fn bitor(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

impl BitOrAssign for BatteryFaults {
    fn bitor_assign(&mut self, other: Self) {
        self.0 |= other.0;
    }
}

/// One reading of a battery. A quantity the device does not report is
/// `None`, never zero or another stand-in value.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BatteryReading {
    /// Volts across the pack's terminals.
    pub voltage: Option<f64>,
    /// Amperes out of the pack: positive while it discharges into the load,
    /// negative while it charges. A device that reports charging current as
    /// positive has its driver negate it.
    pub current: Option<f64>,
    /// Ampere-hours left in the pack.
    pub charge: Option<f64>,
    /// Ampere-hours the pack holds when full, as the device now estimates
    /// it (not the design capacity).
    pub capacity: Option<f64>,
    /// The state of charge in `[0, 1]`: 1 full, 0 empty. A device that
    /// reports percent has its driver divide by 100.
    pub state_of_charge: Option<f64>,
    /// Degrees Celsius of the pack, or of its warmest cell when the device
    /// reports cells only.
    pub temperature: Option<f64>,
    pub status: Option<BatteryStatus>,
    /// The faults the device reports; `Some(BatteryFaults::NONE)` when it
    /// reports none, `None` when it reports no fault state at all.
    pub faults: Option<BatteryFaults>,
}

/// A battery pack, or the battery a robot's driver reports on.
///
/// Like the sensor contracts it is not a [`Robot`](crate::Robot) subtrait: a
/// pack may be a device of its own (a CANopen pack) or part of a base's
/// driver (a Create 3, a VDA 5050 vehicle), so the type that owns the
/// connection implements it.
///
/// [`read_battery`](Self::read_battery) returns the pack's latest state:
/// the driver polls the device, or takes the last status message the
/// device sent. The header's stamp is when the driver received the
/// reading, on the clock it stamps with; `seq` rises with every reading and
/// `frame` names the pack. Stopping the robot on a fault is the
/// application's decision; emergency stops are read where the robot is
/// stopped, not here.
pub trait Battery {
    fn read_battery(&mut self) -> RobotResult<Stamped<BatteryReading>>;
}
