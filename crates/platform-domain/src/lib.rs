#![no_std]

/// Angle in radians.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AngleRad(pub f32);

/// Angular rate in radians per second.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AngularRateRadPerSec(pub f32);

/// Voltage in volts.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Volts(pub f32);

/// Timestamp in microseconds on the firmware monotonic timebase.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct TimestampUs(pub u64);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum StateValidity {
    #[default]
    Invalid,
    Valid,
}

/// State presented to the control domain.
///
/// The type is intentionally specific to the inspected single-wheel plant
/// rather than a generic platform state container. Yaw rate remains observable
/// from the IMU even though the verified assembly has no dedicated yaw actuator.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlatformState {
    pub timestamp: TimestampUs,
    pub roll: AngleRad,
    pub roll_rate: AngularRateRadPerSec,
    pub pitch: AngleRad,
    pub pitch_rate: AngularRateRadPerSec,
    pub reaction_wheel_speed: AngularRateRadPerSec,
    pub drive_wheel_speed: AngularRateRadPerSec,
    pub yaw_rate: AngularRateRadPerSec,
    pub battery: Volts,
    pub validity: StateValidity,
}

/// Generalized control demand for the two physically installed actuation axes.
///
/// There is intentionally no yaw demand: the inspected reference assembly has
/// a reaction-wheel actuator and a ground-drive actuator, while the third PCB
/// motor interface is not populated.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneralizedDemand {
    pub roll: f32,
    pub pitch: f32,
}

/// Platform-semantic actuator identity for the currently verified assembly.
///
/// PCB motor-channel identity remains in `swp-board-one-v2`; the mapping between
/// those channels and these roles belongs to the assembly layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Actuator {
    ReactionWheel,
    DriveWheel,
}

/// Bounded actuator request in the abstract actuator domain.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalizedCommand(f32);

impl NormalizedCommand {
    pub const ZERO: Self = Self(0.0);

    pub fn new(value: f32) -> Option<Self> {
        if (-1.0..=1.0).contains(&value) {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const fn get(self) -> f32 {
        self.0
    }
}

impl Default for NormalizedCommand {
    fn default() -> Self {
        Self::ZERO
    }
}

/// Platform-semantic actuator request before board-specific electrical mapping.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ActuatorCommand {
    pub command: NormalizedCommand,
    pub enabled: bool,
}
