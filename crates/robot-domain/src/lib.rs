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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StateValidity {
    Invalid,
    Valid,
}

impl Default for StateValidity {
    fn default() -> Self {
        Self::Invalid
    }
}

/// State presented to the control domain.
///
/// The type is intentionally specific to the single-wheel plant rather than a
/// generic robotics state container.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RobotState {
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

/// Requested control effort before actuator-specific electrical translation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControlEffort {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Actuator {
    ReactionWheel,
    DriveWheel,
    Spin,
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

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ActuatorCommand {
    pub command: NormalizedCommand,
    pub enabled: bool,
    pub brake_requested: bool,
}
