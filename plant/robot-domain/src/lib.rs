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

/// Torque in newton-metres.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TorqueNm(pub f32);

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
/// rather than a generic robotics state container. Yaw rate remains observable
/// from the IMU even though the verified assembly has no dedicated yaw actuator.
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

/// Physical generalized demand produced by the current state-space controller.
///
/// The current upright plant is synthesized directly in the two populated
/// actuator-effort coordinates. These values are physical torques, never PWM or
/// normalized duty. Reference-assembly allocation owns the mapping from these
/// robot-semantic roles to the plant input / board channels.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneralizedDemand {
    pub drive_wheel_torque: TorqueNm,
    pub reaction_wheel_torque: TorqueNm,
}

/// Robot-semantic actuator identity for the currently verified assembly.
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

/// Robot-semantic actuator request before board-specific electrical mapping.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ActuatorCommand {
    pub command: NormalizedCommand,
    pub enabled: bool,
}
