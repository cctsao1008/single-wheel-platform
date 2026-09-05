#![no_std]

use swp_robot_domain::AngularRateRadPerSec;

/// High-level operating state for the reference single-wheel plant.
///
/// This is intentionally separate from the RTIC task graph. RTIC owns execution;
/// this state owns whether the physical plant may receive closed-loop authority.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OperatingState {
    #[default]
    Boot,
    HardwareCheck,
    Standby,
    CaptureWindow,
    Balancing,
    MomentumLimited,
    Fault,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActuationAuthority {
    Denied,
    ClosedLoop,
}

impl OperatingState {
    pub const fn actuation_authority(self) -> ActuationAuthority {
        match self {
            Self::Balancing | Self::MomentumLimited => ActuationAuthority::ClosedLoop,
            Self::Boot
            | Self::HardwareCheck
            | Self::Standby
            | Self::CaptureWindow
            | Self::Fault => ActuationAuthority::Denied,
        }
    }

    pub const fn is_faulted(self) -> bool {
        matches!(self, Self::Fault)
    }
}

/// Speed-domain evidence for remaining reaction-wheel authority.
///
/// Exact angular momentum requires a verified wheel inertia. Until that is known,
/// reaction-wheel speed is still a useful saturation observable and can be bounded
/// without pretending that momentum itself has been identified.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReactionWheelAuthority {
    Nominal,
    Warning,
    Exhausted,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReactionWheelSpeedLimits {
    warning_abs_rad_per_sec: f32,
    hard_abs_rad_per_sec: f32,
}

impl ReactionWheelSpeedLimits {
    pub fn new(
        warning_abs_rad_per_sec: f32,
        hard_abs_rad_per_sec: f32,
    ) -> Option<Self> {
        if warning_abs_rad_per_sec.is_finite()
            && hard_abs_rad_per_sec.is_finite()
            && warning_abs_rad_per_sec > 0.0
            && hard_abs_rad_per_sec > warning_abs_rad_per_sec
        {
            Some(Self {
                warning_abs_rad_per_sec,
                hard_abs_rad_per_sec,
            })
        } else {
            None
        }
    }

    pub const fn warning_abs_rad_per_sec(self) -> f32 {
        self.warning_abs_rad_per_sec
    }

    pub const fn hard_abs_rad_per_sec(self) -> f32 {
        self.hard_abs_rad_per_sec
    }

    pub fn classify(self, speed: AngularRateRadPerSec) -> ReactionWheelAuthority {
        let absolute_speed = speed.0.abs();
        if absolute_speed >= self.hard_abs_rad_per_sec {
            ReactionWheelAuthority::Exhausted
        } else if absolute_speed >= self.warning_abs_rad_per_sec {
            ReactionWheelAuthority::Warning
        } else {
            ReactionWheelAuthority::Nominal
        }
    }

    /// Normalized remaining speed headroom to the hard limit.
    ///
    /// 1.0 means stationary and 0.0 means at or beyond the configured hard limit.
    pub fn headroom_fraction(self, speed: AngularRateRadPerSec) -> f32 {
        let remaining = 1.0 - speed.0.abs() / self.hard_abs_rad_per_sec;
        remaining.clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn actuation_is_denied_outside_closed_loop_states() {
        for state in [
            OperatingState::Boot,
            OperatingState::HardwareCheck,
            OperatingState::Standby,
            OperatingState::CaptureWindow,
            OperatingState::Fault,
        ] {
            assert_eq!(state.actuation_authority(), ActuationAuthority::Denied);
        }

        assert_eq!(
            OperatingState::Balancing.actuation_authority(),
            ActuationAuthority::ClosedLoop
        );
        assert_eq!(
            OperatingState::MomentumLimited.actuation_authority(),
            ActuationAuthority::ClosedLoop
        );
    }

    #[test]
    fn reaction_wheel_limits_reject_invalid_configuration() {
        assert!(ReactionWheelSpeedLimits::new(0.0, 100.0).is_none());
        assert!(ReactionWheelSpeedLimits::new(100.0, 100.0).is_none());
        assert!(ReactionWheelSpeedLimits::new(120.0, 100.0).is_none());
        assert!(ReactionWheelSpeedLimits::new(f32::NAN, 100.0).is_none());
    }

    #[test]
    fn reaction_wheel_authority_is_symmetric_in_speed_sign() {
        let limits = ReactionWheelSpeedLimits::new(80.0, 100.0).unwrap();

        assert_eq!(
            limits.classify(AngularRateRadPerSec(79.0)),
            ReactionWheelAuthority::Nominal
        );
        assert_eq!(
            limits.classify(AngularRateRadPerSec(-80.0)),
            ReactionWheelAuthority::Warning
        );
        assert_eq!(
            limits.classify(AngularRateRadPerSec(100.0)),
            ReactionWheelAuthority::Exhausted
        );
        assert_eq!(
            limits.classify(AngularRateRadPerSec(-120.0)),
            ReactionWheelAuthority::Exhausted
        );
    }

    #[test]
    fn headroom_is_bounded() {
        let limits = ReactionWheelSpeedLimits::new(80.0, 100.0).unwrap();
        assert_eq!(limits.headroom_fraction(AngularRateRadPerSec(0.0)), 1.0);
        assert!((limits.headroom_fraction(AngularRateRadPerSec(50.0)) - 0.5).abs() < 1.0e-6);
        assert_eq!(limits.headroom_fraction(AngularRateRadPerSec(120.0)), 0.0);
    }
}
