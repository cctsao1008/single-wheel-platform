#![no_std]

use swp_actuator_model::ActuatorPairCommand;
use swp_robot_domain::{AngularRateRadPerSec, StateValidity};

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

    /// Closed-loop authority additionally requires a healthy primary sensor clock.
    pub const fn actuation_authority_with_timing(
        self,
        timing: SensorTimingHealth,
    ) -> ActuationAuthority {
        if timing.closed_loop_eligible() {
            self.actuation_authority()
        } else {
            ActuationAuthority::Denied
        }
    }

    pub const fn is_faulted(self) -> bool {
        matches!(self, Self::Fault)
    }
}

/// Health of the primary sensor-driven real-time boundary.
///
/// This is deliberately independent of the sensor interrupt itself: a separate
/// MCU timer must be able to declare a late or missing observation even when the
/// sensor produces no interrupt at all.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SensorTimingHealth {
    #[default]
    Startup,
    Healthy,
    Late,
    Timeout,
}

impl SensorTimingHealth {
    pub const fn closed_loop_eligible(self) -> bool {
        matches!(self, Self::Healthy)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorTimingLimits {
    expected_period_us: u32,
    late_after_us: u32,
    timeout_after_us: u32,
}

impl SensorTimingLimits {
    pub const fn new(
        expected_period_us: u32,
        late_after_us: u32,
        timeout_after_us: u32,
    ) -> Option<Self> {
        if expected_period_us == 0
            || late_after_us < expected_period_us
            || timeout_after_us <= late_after_us
        {
            None
        } else {
            Some(Self {
                expected_period_us,
                late_after_us,
                timeout_after_us,
            })
        }
    }

    pub const fn expected_period_us(self) -> u32 {
        self.expected_period_us
    }

    pub const fn late_after_us(self) -> u32 {
        self.late_after_us
    }

    pub const fn timeout_after_us(self) -> u32 {
        self.timeout_after_us
    }

    pub const fn classify_elapsed_us(self, elapsed_us: u64) -> SensorTimingHealth {
        if elapsed_us >= self.timeout_after_us as u64 {
            SensorTimingHealth::Timeout
        } else if elapsed_us >= self.late_after_us as u64 {
            SensorTimingHealth::Late
        } else {
            SensorTimingHealth::Healthy
        }
    }
}

/// Stateful liveness monitor for a sensor-driven acquisition boundary.
///
/// `on_event()` is called from the sensor interrupt path. `poll()` is called
/// from an independent MCU timebase. The latter is what makes loss of the
/// sensor interrupt itself observable. One complete inter-event interval is
/// required before the monitor may report `Healthy`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorTimingMonitor {
    limits: SensorTimingLimits,
    started_at_us: u64,
    last_event_at_us: Option<u64>,
    cadence_verified: bool,
    health: SensorTimingHealth,
}

impl SensorTimingMonitor {
    pub const fn new(limits: SensorTimingLimits, started_at_us: u64) -> Self {
        Self {
            limits,
            started_at_us,
            last_event_at_us: None,
            cadence_verified: false,
            health: SensorTimingHealth::Startup,
        }
    }

    pub fn on_event(&mut self, event_at_us: u64) -> SensorTimingHealth {
        self.health = match self.last_event_at_us {
            Some(previous) => {
                self.cadence_verified = true;
                self.limits
                    .classify_elapsed_us(event_at_us.saturating_sub(previous))
            }
            None => SensorTimingHealth::Startup,
        };
        self.last_event_at_us = Some(event_at_us);
        self.health
    }

    pub fn poll(&mut self, now_us: u64) -> SensorTimingHealth {
        let reference = self.last_event_at_us.unwrap_or(self.started_at_us);
        let elapsed = now_us.saturating_sub(reference);

        self.health = if !self.cadence_verified && elapsed < self.limits.timeout_after_us as u64 {
            SensorTimingHealth::Startup
        } else {
            self.limits.classify_elapsed_us(elapsed)
        };
        self.health
    }

    pub const fn health(self) -> SensorTimingHealth {
        self.health
    }

    pub const fn last_event_at_us(self) -> Option<u64> {
        self.last_event_at_us
    }

    pub const fn cadence_verified(self) -> bool {
        self.cadence_verified
    }

    pub const fn limits(self) -> SensorTimingLimits {
        self.limits
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
    pub fn new(warning_abs_rad_per_sec: f32, hard_abs_rad_per_sec: f32) -> Option<Self> {
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

/// Bitwise explanation of why an actuator request was denied or constrained.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AuthorityReasons(u16);

impl AuthorityReasons {
    pub const NONE: Self = Self(0);
    pub const OPERATING_STATE: Self = Self(1 << 0);
    pub const SENSOR_TIMING: Self = Self(1 << 1);
    pub const ESTIMATE_INVALID: Self = Self(1 << 2);
    pub const REACTION_WHEEL_WARNING: Self = Self(1 << 3);
    pub const REACTION_WHEEL_EXHAUSTED: Self = Self(1 << 4);
    pub const DRIVE_SATURATED: Self = Self(1 << 5);
    pub const REACTION_SATURATED: Self = Self(1 << 6);

    pub const fn bits(self) -> u16 {
        self.0
    }

    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    const fn with(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

/// Independent evidence consumed by the physical-output authority gate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AuthorityContext {
    pub operating_state: OperatingState,
    pub timing: SensorTimingHealth,
    pub estimate_validity: StateValidity,
    pub reaction_wheel_authority: ReactionWheelAuthority,
}

/// Result of applying runtime authority to one bounded actuator request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AuthorityDecision {
    pub authority: ActuationAuthority,
    pub reasons: AuthorityReasons,
    pub constrained: bool,
    pub hold_integrator: bool,
}

impl AuthorityDecision {
    pub const fn closed_loop_authorized(self) -> bool {
        matches!(self.authority, ActuationAuthority::ClosedLoop)
    }
}

/// Type-level proof that a bounded actuator request passed runtime authority.
///
/// There is intentionally no public constructor. Board-specific electrical-output
/// code should accept this token rather than raw normalized commands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AuthorizedActuation {
    commands: ActuatorPairCommand,
}

impl AuthorizedActuation {
    pub const fn commands(self) -> ActuatorPairCommand {
        self.commands
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AuthorityOutcome {
    decision: AuthorityDecision,
    authorized: Option<AuthorizedActuation>,
}

impl AuthorityOutcome {
    pub const fn decision(self) -> AuthorityDecision {
        self.decision
    }

    pub const fn authorized(self) -> Option<AuthorizedActuation> {
        self.authorized
    }
}

/// The only semantic promotion boundary from bounded actuator command to a
/// physical-output-authorized command.
pub struct RuntimeAuthority;

impl RuntimeAuthority {
    pub fn evaluate(context: AuthorityContext, commands: ActuatorPairCommand) -> AuthorityOutcome {
        let mut reasons = AuthorityReasons::NONE;
        let mut denied = false;
        let mut constrained = false;

        if context.operating_state.actuation_authority() == ActuationAuthority::Denied {
            reasons = reasons.with(AuthorityReasons::OPERATING_STATE);
            denied = true;
        }
        if !context.timing.closed_loop_eligible() {
            reasons = reasons.with(AuthorityReasons::SENSOR_TIMING);
            denied = true;
        }
        if context.estimate_validity != StateValidity::Valid {
            reasons = reasons.with(AuthorityReasons::ESTIMATE_INVALID);
            denied = true;
        }

        match context.reaction_wheel_authority {
            ReactionWheelAuthority::Nominal => {}
            ReactionWheelAuthority::Warning => {
                reasons = reasons.with(AuthorityReasons::REACTION_WHEEL_WARNING);
                constrained = true;
            }
            ReactionWheelAuthority::Exhausted => {
                reasons = reasons.with(AuthorityReasons::REACTION_WHEEL_EXHAUSTED);
                denied = true;
            }
        }

        if commands.drive.saturated {
            reasons = reasons.with(AuthorityReasons::DRIVE_SATURATED);
            constrained = true;
        }
        if commands.reaction.saturated {
            reasons = reasons.with(AuthorityReasons::REACTION_SATURATED);
            constrained = true;
        }

        let decision = AuthorityDecision {
            authority: if denied {
                ActuationAuthority::Denied
            } else {
                ActuationAuthority::ClosedLoop
            },
            reasons,
            constrained,
            hold_integrator: denied || constrained,
        };

        AuthorityOutcome {
            decision,
            authorized: (!denied).then_some(AuthorizedActuation { commands }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use swp_actuator_model::BoundedActuatorCommand;
    use swp_robot_domain::{NormalizedCommand, TorqueNm};

    fn command(value: f32, saturated: bool) -> BoundedActuatorCommand {
        BoundedActuatorCommand {
            command: NormalizedCommand::new(value).unwrap(),
            saturated,
            predicted_torque_nm: TorqueNm(0.0),
        }
    }

    fn commands(drive_saturated: bool, reaction_saturated: bool) -> ActuatorPairCommand {
        ActuatorPairCommand {
            drive: command(0.25, drive_saturated),
            reaction: command(-0.25, reaction_saturated),
        }
    }

    fn healthy_context() -> AuthorityContext {
        AuthorityContext {
            operating_state: OperatingState::Balancing,
            timing: SensorTimingHealth::Healthy,
            estimate_validity: StateValidity::Valid,
            reaction_wheel_authority: ReactionWheelAuthority::Nominal,
        }
    }

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
    fn timing_health_gates_closed_loop_authority() {
        for timing in [
            SensorTimingHealth::Startup,
            SensorTimingHealth::Late,
            SensorTimingHealth::Timeout,
        ] {
            assert_eq!(
                OperatingState::Balancing.actuation_authority_with_timing(timing),
                ActuationAuthority::Denied
            );
        }

        assert_eq!(
            OperatingState::Balancing.actuation_authority_with_timing(SensorTimingHealth::Healthy),
            ActuationAuthority::ClosedLoop
        );
    }

    #[test]
    fn timing_limits_reject_invalid_configuration() {
        assert!(SensorTimingLimits::new(0, 3_000, 6_000).is_none());
        assert!(SensorTimingLimits::new(2_000, 1_999, 6_000).is_none());
        assert!(SensorTimingLimits::new(2_000, 3_000, 3_000).is_none());
    }

    #[test]
    fn independent_poll_detects_missing_sensor_events() {
        let limits = SensorTimingLimits::new(2_000, 3_000, 6_000).unwrap();
        let mut monitor = SensorTimingMonitor::new(limits, 100_000);

        assert_eq!(monitor.poll(102_000), SensorTimingHealth::Startup);
        assert_eq!(monitor.on_event(102_100), SensorTimingHealth::Startup);
        assert_eq!(monitor.poll(104_900), SensorTimingHealth::Startup);
        assert_eq!(monitor.poll(108_100), SensorTimingHealth::Timeout);
    }

    #[test]
    fn event_cadence_is_classified_at_the_sensor_boundary() {
        let limits = SensorTimingLimits::new(2_000, 3_000, 6_000).unwrap();
        let mut monitor = SensorTimingMonitor::new(limits, 0);

        assert_eq!(monitor.on_event(2_000), SensorTimingHealth::Startup);
        assert!(!monitor.cadence_verified());
        assert_eq!(monitor.on_event(4_100), SensorTimingHealth::Healthy);
        assert!(monitor.cadence_verified());
        assert_eq!(monitor.on_event(7_100), SensorTimingHealth::Late);
        assert_eq!(monitor.on_event(13_100), SensorTimingHealth::Timeout);
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

    #[test]
    fn healthy_runtime_promotes_bounded_commands_to_authorized_token() {
        let outcome = RuntimeAuthority::evaluate(healthy_context(), commands(false, false));
        assert_eq!(outcome.decision().authority, ActuationAuthority::ClosedLoop);
        assert_eq!(outcome.decision().reasons, AuthorityReasons::NONE);
        assert!(!outcome.decision().hold_integrator);
        assert!(outcome.authorized().is_some());
    }

    #[test]
    fn invalid_estimate_denies_physical_output() {
        let mut context = healthy_context();
        context.estimate_validity = StateValidity::Invalid;
        let outcome = RuntimeAuthority::evaluate(context, commands(false, false));
        assert_eq!(outcome.decision().authority, ActuationAuthority::Denied);
        assert!(
            outcome
                .decision()
                .reasons
                .contains(AuthorityReasons::ESTIMATE_INVALID)
        );
        assert!(outcome.decision().hold_integrator);
        assert!(outcome.authorized().is_none());
    }

    #[test]
    fn actuator_saturation_is_authorized_but_explicitly_constrained() {
        let outcome = RuntimeAuthority::evaluate(healthy_context(), commands(true, false));
        assert_eq!(outcome.decision().authority, ActuationAuthority::ClosedLoop);
        assert!(
            outcome
                .decision()
                .reasons
                .contains(AuthorityReasons::DRIVE_SATURATED)
        );
        assert!(outcome.decision().constrained);
        assert!(outcome.decision().hold_integrator);
        assert!(outcome.authorized().is_some());
    }

    #[test]
    fn reaction_wheel_warning_constrains_without_revoking_output() {
        let mut context = healthy_context();
        context.reaction_wheel_authority = ReactionWheelAuthority::Warning;
        let outcome = RuntimeAuthority::evaluate(context, commands(false, false));
        assert_eq!(outcome.decision().authority, ActuationAuthority::ClosedLoop);
        assert!(
            outcome
                .decision()
                .reasons
                .contains(AuthorityReasons::REACTION_WHEEL_WARNING)
        );
        assert!(outcome.decision().hold_integrator);
        assert!(outcome.authorized().is_some());
    }

    #[test]
    fn exhausted_reaction_wheel_revokes_physical_output() {
        let mut context = healthy_context();
        context.reaction_wheel_authority = ReactionWheelAuthority::Exhausted;
        let outcome = RuntimeAuthority::evaluate(context, commands(false, false));
        assert_eq!(outcome.decision().authority, ActuationAuthority::Denied);
        assert!(
            outcome
                .decision()
                .reasons
                .contains(AuthorityReasons::REACTION_WHEEL_EXHAUSTED)
        );
        assert!(outcome.authorized().is_none());
    }
}
