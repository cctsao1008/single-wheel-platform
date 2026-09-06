#![no_std]

use swp_robot_domain::StateValidity;
use swp_runtime_state::{OperatingState, ReactionWheelAuthority, SensorTimingHealth};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RuntimeFaults(u16);

impl RuntimeFaults {
    pub const NONE: Self = Self(0);
    pub const SENSOR_TIMEOUT: Self = Self(1 << 0);
    pub const CONTROL_WATCHDOG_TIMEOUT: Self = Self(1 << 1);
    pub const ESTIMATE_INVALID: Self = Self(1 << 2);
    pub const REACTION_WHEEL_EXHAUSTED: Self = Self(1 << 3);
    pub const CONTROL_NUMERICAL_FAULT: Self = Self(1 << 4);

    pub const fn bits(self) -> u16 { self.0 }
    pub const fn from_bits(bits: u16) -> Self { Self(bits) }
    pub const fn contains(self, other: Self) -> bool { self.0 & other.0 == other.0 }
    pub const fn is_empty(self) -> bool { self.0 == 0 }
    pub const fn with(self, other: Self) -> Self { Self(self.0 | other.0) }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ControlWatchdogHealth {
    #[default]
    Startup,
    Healthy,
    Timeout,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlWatchdog {
    timeout_us: u32,
    armed_at_us: u64,
    last_completed_at_us: Option<u64>,
    health: ControlWatchdogHealth,
}

impl ControlWatchdog {
    pub const fn new(timeout_us: u32, armed_at_us: u64) -> Option<Self> {
        if timeout_us == 0 {
            None
        } else {
            Some(Self {
                timeout_us,
                armed_at_us,
                last_completed_at_us: None,
                health: ControlWatchdogHealth::Startup,
            })
        }
    }

    pub fn observe_control_completion(&mut self, completed_at_us: u64) -> ControlWatchdogHealth {
        self.last_completed_at_us = Some(completed_at_us);
        self.health = ControlWatchdogHealth::Healthy;
        self.health
    }

    pub fn poll(&mut self, now_us: u64) -> ControlWatchdogHealth {
        let reference = self.last_completed_at_us.unwrap_or(self.armed_at_us);
        self.health = if now_us.saturating_sub(reference) >= u64::from(self.timeout_us) {
            ControlWatchdogHealth::Timeout
        } else if self.last_completed_at_us.is_some() {
            ControlWatchdogHealth::Healthy
        } else {
            ControlWatchdogHealth::Startup
        };
        self.health
    }

    pub const fn health(self) -> ControlWatchdogHealth { self.health }
    pub const fn timeout_us(self) -> u32 { self.timeout_us }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FaultLatch {
    faults: RuntimeFaults,
}

impl FaultLatch {
    pub const fn new() -> Self { Self { faults: RuntimeFaults::NONE } }
    pub fn latch(&mut self, faults: RuntimeFaults) { self.faults = self.faults.with(faults); }
    pub fn clear_all(&mut self) { self.faults = RuntimeFaults::NONE; }
    pub const fn faults(self) -> RuntimeFaults { self.faults }
    pub const fn is_faulted(self) -> bool { !self.faults.is_empty() }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RuntimeTransitionError {
    InvalidTransition,
    FaultStillLatched,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RuntimeSupervisor {
    state: OperatingState,
    faults: FaultLatch,
}

impl Default for RuntimeSupervisor {
    fn default() -> Self { Self::new() }
}

impl RuntimeSupervisor {
    pub const fn new() -> Self {
        Self { state: OperatingState::Boot, faults: FaultLatch::new() }
    }

    pub const fn state(self) -> OperatingState { self.state }
    pub const fn faults(self) -> RuntimeFaults { self.faults.faults() }

    pub fn boot_complete(&mut self) -> Result<(), RuntimeTransitionError> {
        if self.state != OperatingState::Boot { return Err(RuntimeTransitionError::InvalidTransition); }
        self.state = OperatingState::HardwareCheck;
        Ok(())
    }

    pub fn hardware_check_passed(&mut self) -> Result<(), RuntimeTransitionError> {
        if self.state != OperatingState::HardwareCheck || self.faults.is_faulted() {
            return Err(RuntimeTransitionError::InvalidTransition);
        }
        self.state = OperatingState::Standby;
        Ok(())
    }

    pub fn request_balance(&mut self) -> Result<(), RuntimeTransitionError> {
        if self.state != OperatingState::Standby || self.faults.is_faulted() {
            return Err(RuntimeTransitionError::InvalidTransition);
        }
        self.state = OperatingState::CaptureWindow;
        Ok(())
    }

    pub fn capture_ready(&mut self) -> Result<(), RuntimeTransitionError> {
        if self.state != OperatingState::CaptureWindow || self.faults.is_faulted() {
            return Err(RuntimeTransitionError::InvalidTransition);
        }
        self.state = OperatingState::Balancing;
        Ok(())
    }

    pub fn latch_fault(&mut self, fault: RuntimeFaults) {
        self.faults.latch(fault);
        self.state = OperatingState::Fault;
    }

    /// Fault recovery is explicit; runtime health never auto-clears a latch.
    pub fn clear_faults_to_standby(&mut self) -> Result<(), RuntimeTransitionError> {
        if self.state != OperatingState::Fault { return Err(RuntimeTransitionError::InvalidTransition); }
        self.faults.clear_all();
        if self.faults.is_faulted() { return Err(RuntimeTransitionError::FaultStillLatched); }
        self.state = OperatingState::Standby;
        Ok(())
    }

    /// Called from an independent MCU timer so loss of the sensor-driven control
    /// task itself remains observable.
    pub fn observe_independent_health(
        &mut self,
        timing: SensorTimingHealth,
        watchdog: ControlWatchdogHealth,
    ) {
        if !matches!(self.state, OperatingState::CaptureWindow | OperatingState::Balancing | OperatingState::MomentumLimited) {
            return;
        }
        if timing == SensorTimingHealth::Timeout { self.latch_fault(RuntimeFaults::SENSOR_TIMEOUT); }
        if watchdog == ControlWatchdogHealth::Timeout { self.latch_fault(RuntimeFaults::CONTROL_WATCHDOG_TIMEOUT); }
    }

    pub fn observe_control_health(
        &mut self,
        estimate_validity: StateValidity,
        reaction_wheel_authority: ReactionWheelAuthority,
    ) {
        if !matches!(self.state, OperatingState::Balancing | OperatingState::MomentumLimited) { return; }
        if estimate_validity != StateValidity::Valid {
            self.latch_fault(RuntimeFaults::ESTIMATE_INVALID);
            return;
        }
        match reaction_wheel_authority {
            ReactionWheelAuthority::Nominal => {
                if self.state == OperatingState::MomentumLimited { self.state = OperatingState::Balancing; }
            }
            ReactionWheelAuthority::Warning => self.state = OperatingState::MomentumLimited,
            ReactionWheelAuthority::Exhausted => self.latch_fault(RuntimeFaults::REACTION_WHEEL_EXHAUSTED),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn active_supervisor() -> RuntimeSupervisor {
        let mut supervisor = RuntimeSupervisor::new();
        supervisor.boot_complete().unwrap();
        supervisor.hardware_check_passed().unwrap();
        supervisor.request_balance().unwrap();
        supervisor.capture_ready().unwrap();
        supervisor
    }

    #[test]
    fn state_machine_requires_explicit_progression() {
        let mut supervisor = RuntimeSupervisor::new();
        assert_eq!(supervisor.state(), OperatingState::Boot);
        supervisor.boot_complete().unwrap();
        assert_eq!(supervisor.state(), OperatingState::HardwareCheck);
        supervisor.hardware_check_passed().unwrap();
        assert_eq!(supervisor.state(), OperatingState::Standby);
        supervisor.request_balance().unwrap();
        assert_eq!(supervisor.state(), OperatingState::CaptureWindow);
        supervisor.capture_ready().unwrap();
        assert_eq!(supervisor.state(), OperatingState::Balancing);
    }

    #[test]
    fn control_watchdog_detects_missing_completion_independently() {
        let mut watchdog = ControlWatchdog::new(15_000, 100_000).unwrap();
        assert_eq!(watchdog.poll(110_000), ControlWatchdogHealth::Startup);
        assert_eq!(watchdog.poll(115_000), ControlWatchdogHealth::Timeout);
        assert_eq!(watchdog.observe_control_completion(116_000), ControlWatchdogHealth::Healthy);
        assert_eq!(watchdog.poll(120_000), ControlWatchdogHealth::Healthy);
    }

    #[test]
    fn timeout_faults_are_latched_and_do_not_self_clear() {
        let mut supervisor = active_supervisor();
        supervisor.observe_independent_health(SensorTimingHealth::Timeout, ControlWatchdogHealth::Healthy);
        assert_eq!(supervisor.state(), OperatingState::Fault);
        assert!(supervisor.faults().contains(RuntimeFaults::SENSOR_TIMEOUT));
        supervisor.observe_independent_health(SensorTimingHealth::Healthy, ControlWatchdogHealth::Healthy);
        assert_eq!(supervisor.state(), OperatingState::Fault);
    }

    #[test]
    fn reaction_warning_is_recoverable_but_exhaustion_is_faulted() {
        let mut supervisor = active_supervisor();
        supervisor.observe_control_health(StateValidity::Valid, ReactionWheelAuthority::Warning);
        assert_eq!(supervisor.state(), OperatingState::MomentumLimited);
        supervisor.observe_control_health(StateValidity::Valid, ReactionWheelAuthority::Nominal);
        assert_eq!(supervisor.state(), OperatingState::Balancing);
        supervisor.observe_control_health(StateValidity::Valid, ReactionWheelAuthority::Exhausted);
        assert_eq!(supervisor.state(), OperatingState::Fault);
        assert!(supervisor.faults().contains(RuntimeFaults::REACTION_WHEEL_EXHAUSTED));
    }

    #[test]
    fn estimate_invalid_latches_fault_only_after_capture() {
        let mut supervisor = RuntimeSupervisor::new();
        supervisor.boot_complete().unwrap();
        supervisor.hardware_check_passed().unwrap();
        supervisor.request_balance().unwrap();
        supervisor.observe_control_health(StateValidity::Invalid, ReactionWheelAuthority::Nominal);
        assert_eq!(supervisor.state(), OperatingState::CaptureWindow);
        supervisor.capture_ready().unwrap();
        supervisor.observe_control_health(StateValidity::Invalid, ReactionWheelAuthority::Nominal);
        assert_eq!(supervisor.state(), OperatingState::Fault);
        assert!(supervisor.faults().contains(RuntimeFaults::ESTIMATE_INVALID));
    }
}
