#![no_std]

use swp_actuation_interface::{ActuationSink, ActuatorIo};
use swp_runtime_state::AuthorizedActuation;

/// Vendor V2.0 TIM3 carrier frequency.
///
/// This is executable-source evidence from `MiniBalance_PWM_Init(7199, 0)` at a
/// 72 MHz timer clock. It is an actuator-interface fact, not a control-loop rate.
pub const VENDOR_V2_PWM_FREQUENCY_HZ: u32 = 10_000;

/// Electrical line state requested for one installed motor interface.
///
/// `pwm_line_high_fraction` describes the physical PWM pin waveform, not motor
/// effort. The vendor V2.0 interface encodes zero effort as an always-high PWM
/// line and increasing effort as an increasing low-active fraction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorElectricalCommand {
    pub direction_high: bool,
    pub pwm_line_high_fraction: f32,
}

/// Actuator-specific electrical frame for the two installed balance actuators.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ElectricalActuation {
    pub drive: MotorElectricalCommand,
    pub reaction: MotorElectricalCommand,
}

impl ElectricalActuation {
    pub const fn zero_effort() -> Self {
        let zero = MotorElectricalCommand {
            direction_high: false,
            pwm_line_high_fraction: 1.0,
        };
        Self {
            drive: zero,
            reaction: zero,
        }
    }
}

/// Convert a runtime-authorized semantic command into the ONE V2 PWM/DIR
/// electrical encoding.
///
/// There is deliberately no entry point that accepts a raw normalized command.
/// `AuthorizedActuation` remains the promotion token that crosses from Supervisor
/// into the physical-actuation side of Firmware.
///
/// The mapping reproduces the vendor V2.0 pin-level convention:
///
/// - negative command -> DIR high
/// - non-negative command -> DIR low
/// - zero magnitude -> PWM line continuously high
/// - unit magnitude -> PWM line continuously low
///
/// The legacy reaction-wheel `+100` timer-count offset is intentionally not
/// reproduced. Dead-zone and actuator nonlinearity belong to the Plant actuator
/// model and require measured/identified evidence rather than hidden electrical
/// heuristics.
pub fn encode_authorized(authorized: AuthorizedActuation) -> ElectricalActuation {
    let commands = authorized.commands();
    ElectricalActuation {
        drive: encode_channel(commands.drive.command.get()),
        reaction: encode_channel(commands.reaction.command.get()),
    }
}

fn encode_channel(command: f32) -> MotorElectricalCommand {
    let magnitude = command.abs().clamp(0.0, 1.0);
    MotorElectricalCommand {
        direction_high: command < 0.0,
        pwm_line_high_fraction: 1.0 - magnitude,
    }
}

/// Portable ONE V2 PWM/DIR actuator adapter.
///
/// `Io` is supplied by the selected control-board target. STM32F103, RP2350, or
/// another MCU may implement `ActuatorIo<ElectricalActuation>` without changing
/// the actuator encoding or the upstream Plant / Supervisor / Control domains.
pub struct OneV2PwmDirAdapter<Io> {
    io: Io,
}

impl<Io> OneV2PwmDirAdapter<Io> {
    pub const fn new(io: Io) -> Self {
        Self { io }
    }

    pub fn io_mut(&mut self) -> &mut Io {
        &mut self.io
    }

    pub fn into_inner(self) -> Io {
        self.io
    }
}

impl<Io> ActuationSink for OneV2PwmDirAdapter<Io>
where
    Io: ActuatorIo<ElectricalActuation>,
{
    type Error = Io::Error;

    fn apply_authorized(&mut self, actuation: AuthorizedActuation) -> Result<(), Self::Error> {
        self.io.write_frame(encode_authorized(actuation))
    }

    fn revoke(&mut self) -> Result<(), Self::Error> {
        self.io.write_frame(ElectricalActuation::zero_effort())
    }
}

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use super::*;
    use swp_actuator_model::{ActuatorPairCommand, BoundedActuatorCommand};
    use swp_robot_domain::{NormalizedCommand, StateValidity, TorqueNm};
    use swp_runtime_state::{
        AuthorityContext, OperatingState, ReactionWheelAuthority, RuntimeAuthority,
        SensorTimingHealth,
    };

    fn bounded(value: f32) -> BoundedActuatorCommand {
        BoundedActuatorCommand {
            command: NormalizedCommand::new(value).unwrap(),
            saturated: false,
            predicted_torque_nm: TorqueNm(0.0),
        }
    }

    fn authorized(drive: f32, reaction: f32) -> AuthorizedActuation {
        RuntimeAuthority::evaluate(
            AuthorityContext {
                operating_state: OperatingState::Balancing,
                timing: SensorTimingHealth::Healthy,
                estimate_validity: StateValidity::Valid,
                reaction_wheel_authority: ReactionWheelAuthority::Nominal,
            },
            ActuatorPairCommand {
                drive: bounded(drive),
                reaction: bounded(reaction),
            },
        )
        .authorized()
        .unwrap()
    }

    #[test]
    fn zero_effort_is_vendor_idle_line_encoding() {
        let electrical = encode_authorized(authorized(0.0, 0.0));
        assert_eq!(electrical, ElectricalActuation::zero_effort());
    }

    #[test]
    fn sign_maps_only_to_direction_and_magnitude_maps_to_low_active_pwm() {
        let electrical = encode_authorized(authorized(0.25, -0.75));
        assert!(!electrical.drive.direction_high);
        assert!((electrical.drive.pwm_line_high_fraction - 0.75).abs() < 1.0e-6);
        assert!(electrical.reaction.direction_high);
        assert!((electrical.reaction.pwm_line_high_fraction - 0.25).abs() < 1.0e-6);
    }

    #[test]
    fn unit_effort_is_continuously_low_pwm_line() {
        let electrical = encode_authorized(authorized(1.0, -1.0));
        assert_eq!(electrical.drive.pwm_line_high_fraction, 0.0);
        assert_eq!(electrical.reaction.pwm_line_high_fraction, 0.0);
        assert!(!electrical.drive.direction_high);
        assert!(electrical.reaction.direction_high);
    }

    #[derive(Default)]
    struct FakeIo {
        last: Option<ElectricalActuation>,
    }

    impl ActuatorIo<ElectricalActuation> for FakeIo {
        type Error = Infallible;

        fn write_frame(&mut self, frame: ElectricalActuation) -> Result<(), Self::Error> {
            self.last = Some(frame);
            Ok(())
        }
    }

    #[test]
    fn actuation_sink_revoke_replaces_the_previous_command_with_zero_effort() {
        let mut sink = OneV2PwmDirAdapter::new(FakeIo::default());
        sink.apply_authorized(authorized(0.4, -0.6)).unwrap();
        assert_ne!(sink.io_mut().last, Some(ElectricalActuation::zero_effort()));

        sink.revoke().unwrap();
        assert_eq!(sink.io_mut().last, Some(ElectricalActuation::zero_effort()));
    }
}
