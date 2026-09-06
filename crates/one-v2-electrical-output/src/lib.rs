#![no_std]

use swp_runtime_state::AuthorizedActuation;

/// Vendor V2.0 TIM3 carrier frequency.
///
/// This is executable-source evidence from `MiniBalance_PWM_Init(7199, 0)` at a
/// 72 MHz timer clock. It is an electrical encoding fact, not a control-loop rate.
pub const VENDOR_V2_PWM_FREQUENCY_HZ: u32 = 10_000;

/// Electrical pin state requested for one installed motor interface.
///
/// `pwm_line_high_fraction` describes the physical PWM pin waveform, not motor
/// effort. The vendor V2.0 source encodes zero effort as an always-high PWM line
/// and increasing effort as an increasing low-active fraction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorElectricalCommand {
    pub direction_high: bool,
    pub pwm_line_high_fraction: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ElectricalActuation {
    pub drive: MotorElectricalCommand,
    pub reaction: MotorElectricalCommand,
}

/// Convert a runtime-authorized semantic command into the ONE V2.0 electrical
/// line encoding.
///
/// There is deliberately no public entry point that accepts a raw normalized
/// command. `AuthorizedActuation` is the only promotion token that can cross
/// into this electrical-output layer.
///
/// The mapping reproduces the vendor V2.0 pin-level convention:
///
/// - negative command -> DIR high
/// - non-negative command -> DIR low
/// - zero magnitude -> PWM line continuously high
/// - unit magnitude -> PWM line continuously low
///
/// The legacy reaction-wheel `+100` timer-count offset is intentionally not
/// reproduced here. Dead-zone and actuator nonlinearity belong to the actuator
/// model and require measured/identified evidence rather than a hidden electrical
/// heuristic.
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

#[cfg(test)]
mod tests {
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
        assert_eq!(
            electrical.drive,
            MotorElectricalCommand {
                direction_high: false,
                pwm_line_high_fraction: 1.0,
            }
        );
        assert_eq!(electrical.reaction, electrical.drive);
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
}
