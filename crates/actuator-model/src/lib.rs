#![no_std]

use swp_robot_domain::{GeneralizedDemand, NormalizedCommand, TorqueNm};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorParameters {
    pub torque_per_effective_command_nm: f32,
    pub command_deadzone: f32,
    pub viscous_friction_nm_per_rad_s: f32,
    pub coulomb_friction_nm: f32,
    pub friction_sign_epsilon_rad_s: f32,
}

impl ActuatorParameters {
    pub fn new(
        torque_per_effective_command_nm: f32,
        command_deadzone: f32,
        viscous_friction_nm_per_rad_s: f32,
        coulomb_friction_nm: f32,
        friction_sign_epsilon_rad_s: f32,
    ) -> Option<Self> {
        let candidate = Self {
            torque_per_effective_command_nm,
            command_deadzone,
            viscous_friction_nm_per_rad_s,
            coulomb_friction_nm,
            friction_sign_epsilon_rad_s,
        };
        candidate.is_valid().then_some(candidate)
    }

    pub fn is_valid(self) -> bool {
        self.torque_per_effective_command_nm.is_finite()
            && self.torque_per_effective_command_nm > 0.0
            && self.command_deadzone.is_finite()
            && (0.0..1.0).contains(&self.command_deadzone)
            && self.viscous_friction_nm_per_rad_s.is_finite()
            && self.viscous_friction_nm_per_rad_s >= 0.0
            && self.coulomb_friction_nm.is_finite()
            && self.coulomb_friction_nm >= 0.0
            && self.friction_sign_epsilon_rad_s.is_finite()
            && self.friction_sign_epsilon_rad_s > 0.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorOperatingPoint {
    pub speed_rad_per_s: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundedActuatorCommand {
    pub command: NormalizedCommand,
    pub saturated: bool,
    pub predicted_torque_nm: TorqueNm,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ActuatorModelError {
    InvalidParameters,
    NonFiniteDemand,
    NonFiniteOperatingPoint,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StaticActuatorModel {
    parameters: ActuatorParameters,
}

impl StaticActuatorModel {
    pub fn new(parameters: ActuatorParameters) -> Option<Self> {
        parameters.is_valid().then_some(Self { parameters })
    }

    pub const fn parameters(self) -> ActuatorParameters {
        self.parameters
    }

    pub fn torque_from_command(
        self,
        command: NormalizedCommand,
        operating_point: ActuatorOperatingPoint,
    ) -> Result<TorqueNm, ActuatorModelError> {
        if !operating_point.speed_rad_per_s.is_finite() {
            return Err(ActuatorModelError::NonFiniteOperatingPoint);
        }
        let effective = effective_command(command.get(), self.parameters.command_deadzone);
        let motor = self.parameters.torque_per_effective_command_nm * effective;
        let loss = friction_torque(self.parameters, operating_point.speed_rad_per_s);
        Ok(TorqueNm(motor - loss))
    }

    pub fn command_for_torque(
        self,
        requested_torque: TorqueNm,
        operating_point: ActuatorOperatingPoint,
    ) -> Result<BoundedActuatorCommand, ActuatorModelError> {
        if !requested_torque.0.is_finite() {
            return Err(ActuatorModelError::NonFiniteDemand);
        }
        if !operating_point.speed_rad_per_s.is_finite() {
            return Err(ActuatorModelError::NonFiniteOperatingPoint);
        }

        let loss = friction_torque(self.parameters, operating_point.speed_rad_per_s);
        let required_motor_torque = requested_torque.0 + loss;
        let required_effective =
            required_motor_torque / self.parameters.torque_per_effective_command_nm;
        let saturated = required_effective.abs() > 1.0;
        let bounded_effective = required_effective.clamp(-1.0, 1.0);
        let raw_command = inverse_effective_command(bounded_effective, self.parameters.command_deadzone);
        let command = NormalizedCommand::new(raw_command).expect("bounded inverse command");
        let predicted_torque_nm = self.torque_from_command(command, operating_point)?;

        Ok(BoundedActuatorCommand {
            command,
            saturated,
            predicted_torque_nm,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorPairModel {
    pub drive: StaticActuatorModel,
    pub reaction: StaticActuatorModel,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorPairOperatingPoint {
    pub drive_speed_rad_per_s: f32,
    pub reaction_speed_rad_per_s: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorPairCommand {
    pub drive: BoundedActuatorCommand,
    pub reaction: BoundedActuatorCommand,
}

impl ActuatorPairModel {
    pub fn command_for_demand(
        self,
        demand: GeneralizedDemand,
        operating_point: ActuatorPairOperatingPoint,
    ) -> Result<ActuatorPairCommand, ActuatorModelError> {
        Ok(ActuatorPairCommand {
            drive: self.drive.command_for_torque(
                demand.drive_wheel_torque,
                ActuatorOperatingPoint {
                    speed_rad_per_s: operating_point.drive_speed_rad_per_s,
                },
            )?,
            reaction: self.reaction.command_for_torque(
                demand.reaction_wheel_torque,
                ActuatorOperatingPoint {
                    speed_rad_per_s: operating_point.reaction_speed_rad_per_s,
                },
            )?,
        })
    }
}

fn effective_command(command: f32, deadzone: f32) -> f32 {
    let magnitude = command.abs();
    if magnitude <= deadzone {
        0.0
    } else {
        command.signum() * (magnitude - deadzone) / (1.0 - deadzone)
    }
}

fn inverse_effective_command(effective: f32, deadzone: f32) -> f32 {
    if effective == 0.0 {
        0.0
    } else {
        effective.signum() * (deadzone + (1.0 - deadzone) * effective.abs())
    }
}

fn friction_torque(parameters: ActuatorParameters, speed_rad_per_s: f32) -> f32 {
    let viscous = parameters.viscous_friction_nm_per_rad_s * speed_rad_per_s;
    let coulomb_sign = if speed_rad_per_s > parameters.friction_sign_epsilon_rad_s {
        1.0
    } else if speed_rad_per_s < -parameters.friction_sign_epsilon_rad_s {
        -1.0
    } else {
        0.0
    };
    viscous + parameters.coulomb_friction_nm * coulomb_sign
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> StaticActuatorModel {
        StaticActuatorModel::new(
            ActuatorParameters::new(0.2, 0.1, 0.001, 0.01, 0.5).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn deadzone_produces_zero_motor_effort() {
        let torque = model()
            .torque_from_command(
                NormalizedCommand::new(0.05).unwrap(),
                ActuatorOperatingPoint { speed_rad_per_s: 0.0 },
            )
            .unwrap();
        assert_eq!(torque, TorqueNm(0.0));
    }

    #[test]
    fn inverse_model_recovers_requested_torque_inside_authority() {
        let operating_point = ActuatorOperatingPoint { speed_rad_per_s: 10.0 };
        let command = model()
            .command_for_torque(TorqueNm(0.08), operating_point)
            .unwrap();
        assert!(!command.saturated);
        assert!((command.predicted_torque_nm.0 - 0.08).abs() < 1.0e-5);
    }

    #[test]
    fn excessive_torque_is_explicitly_saturated() {
        let command = model()
            .command_for_torque(
                TorqueNm(1.0),
                ActuatorOperatingPoint { speed_rad_per_s: 0.0 },
            )
            .unwrap();
        assert!(command.saturated);
        assert_eq!(command.command.get(), 1.0);
    }

    #[test]
    fn invalid_parameters_are_rejected() {
        assert!(ActuatorParameters::new(0.0, 0.1, 0.0, 0.0, 0.1).is_none());
        assert!(ActuatorParameters::new(1.0, 1.0, 0.0, 0.0, 0.1).is_none());
    }
}
