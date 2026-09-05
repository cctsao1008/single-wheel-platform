#![no_std]

use swp_board_one_v2::{EncoderChannel, MotorChannel};
use swp_platform_domain::Actuator;

/// Physical population state of one PCB motor interface in the inspected unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorPopulation {
    Installed(Actuator),
    NotInstalled,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorInstallation {
    pub channel: MotorChannel,
    pub population: MotorPopulation,
}

/// Encoder-channel association established by the corresponding installed motor
/// harness. This does not yet define encoder sign, pulses/revolution, or wheel
/// angular-speed scaling.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EncoderAssociation {
    pub channel: EncoderChannel,
    pub actuator: Actuator,
}

/// Verified assembly topology for the inspected platform.
///
/// Schematic BLDC_1 is the PCB connector silked M2 and is physically traced to
/// the upper reaction-wheel motor. Schematic BLDC_2 is the PCB connector silked
/// M1 and is physically traced to the lower ground-drive motor. BLDC_3 / M3 is
/// present on the PCB but has no motor connected in this unit.
pub const MOTOR_INSTALLATIONS: [MotorInstallation; 3] = [
    MotorInstallation {
        channel: MotorChannel::Bldc1,
        population: MotorPopulation::Installed(Actuator::ReactionWheel),
    },
    MotorInstallation {
        channel: MotorChannel::Bldc2,
        population: MotorPopulation::Installed(Actuator::DriveWheel),
    },
    MotorInstallation {
        channel: MotorChannel::Bldc3,
        population: MotorPopulation::NotInstalled,
    },
];

pub const ENCODER_ASSOCIATIONS: [EncoderAssociation; 2] = [
    EncoderAssociation {
        channel: EncoderChannel::Encoder1,
        actuator: Actuator::ReactionWheel,
    },
    EncoderAssociation {
        channel: EncoderChannel::Encoder2,
        actuator: Actuator::DriveWheel,
    },
];

pub const INSTALLED_ACTUATOR_COUNT: usize = 2;

pub const fn actuator_for_motor(channel: MotorChannel) -> Option<Actuator> {
    match channel {
        MotorChannel::Bldc1 => Some(Actuator::ReactionWheel),
        MotorChannel::Bldc2 => Some(Actuator::DriveWheel),
        MotorChannel::Bldc3 => None,
    }
}

pub const fn actuator_for_encoder(channel: EncoderChannel) -> Option<Actuator> {
    match channel {
        EncoderChannel::Encoder1 => Some(Actuator::ReactionWheel),
        EncoderChannel::Encoder2 => Some(Actuator::DriveWheel),
        EncoderChannel::Encoder3 => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_unit_has_exactly_two_installed_actuators() {
        let installed = MOTOR_INSTALLATIONS
            .iter()
            .filter(|entry| matches!(entry.population, MotorPopulation::Installed(_)))
            .count();
        assert_eq!(installed, INSTALLED_ACTUATOR_COUNT);
    }

    #[test]
    fn verified_channel_roles_do_not_promote_unpopulated_bldc3() {
        assert_eq!(
            actuator_for_motor(MotorChannel::Bldc1),
            Some(Actuator::ReactionWheel)
        );
        assert_eq!(
            actuator_for_motor(MotorChannel::Bldc2),
            Some(Actuator::DriveWheel)
        );
        assert_eq!(actuator_for_motor(MotorChannel::Bldc3), None);
    }
}
