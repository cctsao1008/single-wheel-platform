#![no_std]

pub const FULL_CONFIGURATION_COUNT: usize = 7;
pub const BALANCE_COORDINATE_COUNT: usize = 4;
pub const REDUCED_BALANCE_STATE_COUNT: usize = 7;
pub const REFERENCE_INPUT_COUNT: usize = 2;

/// Full robot configuration before balance-oriented model reduction.
///
/// This is a kinematic configuration contract, not yet a claim that all seven
/// coordinates belong in every controller state. The single-wheel contact is
/// nonholonomic, so the admissible rates are constrained by the rolling/contact
/// model rather than being seven independent generalized velocities.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FullConfiguration {
    pub world_x_m: f32,
    pub world_y_m: f32,
    pub yaw_rad: f32,
    pub pitch_rad: f32,
    pub roll_rad: f32,
    pub drive_wheel_relative_angle_rad: f32,
    pub reaction_wheel_relative_angle_rad: f32,
}

impl FullConfiguration {
    pub const fn as_vector(self) -> [f32; FULL_CONFIGURATION_COUNT] {
        [
            self.world_x_m,
            self.world_y_m,
            self.yaw_rad,
            self.pitch_rad,
            self.roll_rad,
            self.drive_wheel_relative_angle_rad,
            self.reaction_wheel_relative_angle_rad,
        ]
    }
}

/// Local generalized coordinates used for upright / straight-line balance.
///
/// `reaction_wheel_angle_rad` is the reaction-wheel angle relative to the robot
/// body. The drive-wheel coordinate is eliminated through the local pure-rolling
/// relation; `forward_position_m` is the path coordinate used by the reduced
/// balance model.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BalanceCoordinates {
    pub forward_position_m: f32,
    pub pitch_rad: f32,
    pub roll_rad: f32,
    pub reaction_wheel_angle_rad: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BalanceRates {
    pub forward_velocity_m_per_s: f32,
    pub pitch_rate_rad_per_s: f32,
    pub roll_rate_rad_per_s: f32,
    pub reaction_wheel_rate_rad_per_s: f32,
}

/// Reduced state used by the current balance estimator/controller design.
///
/// Reaction-wheel phase is cyclic for the axisymmetric-wheel model, so the
/// reduced state retains relative wheel speed rather than wheel phase.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ReducedBalanceState {
    pub forward_position_m: f32,
    pub forward_velocity_m_per_s: f32,
    pub pitch_rad: f32,
    pub pitch_rate_rad_per_s: f32,
    pub roll_rad: f32,
    pub roll_rate_rad_per_s: f32,
    pub reaction_wheel_rate_rad_per_s: f32,
}

impl ReducedBalanceState {
    pub const fn from_balance(q: BalanceCoordinates, dq: BalanceRates) -> Self {
        Self {
            forward_position_m: q.forward_position_m,
            forward_velocity_m_per_s: dq.forward_velocity_m_per_s,
            pitch_rad: q.pitch_rad,
            pitch_rate_rad_per_s: dq.pitch_rate_rad_per_s,
            roll_rad: q.roll_rad,
            roll_rate_rad_per_s: dq.roll_rate_rad_per_s,
            reaction_wheel_rate_rad_per_s: dq.reaction_wheel_rate_rad_per_s,
        }
    }

    pub const fn as_vector(self) -> [f32; REDUCED_BALANCE_STATE_COUNT] {
        [
            self.forward_position_m,
            self.forward_velocity_m_per_s,
            self.pitch_rad,
            self.pitch_rate_rad_per_s,
            self.roll_rad,
            self.roll_rate_rad_per_s,
            self.reaction_wheel_rate_rad_per_s,
        ]
    }
}

/// Physical actuator inputs for the populated reference assembly.
///
/// Inputs are motor torques, never PWM values. Positive drive torque increases
/// drive-wheel rotation corresponding to positive forward travel. Positive
/// reaction-wheel torque increases the reaction wheel's rotation relative to
/// the robot body.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ReferencePlantInput {
    pub drive_torque_nm: f32,
    pub reaction_wheel_torque_nm: f32,
}

impl ReferencePlantInput {
    pub const fn as_vector(self) -> [f32; REFERENCE_INPUT_COUNT] {
        [self.drive_torque_nm, self.reaction_wheel_torque_nm]
    }
}

/// Generalized force vector corresponding to the reduced balance coordinates
/// `[forward_position, pitch, roll, reaction_wheel_relative_angle]`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BalanceGeneralizedForces {
    pub forward_force_n: f32,
    pub pitch_torque_nm: f32,
    pub roll_torque_nm: f32,
    pub reaction_wheel_relative_torque_nm: f32,
}

impl BalanceGeneralizedForces {
    pub const fn as_vector(self) -> [f32; BALANCE_COORDINATE_COUNT] {
        [
            self.forward_force_n,
            self.pitch_torque_nm,
            self.roll_torque_nm,
            self.reaction_wheel_relative_torque_nm,
        ]
    }
}

/// Physical parameters required by the canonical plant model.
///
/// This type contains no defaults. Unknown parameters must be measured or
/// identified before constructing a numeric model; they must not be replaced by
/// convenient textbook values.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlantParameters {
    pub gravity_m_per_s2: f32,

    pub body_mass_kg: f32,
    pub body_com_height_m: f32,
    pub body_inertia_roll_kg_m2: f32,
    pub body_inertia_pitch_kg_m2: f32,
    pub body_inertia_yaw_kg_m2: f32,

    pub drive_wheel_mass_kg: f32,
    pub drive_wheel_radius_m: f32,
    pub drive_wheel_spin_inertia_kg_m2: f32,

    pub reaction_wheel_mass_kg: f32,
    pub reaction_wheel_com_height_m: f32,
    pub reaction_wheel_spin_inertia_kg_m2: f32,
    pub reaction_wheel_transverse_inertia_kg_m2: f32,
}

impl PlantParameters {
    pub fn is_physically_valid(self) -> bool {
        let positive = [
            self.gravity_m_per_s2,
            self.body_mass_kg,
            self.body_com_height_m,
            self.body_inertia_roll_kg_m2,
            self.body_inertia_pitch_kg_m2,
            self.body_inertia_yaw_kg_m2,
            self.drive_wheel_mass_kg,
            self.drive_wheel_radius_m,
            self.drive_wheel_spin_inertia_kg_m2,
            self.reaction_wheel_mass_kg,
            self.reaction_wheel_com_height_m,
            self.reaction_wheel_spin_inertia_kg_m2,
            self.reaction_wheel_transverse_inertia_kg_m2,
        ];

        positive
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
    }
}

/// Map reference-assembly motor torques into the reduced balance coordinates.
///
/// For the drive motor, the wheel's absolute rolling angle is eliminated using
/// the local no-slip relation. Motor relative rotation is therefore
/// `forward_position / radius - pitch`, which gives `tau/r` in translation and
/// `-tau` in body pitch by virtual work.
///
/// The reaction-wheel coordinate is already the wheel angle *relative to the
/// body*. Its motor torque therefore enters only that relative coordinate. The
/// equal/opposite body reaction is generated by the coupled inertia terms of the
/// body + wheel kinetic energy; adding a second explicit `-tau` roll force here
/// would double-count the internal action/reaction pair.
pub fn balance_generalized_forces(
    input: ReferencePlantInput,
    parameters: PlantParameters,
) -> Option<BalanceGeneralizedForces> {
    if !parameters.is_physically_valid() {
        return None;
    }

    Some(BalanceGeneralizedForces {
        forward_force_n: input.drive_torque_nm / parameters.drive_wheel_radius_m,
        pitch_torque_nm: -input.drive_torque_nm,
        roll_torque_nm: 0.0,
        reaction_wheel_relative_torque_nm: input.reaction_wheel_torque_nm,
    })
}

/// Continuous linearization about a specified operating point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContinuousLinearPlant {
    pub a: [[f32; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT],
    pub b: [[f32; REFERENCE_INPUT_COUNT]; REDUCED_BALANCE_STATE_COUNT],
}

/// Zero-order-hold discrete plant used by the real-time estimator/controller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiscreteLinearPlant {
    pub sample_period_s: f32,
    pub a_d: [[f32; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT],
    pub b_d: [[f32; REFERENCE_INPUT_COUNT]; REDUCED_BALANCE_STATE_COUNT],
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parameters() -> PlantParameters {
        PlantParameters {
            gravity_m_per_s2: 9.80665,
            body_mass_kg: 1.0,
            body_com_height_m: 0.1,
            body_inertia_roll_kg_m2: 0.01,
            body_inertia_pitch_kg_m2: 0.01,
            body_inertia_yaw_kg_m2: 0.01,
            drive_wheel_mass_kg: 0.1,
            drive_wheel_radius_m: 0.05,
            drive_wheel_spin_inertia_kg_m2: 0.001,
            reaction_wheel_mass_kg: 0.1,
            reaction_wheel_com_height_m: 0.1,
            reaction_wheel_spin_inertia_kg_m2: 0.001,
            reaction_wheel_transverse_inertia_kg_m2: 0.0005,
        }
    }

    #[test]
    fn invalid_parameters_are_not_promoted_into_a_numeric_model() {
        let mut p = parameters();
        p.drive_wheel_radius_m = 0.0;
        assert!(!p.is_physically_valid());
        assert!(balance_generalized_forces(ReferencePlantInput::default(), p).is_none());
    }

    #[test]
    fn drive_torque_maps_to_translation_and_opposite_body_pitch_torque() {
        let input = ReferencePlantInput {
            drive_torque_nm: 0.5,
            reaction_wheel_torque_nm: 0.0,
        };
        let forces = balance_generalized_forces(input, parameters()).unwrap();
        assert!((forces.forward_force_n - 10.0).abs() < 1.0e-6);
        assert!((forces.pitch_torque_nm + 0.5).abs() < 1.0e-6);
    }

    #[test]
    fn reaction_wheel_torque_enters_only_the_relative_wheel_coordinate() {
        let input = ReferencePlantInput {
            drive_torque_nm: 0.0,
            reaction_wheel_torque_nm: 0.25,
        };
        let forces = balance_generalized_forces(input, parameters()).unwrap();
        assert_eq!(forces.roll_torque_nm, 0.0);
        assert!((forces.reaction_wheel_relative_torque_nm - 0.25).abs() < 1.0e-6);
    }

    #[test]
    fn reduced_state_discards_only_cyclic_reaction_wheel_angle() {
        let q = BalanceCoordinates {
            forward_position_m: 1.0,
            pitch_rad: 2.0,
            roll_rad: 3.0,
            reaction_wheel_angle_rad: 4.0,
        };
        let dq = BalanceRates {
            forward_velocity_m_per_s: 5.0,
            pitch_rate_rad_per_s: 6.0,
            roll_rate_rad_per_s: 7.0,
            reaction_wheel_rate_rad_per_s: 8.0,
        };
        assert_eq!(
            ReducedBalanceState::from_balance(q, dq).as_vector(),
            [1.0, 5.0, 2.0, 6.0, 3.0, 7.0, 8.0]
        );
    }

    #[test]
    fn full_configuration_keeps_yaw_and_planar_pose_available_for_mobility_modeling() {
        let q = FullConfiguration {
            world_x_m: 1.0,
            world_y_m: 2.0,
            yaw_rad: 3.0,
            pitch_rad: 4.0,
            roll_rad: 5.0,
            drive_wheel_relative_angle_rad: 6.0,
            reaction_wheel_relative_angle_rad: 7.0,
        };
        assert_eq!(q.as_vector(), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);
    }
}
