#![no_std]

pub const GENERALIZED_COORDINATE_COUNT: usize = 4;
pub const REDUCED_STATE_COUNT: usize = 7;
pub const INPUT_COUNT: usize = 2;

/// Canonical generalized coordinates for the balance plant.
///
/// `reaction_wheel_angle_rad` is relative to the robot body. Its absolute angle
/// is cyclic for an axisymmetric reaction wheel and is therefore omitted from
/// the reduced control state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneralizedCoordinates {
    pub forward_position_m: f32,
    pub pitch_rad: f32,
    pub roll_rad: f32,
    pub reaction_wheel_angle_rad: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneralizedRates {
    pub forward_velocity_m_per_s: f32,
    pub pitch_rate_rad_per_s: f32,
    pub roll_rate_rad_per_s: f32,
    pub reaction_wheel_rate_rad_per_s: f32,
}

/// Reduced state used by estimation and control.
///
/// The reaction-wheel angle itself is omitted because the current plant model
/// depends on wheel speed and torque, not on absolute wheel phase.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ReducedPlantState {
    pub forward_position_m: f32,
    pub forward_velocity_m_per_s: f32,
    pub pitch_rad: f32,
    pub pitch_rate_rad_per_s: f32,
    pub roll_rad: f32,
    pub roll_rate_rad_per_s: f32,
    pub reaction_wheel_rate_rad_per_s: f32,
}

impl ReducedPlantState {
    pub const fn from_generalized(q: GeneralizedCoordinates, dq: GeneralizedRates) -> Self {
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

    pub const fn as_vector(self) -> [f32; REDUCED_STATE_COUNT] {
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

/// Physical actuator inputs to the plant.
///
/// Positive drive torque increases drive-wheel rotation corresponding to
/// positive forward travel and applies the equal/opposite reaction torque to
/// the body. Positive reaction-wheel torque increases the wheel's relative
/// rotation and applies the equal/opposite roll torque to the body.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlantInput {
    pub drive_torque_nm: f32,
    pub reaction_wheel_torque_nm: f32,
}

impl PlantInput {
    pub const fn as_vector(self) -> [f32; INPUT_COUNT] {
        [self.drive_torque_nm, self.reaction_wheel_torque_nm]
    }
}

/// Generalized force vector corresponding to
/// `[forward_position, pitch, roll, reaction_wheel_angle]`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneralizedForces {
    pub forward_force_n: f32,
    pub pitch_torque_nm: f32,
    pub roll_torque_nm: f32,
    pub reaction_wheel_torque_nm: f32,
}

impl GeneralizedForces {
    pub const fn as_vector(self) -> [f32; GENERALIZED_COORDINATE_COUNT] {
        [
            self.forward_force_n,
            self.pitch_torque_nm,
            self.roll_torque_nm,
            self.reaction_wheel_torque_nm,
        ]
    }
}

/// Physical parameters required by the canonical nonlinear plant model.
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

        positive.iter().all(|value| value.is_finite() && *value > 0.0)
    }
}

/// Map physical motor torques to generalized forces by virtual work.
///
/// The drive-wheel absolute rotation is eliminated using the no-slip relation
/// `alpha = forward_position / drive_wheel_radius`. Motor torque therefore
/// contributes `tau / r` to forward generalized force and the equal/opposite
/// torque to body pitch. Reaction-wheel torque acts internally between the body
/// roll coordinate and the wheel's relative spin coordinate.
pub fn generalized_forces(
    input: PlantInput,
    parameters: PlantParameters,
) -> Option<GeneralizedForces> {
    if !parameters.is_physically_valid() {
        return None;
    }

    Some(GeneralizedForces {
        forward_force_n: input.drive_torque_nm / parameters.drive_wheel_radius_m,
        pitch_torque_nm: -input.drive_torque_nm,
        roll_torque_nm: -input.reaction_wheel_torque_nm,
        reaction_wheel_torque_nm: input.reaction_wheel_torque_nm,
    })
}

/// Continuous linearization about a specified operating point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContinuousLinearPlant {
    pub a: [[f32; REDUCED_STATE_COUNT]; REDUCED_STATE_COUNT],
    pub b: [[f32; INPUT_COUNT]; REDUCED_STATE_COUNT],
}

/// Zero-order-hold discrete plant used by the real-time estimator/controller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiscreteLinearPlant {
    pub sample_period_s: f32,
    pub a_d: [[f32; REDUCED_STATE_COUNT]; REDUCED_STATE_COUNT],
    pub b_d: [[f32; INPUT_COUNT]; REDUCED_STATE_COUNT],
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
        assert!(generalized_forces(PlantInput::default(), p).is_none());
    }

    #[test]
    fn drive_torque_maps_to_translation_and_opposite_body_pitch_torque() {
        let input = PlantInput {
            drive_torque_nm: 0.5,
            reaction_wheel_torque_nm: 0.0,
        };
        let forces = generalized_forces(input, parameters()).unwrap();
        assert!((forces.forward_force_n - 10.0).abs() < 1.0e-6);
        assert!((forces.pitch_torque_nm + 0.5).abs() < 1.0e-6);
    }

    #[test]
    fn reaction_wheel_torque_is_internal_action_reaction_pair() {
        let input = PlantInput {
            drive_torque_nm: 0.0,
            reaction_wheel_torque_nm: 0.25,
        };
        let forces = generalized_forces(input, parameters()).unwrap();
        assert!((forces.roll_torque_nm + forces.reaction_wheel_torque_nm).abs() < 1.0e-6);
    }

    #[test]
    fn reduced_state_discards_only_cyclic_reaction_wheel_angle() {
        let q = GeneralizedCoordinates {
            forward_position_m: 1.0,
            pitch_rad: 2.0,
            roll_rad: 3.0,
            reaction_wheel_angle_rad: 4.0,
        };
        let dq = GeneralizedRates {
            forward_velocity_m_per_s: 5.0,
            pitch_rate_rad_per_s: 6.0,
            roll_rate_rad_per_s: 7.0,
            reaction_wheel_rate_rad_per_s: 8.0,
        };
        assert_eq!(
            ReducedPlantState::from_generalized(q, dq).as_vector(),
            [1.0, 5.0, 2.0, 6.0, 3.0, 7.0, 8.0]
        );
    }
}
