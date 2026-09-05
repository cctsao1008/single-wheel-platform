#![no_std]

use swp_plant_model::{
    PlantParameters, REDUCED_BALANCE_STATE_COUNT, REFERENCE_INPUT_COUNT,
    linearize_stationary_upright,
};

pub const UPRIGHT_MEASUREMENT_COUNT: usize = 8;

pub const ACCEL_X: usize = 0;
pub const ACCEL_Y: usize = 1;
pub const ACCEL_Z: usize = 2;
pub const GYRO_X: usize = 3;
pub const GYRO_Y: usize = 4;
pub const GYRO_Z: usize = 5;
pub const DRIVE_ENCODER_RELATIVE_ANGLE: usize = 6;
pub const REACTION_WHEEL_RELATIVE_RATE: usize = 7;

/// IMU position in the canonical robot body frame, measured from the drive-wheel
/// axle / reduced-model body origin.
///
/// Orientation is intentionally absent: calibrated IMU vectors are rotated into
/// the body frame before reaching this model.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuPlacement {
    pub forward_x_m: f32,
    pub left_y_m: f32,
    pub up_z_m: f32,
}

impl ImuPlacement {
    pub fn is_finite(self) -> bool {
        [self.forward_x_m, self.left_y_m, self.up_z_m]
            .iter()
            .all(|value| value.is_finite())
    }
}

/// Affine first-order sensor model around stationary upright.
///
/// State order is
/// `[s, s_dot, theta, theta_dot, phi, phi_dot, psi_r_dot]`.
/// Input order is `[tau_drive, tau_reaction]`.
/// Measurement order is
/// `[a_x, a_y, a_z, gyro_x, gyro_y, gyro_z, delta_drive, psi_r_dot]`.
///
/// The body-frame accelerometer output is specific force, not geometric tilt.
/// `nominal[ACCEL_Z] = g` at stationary upright.  The direct input matrix is
/// intentionally retained because actuator torque changes translational and
/// angular acceleration during the same sample interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UprightMeasurementModel {
    pub nominal: [f32; UPRIGHT_MEASUREMENT_COUNT],
    pub c: [[f32; REDUCED_BALANCE_STATE_COUNT]; UPRIGHT_MEASUREMENT_COUNT],
    pub d: [[f32; REFERENCE_INPUT_COUNT]; UPRIGHT_MEASUREMENT_COUNT],
}

impl UprightMeasurementModel {
    pub fn predict(
        self,
        state: [f32; REDUCED_BALANCE_STATE_COUNT],
        input: [f32; REFERENCE_INPUT_COUNT],
    ) -> [f32; UPRIGHT_MEASUREMENT_COUNT] {
        let mut measurement = self.nominal;

        for (row, output) in measurement.iter_mut().enumerate() {
            for (column, value) in state.iter().enumerate() {
                *output += self.c[row][column] * value;
            }
            for (column, value) in input.iter().enumerate() {
                *output += self.d[row][column] * value;
            }
        }

        measurement
    }
}

/// Build the first-order physical measurement equation
///
/// `y = y_0 + C x + D u`
///
/// about stationary upright.
///
/// The specific-force linearization uses the rigid-body point-acceleration
/// relation at the IMU location.  Around upright and zero rates:
///
/// `a_x = s_ddot - g theta + z_i theta_ddot`
/// `a_y = g phi - z_i phi_ddot`
/// `a_z = g - x_i theta_ddot + y_i phi_ddot`
///
/// Centripetal terms are second order at this operating point and therefore do
/// not appear in the Jacobian.
pub fn linearize_stationary_upright_measurement(
    parameters: PlantParameters,
    imu: ImuPlacement,
) -> Option<UprightMeasurementModel> {
    if !imu.is_finite() {
        return None;
    }

    let plant = linearize_stationary_upright(parameters)?;
    let mut nominal = [0.0; UPRIGHT_MEASUREMENT_COUNT];
    let mut c = [[0.0; REDUCED_BALANCE_STATE_COUNT]; UPRIGHT_MEASUREMENT_COUNT];
    let mut d = [[0.0; REFERENCE_INPUT_COUNT]; UPRIGHT_MEASUREMENT_COUNT];

    let g = parameters.gravity_m_per_s2;
    let r = parameters.drive_wheel_radius_m;
    let x_i = imu.forward_x_m;
    let y_i = imu.left_y_m;
    let z_i = imu.up_z_m;

    nominal[ACCEL_Z] = g;

    // Body-frame specific force at the IMU point.
    for state_index in 0..REDUCED_BALANCE_STATE_COUNT {
        c[ACCEL_X][state_index] = plant.a[1][state_index] + z_i * plant.a[3][state_index];
        c[ACCEL_Y][state_index] = -z_i * plant.a[5][state_index];
        c[ACCEL_Z][state_index] = -x_i * plant.a[3][state_index] + y_i * plant.a[5][state_index];
    }
    c[ACCEL_X][2] -= g;
    c[ACCEL_Y][4] += g;

    for input_index in 0..REFERENCE_INPUT_COUNT {
        d[ACCEL_X][input_index] = plant.b[1][input_index] + z_i * plant.b[3][input_index];
        d[ACCEL_Y][input_index] = -z_i * plant.b[5][input_index];
        d[ACCEL_Z][input_index] = -x_i * plant.b[3][input_index] + y_i * plant.b[5][input_index];
    }

    // Body angular rate for R = R_y(theta) R_x(phi).  At upright the first-order
    // rates are gyro_x = phi_dot, gyro_y = theta_dot, while gyro_z is second order
    // in the reduced yaw-free balance coordinates.
    c[GYRO_X][5] = 1.0;
    c[GYRO_Y][3] = 1.0;

    // Drive encoder measures motor-relative wheel angle:
    // delta_d = s / r - theta.
    c[DRIVE_ENCODER_RELATIVE_ANGLE][0] = 1.0 / r;
    c[DRIVE_ENCODER_RELATIVE_ANGLE][2] = -1.0;

    // The reduced control state omits reaction-wheel phase but retains the
    // relative wheel rate obtained from unwrapped encoder motion.
    c[REACTION_WHEEL_RELATIVE_RATE][6] = 1.0;

    Some(UprightMeasurementModel { nominal, c, d })
}

/// Structural local observability certificates using only encoder/gyro channels.
///
/// These indicators deliberately exclude accelerometer information.  They show
/// that the ideal seven-state stationary-upright plant is already locally
/// observable from the drive relative-angle channel, pitch/roll gyro rates, and
/// reaction-wheel relative rate.  This is a structural statement only: scale,
/// sign, bias, timing, model error, and noise still matter in the real estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UprightObservabilityIndicators {
    pub pitch_minor: f32,
    pub roll_minor: f32,
}

impl UprightObservabilityIndicators {
    pub fn is_structurally_observable(self) -> bool {
        [self.pitch_minor, self.roll_minor]
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
    }
}

pub fn upright_observability_indicators(
    parameters: PlantParameters,
) -> Option<UprightObservabilityIndicators> {
    let p = parameters.upright_aggregates()?;
    let h = p.gravitational_first_moment_kg_m;
    let m_s = p.equivalent_translation_mass_kg;
    let j_phi = p.roll_body_inertia_kg_m2;
    let delta = p.pitch_inertia_determinant_kg2_m2;
    let g = parameters.gravity_m_per_s2;
    let r = parameters.drive_wheel_radius_m;

    // A nonzero 4x4 minor of O_pitch built from
    // y = [s/r - theta, theta_dot] is (H M_s g / Delta) / r^2.
    let pitch_minor = h * m_s * g / (delta * r * r);

    // A nonzero 3x3 minor of O_roll built from
    // y = [phi_dot, psi_r_dot] is H g / J_phi.
    let roll_minor = h * g / j_phi;

    let result = UprightObservabilityIndicators {
        pitch_minor,
        roll_minor,
    };

    result.is_structurally_observable().then_some(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parameters() -> PlantParameters {
        // Test fixture only; these are not reference-platform facts.
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
    fn stationary_upright_specific_force_is_positive_body_z_gravity() {
        let model = linearize_stationary_upright_measurement(parameters(), ImuPlacement::default())
            .unwrap();
        let y = model.predict(
            [0.0; REDUCED_BALANCE_STATE_COUNT],
            [0.0; REFERENCE_INPUT_COUNT],
        );

        assert!(y[ACCEL_X].abs() < 1.0e-6);
        assert!(y[ACCEL_Y].abs() < 1.0e-6);
        assert!((y[ACCEL_Z] - parameters().gravity_m_per_s2).abs() < 1.0e-6);
    }

    #[test]
    fn gyro_and_encoder_rows_follow_physical_coordinate_contract() {
        let model = linearize_stationary_upright_measurement(parameters(), ImuPlacement::default())
            .unwrap();
        let state = [1.0, 2.0, 0.25, 3.0, -0.5, 4.0, 5.0];
        let y = model.predict(state, [0.0, 0.0]);

        assert_eq!(y[GYRO_X], 4.0);
        assert_eq!(y[GYRO_Y], 3.0);
        assert_eq!(y[GYRO_Z], 0.0);
        assert!((y[DRIVE_ENCODER_RELATIVE_ANGLE] - (1.0 / 0.05 - 0.25)).abs() < 1.0e-5);
        assert_eq!(y[REACTION_WHEEL_RELATIVE_RATE], 5.0);
    }

    #[test]
    fn accelerometer_has_direct_actuation_feedthrough() {
        let model = linearize_stationary_upright_measurement(
            parameters(),
            ImuPlacement {
                forward_x_m: 0.0,
                left_y_m: 0.0,
                up_z_m: 0.03,
            },
        )
        .unwrap();
        let y = model.predict([0.0; REDUCED_BALANCE_STATE_COUNT], [0.2, 0.1]);

        assert!(y[ACCEL_X].abs() > 1.0e-6);
        assert!(y[ACCEL_Y].abs() > 1.0e-6);
    }

    #[test]
    fn ideal_upright_model_is_structurally_observable_without_accelerometer() {
        assert!(
            upright_observability_indicators(parameters())
                .unwrap()
                .is_structurally_observable()
        );
    }

    #[test]
    fn nonfinite_imu_placement_is_rejected() {
        assert!(
            linearize_stationary_upright_measurement(
                parameters(),
                ImuPlacement {
                    forward_x_m: f32::NAN,
                    left_y_m: 0.0,
                    up_z_m: 0.0,
                },
            )
            .is_none()
        );
    }
}
