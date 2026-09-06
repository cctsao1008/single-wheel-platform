#![no_std]

use swp_dsp_kernel::dot_f32;
use swp_measurement_model::{
    ACCEL_X, ACCEL_Y, ACCEL_Z, UPRIGHT_MEASUREMENT_COUNT, UprightMeasurementModel,
};
use swp_plant_model::{
    DiscreteLinearPlant, REDUCED_BALANCE_STATE_COUNT, REFERENCE_INPUT_COUNT, ReducedBalanceState,
    ReferencePlantInput,
};
use swp_robot_domain::StateValidity;
use swp_state_estimator::{
    BalanceStateEstimator, EstimateError, EstimatedBalanceState, EstimatorMeasurement,
    MeasurementMask,
};

const MIN_INNOVATION_VARIANCE: f32 = 1.0e-12;
const MIN_COVARIANCE_DIAGONAL: f32 = 1.0e-12;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EkfNoise {
    pub process_variance: [f32; REDUCED_BALANCE_STATE_COUNT],
    pub measurement_variance: [f32; UPRIGHT_MEASUREMENT_COUNT],
}

impl EkfNoise {
    pub fn new(
        process_variance: [f32; REDUCED_BALANCE_STATE_COUNT],
        measurement_variance: [f32; UPRIGHT_MEASUREMENT_COUNT],
    ) -> Option<Self> {
        let process_valid = process_variance
            .iter()
            .all(|value| value.is_finite() && *value >= 0.0);
        let measurement_valid = measurement_variance
            .iter()
            .all(|value| value.is_finite() && *value > 0.0);
        (process_valid && measurement_valid).then_some(Self {
            process_variance,
            measurement_variance,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EkfDesign {
    pub plant: DiscreteLinearPlant,
    pub measurement: UprightMeasurementModel,
    pub gravity_m_per_s2: f32,
    pub noise: EkfNoise,
    pub used_measurements: MeasurementMask,
    pub required_measurements: MeasurementMask,
}

impl EkfDesign {
    pub fn new(
        plant: DiscreteLinearPlant,
        measurement: UprightMeasurementModel,
        gravity_m_per_s2: f32,
        noise: EkfNoise,
        used_measurements: MeasurementMask,
        required_measurements: MeasurementMask,
    ) -> Option<Self> {
        let valid_mask = (1_u16 << UPRIGHT_MEASUREMENT_COUNT) - 1;
        let masks_valid = used_measurements.bits() != 0
            && used_measurements.bits() & !valid_mask == 0
            && required_measurements.bits() & !valid_mask == 0
            && required_measurements.bits() & !used_measurements.bits() == 0;
        let plant_valid = plant.sample_period_s.is_finite()
            && plant.sample_period_s > 0.0
            && plant
                .a_d
                .iter()
                .flatten()
                .chain(plant.b_d.iter().flatten())
                .all(|value| value.is_finite());
        let measurement_valid = measurement
            .nominal
            .iter()
            .chain(measurement.c.iter().flatten())
            .chain(measurement.d.iter().flatten())
            .all(|value| value.is_finite());
        let gravity_valid = gravity_m_per_s2.is_finite() && gravity_m_per_s2 > 0.0;

        (masks_valid && plant_valid && measurement_valid && gravity_valid).then_some(Self {
            plant,
            measurement,
            gravity_m_per_s2,
            noise,
            used_measurements,
            required_measurements,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EkfEstimate {
    pub estimate: EstimatedBalanceState,
    pub covariance: [[f32; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT],
    pub corrected_measurements: MeasurementMask,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExtendedKalmanFilter {
    design: EkfDesign,
    state: [f32; REDUCED_BALANCE_STATE_COUNT],
    covariance: [[f32; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT],
    reset_variance: [f32; REDUCED_BALANCE_STATE_COUNT],
    validity: StateValidity,
}

impl ExtendedKalmanFilter {
    pub fn new(
        design: EkfDesign,
        initial_state: ReducedBalanceState,
        initial_variance: [f32; REDUCED_BALANCE_STATE_COUNT],
    ) -> Option<Self> {
        let state = initial_state.as_vector();
        if !state.iter().all(|value| value.is_finite())
            || !initial_variance
                .iter()
                .all(|value| value.is_finite() && *value > 0.0)
        {
            return None;
        }

        let mut covariance = [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        for index in 0..REDUCED_BALANCE_STATE_COUNT {
            covariance[index][index] = initial_variance[index];
        }

        Some(Self {
            design,
            state,
            covariance,
            reset_variance: initial_variance,
            validity: StateValidity::Invalid,
        })
    }

    pub const fn design(&self) -> EkfDesign {
        self.design
    }

    pub const fn covariance(
        &self,
    ) -> [[f32; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT] {
        self.covariance
    }

    pub fn estimate(&self) -> EstimatedBalanceState {
        EstimatedBalanceState {
            state: state_from_vector(self.state),
            validity: self.validity,
        }
    }

    pub fn reset(
        &mut self,
        state: ReducedBalanceState,
        variance: [f32; REDUCED_BALANCE_STATE_COUNT],
    ) -> bool {
        let vector = state.as_vector();
        if !vector.iter().all(|value| value.is_finite())
            || !variance
                .iter()
                .all(|value| value.is_finite() && *value > 0.0)
        {
            self.validity = StateValidity::Invalid;
            return false;
        }

        self.state = vector;
        self.reset_variance = variance;
        self.covariance = [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        for index in 0..REDUCED_BALANCE_STATE_COUNT {
            self.covariance[index][index] = variance[index];
        }
        self.validity = StateValidity::Invalid;
        true
    }

    /// Execute one 500 Hz-class EKF opportunity.
    ///
    /// The prediction uses the exact-discrete local plant supplied by host
    /// synthesis. The measurement function keeps the locally derived rigid-body
    /// acceleration/feedthrough terms and restores the exact trigonometric gravity
    /// projection, making the correction genuinely nonlinear away from upright.
    /// Measurement channels are applied sequentially as scalar EKF updates. This
    /// avoids an 8x8 matrix inversion and naturally supports availability masks.
    pub fn step(
        &mut self,
        previous_input: ReferencePlantInput,
        measurement_input: ReferencePlantInput,
        measurement: EstimatorMeasurement,
    ) -> Result<EkfEstimate, EstimateError> {
        self.validate_inputs(previous_input, measurement_input, measurement)?;

        let previous_input = previous_input.as_vector();
        let measurement_input = measurement_input.as_vector();
        self.predict(previous_input);

        let mut corrected = MeasurementMask::NONE;
        for channel in 0..UPRIGHT_MEASUREMENT_COUNT {
            if !self.design.used_measurements.contains(channel)
                || !measurement.available.contains(channel)
            {
                continue;
            }
            self.correct_scalar(channel, measurement.values[channel], measurement_input)?;
            corrected = corrected.with(channel);
        }

        if !self.state.iter().all(|value| value.is_finite()) || !covariance_valid(self.covariance) {
            self.validity = StateValidity::Invalid;
            return Err(EstimateError::NumericalFault);
        }

        self.validity = StateValidity::Valid;
        Ok(EkfEstimate {
            estimate: self.estimate(),
            covariance: self.covariance,
            corrected_measurements: corrected,
        })
    }

    fn validate_inputs(
        &mut self,
        previous_input: ReferencePlantInput,
        measurement_input: ReferencePlantInput,
        measurement: EstimatorMeasurement,
    ) -> Result<(), EstimateError> {
        if !measurement.timing_valid {
            self.validity = StateValidity::Invalid;
            return Err(EstimateError::TimingInvalid);
        }
        if !previous_input
            .as_vector()
            .iter()
            .chain(measurement_input.as_vector().iter())
            .all(|value| value.is_finite())
        {
            self.validity = StateValidity::Invalid;
            return Err(EstimateError::NonFiniteInput);
        }

        for channel in 0..UPRIGHT_MEASUREMENT_COUNT {
            if self.design.required_measurements.contains(channel)
                && !measurement.available.contains(channel)
            {
                self.validity = StateValidity::Invalid;
                return Err(EstimateError::MissingRequiredMeasurement(channel));
            }
            if measurement.available.contains(channel) && !measurement.values[channel].is_finite() {
                self.validity = StateValidity::Invalid;
                return Err(EstimateError::NonFiniteMeasurement(channel));
            }
        }
        Ok(())
    }

    fn predict(&mut self, input: [f32; REFERENCE_INPUT_COUNT]) {
        let mut predicted_state = [0.0; REDUCED_BALANCE_STATE_COUNT];
        for (row, output) in predicted_state.iter_mut().enumerate() {
            *output = dot_f32(&self.design.plant.a_d[row], &self.state)
                + dot_f32(&self.design.plant.b_d[row], &input);
        }
        self.state = predicted_state;

        let a = self.design.plant.a_d;
        let mut a_p = [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        for row in 0..REDUCED_BALANCE_STATE_COUNT {
            for col in 0..REDUCED_BALANCE_STATE_COUNT {
                let mut p_col = [0.0; REDUCED_BALANCE_STATE_COUNT];
                for k in 0..REDUCED_BALANCE_STATE_COUNT {
                    p_col[k] = self.covariance[k][col];
                }
                a_p[row][col] = dot_f32(&a[row], &p_col);
            }
        }

        let mut predicted_covariance =
            [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        for row in 0..REDUCED_BALANCE_STATE_COUNT {
            for col in 0..REDUCED_BALANCE_STATE_COUNT {
                predicted_covariance[row][col] = dot_f32(&a_p[row], &a[col]);
            }
            predicted_covariance[row][row] += self.design.noise.process_variance[row];
        }
        self.covariance = symmetrize(predicted_covariance);
    }

    fn correct_scalar(
        &mut self,
        channel: usize,
        measured_value: f32,
        input: [f32; REFERENCE_INPUT_COUNT],
    ) -> Result<(), EstimateError> {
        let predicted_measurement = nonlinear_measurement(
            self.design.measurement,
            self.design.gravity_m_per_s2,
            self.state,
            input,
        );
        let jacobian = nonlinear_measurement_jacobian(
            self.design.measurement,
            self.design.gravity_m_per_s2,
            self.state,
        );
        let h = jacobian[channel];

        let mut p_h = [0.0; REDUCED_BALANCE_STATE_COUNT];
        for row in 0..REDUCED_BALANCE_STATE_COUNT {
            p_h[row] = dot_f32(&self.covariance[row], &h);
        }
        let innovation_variance =
            dot_f32(&h, &p_h) + self.design.noise.measurement_variance[channel];
        if !innovation_variance.is_finite() || innovation_variance <= MIN_INNOVATION_VARIANCE {
            self.validity = StateValidity::Invalid;
            return Err(EstimateError::NumericalFault);
        }

        let innovation = measured_value - predicted_measurement[channel];
        let mut gain = [0.0; REDUCED_BALANCE_STATE_COUNT];
        for index in 0..REDUCED_BALANCE_STATE_COUNT {
            gain[index] = p_h[index] / innovation_variance;
            self.state[index] += gain[index] * innovation;
        }

        let mut i_minus_kh = [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        for row in 0..REDUCED_BALANCE_STATE_COUNT {
            for col in 0..REDUCED_BALANCE_STATE_COUNT {
                i_minus_kh[row][col] = if row == col { 1.0 } else { 0.0 } - gain[row] * h[col];
            }
        }

        let mut left = [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        for row in 0..REDUCED_BALANCE_STATE_COUNT {
            for col in 0..REDUCED_BALANCE_STATE_COUNT {
                let mut p_col = [0.0; REDUCED_BALANCE_STATE_COUNT];
                for k in 0..REDUCED_BALANCE_STATE_COUNT {
                    p_col[k] = self.covariance[k][col];
                }
                left[row][col] = dot_f32(&i_minus_kh[row], &p_col);
            }
        }

        let mut joseph = [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        let r = self.design.noise.measurement_variance[channel];
        for row in 0..REDUCED_BALANCE_STATE_COUNT {
            for col in 0..REDUCED_BALANCE_STATE_COUNT {
                joseph[row][col] =
                    dot_f32(&left[row], &i_minus_kh[col]) + gain[row] * r * gain[col];
            }
        }
        self.covariance = symmetrize(joseph);
        Ok(())
    }
}

/// Nonlinear measurement used by the production EKF.
///
/// The upright model already contains the first-order gravity terms
/// `a_x=-g*theta`, `a_y=g*phi`, `a_z=g`. These corrections replace that local
/// gravity projection with the exact trigonometric projection while preserving
/// the locally derived translational/angular-acceleration and direct-input terms.
pub fn nonlinear_measurement(
    model: UprightMeasurementModel,
    gravity_m_per_s2: f32,
    state: [f32; REDUCED_BALANCE_STATE_COUNT],
    input: [f32; REFERENCE_INPUT_COUNT],
) -> [f32; UPRIGHT_MEASUREMENT_COUNT] {
    let mut output = model.predict(state, input);
    let theta = state[2];
    let phi = state[4];
    let g = gravity_m_per_s2;

    output[ACCEL_X] += g * (theta - libm::sinf(theta));
    output[ACCEL_Y] += g * (libm::sinf(phi) * libm::cosf(theta) - phi);
    output[ACCEL_Z] += g * (libm::cosf(phi) * libm::cosf(theta) - 1.0);
    output
}

pub fn nonlinear_measurement_jacobian(
    model: UprightMeasurementModel,
    gravity_m_per_s2: f32,
    state: [f32; REDUCED_BALANCE_STATE_COUNT],
) -> [[f32; REDUCED_BALANCE_STATE_COUNT]; UPRIGHT_MEASUREMENT_COUNT] {
    let mut h = model.c;
    let theta = state[2];
    let phi = state[4];
    let g = gravity_m_per_s2;

    h[ACCEL_X][2] += g * (1.0 - libm::cosf(theta));
    h[ACCEL_Y][2] += -g * libm::sinf(phi) * libm::sinf(theta);
    h[ACCEL_Y][4] += g * (libm::cosf(phi) * libm::cosf(theta) - 1.0);
    h[ACCEL_Z][2] += -g * libm::cosf(phi) * libm::sinf(theta);
    h[ACCEL_Z][4] += -g * libm::sinf(phi) * libm::cosf(theta);
    h
}

impl BalanceStateEstimator for ExtendedKalmanFilter {
    fn reset(&mut self, state: ReducedBalanceState) -> bool {
        let variance = self.reset_variance;
        ExtendedKalmanFilter::reset(self, state, variance)
    }

    fn estimate(&self) -> EstimatedBalanceState {
        ExtendedKalmanFilter::estimate(self)
    }

    fn step(
        &mut self,
        previous_input: ReferencePlantInput,
        measurement_input: ReferencePlantInput,
        measurement: EstimatorMeasurement,
    ) -> Result<EstimatedBalanceState, EstimateError> {
        ExtendedKalmanFilter::step(self, previous_input, measurement_input, measurement)
            .map(|result| result.estimate)
    }
}

fn symmetrize(
    mut matrix: [[f32; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT],
) -> [[f32; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT] {
    for row in 0..REDUCED_BALANCE_STATE_COUNT {
        for col in (row + 1)..REDUCED_BALANCE_STATE_COUNT {
            let average = 0.5 * (matrix[row][col] + matrix[col][row]);
            matrix[row][col] = average;
            matrix[col][row] = average;
        }
        if matrix[row][row] < MIN_COVARIANCE_DIAGONAL {
            matrix[row][row] = MIN_COVARIANCE_DIAGONAL;
        }
    }
    matrix
}

fn covariance_valid(
    covariance: [[f32; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT],
) -> bool {
    covariance.iter().flatten().all(|value| value.is_finite())
        && (0..REDUCED_BALANCE_STATE_COUNT)
            .all(|index| covariance[index][index] >= MIN_COVARIANCE_DIAGONAL)
}

fn state_from_vector(value: [f32; REDUCED_BALANCE_STATE_COUNT]) -> ReducedBalanceState {
    ReducedBalanceState {
        forward_position_m: value[0],
        forward_velocity_m_per_s: value[1],
        pitch_rad: value[2],
        pitch_rate_rad_per_s: value[3],
        roll_rad: value[4],
        roll_rate_rad_per_s: value[5],
        reaction_wheel_rate_rad_per_s: value[6],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use swp_measurement_model::{
        DRIVE_ENCODER_RELATIVE_ANGLE, GYRO_X, GYRO_Y, REACTION_WHEEL_RELATIVE_RATE,
    };

    fn design() -> EkfDesign {
        let mut a_d = [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        for (index, row) in a_d.iter_mut().enumerate() {
            row[index] = 1.0;
        }
        let plant = DiscreteLinearPlant {
            sample_period_s: 0.002,
            a_d,
            b_d: [[0.0; REFERENCE_INPUT_COUNT]; REDUCED_BALANCE_STATE_COUNT],
        };

        let mut c = [[0.0; REDUCED_BALANCE_STATE_COUNT]; UPRIGHT_MEASUREMENT_COUNT];
        c[GYRO_X][5] = 1.0;
        c[GYRO_Y][3] = 1.0;
        c[DRIVE_ENCODER_RELATIVE_ANGLE][0] = 20.0;
        c[DRIVE_ENCODER_RELATIVE_ANGLE][2] = -1.0;
        c[REACTION_WHEEL_RELATIVE_RATE][6] = 1.0;
        let measurement = UprightMeasurementModel {
            nominal: [0.0, 0.0, 9.80665, 0.0, 0.0, 0.0, 0.0, 0.0],
            c,
            d: [[0.0; REFERENCE_INPUT_COUNT]; UPRIGHT_MEASUREMENT_COUNT],
        };
        let all = MeasurementMask::from_bits((1_u16 << UPRIGHT_MEASUREMENT_COUNT) - 1);
        EkfDesign::new(
            plant,
            measurement,
            9.80665,
            EkfNoise::new(
                [1.0e-5; REDUCED_BALANCE_STATE_COUNT],
                [1.0e-3; UPRIGHT_MEASUREMENT_COUNT],
            )
            .unwrap(),
            all,
            all,
        )
        .unwrap()
    }

    #[test]
    fn nonlinear_gravity_projection_matches_upright_and_curves_away_from_it() {
        let model = design().measurement;
        let upright = nonlinear_measurement(
            model,
            9.80665,
            [0.0; REDUCED_BALANCE_STATE_COUNT],
            [0.0; REFERENCE_INPUT_COUNT],
        );
        assert!((upright[ACCEL_Z] - 9.80665).abs() < 1.0e-6);

        let mut state = [0.0; REDUCED_BALANCE_STATE_COUNT];
        state[2] = 0.5;
        state[4] = -0.3;
        let nonlinear = nonlinear_measurement(model, 9.80665, state, [0.0, 0.0]);
        let linear = model.predict(state, [0.0, 0.0]);
        assert!((nonlinear[ACCEL_X] - linear[ACCEL_X]).abs() > 1.0e-3);
        assert!(nonlinear[ACCEL_Z] < 9.80665);
    }

    #[test]
    fn complete_measurement_produces_valid_estimate_and_finite_covariance() {
        let mut ekf = ExtendedKalmanFilter::new(
            design(),
            ReducedBalanceState::default(),
            [0.1; REDUCED_BALANCE_STATE_COUNT],
        )
        .unwrap();
        let values = design().measurement.predict(
            [0.0; REDUCED_BALANCE_STATE_COUNT],
            [0.0; REFERENCE_INPUT_COUNT],
        );
        let all = MeasurementMask::from_bits((1_u16 << UPRIGHT_MEASUREMENT_COUNT) - 1);
        let result = ekf
            .step(
                ReferencePlantInput::default(),
                ReferencePlantInput::default(),
                EstimatorMeasurement::new(values, all, true),
            )
            .unwrap();
        assert_eq!(result.estimate.validity, StateValidity::Valid);
        assert!(covariance_valid(result.covariance));
        assert_eq!(result.corrected_measurements.bits(), all.bits());
    }

    #[test]
    fn missing_required_channel_invalidates_estimate() {
        let mut ekf = ExtendedKalmanFilter::new(
            design(),
            ReducedBalanceState::default(),
            [0.1; REDUCED_BALANCE_STATE_COUNT],
        )
        .unwrap();
        let available = MeasurementMask::from_bits(
            ((1_u16 << UPRIGHT_MEASUREMENT_COUNT) - 1) & !(1_u16 << GYRO_Y),
        );
        let error = ekf
            .step(
                ReferencePlantInput::default(),
                ReferencePlantInput::default(),
                EstimatorMeasurement::new([0.0; UPRIGHT_MEASUREMENT_COUNT], available, true),
            )
            .unwrap_err();
        assert_eq!(error, EstimateError::MissingRequiredMeasurement(GYRO_Y));
        assert_eq!(ekf.estimate().validity, StateValidity::Invalid);
    }
}
