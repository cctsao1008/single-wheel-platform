#![no_std]

use swp_dsp_kernel::dot_f32;
use swp_measurement_model::{UPRIGHT_MEASUREMENT_COUNT, UprightMeasurementModel};
use swp_plant_model::{
    DiscreteLinearPlant, REDUCED_BALANCE_STATE_COUNT, ReducedBalanceState, ReferencePlantInput,
};
use swp_robot_domain::StateValidity;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MeasurementMask(u16);

impl MeasurementMask {
    pub const NONE: Self = Self(0);

    pub const fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u16 {
        self.0
    }

    pub const fn contains(self, index: usize) -> bool {
        index < 16 && (self.0 & (1_u16 << index)) != 0
    }

    pub const fn with(self, index: usize) -> Self {
        if index < 16 {
            Self(self.0 | (1_u16 << index))
        } else {
            self
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimatorMeasurement {
    pub values: [f32; UPRIGHT_MEASUREMENT_COUNT],
    pub available: MeasurementMask,
    pub timing_valid: bool,
}

impl EstimatorMeasurement {
    pub const fn new(
        values: [f32; UPRIGHT_MEASUREMENT_COUNT],
        available: MeasurementMask,
        timing_valid: bool,
    ) -> Self {
        Self {
            values,
            available,
            timing_valid,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ObserverGain {
    pub l: [[f32; UPRIGHT_MEASUREMENT_COUNT]; REDUCED_BALANCE_STATE_COUNT],
}

impl ObserverGain {
    pub fn new(l: [[f32; UPRIGHT_MEASUREMENT_COUNT]; REDUCED_BALANCE_STATE_COUNT]) -> Option<Self> {
        l.iter()
            .flatten()
            .all(|value| value.is_finite())
            .then_some(Self { l })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ObserverDesign {
    pub plant: DiscreteLinearPlant,
    pub measurement: UprightMeasurementModel,
    pub gain: ObserverGain,
    pub required_measurements: MeasurementMask,
}

impl ObserverDesign {
    pub fn new(
        plant: DiscreteLinearPlant,
        measurement: UprightMeasurementModel,
        gain: ObserverGain,
        required_measurements: MeasurementMask,
    ) -> Option<Self> {
        let sample_period_valid = plant.sample_period_s.is_finite() && plant.sample_period_s > 0.0;
        let plant_finite = plant.a_d.iter().flatten().all(|value| value.is_finite())
            && plant.b_d.iter().flatten().all(|value| value.is_finite());
        let measurement_finite = measurement
            .nominal
            .iter()
            .chain(measurement.c.iter().flatten())
            .chain(measurement.d.iter().flatten())
            .all(|value| value.is_finite());
        let mask_bits = required_measurements.bits();
        let mask_valid =
            mask_bits != 0 && mask_bits & !((1_u16 << UPRIGHT_MEASUREMENT_COUNT) - 1) == 0;

        (sample_period_valid && plant_finite && measurement_finite && mask_valid).then_some(Self {
            plant,
            measurement,
            gain,
            required_measurements,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimatedBalanceState {
    pub state: ReducedBalanceState,
    pub validity: StateValidity,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EstimateError {
    TimingInvalid,
    MissingRequiredMeasurement(usize),
    NonFiniteMeasurement(usize),
    NonFiniteInput,
    NumericalFault,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearObserver {
    design: ObserverDesign,
    state: [f32; REDUCED_BALANCE_STATE_COUNT],
    validity: StateValidity,
}

impl LinearObserver {
    pub fn new(design: ObserverDesign, initial_state: ReducedBalanceState) -> Option<Self> {
        let state = initial_state.as_vector();
        state.iter().all(|value| value.is_finite()).then_some(Self {
            design,
            state,
            validity: StateValidity::Invalid,
        })
    }

    pub const fn design(&self) -> ObserverDesign {
        self.design
    }

    pub fn reset(&mut self, state: ReducedBalanceState) -> bool {
        let vector = state.as_vector();
        if !vector.iter().all(|value| value.is_finite()) {
            self.validity = StateValidity::Invalid;
            return false;
        }
        self.state = vector;
        self.validity = StateValidity::Invalid;
        true
    }

    pub fn estimate(&self) -> EstimatedBalanceState {
        EstimatedBalanceState {
            state: state_from_vector(self.state),
            validity: self.validity,
        }
    }

    /// Execute one fixed-rate predictor/corrector step.
    ///
    /// All fixed-size linear algebra in the estimator execution path is routed
    /// through `swp-dsp-kernel`, whose Cortex-M backend is CMSIS-DSP.
    ///
    /// `previous_input` is the physical actuator input applied over the interval
    /// from k-1 to k and therefore drives the discrete state prediction.
    /// `measurement_input` is the input associated with y[k] and is kept
    /// separate because the measurement equation can contain direct feedthrough.
    pub fn step(
        &mut self,
        previous_input: ReferencePlantInput,
        measurement_input: ReferencePlantInput,
        measurement: EstimatorMeasurement,
    ) -> Result<EstimatedBalanceState, EstimateError> {
        if !measurement.timing_valid {
            self.validity = StateValidity::Invalid;
            return Err(EstimateError::TimingInvalid);
        }

        let previous_input = previous_input.as_vector();
        let measurement_input = measurement_input.as_vector();
        if !previous_input
            .iter()
            .chain(measurement_input.iter())
            .all(|value| value.is_finite())
        {
            self.validity = StateValidity::Invalid;
            return Err(EstimateError::NonFiniteInput);
        }

        for (index, value) in measurement.values.iter().enumerate() {
            if !self.design.required_measurements.contains(index) {
                continue;
            }
            if !measurement.available.contains(index) {
                self.validity = StateValidity::Invalid;
                return Err(EstimateError::MissingRequiredMeasurement(index));
            }
            if !value.is_finite() {
                self.validity = StateValidity::Invalid;
                return Err(EstimateError::NonFiniteMeasurement(index));
            }
        }

        let mut predicted = [0.0; REDUCED_BALANCE_STATE_COUNT];
        for (row, output) in predicted.iter_mut().enumerate() {
            *output = dot_f32(&self.design.plant.a_d[row], &self.state)
                + dot_f32(&self.design.plant.b_d[row], &previous_input);
        }

        let predicted_measurement = self
            .design
            .measurement
            .predict(predicted, measurement_input);
        let mut innovation = [0.0; UPRIGHT_MEASUREMENT_COUNT];
        for (index, output) in innovation.iter_mut().enumerate() {
            if self.design.required_measurements.contains(index) {
                *output = measurement.values[index] - predicted_measurement[index];
            }
        }

        let mut corrected = predicted;
        for (row, output) in corrected.iter_mut().enumerate() {
            *output += dot_f32(&self.design.gain.l[row], &innovation);
        }

        if !corrected.iter().all(|value| value.is_finite()) {
            self.validity = StateValidity::Invalid;
            return Err(EstimateError::NumericalFault);
        }

        self.state = corrected;
        self.validity = StateValidity::Valid;
        Ok(self.estimate())
    }
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
    use swp_plant_model::REFERENCE_INPUT_COUNT;

    fn identity_plant() -> DiscreteLinearPlant {
        let mut a_d = [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        for (index, row) in a_d.iter_mut().enumerate() {
            row[index] = 1.0;
        }
        DiscreteLinearPlant {
            sample_period_s: 0.002,
            a_d,
            b_d: [[0.0; REFERENCE_INPUT_COUNT]; REDUCED_BALANCE_STATE_COUNT],
        }
    }

    fn direct_measurement_model() -> UprightMeasurementModel {
        let mut c = [[0.0; REDUCED_BALANCE_STATE_COUNT]; UPRIGHT_MEASUREMENT_COUNT];
        for (index, row) in c.iter_mut().take(REDUCED_BALANCE_STATE_COUNT).enumerate() {
            row[index] = 1.0;
        }
        UprightMeasurementModel {
            nominal: [0.0; UPRIGHT_MEASUREMENT_COUNT],
            c,
            d: [[0.0; REFERENCE_INPUT_COUNT]; UPRIGHT_MEASUREMENT_COUNT],
        }
    }

    fn design() -> ObserverDesign {
        let mut l = [[0.0; UPRIGHT_MEASUREMENT_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        let mut required = MeasurementMask::NONE;
        for (index, row) in l.iter_mut().enumerate() {
            row[index] = 1.0;
            required = required.with(index);
        }
        ObserverDesign::new(
            identity_plant(),
            direct_measurement_model(),
            ObserverGain::new(l).unwrap(),
            required,
        )
        .unwrap()
    }

    #[test]
    fn correction_promotes_direct_measurements_into_estimated_state() {
        let mut observer = LinearObserver::new(design(), ReducedBalanceState::default()).unwrap();
        let values = [1.0, 2.0, 0.3, 4.0, -0.5, 6.0, 7.0, 99.0];
        let measurement = EstimatorMeasurement::new(
            values,
            MeasurementMask::from_bits((1_u16 << REDUCED_BALANCE_STATE_COUNT) - 1),
            true,
        );

        let estimate = observer
            .step(
                ReferencePlantInput::default(),
                ReferencePlantInput::default(),
                measurement,
            )
            .unwrap();

        assert_eq!(estimate.validity, StateValidity::Valid);
        assert_eq!(
            estimate.state.as_vector(),
            [1.0, 2.0, 0.3, 4.0, -0.5, 6.0, 7.0]
        );
    }

    #[test]
    fn missing_required_measurement_invalidates_estimate() {
        let mut observer = LinearObserver::new(design(), ReducedBalanceState::default()).unwrap();
        let result = observer.step(
            ReferencePlantInput::default(),
            ReferencePlantInput::default(),
            EstimatorMeasurement::new(
                [0.0; UPRIGHT_MEASUREMENT_COUNT],
                MeasurementMask::NONE,
                true,
            ),
        );

        assert_eq!(result, Err(EstimateError::MissingRequiredMeasurement(0)));
        assert_eq!(observer.estimate().validity, StateValidity::Invalid);
    }

    #[test]
    fn timing_invalid_measurement_cannot_drive_the_observer() {
        let mut observer = LinearObserver::new(design(), ReducedBalanceState::default()).unwrap();
        let result = observer.step(
            ReferencePlantInput::default(),
            ReferencePlantInput::default(),
            EstimatorMeasurement::new(
                [0.0; UPRIGHT_MEASUREMENT_COUNT],
                MeasurementMask::from_bits((1_u16 << REDUCED_BALANCE_STATE_COUNT) - 1),
                false,
            ),
        );

        assert_eq!(result, Err(EstimateError::TimingInvalid));
    }

    #[test]
    fn design_rejects_nonfinite_gain() {
        let mut l = [[0.0; UPRIGHT_MEASUREMENT_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        l[0][0] = f32::NAN;
        assert!(ObserverGain::new(l).is_none());
    }
}
