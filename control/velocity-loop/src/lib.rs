#![no_std]

use swp_plant_model::ReducedBalanceState;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VelocityLoopParameters {
    /// Proportional conversion from forward-velocity error to pitch reference.
    pub kp_pitch_rad_per_m_per_s: f32,
    /// Integral conversion from accumulated forward-position error to pitch reference.
    pub ki_pitch_rad_per_m: f32,
    /// Maximum absolute pitch reference emitted by the outer loop.
    pub max_abs_pitch_reference_rad: f32,
    /// Maximum absolute accumulated velocity-error integral in metres.
    pub max_abs_integral_m: f32,
}

impl VelocityLoopParameters {
    pub fn new(
        kp_pitch_rad_per_m_per_s: f32,
        ki_pitch_rad_per_m: f32,
        max_abs_pitch_reference_rad: f32,
        max_abs_integral_m: f32,
    ) -> Option<Self> {
        let candidate = Self {
            kp_pitch_rad_per_m_per_s,
            ki_pitch_rad_per_m,
            max_abs_pitch_reference_rad,
            max_abs_integral_m,
        };
        candidate.is_valid().then_some(candidate)
    }

    pub fn is_valid(self) -> bool {
        self.kp_pitch_rad_per_m_per_s.is_finite()
            && self.ki_pitch_rad_per_m.is_finite()
            && self.max_abs_pitch_reference_rad.is_finite()
            && self.max_abs_pitch_reference_rad > 0.0
            && self.max_abs_integral_m.is_finite()
            && self.max_abs_integral_m > 0.0
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VelocityTarget {
    pub forward_velocity_m_per_s: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VelocityIntegratorUpdate {
    Integrate,
    Hold,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VelocityLoopError {
    InvalidParameters,
    InvalidSamplePeriod,
    NonFiniteState,
    NonFiniteTarget,
    NumericalFault,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VelocityLoopOutput {
    pub reference: ReducedBalanceState,
    pub velocity_error_m_per_s: f32,
    pub integral_error_m: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VelocityLoop {
    parameters: VelocityLoopParameters,
    sample_period_s: f32,
    integral_error_m: f32,
}

impl VelocityLoop {
    pub fn new(parameters: VelocityLoopParameters, sample_period_s: f32) -> Option<Self> {
        if !parameters.is_valid() || !sample_period_s.is_finite() || sample_period_s <= 0.0 {
            return None;
        }
        Some(Self {
            parameters,
            sample_period_s,
            integral_error_m: 0.0,
        })
    }

    pub const fn parameters(self) -> VelocityLoopParameters {
        self.parameters
    }

    pub const fn sample_period_s(self) -> f32 {
        self.sample_period_s
    }

    pub const fn integral_error_m(self) -> f32 {
        self.integral_error_m
    }

    pub fn reset(&mut self) {
        self.integral_error_m = 0.0;
    }

    /// Convert forward-velocity error into a balance reference.
    ///
    /// This outer loop does not produce actuator effort. It only shapes the
    /// physical state reference consumed by the faster balance controller.
    /// Consequently it remains inside Control and preserves the canonical
    /// `EstimatedState + Reference -> GeneralizedDemand` boundary.
    pub fn update(
        &mut self,
        estimated_state: ReducedBalanceState,
        target: VelocityTarget,
        integrator_update: VelocityIntegratorUpdate,
    ) -> Result<VelocityLoopOutput, VelocityLoopError> {
        if !estimated_state
            .as_vector()
            .iter()
            .all(|value| value.is_finite())
        {
            return Err(VelocityLoopError::NonFiniteState);
        }
        if !target.forward_velocity_m_per_s.is_finite() {
            return Err(VelocityLoopError::NonFiniteTarget);
        }

        let error = target.forward_velocity_m_per_s - estimated_state.forward_velocity_m_per_s;
        if integrator_update == VelocityIntegratorUpdate::Integrate {
            self.integral_error_m = (self.integral_error_m + error * self.sample_period_s).clamp(
                -self.parameters.max_abs_integral_m,
                self.parameters.max_abs_integral_m,
            );
        }

        let pitch_reference = self.parameters.kp_pitch_rad_per_m_per_s * error
            + self.parameters.ki_pitch_rad_per_m * self.integral_error_m;
        if !pitch_reference.is_finite() {
            return Err(VelocityLoopError::NumericalFault);
        }
        let bounded_pitch = pitch_reference.clamp(
            -self.parameters.max_abs_pitch_reference_rad,
            self.parameters.max_abs_pitch_reference_rad,
        );

        Ok(VelocityLoopOutput {
            reference: ReducedBalanceState {
                forward_velocity_m_per_s: target.forward_velocity_m_per_s,
                pitch_rad: bounded_pitch,
                ..ReducedBalanceState::default()
            },
            velocity_error_m_per_s: error,
            integral_error_m: self.integral_error_m,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn loop_100_hz() -> VelocityLoop {
        VelocityLoop::new(
            VelocityLoopParameters::new(0.2, 0.1, 0.15, 0.5).unwrap(),
            0.01,
        )
        .unwrap()
    }

    #[test]
    fn outer_loop_outputs_reference_not_actuation() {
        let mut controller = loop_100_hz();
        let output = controller
            .update(
                ReducedBalanceState {
                    forward_velocity_m_per_s: 0.5,
                    ..ReducedBalanceState::default()
                },
                VelocityTarget {
                    forward_velocity_m_per_s: 1.0,
                },
                VelocityIntegratorUpdate::Integrate,
            )
            .unwrap();

        assert_eq!(output.reference.forward_velocity_m_per_s, 1.0);
        assert!(output.reference.pitch_rad > 0.0);
        assert_eq!(output.reference.roll_rad, 0.0);
    }

    #[test]
    fn pitch_reference_and_integral_are_bounded() {
        let mut controller = loop_100_hz();
        for _ in 0..10_000 {
            controller
                .update(
                    ReducedBalanceState::default(),
                    VelocityTarget {
                        forward_velocity_m_per_s: 100.0,
                    },
                    VelocityIntegratorUpdate::Integrate,
                )
                .unwrap();
        }
        assert_eq!(controller.integral_error_m(), 0.5);
        let output = controller
            .update(
                ReducedBalanceState::default(),
                VelocityTarget {
                    forward_velocity_m_per_s: 100.0,
                },
                VelocityIntegratorUpdate::Hold,
            )
            .unwrap();
        assert_eq!(output.reference.pitch_rad, 0.15);
    }

    #[test]
    fn hold_freezes_outer_integrator() {
        let mut controller = loop_100_hz();
        controller
            .update(
                ReducedBalanceState::default(),
                VelocityTarget {
                    forward_velocity_m_per_s: 1.0,
                },
                VelocityIntegratorUpdate::Integrate,
            )
            .unwrap();
        let before = controller.integral_error_m();
        controller
            .update(
                ReducedBalanceState::default(),
                VelocityTarget {
                    forward_velocity_m_per_s: 1.0,
                },
                VelocityIntegratorUpdate::Hold,
            )
            .unwrap();
        assert_eq!(controller.integral_error_m(), before);
    }
}
