#![no_std]

use swp_plant_model::{REDUCED_BALANCE_STATE_COUNT, ReducedBalanceState};
use swp_robot_domain::{GeneralizedDemand, TorqueNm};

pub const INTEGRAL_STATE_COUNT: usize = 2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateFeedbackGain {
    pub k: [[f32; REDUCED_BALANCE_STATE_COUNT]; 2],
}

impl StateFeedbackGain {
    pub fn new(k: [[f32; REDUCED_BALANCE_STATE_COUNT]; 2]) -> Option<Self> {
        k.iter()
            .flatten()
            .all(|value| value.is_finite())
            .then_some(Self { k })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlError {
    NonFiniteState,
    NonFiniteReference,
    NonFiniteFeedforward,
    NumericalFault,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LqrController {
    gain: StateFeedbackGain,
}

impl LqrController {
    pub const fn new(gain: StateFeedbackGain) -> Self {
        Self { gain }
    }

    pub const fn gain(&self) -> StateFeedbackGain {
        self.gain
    }

    /// Compute the physical two-input state-feedback demand
    ///
    /// `u = u_ff - K (x - x_ref)`.
    ///
    /// The result is intentionally unconstrained. Allocation, actuator limits,
    /// momentum headroom, and electrical mapping remain downstream authority
    /// concerns rather than hidden controller behavior.
    pub fn command(
        &self,
        state: ReducedBalanceState,
        reference: ReducedBalanceState,
        feedforward: GeneralizedDemand,
    ) -> Result<GeneralizedDemand, ControlError> {
        let error = state_error(state, reference)?;
        require_finite_demand(feedforward)?;

        let drive = feedforward.drive_wheel_torque.0 - dot(self.gain.k[0], error);
        let reaction = feedforward.reaction_wheel_torque.0 - dot(self.gain.k[1], error);
        demand_from_values(drive, reaction)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IntegralProjection {
    /// Rows select the two tracked linear combinations of the seven-state error.
    pub c_i: [[f32; REDUCED_BALANCE_STATE_COUNT]; INTEGRAL_STATE_COUNT],
}

impl IntegralProjection {
    pub fn new(
        c_i: [[f32; REDUCED_BALANCE_STATE_COUNT]; INTEGRAL_STATE_COUNT],
    ) -> Option<Self> {
        c_i.iter()
            .flatten()
            .all(|value| value.is_finite())
            .then_some(Self { c_i })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IntegralGain {
    pub k_i: [[f32; INTEGRAL_STATE_COUNT]; 2],
}

impl IntegralGain {
    pub fn new(k_i: [[f32; INTEGRAL_STATE_COUNT]; 2]) -> Option<Self> {
        k_i.iter()
            .flatten()
            .all(|value| value.is_finite())
            .then_some(Self { k_i })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IntegralBounds {
    pub max_abs: [f32; INTEGRAL_STATE_COUNT],
}

impl IntegralBounds {
    pub fn new(max_abs: [f32; INTEGRAL_STATE_COUNT]) -> Option<Self> {
        max_abs
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
            .then_some(Self { max_abs })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntegratorUpdate {
    Integrate,
    Hold,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LqiController {
    state_gain: StateFeedbackGain,
    integral_gain: IntegralGain,
    projection: IntegralProjection,
    bounds: IntegralBounds,
    sample_period_s: f32,
    integral_state: [f32; INTEGRAL_STATE_COUNT],
}

impl LqiController {
    pub fn new(
        state_gain: StateFeedbackGain,
        integral_gain: IntegralGain,
        projection: IntegralProjection,
        bounds: IntegralBounds,
        sample_period_s: f32,
    ) -> Option<Self> {
        (sample_period_s.is_finite() && sample_period_s > 0.0).then_some(Self {
            state_gain,
            integral_gain,
            projection,
            bounds,
            sample_period_s,
            integral_state: [0.0; INTEGRAL_STATE_COUNT],
        })
    }

    pub const fn integral_state(&self) -> [f32; INTEGRAL_STATE_COUNT] {
        self.integral_state
    }

    pub fn reset_integral(&mut self) {
        self.integral_state = [0.0; INTEGRAL_STATE_COUNT];
    }

    /// LQI convention:
    ///
    /// `z[k+1] = z[k] + Ts * C_i * (x[k] - x_ref[k])`
    ///
    /// `u[k] = u_ff[k] - K x_error[k] - K_i z[k+1]`
    ///
    /// Runtime authority can hold the integrator while an actuator demand is
    /// constrained, denied, or momentum-limited. The controller itself does not
    /// guess actuator limits.
    pub fn command(
        &mut self,
        state: ReducedBalanceState,
        reference: ReducedBalanceState,
        feedforward: GeneralizedDemand,
        integrator_update: IntegratorUpdate,
    ) -> Result<GeneralizedDemand, ControlError> {
        let error = state_error(state, reference)?;
        require_finite_demand(feedforward)?;

        if integrator_update == IntegratorUpdate::Integrate {
            for row in 0..INTEGRAL_STATE_COUNT {
                let delta = self.sample_period_s * dot(self.projection.c_i[row], error);
                let next = self.integral_state[row] + delta;
                self.integral_state[row] =
                    next.clamp(-self.bounds.max_abs[row], self.bounds.max_abs[row]);
            }
        }

        let drive = feedforward.drive_wheel_torque.0
            - dot(self.state_gain.k[0], error)
            - dot_integral(self.integral_gain.k_i[0], self.integral_state);
        let reaction = feedforward.reaction_wheel_torque.0
            - dot(self.state_gain.k[1], error)
            - dot_integral(self.integral_gain.k_i[1], self.integral_state);
        demand_from_values(drive, reaction)
    }
}

fn state_error(
    state: ReducedBalanceState,
    reference: ReducedBalanceState,
) -> Result<[f32; REDUCED_BALANCE_STATE_COUNT], ControlError> {
    let state = state.as_vector();
    let reference = reference.as_vector();

    if !state.iter().all(|value| value.is_finite()) {
        return Err(ControlError::NonFiniteState);
    }
    if !reference.iter().all(|value| value.is_finite()) {
        return Err(ControlError::NonFiniteReference);
    }

    let mut error = [0.0; REDUCED_BALANCE_STATE_COUNT];
    for index in 0..REDUCED_BALANCE_STATE_COUNT {
        error[index] = state[index] - reference[index];
    }
    Ok(error)
}

fn require_finite_demand(demand: GeneralizedDemand) -> Result<(), ControlError> {
    if demand.drive_wheel_torque.0.is_finite() && demand.reaction_wheel_torque.0.is_finite() {
        Ok(())
    } else {
        Err(ControlError::NonFiniteFeedforward)
    }
}

fn demand_from_values(drive: f32, reaction: f32) -> Result<GeneralizedDemand, ControlError> {
    if drive.is_finite() && reaction.is_finite() {
        Ok(GeneralizedDemand {
            drive_wheel_torque: TorqueNm(drive),
            reaction_wheel_torque: TorqueNm(reaction),
        })
    } else {
        Err(ControlError::NumericalFault)
    }
}

fn dot(
    row: [f32; REDUCED_BALANCE_STATE_COUNT],
    vector: [f32; REDUCED_BALANCE_STATE_COUNT],
) -> f32 {
    row.iter()
        .zip(vector.iter())
        .map(|(lhs, rhs)| lhs * rhs)
        .sum()
}

fn dot_integral(
    row: [f32; INTEGRAL_STATE_COUNT],
    vector: [f32; INTEGRAL_STATE_COUNT],
) -> f32 {
    row.iter()
        .zip(vector.iter())
        .map(|(lhs, rhs)| lhs * rhs)
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn zero_demand() -> GeneralizedDemand {
        GeneralizedDemand::default()
    }

    #[test]
    fn lqr_uses_physical_state_error_and_torque_demand() {
        let mut k = [[0.0; REDUCED_BALANCE_STATE_COUNT]; 2];
        k[0][2] = 2.0;
        k[1][4] = 3.0;
        let controller = LqrController::new(StateFeedbackGain::new(k).unwrap());
        let state = ReducedBalanceState {
            pitch_rad: 0.1,
            roll_rad: -0.2,
            ..ReducedBalanceState::default()
        };

        let demand = controller
            .command(state, ReducedBalanceState::default(), zero_demand())
            .unwrap();

        assert!((demand.drive_wheel_torque.0 + 0.2).abs() < 1.0e-6);
        assert!((demand.reaction_wheel_torque.0 - 0.6).abs() < 1.0e-6);
    }

    #[test]
    fn zero_state_error_preserves_feedforward() {
        let controller = LqrController::new(
            StateFeedbackGain::new([[1.0; REDUCED_BALANCE_STATE_COUNT]; 2]).unwrap(),
        );
        let feedforward = GeneralizedDemand {
            drive_wheel_torque: TorqueNm(0.3),
            reaction_wheel_torque: TorqueNm(-0.4),
        };

        assert_eq!(
            controller
                .command(
                    ReducedBalanceState::default(),
                    ReducedBalanceState::default(),
                    feedforward,
                )
                .unwrap(),
            feedforward
        );
    }

    #[test]
    fn lqi_integrator_can_be_held_by_runtime_authority() {
        let state_gain = StateFeedbackGain::new([[0.0; REDUCED_BALANCE_STATE_COUNT]; 2]).unwrap();
        let integral_gain = IntegralGain::new([[1.0, 0.0], [0.0, 1.0]]).unwrap();
        let mut c_i = [[0.0; REDUCED_BALANCE_STATE_COUNT]; INTEGRAL_STATE_COUNT];
        c_i[0][1] = 1.0;
        c_i[1][6] = 1.0;
        let projection = IntegralProjection::new(c_i).unwrap();
        let bounds = IntegralBounds::new([1.0, 1.0]).unwrap();
        let mut controller =
            LqiController::new(state_gain, integral_gain, projection, bounds, 0.1).unwrap();
        let state = ReducedBalanceState {
            forward_velocity_m_per_s: 2.0,
            reaction_wheel_rate_rad_per_s: -3.0,
            ..ReducedBalanceState::default()
        };

        let first = controller
            .command(
                state,
                ReducedBalanceState::default(),
                zero_demand(),
                IntegratorUpdate::Integrate,
            )
            .unwrap();
        assert!((first.drive_wheel_torque.0 + 0.2).abs() < 1.0e-6);
        assert!((first.reaction_wheel_torque.0 - 0.3).abs() < 1.0e-6);

        let integral = controller.integral_state();
        controller
            .command(
                state,
                ReducedBalanceState::default(),
                zero_demand(),
                IntegratorUpdate::Hold,
            )
            .unwrap();
        assert_eq!(controller.integral_state(), integral);
    }

    #[test]
    fn nonfinite_state_is_rejected() {
        let controller = LqrController::new(
            StateFeedbackGain::new([[0.0; REDUCED_BALANCE_STATE_COUNT]; 2]).unwrap(),
        );
        let state = ReducedBalanceState {
            roll_rad: f32::NAN,
            ..ReducedBalanceState::default()
        };
        assert_eq!(
            controller.command(state, ReducedBalanceState::default(), zero_demand()),
            Err(ControlError::NonFiniteState)
        );
    }
}
