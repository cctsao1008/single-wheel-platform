#![no_std]

use swp_actuator_model::{
    ActuatorModelError, ActuatorPairCommand, ActuatorPairModel, ActuatorPairOperatingPoint,
};
use swp_plant_model::{ReducedBalanceState, ReferencePlantInput};
use swp_robot_domain::{AngularRateRadPerSec, GeneralizedDemand, StateValidity, TorqueNm};
use swp_runtime_state::{
    ActuationAuthority, AuthorityContext, AuthorityDecision, AuthorityOutcome, AuthorizedActuation,
    OperatingState, ReactionWheelSpeedLimits, RuntimeAuthority, SensorTimingHealth,
};
use swp_state_estimator::{
    EstimateError, EstimatedBalanceState, EstimatorMeasurement, LinearObserver,
};
use swp_state_feedback::{
    ControlError, IntegratorUpdate, LqiController, LqrController,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StateFeedbackController {
    Lqr(LqrController),
    Lqi(LqiController),
}

impl StateFeedbackController {
    pub fn reset(&mut self) {
        if let Self::Lqi(controller) = self {
            controller.reset_integral();
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlStepInput {
    pub measurement: EstimatorMeasurement,
    pub operating_state: OperatingState,
    pub timing: SensorTimingHealth,
    pub reference: ReducedBalanceState,
    pub feedforward: GeneralizedDemand,
    pub actuator_operating_point: ActuatorPairOperatingPoint,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlStepResult {
    pub estimate: EstimatedBalanceState,
    pub requested_demand: GeneralizedDemand,
    pub bounded_commands: ActuatorPairCommand,
    pub authority: AuthorityDecision,
    pub authorized_actuation: Option<AuthorizedActuation>,
    pub applied_input: ReferencePlantInput,
    pub integrator_advanced: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ControlRuntimeError {
    Estimate(EstimateError),
    Control(ControlError),
    Actuator(ActuatorModelError),
}

/// Executable composition of the model-based balance path.
///
/// This crate owns causality between estimator, controller, inverse actuator
/// model, and runtime authority. It does not own sensors, board peripherals, or
/// generated numeric design data.
///
/// The previous physically authorized actuator effort is retained as the plant
/// input for the next observer prediction. A denied step records zero applied
/// input. One call represents one measurement opportunity; callers must never
/// execute backlog calls to catch up a missed real-time period.
pub struct ControlRuntime {
    observer: LinearObserver,
    controller: StateFeedbackController,
    actuators: ActuatorPairModel,
    reaction_wheel_limits: ReactionWheelSpeedLimits,
    previous_applied_input: ReferencePlantInput,
}

impl ControlRuntime {
    pub const fn new(
        observer: LinearObserver,
        controller: StateFeedbackController,
        actuators: ActuatorPairModel,
        reaction_wheel_limits: ReactionWheelSpeedLimits,
    ) -> Self {
        Self {
            observer,
            controller,
            actuators,
            reaction_wheel_limits,
            previous_applied_input: ReferencePlantInput {
                drive_torque_nm: 0.0,
                reaction_wheel_torque_nm: 0.0,
            },
        }
    }

    pub const fn previous_applied_input(&self) -> ReferencePlantInput {
        self.previous_applied_input
    }

    pub const fn controller(&self) -> StateFeedbackController {
        self.controller
    }

    /// Reset all dynamic control state before a new closed-loop capture.
    ///
    /// The observer is reset to the supplied physical state, the LQI integrator
    /// is cleared, and the remembered plant input returns to zero. Entering a
    /// balancing session must therefore never inherit stale control history.
    pub fn reset(&mut self, initial_state: ReducedBalanceState) -> bool {
        self.previous_applied_input = ReferencePlantInput::default();
        self.controller.reset();
        self.observer.reset(initial_state)
    }

    /// Execute exactly one model-based control opportunity.
    ///
    /// The observer uses the previously authorized physical effort for both the
    /// ZOH prediction interval and the local direct-feedthrough measurement input.
    /// The new request is not considered applied until `RuntimeAuthority` creates
    /// an `AuthorizedActuation` token.
    pub fn step(&mut self, input: ControlStepInput) -> Result<ControlStepResult, ControlRuntimeError> {
        let estimate = self
            .observer
            .step(
                self.previous_applied_input,
                self.previous_applied_input,
                input.measurement,
            )
            .map_err(ControlRuntimeError::Estimate)?;

        let reaction_wheel_authority = self.reaction_wheel_limits.classify(
            AngularRateRadPerSec(estimate.state.reaction_wheel_rate_rad_per_s),
        );
        let authority_context = AuthorityContext {
            operating_state: input.operating_state,
            timing: input.timing,
            estimate_validity: estimate.validity,
            reaction_wheel_authority,
        };

        let evaluated = match &mut self.controller {
            StateFeedbackController::Lqr(controller) => {
                let demand = controller
                    .command(estimate.state, input.reference, input.feedforward)
                    .map_err(ControlRuntimeError::Control)?;
                let (commands, outcome) = evaluate_demand(
                    self.actuators,
                    demand,
                    input.actuator_operating_point,
                    authority_context,
                )?;
                EvaluatedControl {
                    demand,
                    commands,
                    outcome,
                    integrator_advanced: false,
                }
            }
            StateFeedbackController::Lqi(controller) => {
                // Preflight with the current integral state. Only commit an
                // integral update when both the current request and the updated
                // request remain fully authorized and unconstrained. This avoids
                // even one sample of wind-up at the onset of saturation or
                // momentum limiting.
                let hold_demand = controller
                    .command(
                        estimate.state,
                        input.reference,
                        input.feedforward,
                        IntegratorUpdate::Hold,
                    )
                    .map_err(ControlRuntimeError::Control)?;
                let (hold_commands, hold_outcome) = evaluate_demand(
                    self.actuators,
                    hold_demand,
                    input.actuator_operating_point,
                    authority_context,
                )?;

                if fully_unconstrained(hold_outcome.decision()) {
                    let mut candidate = *controller;
                    let integrated_demand = candidate
                        .command(
                            estimate.state,
                            input.reference,
                            input.feedforward,
                            IntegratorUpdate::Integrate,
                        )
                        .map_err(ControlRuntimeError::Control)?;
                    let (integrated_commands, integrated_outcome) = evaluate_demand(
                        self.actuators,
                        integrated_demand,
                        input.actuator_operating_point,
                        authority_context,
                    )?;

                    if fully_unconstrained(integrated_outcome.decision()) {
                        *controller = candidate;
                        EvaluatedControl {
                            demand: integrated_demand,
                            commands: integrated_commands,
                            outcome: integrated_outcome,
                            integrator_advanced: true,
                        }
                    } else {
                        EvaluatedControl {
                            demand: hold_demand,
                            commands: hold_commands,
                            outcome: hold_outcome,
                            integrator_advanced: false,
                        }
                    }
                } else {
                    EvaluatedControl {
                        demand: hold_demand,
                        commands: hold_commands,
                        outcome: hold_outcome,
                        integrator_advanced: false,
                    }
                }
            }
        };

        let authorized_actuation = evaluated.outcome.authorized();
        let applied_input = authorized_actuation
            .map(applied_input_from_authorized)
            .unwrap_or_default();
        self.previous_applied_input = applied_input;

        Ok(ControlStepResult {
            estimate,
            requested_demand: evaluated.demand,
            bounded_commands: evaluated.commands,
            authority: evaluated.outcome.decision(),
            authorized_actuation,
            applied_input,
            integrator_advanced: evaluated.integrator_advanced,
        })
    }
}

#[derive(Clone, Copy)]
struct EvaluatedControl {
    demand: GeneralizedDemand,
    commands: ActuatorPairCommand,
    outcome: AuthorityOutcome,
    integrator_advanced: bool,
}

fn evaluate_demand(
    actuators: ActuatorPairModel,
    demand: GeneralizedDemand,
    operating_point: ActuatorPairOperatingPoint,
    authority_context: AuthorityContext,
) -> Result<(ActuatorPairCommand, AuthorityOutcome), ControlRuntimeError> {
    let commands = actuators
        .command_for_demand(demand, operating_point)
        .map_err(ControlRuntimeError::Actuator)?;
    let outcome = RuntimeAuthority::evaluate(authority_context, commands);
    Ok((commands, outcome))
}

fn fully_unconstrained(decision: AuthorityDecision) -> bool {
    decision.authority == ActuationAuthority::ClosedLoop && !decision.constrained
}

fn applied_input_from_authorized(authorized: AuthorizedActuation) -> ReferencePlantInput {
    let commands = authorized.commands();
    ReferencePlantInput {
        drive_torque_nm: commands.drive.predicted_torque_nm.0,
        reaction_wheel_torque_nm: commands.reaction.predicted_torque_nm.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use swp_actuator_model::{ActuatorParameters, StaticActuatorModel};
    use swp_measurement_model::{UPRIGHT_MEASUREMENT_COUNT, UprightMeasurementModel};
    use swp_plant_model::{
        DiscreteLinearPlant, REDUCED_BALANCE_STATE_COUNT, REFERENCE_INPUT_COUNT,
    };
    use swp_runtime_state::AuthorityReasons;
    use swp_state_estimator::{MeasurementMask, ObserverDesign, ObserverGain};
    use swp_state_feedback::{
        IntegralBounds, IntegralGain, IntegralProjection, StateFeedbackGain,
    };

    fn observer() -> LinearObserver {
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
        let mut l = [[0.0; UPRIGHT_MEASUREMENT_COUNT]; REDUCED_BALANCE_STATE_COUNT];
        let mut mask = MeasurementMask::NONE;
        for index in 0..REDUCED_BALANCE_STATE_COUNT {
            c[index][index] = 1.0;
            l[index][index] = 1.0;
            mask = mask.with(index);
        }
        let measurement = UprightMeasurementModel {
            nominal: [0.0; UPRIGHT_MEASUREMENT_COUNT],
            c,
            d: [[0.0; REFERENCE_INPUT_COUNT]; UPRIGHT_MEASUREMENT_COUNT],
        };
        let design = ObserverDesign::new(
            plant,
            measurement,
            ObserverGain::new(l).unwrap(),
            mask,
        )
        .unwrap();
        LinearObserver::new(design, ReducedBalanceState::default()).unwrap()
    }

    fn actuator_pair(torque_gain: f32) -> ActuatorPairModel {
        let parameters = ActuatorParameters::new(torque_gain, 0.1, 0.0, 0.0, 0.1).unwrap();
        let model = StaticActuatorModel::new(parameters).unwrap();
        ActuatorPairModel {
            drive: model,
            reaction: model,
        }
    }

    fn measurement(values: [f32; UPRIGHT_MEASUREMENT_COUNT]) -> EstimatorMeasurement {
        EstimatorMeasurement::new(
            values,
            MeasurementMask::from_bits((1_u16 << REDUCED_BALANCE_STATE_COUNT) - 1),
            true,
        )
    }

    fn step_input(values: [f32; UPRIGHT_MEASUREMENT_COUNT]) -> ControlStepInput {
        ControlStepInput {
            measurement: measurement(values),
            operating_state: OperatingState::Balancing,
            timing: SensorTimingHealth::Healthy,
            reference: ReducedBalanceState::default(),
            feedforward: GeneralizedDemand::default(),
            actuator_operating_point: ActuatorPairOperatingPoint {
                drive_speed_rad_per_s: 0.0,
                reaction_speed_rad_per_s: 0.0,
            },
        }
    }

    fn runtime(controller: StateFeedbackController, torque_gain: f32) -> ControlRuntime {
        ControlRuntime::new(
            observer(),
            controller,
            actuator_pair(torque_gain),
            ReactionWheelSpeedLimits::new(80.0, 100.0).unwrap(),
        )
    }

    #[test]
    fn healthy_lqr_step_produces_type_authorized_output() {
        let controller = LqrController::new(
            StateFeedbackGain::new([[0.0; REDUCED_BALANCE_STATE_COUNT]; 2]).unwrap(),
        );
        let mut runtime = runtime(StateFeedbackController::Lqr(controller), 1.0);
        let result = runtime.step(step_input([0.0; UPRIGHT_MEASUREMENT_COUNT])).unwrap();

        assert_eq!(result.estimate.validity, StateValidity::Valid);
        assert_eq!(result.authority.authority, ActuationAuthority::ClosedLoop);
        assert!(result.authorized_actuation.is_some());
        assert_eq!(result.applied_input, ReferencePlantInput::default());
    }

    #[test]
    fn late_sensor_timing_denies_output_and_zeroes_remembered_input() {
        let controller = LqrController::new(
            StateFeedbackGain::new([[0.0; REDUCED_BALANCE_STATE_COUNT]; 2]).unwrap(),
        );
        let mut runtime = runtime(StateFeedbackController::Lqr(controller), 1.0);
        let mut input = step_input([0.0; UPRIGHT_MEASUREMENT_COUNT]);
        input.feedforward = GeneralizedDemand {
            drive_wheel_torque: TorqueNm(0.2),
            reaction_wheel_torque: TorqueNm(0.0),
        };
        input.timing = SensorTimingHealth::Late;

        let result = runtime.step(input).unwrap();
        assert_eq!(result.authority.authority, ActuationAuthority::Denied);
        assert!(result.authority.reasons.contains(AuthorityReasons::SENSOR_TIMING));
        assert!(result.authorized_actuation.is_none());
        assert_eq!(runtime.previous_applied_input(), ReferencePlantInput::default());
    }

    #[test]
    fn lqi_does_not_commit_integrator_update_that_would_create_saturation() {
        let state_gain = StateFeedbackGain::new([[0.0; REDUCED_BALANCE_STATE_COUNT]; 2]).unwrap();
        let integral_gain = IntegralGain::new([[1.0, 0.0], [0.0, 0.0]]).unwrap();
        let mut c_i = [[0.0; REDUCED_BALANCE_STATE_COUNT]; 2];
        c_i[0][1] = 1.0;
        let controller = LqiController::new(
            state_gain,
            integral_gain,
            IntegralProjection::new(c_i).unwrap(),
            IntegralBounds::new([10.0, 10.0]).unwrap(),
            0.1,
        )
        .unwrap();
        let mut runtime = runtime(StateFeedbackController::Lqi(controller), 0.05);
        let mut values = [0.0; UPRIGHT_MEASUREMENT_COUNT];
        values[1] = 2.0;

        let result = runtime.step(step_input(values)).unwrap();
        assert!(!result.integrator_advanced);
        assert_eq!(result.requested_demand, GeneralizedDemand::default());
        assert!(!result.authority.constrained);
        match runtime.controller() {
            StateFeedbackController::Lqi(controller) => {
                assert_eq!(controller.integral_state(), [0.0, 0.0]);
            }
            StateFeedbackController::Lqr(_) => panic!("expected LQI controller"),
        }
    }

    #[test]
    fn lqi_commits_integrator_only_when_updated_request_remains_unconstrained() {
        let state_gain = StateFeedbackGain::new([[0.0; REDUCED_BALANCE_STATE_COUNT]; 2]).unwrap();
        let integral_gain = IntegralGain::new([[1.0, 0.0], [0.0, 0.0]]).unwrap();
        let mut c_i = [[0.0; REDUCED_BALANCE_STATE_COUNT]; 2];
        c_i[0][1] = 1.0;
        let controller = LqiController::new(
            state_gain,
            integral_gain,
            IntegralProjection::new(c_i).unwrap(),
            IntegralBounds::new([10.0, 10.0]).unwrap(),
            0.1,
        )
        .unwrap();
        let mut runtime = runtime(StateFeedbackController::Lqi(controller), 1.0);
        let mut values = [0.0; UPRIGHT_MEASUREMENT_COUNT];
        values[1] = 1.0;

        let result = runtime.step(step_input(values)).unwrap();
        assert!(result.integrator_advanced);
        assert!((result.requested_demand.drive_wheel_torque.0 + 0.1).abs() < 1.0e-6);
        match runtime.controller() {
            StateFeedbackController::Lqi(controller) => {
                assert!((controller.integral_state()[0] - 0.1).abs() < 1.0e-6);
            }
            StateFeedbackController::Lqr(_) => panic!("expected LQI controller"),
        }
    }

    #[test]
    fn reset_clears_dynamic_control_history() {
        let controller = LqrController::new(
            StateFeedbackGain::new([[0.0; REDUCED_BALANCE_STATE_COUNT]; 2]).unwrap(),
        );
        let mut runtime = runtime(StateFeedbackController::Lqr(controller), 1.0);
        let mut input = step_input([0.0; UPRIGHT_MEASUREMENT_COUNT]);
        input.feedforward = GeneralizedDemand {
            drive_wheel_torque: TorqueNm(0.2),
            reaction_wheel_torque: TorqueNm(0.0),
        };
        runtime.step(input).unwrap();
        assert!(runtime.previous_applied_input().drive_torque_nm > 0.0);

        assert!(runtime.reset(ReducedBalanceState::default()));
        assert_eq!(runtime.previous_applied_input(), ReferencePlantInput::default());
        assert_eq!(runtime.observer.estimate().validity, StateValidity::Invalid);
    }
}
