use swp_actuator_model::{
    ActuatorPairModel, ActuatorPairOperatingPoint, ActuatorParameters, StaticActuatorModel,
};
use swp_control_runtime::{ControlRuntime, ControlStepInput, StateFeedbackController};
use swp_ekf::{EkfDesign, EkfNoise, ExtendedKalmanFilter};
use swp_measurement_model::{UPRIGHT_MEASUREMENT_COUNT, UprightMeasurementModel};
use swp_plant_model::{
    DiscreteLinearPlant, REDUCED_BALANCE_STATE_COUNT, REFERENCE_INPUT_COUNT, ReducedBalanceState,
};
use swp_robot_domain::{GeneralizedDemand, StateValidity};
use swp_runtime_state::{
    ActuationAuthority, OperatingState, ReactionWheelSpeedLimits, SensorTimingHealth,
};
use swp_state_estimator::{EstimatorMeasurement, MeasurementMask};
use swp_state_feedback::{LqrController, StateFeedbackGain};

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
    let mut nominal = [0.0; UPRIGHT_MEASUREMENT_COUNT];
    nominal[2] = 9.806_65;
    UprightMeasurementModel {
        nominal,
        c,
        d: [[0.0; REFERENCE_INPUT_COUNT]; UPRIGHT_MEASUREMENT_COUNT],
    }
}

fn ekf() -> ExtendedKalmanFilter {
    let required = MeasurementMask::from_bits((1_u16 << REDUCED_BALANCE_STATE_COUNT) - 1);
    let design = EkfDesign::new(
        identity_plant(),
        direct_measurement_model(),
        9.806_65,
        EkfNoise::new(
            [1.0e-5; REDUCED_BALANCE_STATE_COUNT],
            [1.0e-3; UPRIGHT_MEASUREMENT_COUNT],
        )
        .unwrap(),
        required,
        required,
    )
    .unwrap();
    ExtendedKalmanFilter::new(
        design,
        ReducedBalanceState::default(),
        [0.1; REDUCED_BALANCE_STATE_COUNT],
    )
    .unwrap()
}

fn actuators() -> ActuatorPairModel {
    let parameters = ActuatorParameters::new(1.0, 0.05, 0.0, 0.0, 0.1).unwrap();
    let model = StaticActuatorModel::new(parameters).unwrap();
    ActuatorPairModel {
        drive: model,
        reaction: model,
    }
}

#[test]
fn ekf_implements_the_same_closed_loop_runtime_contract() {
    let controller = StateFeedbackController::Lqr(LqrController::new(
        StateFeedbackGain::new([[0.0; REDUCED_BALANCE_STATE_COUNT]; REFERENCE_INPUT_COUNT]).unwrap(),
    ));
    let mut runtime = ControlRuntime::new(
        ekf(),
        controller,
        actuators(),
        ReactionWheelSpeedLimits::new(80.0, 100.0).unwrap(),
    );

    let required = MeasurementMask::from_bits((1_u16 << REDUCED_BALANCE_STATE_COUNT) - 1);
    let mut values = [0.0; UPRIGHT_MEASUREMENT_COUNT];
    values[2] = 9.806_65;

    let result = runtime
        .step(ControlStepInput {
            measurement: EstimatorMeasurement::new(values, required, true),
            operating_state: OperatingState::Balancing,
            timing: SensorTimingHealth::Healthy,
            reference: ReducedBalanceState::default(),
            feedforward: GeneralizedDemand::default(),
            actuator_operating_point: ActuatorPairOperatingPoint {
                drive_speed_rad_per_s: 0.0,
                reaction_speed_rad_per_s: 0.0,
            },
        })
        .unwrap();

    assert_eq!(result.estimate.validity, StateValidity::Valid);
    assert_eq!(result.authority.authority, ActuationAuthority::ClosedLoop);
    assert!(result.authorized_actuation.is_some());
}
