#![no_std]
#![no_main]
#![deny(unsafe_code)]

use panic_halt as _;

/// Non-production Cortex-M linkage/footprint probe.
///
/// The numeric fixture below is deliberately synthetic and must never be treated
/// as reference-platform evidence. The binary owns no motor peripherals and
/// creates no electrical output. Its only purpose is to force the complete
/// estimator -> controller -> actuator model -> runtime-authority path through
/// the real STM32F103/CMSIS-DSP link so Flash/RAM cost can be measured before
/// physical parameters are available.
#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use cortex_m::peripheral::DWT;
    use swp_actuator_model::{
        ActuatorPairModel, ActuatorPairOperatingPoint, ActuatorParameters, StaticActuatorModel,
    };
    use swp_control_runtime::{
        ControlRuntime, ControlRuntimeError, ControlStepInput, ControlStepResult,
        StateFeedbackController,
    };
    use swp_measurement_model::{UPRIGHT_MEASUREMENT_COUNT, UprightMeasurementModel};
    use swp_plant_model::{
        DiscreteLinearPlant, REDUCED_BALANCE_STATE_COUNT, REFERENCE_INPUT_COUNT,
        ReducedBalanceState,
    };
    use swp_robot_domain::GeneralizedDemand;
    use swp_runtime_state::{OperatingState, ReactionWheelSpeedLimits, SensorTimingHealth};
    use swp_state_estimator::{
        EstimatorMeasurement, LinearObserver, MeasurementMask, ObserverDesign, ObserverGain,
    };
    use swp_state_feedback::{LqrController, StateFeedbackGain};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let mut dcb = ctx.core.DCB;
        let mut dwt = ctx.core.DWT;
        dcb.enable_trace();
        dwt.enable_cycle_counter();

        // DWT makes the fixture data runtime-dependent so LTO cannot fold the
        // complete control path into a compile-time constant.
        let seed = DWT::cycle_count();
        core::hint::black_box(run_probe(seed));

        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    fn run_probe(seed: u32) -> Result<ControlStepResult, ControlRuntimeError> {
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
        let mut required = MeasurementMask::NONE;
        for index in 0..REDUCED_BALANCE_STATE_COUNT {
            c[index][index] = 1.0;
            l[index][index] = 1.0;
            required = required.with(index);
        }
        let measurement_model = UprightMeasurementModel {
            nominal: [0.0; UPRIGHT_MEASUREMENT_COUNT],
            c,
            d: [[0.0; REFERENCE_INPUT_COUNT]; UPRIGHT_MEASUREMENT_COUNT],
        };
        let observer_design = ObserverDesign::new(
            plant,
            measurement_model,
            ObserverGain::new(l).expect("synthetic observer gain"),
            required,
        )
        .expect("synthetic observer design");
        let observer = LinearObserver::new(observer_design, ReducedBalanceState::default())
            .expect("synthetic observer state");

        let mut k = [[0.0; REDUCED_BALANCE_STATE_COUNT]; 2];
        k[0][2] = 0.8;
        k[0][3] = 0.05;
        k[1][4] = 0.8;
        k[1][5] = 0.05;
        let controller = StateFeedbackController::Lqr(LqrController::new(
            StateFeedbackGain::new(k).expect("synthetic state feedback gain"),
        ));

        let actuator_parameters = ActuatorParameters::new(1.0, 0.05, 0.001, 0.002, 0.1)
            .expect("synthetic actuator parameters");
        let actuator =
            StaticActuatorModel::new(actuator_parameters).expect("synthetic actuator model");
        let actuators = ActuatorPairModel {
            drive: actuator,
            reaction: actuator,
        };

        let mut runtime = ControlRuntime::new(
            observer,
            controller,
            actuators,
            ReactionWheelSpeedLimits::new(80.0, 100.0).expect("synthetic wheel limits"),
        );

        let perturbation = (seed & 0x03ff) as f32 * 1.0e-6;
        let mut values = [0.0; UPRIGHT_MEASUREMENT_COUNT];
        values[2] = 0.02 + perturbation;
        values[3] = -0.01;
        values[4] = -0.015 - perturbation;
        values[5] = 0.008;

        runtime.step(ControlStepInput {
            measurement: EstimatorMeasurement::new(values, required, true),
            operating_state: OperatingState::Balancing,
            timing: SensorTimingHealth::Healthy,
            reference: ReducedBalanceState::default(),
            feedforward: GeneralizedDemand::default(),
            actuator_operating_point: ActuatorPairOperatingPoint {
                drive_speed_rad_per_s: perturbation,
                reaction_speed_rad_per_s: -perturbation,
            },
        })
    }
}
