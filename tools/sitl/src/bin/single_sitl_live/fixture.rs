//! Synthetic fixture for the host-only persistent live SITL console.
//!
//! These values mirror the repository's existing closed-loop integration fixture.
//! They are development-only values, not accepted ONE V2 physical parameters.

use swp_actuator_model::{ActuatorPairModel, ActuatorParameters, StaticActuatorModel};
use swp_control_runtime::StateFeedbackController;
use swp_dynamics_model::{
    DiscreteLinearPlant, PlantParameters, ReducedBalanceState, REDUCED_BALANCE_STATE_COUNT,
    REFERENCE_INPUT_COUNT, linearize_stationary_upright,
};
use swp_frame_transform::{FrameEvidence, FrameEvidenceBasis, SensorToBodyRotation};
use swp_measurement_model::{ImuPlacement, linearize_stationary_upright_measurement};
use swp_mpu6050::{AccelRange, Config as MpuConfig, Dlpf, GyroRange};
use swp_runtime_state::{ReactionWheelSpeedLimits, SensorTimingLimits};
use swp_sensor_calibration::encoder::{
    EncoderPositiveDirection, EncoderTransfer, EncoderTransferBasis, EncoderTransferEvidence,
};
use swp_sensor_calibration::{
    AffineCalibration3, CalibrationBasis, CalibrationEvidence, ImuCalibration,
};
use swp_state_estimator::{LinearObserver, MeasurementMask, ObserverDesign, ObserverGain};
use swp_state_feedback::{LqrController, StateFeedbackGain};
use swp_velocity_loop::{VelocityLoop, VelocityLoopParameters, VelocityTarget};
use swp_sitl::closed_loop::{ClosedLoopSimulation, ProductionPathConfig};
use swp_sitl::simulation_world::{SimulationWorld, SimulationWorldConfig};

pub const LIVE_PERIOD_US: u64 = 2_000;
pub const LIVE_INNER_HZ: u32 = 500;
pub const LIVE_OUTER_HZ: u32 = 100;
const SAMPLE_PERIOD_S: f32 = 0.002;

pub fn default_initial_state() -> ReducedBalanceState {
    ReducedBalanceState {
        pitch_rad: 0.03,
        roll_rad: -0.025,
        ..ReducedBalanceState::default()
    }
}

fn plant_parameters() -> PlantParameters {
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

fn euler_discrete_plant() -> DiscreteLinearPlant {
    let continuous = linearize_stationary_upright(plant_parameters()).expect("synthetic plant");
    let mut a_d = [[0.0; REDUCED_BALANCE_STATE_COUNT]; REDUCED_BALANCE_STATE_COUNT];
    let mut b_d = [[0.0; REFERENCE_INPUT_COUNT]; REDUCED_BALANCE_STATE_COUNT];
    for row in 0..REDUCED_BALANCE_STATE_COUNT {
        for column in 0..REDUCED_BALANCE_STATE_COUNT {
            a_d[row][column] = continuous.a[row][column] * SAMPLE_PERIOD_S;
        }
        a_d[row][row] += 1.0;
        for column in 0..REFERENCE_INPUT_COUNT {
            b_d[row][column] = continuous.b[row][column] * SAMPLE_PERIOD_S;
        }
    }
    DiscreteLinearPlant {
        sample_period_s: SAMPLE_PERIOD_S,
        a_d,
        b_d,
    }
}

fn observer() -> LinearObserver {
    let measurement =
        linearize_stationary_upright_measurement(plant_parameters(), ImuPlacement::default())
            .expect("synthetic measurement model");
    let l = [
        [-1.7256453e-3, 0.0, 0.0, 0.0, 1.0308802e-5, 0.0, 4.9912117e-2, 0.0],
        [-1.6453623e-2, 0.0, 0.0, 0.0, 9.283811e-5, 0.0, 4.9412607e-1, 0.0],
        [-3.452375e-2, 0.0, 0.0, 0.0, 2.0616585e-4, 0.0, -1.1468928e-3, 0.0],
        [-1.9492655e-4, 0.0, 0.0, 0.0, 9.4427447e-1, 0.0, 1.0196192e-6, 0.0],
        [0.0, 3.9227562e-2, 0.0, 3.00559e-4, 0.0, 0.0, 0.0, -7.761855e-6],
        [0.0, 1.8421731e-4, 0.0, 9.4427395e-1, 0.0, 0.0, 0.0, -6.225035e-8],
        [0.0, -4.757362e-4, 0.0, -6.225035e-6, 0.0, 0.0, 0.0, 8.284273e-1],
    ];
    let required = MeasurementMask::from_bits(
        (1_u16 << 0) | (1_u16 << 1) | (1_u16 << 3) | (1_u16 << 4) | (1_u16 << 6) | (1_u16 << 7),
    );
    let design = ObserverDesign::new(
        euler_discrete_plant(),
        measurement,
        ObserverGain::new(l).expect("synthetic observer gain"),
        required,
    )
    .expect("synthetic observer design");
    LinearObserver::new(design, ReducedBalanceState::default()).expect("synthetic observer")
}

fn controller() -> StateFeedbackController {
    let k = [
        [-0.90437967, -1.6302925, -6.2296453, -0.7720527, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, -7.7666373, -0.9868621, -0.009627679],
    ];
    StateFeedbackController::Lqr(LqrController::new(
        StateFeedbackGain::new(k).expect("synthetic state-feedback gain"),
    ))
}

fn actuators() -> ActuatorPairModel {
    let actuator = StaticActuatorModel::new(
        ActuatorParameters::new(2.0, 0.01, 0.0, 0.0, 0.1)
            .expect("synthetic actuator parameters"),
    )
    .expect("synthetic actuator model");
    ActuatorPairModel {
        drive: actuator,
        reaction: actuator,
    }
}

fn encoder_transfer() -> EncoderTransfer {
    EncoderTransfer::new(
        65_536,
        EncoderPositiveDirection::CounterIncreasing,
        20_000,
        EncoderTransferEvidence {
            revision: 1,
            basis: EncoderTransferBasis::DatasheetDerived,
        },
    )
    .expect("synthetic encoder transfer")
}

fn identity_calibration() -> ImuCalibration {
    let identity = AffineCalibration3::new(
        [0.0; 3],
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    )
    .expect("identity calibration");
    ImuCalibration {
        accelerometer: identity,
        gyroscope: identity,
        evidence: CalibrationEvidence {
            revision: 1,
            basis: CalibrationBasis::ImportedMeasured,
        },
    }
}

fn world(initial_state: ReducedBalanceState) -> SimulationWorld {
    SimulationWorld::new(
        SimulationWorldConfig {
            plant: plant_parameters(),
            accel_range: AccelRange::G4,
            gyro_range: GyroRange::Dps1000,
            drive_encoder_counts_per_revolution: 65_536,
            reaction_encoder_counts_per_revolution: 65_536,
            imu_temperature_celsius: 36.53,
        },
        initial_state,
        0.0,
    )
    .expect("synthetic simulation world")
}

pub fn build_live_simulation(initial_state: ReducedBalanceState) -> ClosedLoopSimulation {
    let config = ProductionPathConfig {
        mpu: MpuConfig {
            gyro_range: GyroRange::Dps1000,
            accel_range: AccelRange::G4,
            dlpf: Dlpf::Config3,
            sample_rate_hz: LIVE_INNER_HZ as u16,
            data_ready_interrupt: true,
        },
        imu_calibration: identity_calibration(),
        sensor_to_body: SensorToBodyRotation::new([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        .expect("identity frame transform"),
        frame_evidence: FrameEvidence {
            revision: 1,
            basis: FrameEvidenceBasis::ImportedMeasured,
        },
        drive_encoder_transfer: encoder_transfer(),
        reaction_encoder_transfer: encoder_transfer(),
        reaction_wheel_limits: ReactionWheelSpeedLimits::new(80.0, 120.0)
            .expect("synthetic reaction limits"),
        sensor_timing_limits: SensorTimingLimits::new(2_000, 3_000, 6_000)
            .expect("synthetic sensor timing limits"),
        drive_wheel_radius_m: plant_parameters().drive_wheel_radius_m,
        velocity_target: VelocityTarget {
            forward_velocity_m_per_s: 0.0,
        },
        outer_loop_decimation: LIVE_INNER_HZ / LIVE_OUTER_HZ,
    };
    let velocity_loop = VelocityLoop::new(
        VelocityLoopParameters::new(0.05, 0.01, 0.1, 1.0)
            .expect("synthetic velocity-loop parameters"),
        1.0 / LIVE_OUTER_HZ as f32,
    )
    .expect("synthetic velocity loop");
    let mut simulation = ClosedLoopSimulation::new(
        world(initial_state),
        config,
        observer(),
        controller(),
        actuators(),
        velocity_loop,
    )
    .expect("synthetic closed-loop simulation");
    simulation.arm_balance().expect("synthetic supervisor arming");
    simulation
}
