use std::error::Error;
use std::io::{self, BufRead, BufWriter, Write};

use serde::{Deserialize, Serialize};
use swp_actuator_model::{
    ActuatorPairModel, ActuatorPairOperatingPoint, ActuatorParameters, StaticActuatorModel,
};
use swp_control_runtime::{ControlRuntime, ControlStepInput, StateFeedbackController};
use swp_dynamics_model::{
    DiscreteLinearPlant, PlantParameters, REDUCED_BALANCE_STATE_COUNT, REFERENCE_INPUT_COUNT,
    ReducedBalanceState, linearize_stationary_upright,
};
use swp_estimator_input::{EncoderChannelStatus, EstimatorInputBuilder};
use swp_frame_transform::{
    FrameEvidence, FrameEvidenceBasis, SensorToBodyRotation, map_calibrated_imu_to_body,
};
use swp_measurement_model::{ImuPlacement, linearize_stationary_upright_measurement};
use swp_mpu6050::{AccelRange, Config as MpuConfig, Dlpf, GyroRange};
use swp_plant_observation::{
    AcquisitionStatus, MeasurementQuality, RawBatteryObservation, RawEncoderObservation,
    RawImuObservation, RawObservation, TimestampEvidence,
};
use swp_robot_domain::{AngularRateRadPerSec, GeneralizedDemand, StateValidity};
use swp_runtime_state::{
    ActuationAuthority, AuthorityDecision, OperatingState, ReactionWheelSpeedLimits,
    SensorTimingHealth, SensorTimingLimits, SensorTimingMonitor,
};
use swp_runtime_supervisor::{RuntimeFaults, RuntimeSupervisor};
use swp_sensor_calibration::encoder::{
    EncoderPositiveDirection, EncoderTransfer, EncoderTransferBasis, EncoderTransferEvidence,
};
use swp_sensor_calibration::{
    AffineCalibration3, CalibrationBasis, CalibrationEvidence, ImuCalibration, calibrate_imu,
    scale_mpu6050,
};
use swp_sitl::closed_loop::ProductionPathConfig;
use swp_state_estimator::{LinearObserver, MeasurementMask, ObserverDesign, ObserverGain};
use swp_state_feedback::{LqrController, StateFeedbackGain};
use swp_velocity_loop::{
    VelocityIntegratorUpdate, VelocityLoop, VelocityLoopParameters, VelocityTarget,
};

const WIRE_SCHEMA: u32 = 1;
const MAPPING_ID: &str = "webots-body-identity-v1";
const SAMPLE_PERIOD_S: f32 = 0.002;

#[derive(Debug, Deserialize)]
struct RawWireSample {
    schema: u32,
    mapping_id: String,
    sample_index: u32,
    timestamp_us: u64,
    accel_raw: [i16; 3],
    temperature_raw: i16,
    gyro_raw: [i16; 3],
    drive_encoder_count: u16,
    reaction_encoder_count: u16,
}

impl RawWireSample {
    fn into_raw(self) -> Result<RawObservation, String> {
        if self.schema != WIRE_SCHEMA {
            return Err(format!("unsupported Webots bridge schema {}", self.schema));
        }
        if self.mapping_id != MAPPING_ID {
            return Err(format!(
                "unexpected Webots mapping {:?}; hidden axis/sign compensation is forbidden",
                self.mapping_id
            ));
        }

        let timestamp = TimestampEvidence::Known(self.timestamp_us);
        let quality = healthy_quality();
        Ok(RawObservation {
            sample_index: self.sample_index,
            acquisition_started_us: self.timestamp_us,
            acquisition_completed_us: self.timestamp_us,
            imu: RawImuObservation {
                source_sample_at_us: timestamp,
                read_started_at_us: timestamp,
                read_completed_at_us: timestamp,
                accel_raw: self.accel_raw,
                temperature_raw: self.temperature_raw,
                gyro_raw: self.gyro_raw,
                quality,
            },
            encoders: [
                RawEncoderObservation {
                    captured_at_us: timestamp,
                    count: self.drive_encoder_count,
                    quality,
                },
                RawEncoderObservation {
                    captured_at_us: timestamp,
                    count: self.reaction_encoder_count,
                    quality,
                },
            ],
            battery: RawBatteryObservation {
                read_completed_at_us: TimestampEvidence::Unknown,
                adc_raw: 0,
                quality: MeasurementQuality::NONE,
            },
            acquisition_status: AcquisitionStatus::BUS_READY
                | AcquisitionStatus::IMU_PRESENT
                | AcquisitionStatus::IMU_CONFIGURED
                | AcquisitionStatus::IMU_DATA_READY_SEEN
                | AcquisitionStatus::IMU_TIMING_HEALTHY,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum BridgeActuation {
    Apply {
        drive_torque_nm: f32,
        reaction_torque_nm: f32,
    },
    Revoke,
}

#[derive(Clone, Copy, Debug)]
struct BridgeStep {
    sample_index: u32,
    estimate: Option<ReducedBalanceState>,
    reference: ReducedBalanceState,
    estimate_validity: Option<StateValidity>,
    operating_state: OperatingState,
    runtime_faults: RuntimeFaults,
    sensor_timing: SensorTimingHealth,
    authority: Option<AuthorityDecision>,
    actuation: BridgeActuation,
    runtime_steps: u64,
    skipped_unready_observations: u64,
}

struct ProductionSemanticBridge {
    config: ProductionPathConfig,
    estimator_input: EstimatorInputBuilder,
    runtime: ControlRuntime<LinearObserver>,
    velocity_loop: VelocityLoop,
    supervisor: RuntimeSupervisor,
    sensor_timing: SensorTimingMonitor,
    reference: ReducedBalanceState,
    last_estimate: ReducedBalanceState,
    consumed_sample_index: Option<u32>,
    outer_divider: u32,
    runtime_steps: u64,
    skipped_unready_observations: u64,
}

impl ProductionSemanticBridge {
    fn new(
        config: ProductionPathConfig,
        observer: LinearObserver,
        controller: StateFeedbackController,
        actuators: ActuatorPairModel,
        velocity_loop: VelocityLoop,
    ) -> Result<Self, String> {
        if !config.drive_wheel_radius_m.is_finite()
            || config.drive_wheel_radius_m <= 0.0
            || config.outer_loop_decimation == 0
        {
            return Err("invalid production bridge configuration".to_owned());
        }

        let mut bridge = Self {
            config,
            estimator_input: EstimatorInputBuilder::new(
                config.drive_encoder_transfer,
                config.reaction_encoder_transfer,
            ),
            runtime: ControlRuntime::new(
                observer,
                controller,
                actuators,
                config.reaction_wheel_limits,
            ),
            velocity_loop,
            supervisor: RuntimeSupervisor::new(),
            sensor_timing: SensorTimingMonitor::new(config.sensor_timing_limits, 0),
            reference: ReducedBalanceState::default(),
            last_estimate: ReducedBalanceState::default(),
            consumed_sample_index: None,
            outer_divider: 0,
            runtime_steps: 0,
            skipped_unready_observations: 0,
        };
        bridge.arm_balance()?;
        Ok(bridge)
    }

    fn arm_balance(&mut self) -> Result<(), String> {
        self.supervisor
            .boot_complete()
            .map_err(|error| format!("boot transition failed: {error:?}"))?;
        self.supervisor
            .hardware_check_passed()
            .map_err(|error| format!("hardware-check transition failed: {error:?}"))?;
        self.supervisor
            .request_balance()
            .map_err(|error| format!("balance request failed: {error:?}"))?;
        self.estimator_input.reset();
        self.velocity_loop.reset();
        self.reference = ReducedBalanceState::default();
        Ok(())
    }

    fn step_raw(&mut self, raw: RawObservation) -> Result<BridgeStep, String> {
        if self.consumed_sample_index == Some(raw.sample_index) {
            return Err(format!(
                "production bridge attempted to replay sample {}",
                raw.sample_index
            ));
        }

        self.sensor_timing.on_event(raw.acquisition_started_us);
        let scaled = scale_mpu6050(raw.imu, self.config.mpu)
            .map_err(|error| format!("production sensor scaling failed: {error:?}"))?;
        let calibrated = calibrate_imu(scaled, self.config.imu_calibration);
        let body = map_calibrated_imu_to_body(
            calibrated,
            self.config.sensor_to_body,
            self.config.frame_evidence,
        )
        .map_err(|error| format!("production frame transform failed: {error:?}"))?;
        let frame = self
            .estimator_input
            .build(body, raw.encoders[0], raw.encoders[1]);
        self.consumed_sample_index = Some(raw.sample_index);

        if matches!(
            frame.drive_encoder_status,
            EncoderChannelStatus::Rejected(_)
        ) || matches!(
            frame.reaction_encoder_status,
            EncoderChannelStatus::Rejected(_)
        ) {
            return Err("production encoder adapter rejected Webots device evidence".to_owned());
        }

        if !matches!(frame.drive_encoder_status, EncoderChannelStatus::Ready)
            || !matches!(frame.reaction_encoder_status, EncoderChannelStatus::Ready)
        {
            self.skipped_unready_observations = self.skipped_unready_observations.saturating_add(1);
            return Ok(self.snapshot(raw.sample_index, None, None, BridgeActuation::Revoke));
        }

        let drive_speed_rad_per_s = self.last_estimate.forward_velocity_m_per_s
            / self.config.drive_wheel_radius_m
            - self.last_estimate.pitch_rate_rad_per_s;
        let operating_point = ActuatorPairOperatingPoint {
            drive_speed_rad_per_s,
            reaction_speed_rad_per_s: self.last_estimate.reaction_wheel_rate_rad_per_s,
        };
        let timing = self.sensor_timing.health();
        let result = self
            .runtime
            .step(ControlStepInput {
                measurement: frame.measurement,
                operating_state: self.supervisor.state(),
                timing,
                reference: self.reference,
                feedforward: GeneralizedDemand::default(),
                actuator_operating_point: operating_point,
            })
            .map_err(|error| format!("production control runtime failed: {error:?}"))?;

        self.runtime_steps = self.runtime_steps.saturating_add(1);
        self.last_estimate = result.estimate.state;

        let reaction_authority = self
            .config
            .reaction_wheel_limits
            .classify(AngularRateRadPerSec(
                result.estimate.state.reaction_wheel_rate_rad_per_s,
            ));

        if self.supervisor.state() == OperatingState::CaptureWindow
            && result.estimate.validity == StateValidity::Valid
            && timing == SensorTimingHealth::Healthy
        {
            self.supervisor
                .capture_ready()
                .map_err(|error| format!("capture-ready transition failed: {error:?}"))?;
        }
        self.supervisor
            .observe_control_health(result.estimate.validity, reaction_authority);

        self.outer_divider = self.outer_divider.saturating_add(1);
        if self.outer_divider >= self.config.outer_loop_decimation {
            self.outer_divider = 0;
            let integrator_update = if result.authority.hold_integrator {
                VelocityIntegratorUpdate::Hold
            } else {
                VelocityIntegratorUpdate::Integrate
            };
            self.reference = self
                .velocity_loop
                .update(
                    result.estimate.state,
                    self.config.velocity_target,
                    integrator_update,
                )
                .map_err(|error| format!("production velocity loop failed: {error:?}"))?
                .reference;
        }

        let actuation = match result.authorized_actuation {
            Some(authorized) => {
                let commands = authorized.commands();
                BridgeActuation::Apply {
                    drive_torque_nm: commands.drive.predicted_torque_nm.0,
                    reaction_torque_nm: commands.reaction.predicted_torque_nm.0,
                }
            }
            None => BridgeActuation::Revoke,
        };

        Ok(self.snapshot(
            raw.sample_index,
            Some(result.estimate.state),
            Some((result.estimate.validity, result.authority)),
            actuation,
        ))
    }

    fn snapshot(
        &self,
        sample_index: u32,
        estimate: Option<ReducedBalanceState>,
        runtime_evidence: Option<(StateValidity, AuthorityDecision)>,
        actuation: BridgeActuation,
    ) -> BridgeStep {
        BridgeStep {
            sample_index,
            estimate,
            reference: self.reference,
            estimate_validity: runtime_evidence.map(|item| item.0),
            operating_state: self.supervisor.state(),
            runtime_faults: self.supervisor.faults(),
            sensor_timing: self.sensor_timing.health(),
            authority: runtime_evidence.map(|item| item.1),
            actuation,
            runtime_steps: self.runtime_steps,
            skipped_unready_observations: self.skipped_unready_observations,
        }
    }
}

#[derive(Serialize)]
struct StateWire {
    forward_position_m: f32,
    forward_velocity_m_per_s: f32,
    body_pitch_rad: f32,
    body_pitch_rate_rad_per_s: f32,
    body_roll_rad: f32,
    body_roll_rate_rad_per_s: f32,
    reaction_rate_rad_per_s: f32,
}

impl From<ReducedBalanceState> for StateWire {
    fn from(state: ReducedBalanceState) -> Self {
        Self {
            forward_position_m: state.forward_position_m,
            forward_velocity_m_per_s: state.forward_velocity_m_per_s,
            body_pitch_rad: state.pitch_rad,
            body_pitch_rate_rad_per_s: state.pitch_rate_rad_per_s,
            body_roll_rad: state.roll_rad,
            body_roll_rate_rad_per_s: state.roll_rate_rad_per_s,
            reaction_rate_rad_per_s: state.reaction_wheel_rate_rad_per_s,
        }
    }
}

#[derive(Serialize)]
struct BridgeOutput {
    schema: u32,
    sample_index: u32,
    operating_state: &'static str,
    runtime_fault_bits: u16,
    sensor_timing: &'static str,
    estimate_validity: Option<&'static str>,
    authority: Option<&'static str>,
    authority_reason_bits: u16,
    constrained: bool,
    hold_integrator: bool,
    actuation: &'static str,
    drive_torque_nm: f32,
    reaction_torque_nm: f32,
    estimate: Option<StateWire>,
    reference: StateWire,
    runtime_steps: u64,
    skipped_unready_observations: u64,
}

impl From<BridgeStep> for BridgeOutput {
    fn from(step: BridgeStep) -> Self {
        let (actuation, drive_torque_nm, reaction_torque_nm) = match step.actuation {
            BridgeActuation::Apply {
                drive_torque_nm,
                reaction_torque_nm,
            } => ("apply", drive_torque_nm, reaction_torque_nm),
            BridgeActuation::Revoke => ("revoke", 0.0, 0.0),
        };
        Self {
            schema: WIRE_SCHEMA,
            sample_index: step.sample_index,
            operating_state: operating_state_name(step.operating_state),
            runtime_fault_bits: step.runtime_faults.bits(),
            sensor_timing: sensor_timing_name(step.sensor_timing),
            estimate_validity: step.estimate_validity.map(state_validity_name),
            authority: step.authority.map(|decision| match decision.authority {
                ActuationAuthority::Denied => "denied",
                ActuationAuthority::ClosedLoop => "closed_loop",
            }),
            authority_reason_bits: step
                .authority
                .map(|decision| decision.reasons.bits())
                .unwrap_or(0),
            constrained: step
                .authority
                .map(|decision| decision.constrained)
                .unwrap_or(false),
            hold_integrator: step
                .authority
                .map(|decision| decision.hold_integrator)
                .unwrap_or(true),
            actuation,
            drive_torque_nm,
            reaction_torque_nm,
            estimate: step.estimate.map(StateWire::from),
            reference: StateWire::from(step.reference),
            runtime_steps: step.runtime_steps,
            skipped_unready_observations: step.skipped_unready_observations,
        }
    }
}

fn operating_state_name(state: OperatingState) -> &'static str {
    match state {
        OperatingState::Boot => "boot",
        OperatingState::HardwareCheck => "hardware_check",
        OperatingState::Standby => "standby",
        OperatingState::CaptureWindow => "capture_window",
        OperatingState::Balancing => "balancing",
        OperatingState::MomentumLimited => "momentum_limited",
        OperatingState::Fault => "fault",
    }
}

fn sensor_timing_name(health: SensorTimingHealth) -> &'static str {
    match health {
        SensorTimingHealth::Startup => "startup",
        SensorTimingHealth::Healthy => "healthy",
        SensorTimingHealth::Late => "late",
        SensorTimingHealth::Timeout => "timeout",
    }
}

fn state_validity_name(validity: StateValidity) -> &'static str {
    match validity {
        StateValidity::Invalid => "invalid",
        StateValidity::Valid => "valid",
    }
}

fn healthy_quality() -> MeasurementQuality {
    MeasurementQuality::AVAILABLE
        | MeasurementQuality::IO_OK
        | MeasurementQuality::TIMING_VALID
        | MeasurementQuality::FRESHNESS_VERIFIED
}

fn parameters() -> PlantParameters {
    // Synthetic integration fixture only; these are not ONE V2 facts.
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
    let continuous = linearize_stationary_upright(parameters()).expect("synthetic plant");
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
        linearize_stationary_upright_measurement(parameters(), ImuPlacement::default())
            .expect("synthetic measurement model");
    let l = [
        [
            -1.7256453e-3,
            0.0,
            0.0,
            0.0,
            1.0308802e-5,
            0.0,
            4.9912117e-2,
            0.0,
        ],
        [
            -1.6453623e-2,
            0.0,
            0.0,
            0.0,
            9.283811e-5,
            0.0,
            4.9412607e-1,
            0.0,
        ],
        [
            -3.452375e-2,
            0.0,
            0.0,
            0.0,
            2.0616585e-4,
            0.0,
            -1.1468928e-3,
            0.0,
        ],
        [
            -1.9492655e-4,
            0.0,
            0.0,
            0.0,
            9.4427447e-1,
            0.0,
            1.0196192e-6,
            0.0,
        ],
        [
            0.0,
            3.9227562e-2,
            0.0,
            3.00559e-4,
            0.0,
            0.0,
            0.0,
            -7.761855e-6,
        ],
        [
            0.0,
            1.8421731e-4,
            0.0,
            9.4427395e-1,
            0.0,
            0.0,
            0.0,
            -6.225035e-8,
        ],
        [
            0.0,
            -4.757362e-4,
            0.0,
            -6.225035e-6,
            0.0,
            0.0,
            0.0,
            8.284273e-1,
        ],
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
        [
            -0.90437967,
            -1.6302925,
            -6.2296453,
            -0.7720527,
            0.0,
            0.0,
            0.0,
        ],
        [0.0, 0.0, 0.0, 0.0, -7.7666373, -0.9868621, -0.009627679],
    ];
    StateFeedbackController::Lqr(LqrController::new(
        StateFeedbackGain::new(k).expect("synthetic state-feedback gain"),
    ))
}

fn actuators() -> ActuatorPairModel {
    let actuator = StaticActuatorModel::new(
        ActuatorParameters::new(2.0, 0.01, 0.0, 0.0, 0.1).expect("synthetic actuator parameters"),
    )
    .expect("synthetic actuator model");
    ActuatorPairModel {
        drive: actuator,
        reaction: actuator,
    }
}

fn transfer() -> EncoderTransfer {
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

fn synthetic_bridge() -> ProductionSemanticBridge {
    let reaction_limits = ReactionWheelSpeedLimits::new(80.0, 120.0).expect("reaction limits");
    let config = ProductionPathConfig {
        mpu: MpuConfig {
            gyro_range: GyroRange::Dps1000,
            accel_range: AccelRange::G4,
            dlpf: Dlpf::Config3,
            sample_rate_hz: 500,
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
        drive_encoder_transfer: transfer(),
        reaction_encoder_transfer: transfer(),
        reaction_wheel_limits: reaction_limits,
        sensor_timing_limits: SensorTimingLimits::new(2_000, 3_000, 6_000)
            .expect("sensor timing limits"),
        drive_wheel_radius_m: parameters().drive_wheel_radius_m,
        velocity_target: VelocityTarget {
            forward_velocity_m_per_s: 0.0,
        },
        outer_loop_decimation: 5,
    };
    let velocity_loop = VelocityLoop::new(
        VelocityLoopParameters::new(0.05, 0.01, 0.1, 1.0)
            .expect("synthetic velocity-loop parameters"),
        0.01,
    )
    .expect("synthetic velocity loop");
    ProductionSemanticBridge::new(config, observer(), controller(), actuators(), velocity_loop)
        .expect("synthetic production bridge")
}

fn run() -> Result<(), Box<dyn Error>> {
    let stdin = io::stdin();
    let mut stdout = BufWriter::new(io::stdout().lock());
    let mut bridge = synthetic_bridge();

    for line in stdin.lock().lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let wire: RawWireSample = serde_json::from_str(&line)?;
        let raw = wire.into_raw().map_err(io::Error::other)?;
        let step = bridge.step_raw(raw).map_err(io::Error::other)?;
        serde_json::to_writer(&mut stdout, &BridgeOutput::from(step))?;
        writeln!(stdout)?;
        stdout.flush()?;
    }
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("webots production bridge error: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wire(sample_index: u32, timestamp_us: u64) -> RawWireSample {
        RawWireSample {
            schema: WIRE_SCHEMA,
            mapping_id: MAPPING_ID.to_owned(),
            sample_index,
            timestamp_us,
            accel_raw: [0, 0, 8192],
            temperature_raw: 0,
            gyro_raw: [0, 0, 0],
            drive_encoder_count: 0,
            reaction_encoder_count: 0,
        }
    }

    #[test]
    fn startup_requires_fresh_encoder_and_timing_evidence_before_authority() {
        let mut bridge = synthetic_bridge();

        let first = bridge.step_raw(wire(0, 2_000).into_raw().unwrap()).unwrap();
        assert_eq!(first.actuation, BridgeActuation::Revoke);
        assert_eq!(first.operating_state, OperatingState::CaptureWindow);
        assert_eq!(first.sensor_timing, SensorTimingHealth::Startup);

        let second = bridge.step_raw(wire(1, 4_000).into_raw().unwrap()).unwrap();
        assert_eq!(second.actuation, BridgeActuation::Revoke);
        assert_eq!(second.operating_state, OperatingState::Balancing);
        assert_eq!(second.sensor_timing, SensorTimingHealth::Healthy);

        let third = bridge.step_raw(wire(2, 6_000).into_raw().unwrap()).unwrap();
        assert!(matches!(third.actuation, BridgeActuation::Apply { .. }));
        assert_eq!(third.operating_state, OperatingState::Balancing);
        assert!(third.authority.unwrap().closed_loop_authorized());
    }

    #[test]
    fn replayed_raw_observation_is_rejected() {
        let mut bridge = synthetic_bridge();
        bridge.step_raw(wire(7, 2_000).into_raw().unwrap()).unwrap();
        let error = bridge
            .step_raw(wire(7, 4_000).into_raw().unwrap())
            .unwrap_err();
        assert!(error.contains("replay sample 7"));
    }

    #[test]
    fn wrong_mapping_id_fails_before_production_runtime() {
        let mut sample = wire(0, 2_000);
        sample.mapping_id = "broken-pitch-sign".to_owned();
        let error = sample.into_raw().unwrap_err();
        assert!(error.contains("hidden axis/sign compensation"));
    }
}
