use std::error::Error;
use std::fmt::{Display, Formatter};

use swp_actuation_interface::ActuationSink;
use swp_actuator_model::{ActuatorPairModel, ActuatorPairOperatingPoint};
use swp_control_runtime::{
    ControlRuntime, ControlRuntimeError, ControlStepInput, StateFeedbackController,
};
use swp_dynamics_model::ReducedBalanceState;
use swp_estimator_input::{EncoderChannelStatus, EstimatorInputBuilder, EstimatorInputFrame};
use swp_frame_transform::{
    FrameEvidence, FrameTransformError, SensorToBodyRotation, map_calibrated_imu_to_body,
};
use swp_mpu6050::Config as MpuConfig;
use swp_plant_observation::RawObservation;
use swp_robot_domain::{AngularRateRadPerSec, GeneralizedDemand, StateValidity};
use swp_runtime_state::{
    AuthorityDecision, AuthorizedActuation, OperatingState, ReactionWheelSpeedLimits,
    SensorTimingHealth, SensorTimingLimits, SensorTimingMonitor,
};
use swp_runtime_supervisor::{RuntimeFaults, RuntimeSupervisor, RuntimeTransitionError};
use swp_sensor_calibration::encoder::EncoderTransfer;
use swp_sensor_calibration::{
    CalibrationError, ImuCalibration, calibrate_imu, scale_mpu6050,
};
use swp_state_estimator::LinearObserver;
use swp_velocity_loop::{
    VelocityIntegratorUpdate, VelocityLoop, VelocityLoopError, VelocityTarget,
};

use crate::scheduler::{EventKind, ScheduledEvent};
use crate::simulation_world::{SimulationTruth, SimulationWorld, SimulationWorldError};
use crate::virtual_time::VirtualTime;
use crate::{PhysicalTimeAdvance, ScenarioExecution};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProductionPathConfig {
    pub mpu: MpuConfig,
    pub imu_calibration: ImuCalibration,
    pub sensor_to_body: SensorToBodyRotation,
    pub frame_evidence: FrameEvidence,
    pub drive_encoder_transfer: EncoderTransfer,
    pub reaction_encoder_transfer: EncoderTransfer,
    pub reaction_wheel_limits: ReactionWheelSpeedLimits,
    pub sensor_timing_limits: SensorTimingLimits,
    pub drive_wheel_radius_m: f32,
    pub velocity_target: VelocityTarget,
    pub outer_loop_decimation: u32,
}

impl ProductionPathConfig {
    fn is_valid(self) -> bool {
        self.drive_wheel_radius_m.is_finite()
            && self.drive_wheel_radius_m > 0.0
            && self.outer_loop_decimation > 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PendingActuation {
    Apply(AuthorizedActuation),
    Revoke,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct DeliveredObservation {
    sample_index: u32,
    frame: EstimatorInputFrame,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosedLoopSnapshot {
    pub truth: SimulationTruth,
    pub latest_estimate: Option<ReducedBalanceState>,
    pub reference: ReducedBalanceState,
    pub operating_state: OperatingState,
    pub runtime_faults: RuntimeFaults,
    pub sensor_timing: SensorTimingHealth,
    pub latest_authority: Option<AuthorityDecision>,
    pub sensor_samples: u64,
    pub delivered_observations: u64,
    pub runtime_steps: u64,
    pub skipped_unready_observations: u64,
    pub missed_runtime_opportunities: u64,
    pub observations_discarded_on_miss: u64,
    pub authorized_commits: u64,
    pub revocations: u64,
}

#[derive(Debug)]
pub enum ClosedLoopError {
    InvalidConfig,
    World(SimulationWorldError),
    Calibration(CalibrationError),
    FrameTransform(FrameTransformError),
    ControlRuntime(ControlRuntimeError),
    VelocityLoop(VelocityLoopError),
    SupervisorTransition(RuntimeTransitionError),
    SensorSampleAlreadyPending,
    ObservationDeliveryWithoutSample,
    RuntimeWithoutObservation,
    RuntimeWithoutFreshObservation(u32),
    EncoderRejected,
    ActuationCommitWithoutRuntime,
}

impl Display for ClosedLoopError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidConfig => write!(formatter, "invalid closed-loop SITL configuration"),
            Self::World(error) => write!(formatter, "simulation world failed: {error}"),
            Self::Calibration(error) => {
                write!(formatter, "production sensor scaling failed: {error:?}")
            }
            Self::FrameTransform(error) => {
                write!(formatter, "production frame transform failed: {error:?}")
            }
            Self::ControlRuntime(error) => {
                write!(formatter, "production control runtime failed: {error:?}")
            }
            Self::VelocityLoop(error) => {
                write!(formatter, "production velocity loop failed: {error:?}")
            }
            Self::SupervisorTransition(error) => {
                write!(formatter, "runtime supervisor transition failed: {error:?}")
            }
            Self::SensorSampleAlreadyPending => {
                write!(formatter, "new virtual sensor sample arrived before prior delivery")
            }
            Self::ObservationDeliveryWithoutSample => {
                write!(formatter, "observation delivery occurred without a sampled observation")
            }
            Self::RuntimeWithoutObservation => {
                write!(formatter, "production runtime occurred without a delivered observation")
            }
            Self::RuntimeWithoutFreshObservation(sample_index) => write!(
                formatter,
                "production runtime attempted to reuse observation sample {sample_index}"
            ),
            Self::EncoderRejected => {
                write!(formatter, "production encoder adapter rejected virtual sensor evidence")
            }
            Self::ActuationCommitWithoutRuntime => {
                write!(formatter, "actuation commit occurred without a runtime decision")
            }
        }
    }
}

impl Error for ClosedLoopError {}

impl From<SimulationWorldError> for ClosedLoopError {
    fn from(error: SimulationWorldError) -> Self {
        Self::World(error)
    }
}

/// Nominal software-in-the-loop composition.
///
/// `SimulationWorld` owns physical truth and emits only device-like `RawObservation`.
/// The production sensor adapters, estimator, Control, actuator model, Supervisor
/// authority, and `ActuationSink` then close the loop back into the virtual world.
/// Physical truth is available only through `snapshot()` for host-side evidence;
/// it is never passed into the production runtime.
pub struct ClosedLoopSimulation {
    world: SimulationWorld,
    config: ProductionPathConfig,
    estimator_input: EstimatorInputBuilder,
    runtime: ControlRuntime<LinearObserver>,
    velocity_loop: VelocityLoop,
    supervisor: RuntimeSupervisor,
    sensor_timing: SensorTimingMonitor,
    reference: ReducedBalanceState,
    last_estimate: ReducedBalanceState,
    latest_authority: Option<AuthorityDecision>,
    pending_raw: Option<RawObservation>,
    delivered: Option<DeliveredObservation>,
    consumed_sample_index: Option<u32>,
    pending_actuation: Option<PendingActuation>,
    outer_divider: u32,
    sensor_samples: u64,
    delivered_observations: u64,
    runtime_steps: u64,
    skipped_unready_observations: u64,
    missed_runtime_opportunities: u64,
    observations_discarded_on_miss: u64,
    authorized_commits: u64,
    revocations: u64,
}

impl ClosedLoopSimulation {
    pub fn new(
        world: SimulationWorld,
        config: ProductionPathConfig,
        observer: LinearObserver,
        controller: StateFeedbackController,
        actuators: ActuatorPairModel,
        velocity_loop: VelocityLoop,
    ) -> Result<Self, ClosedLoopError> {
        if !config.is_valid() {
            return Err(ClosedLoopError::InvalidConfig);
        }

        Ok(Self {
            world,
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
            latest_authority: None,
            pending_raw: None,
            delivered: None,
            consumed_sample_index: None,
            pending_actuation: None,
            outer_divider: 0,
            sensor_samples: 0,
            delivered_observations: 0,
            runtime_steps: 0,
            skipped_unready_observations: 0,
            missed_runtime_opportunities: 0,
            observations_discarded_on_miss: 0,
            authorized_commits: 0,
            revocations: 0,
        })
    }

    /// Explicitly progress the production Supervisor into `CaptureWindow`.
    /// Closed-loop authority is still denied until a healthy, estimator-valid
    /// runtime opportunity promotes the Supervisor to `Balancing`.
    pub fn arm_balance(&mut self) -> Result<(), ClosedLoopError> {
        self.supervisor
            .boot_complete()
            .map_err(ClosedLoopError::SupervisorTransition)?;
        self.supervisor
            .hardware_check_passed()
            .map_err(ClosedLoopError::SupervisorTransition)?;
        self.supervisor
            .request_balance()
            .map_err(ClosedLoopError::SupervisorTransition)?;
        self.estimator_input.reset();
        self.velocity_loop.reset();
        self.reference = ReducedBalanceState::default();
        ActuationSink::revoke(&mut self.world)?;
        Ok(())
    }

    pub fn snapshot(&self) -> ClosedLoopSnapshot {
        ClosedLoopSnapshot {
            truth: self.world.truth(),
            latest_estimate: (self.runtime_steps > 0).then_some(self.last_estimate),
            reference: self.reference,
            operating_state: self.supervisor.state(),
            runtime_faults: self.supervisor.faults(),
            sensor_timing: self.sensor_timing.health(),
            latest_authority: self.latest_authority,
            sensor_samples: self.sensor_samples,
            delivered_observations: self.delivered_observations,
            runtime_steps: self.runtime_steps,
            skipped_unready_observations: self.skipped_unready_observations,
            missed_runtime_opportunities: self.missed_runtime_opportunities,
            observations_discarded_on_miss: self.observations_discarded_on_miss,
            authorized_commits: self.authorized_commits,
            revocations: self.revocations,
        }
    }

    fn sample_sensor(&mut self, at: VirtualTime) -> Result<(), ClosedLoopError> {
        if self.pending_raw.is_some() {
            return Err(ClosedLoopError::SensorSampleAlreadyPending);
        }
        let raw = self.world.sample_raw_observation(at)?;
        self.sensor_timing.on_event(at.as_micros());
        self.pending_raw = Some(raw);
        self.sensor_samples = self.sensor_samples.saturating_add(1);
        Ok(())
    }

    fn deliver_observation(&mut self) -> Result<(), ClosedLoopError> {
        let raw = self
            .pending_raw
            .take()
            .ok_or(ClosedLoopError::ObservationDeliveryWithoutSample)?;
        let scaled = scale_mpu6050(raw.imu, self.config.mpu)
            .map_err(ClosedLoopError::Calibration)?;
        let calibrated = calibrate_imu(scaled, self.config.imu_calibration);
        let body = map_calibrated_imu_to_body(
            calibrated,
            self.config.sensor_to_body,
            self.config.frame_evidence,
        )
        .map_err(ClosedLoopError::FrameTransform)?;

        // SimulationWorld exposes its two virtual encoder channels in
        // [drive, reaction] order. Board-channel permutation belongs to the
        // hardware assembly target and is not fabricated by host SITL.
        let frame = self
            .estimator_input
            .build(body, raw.encoders[0], raw.encoders[1]);
        self.delivered = Some(DeliveredObservation {
            sample_index: raw.sample_index,
            frame,
        });
        self.delivered_observations = self.delivered_observations.saturating_add(1);
        Ok(())
    }

    fn run_production_runtime(&mut self) -> Result<(), ClosedLoopError> {
        let delivered = self
            .delivered
            .ok_or(ClosedLoopError::RuntimeWithoutObservation)?;
        if self.consumed_sample_index == Some(delivered.sample_index) {
            return Err(ClosedLoopError::RuntimeWithoutFreshObservation(
                delivered.sample_index,
            ));
        }

        if matches!(
            delivered.frame.drive_encoder_status,
            EncoderChannelStatus::Rejected(_)
        ) || matches!(
            delivered.frame.reaction_encoder_status,
            EncoderChannelStatus::Rejected(_)
        ) {
            return Err(ClosedLoopError::EncoderRejected);
        }

        self.consumed_sample_index = Some(delivered.sample_index);

        if !matches!(
            delivered.frame.drive_encoder_status,
            EncoderChannelStatus::Ready
        ) || !matches!(
            delivered.frame.reaction_encoder_status,
            EncoderChannelStatus::Ready
        ) {
            self.skipped_unready_observations =
                self.skipped_unready_observations.saturating_add(1);
            self.pending_actuation = Some(PendingActuation::Revoke);
            return Ok(());
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
                measurement: delivered.frame.measurement,
                operating_state: self.supervisor.state(),
                timing,
                reference: self.reference,
                feedforward: GeneralizedDemand::default(),
                actuator_operating_point: operating_point,
            })
            .map_err(ClosedLoopError::ControlRuntime)?;

        self.runtime_steps = self.runtime_steps.saturating_add(1);
        self.last_estimate = result.estimate.state;
        self.latest_authority = Some(result.authority);

        let reaction_authority = self.config.reaction_wheel_limits.classify(
            AngularRateRadPerSec(result.estimate.state.reaction_wheel_rate_rad_per_s),
        );

        if self.supervisor.state() == OperatingState::CaptureWindow
            && result.estimate.validity == StateValidity::Valid
            && timing == SensorTimingHealth::Healthy
        {
            self.supervisor
                .capture_ready()
                .map_err(ClosedLoopError::SupervisorTransition)?;
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
                .map_err(ClosedLoopError::VelocityLoop)?
                .reference;
        }

        self.pending_actuation = Some(match result.authorized_actuation {
            Some(authorized) => PendingActuation::Apply(authorized),
            None => PendingActuation::Revoke,
        });
        Ok(())
    }

    fn miss_runtime_opportunity(&mut self) {
        self.missed_runtime_opportunities = self.missed_runtime_opportunities.saturating_add(1);
        if let Some(delivered) = self.delivered
            && self.consumed_sample_index != Some(delivered.sample_index)
        {
            self.consumed_sample_index = Some(delivered.sample_index);
            self.observations_discarded_on_miss =
                self.observations_discarded_on_miss.saturating_add(1);
        }
    }

    fn commit_actuation(&mut self) -> Result<(), ClosedLoopError> {
        let pending = self
            .pending_actuation
            .take()
            .ok_or(ClosedLoopError::ActuationCommitWithoutRuntime)?;
        match pending {
            PendingActuation::Apply(authorized) => {
                ActuationSink::apply_authorized(&mut self.world, authorized)?;
                self.authorized_commits = self.authorized_commits.saturating_add(1);
            }
            PendingActuation::Revoke => {
                ActuationSink::revoke(&mut self.world)?;
                self.revocations = self.revocations.saturating_add(1);
            }
        }
        Ok(())
    }

    fn dispatch(&mut self, event: ScheduledEvent) -> Result<(), ClosedLoopError> {
        match event.kind {
            EventKind::ScenarioStart => Ok(()),
            EventKind::SensorSample => self.sample_sensor(event.at),
            EventKind::ObservationDelivery => self.deliver_observation(),
            EventKind::ProductionRuntime => self.run_production_runtime(),
            EventKind::RuntimeOpportunityMissed => {
                self.miss_runtime_opportunity();
                Ok(())
            }
            EventKind::ActuationCommit => self.commit_actuation(),
        }
    }
}

impl PhysicalTimeAdvance for ClosedLoopSimulation {
    fn advance(
        &mut self,
        from: VirtualTime,
        to: VirtualTime,
    ) -> Result<(), Box<dyn Error + 'static>> {
        self.world.advance(from, to)
    }
}

impl ScenarioExecution for ClosedLoopSimulation {
    fn dispatch_event(
        &mut self,
        event: ScheduledEvent,
    ) -> Result<(), Box<dyn Error + 'static>> {
        self.dispatch(event)
            .map_err(|error| Box::new(error) as Box<dyn Error + 'static>)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use swp_actuator_model::{ActuatorParameters, StaticActuatorModel};
    use swp_dynamics_model::{
        DiscreteLinearPlant, PlantParameters, REDUCED_BALANCE_STATE_COUNT, REFERENCE_INPUT_COUNT,
        linearize_stationary_upright,
    };
    use swp_frame_transform::{FrameEvidenceBasis, SensorToBodyRotation};
    use swp_measurement_model::{
        ImuPlacement, UPRIGHT_MEASUREMENT_COUNT, linearize_stationary_upright_measurement,
    };
    use swp_mpu6050::{AccelRange, Dlpf, GyroRange};
    use swp_sensor_calibration::encoder::{
        EncoderPositiveDirection, EncoderTransferBasis, EncoderTransferEvidence,
    };
    use swp_sensor_calibration::{
        AffineCalibration3, CalibrationBasis, CalibrationEvidence,
    };
    use swp_state_estimator::{LinearObserver, MeasurementMask, ObserverDesign, ObserverGain};
    use swp_state_feedback::{LqrController, StateFeedbackGain};
    use swp_velocity_loop::VelocityLoopParameters;

    use crate::scenario::Scenario;
    use crate::{RunContext, run_scenario_with_execution};

    const SAMPLE_PERIOD_S: f32 = 0.002;

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
        let continuous = linearize_stationary_upright(parameters()).unwrap();
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
                .unwrap();
        // Synthetic steady-state observer gain generated independently from the
        // same explicit fixture. It is test evidence, not reference-robot data.
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
            (1_u16 << 0)
                | (1_u16 << 1)
                | (1_u16 << 3)
                | (1_u16 << 4)
                | (1_u16 << 6)
                | (1_u16 << 7),
        );
        let design = ObserverDesign::new(
            euler_discrete_plant(),
            measurement,
            ObserverGain::new(l).unwrap(),
            required,
        )
        .unwrap();
        LinearObserver::new(design, ReducedBalanceState::default()).unwrap()
    }

    fn controller() -> StateFeedbackController {
        let k = [
            [-0.90437967, -1.6302925, -6.2296453, -0.7720527, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, -7.7666373, -0.9868621, -0.009627679],
        ];
        StateFeedbackController::Lqr(LqrController::new(
            StateFeedbackGain::new(k).unwrap(),
        ))
    }

    fn actuators() -> ActuatorPairModel {
        let actuator = StaticActuatorModel::new(
            ActuatorParameters::new(2.0, 0.0, 0.0, 0.0, 0.1).unwrap(),
        )
        .unwrap();
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
        .unwrap()
    }

    fn identity_calibration() -> ImuCalibration {
        let identity = AffineCalibration3::new(
            [0.0; 3],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )
        .unwrap();
        // Test-only evidence token. No physical calibration claim is made by this fixture.
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
            crate::simulation_world::SimulationWorldConfig {
                plant: parameters(),
                accel_range: AccelRange::G4,
                gyro_range: GyroRange::Dps1000,
                drive_encoder_counts_per_revolution: 65_536,
                reaction_encoder_counts_per_revolution: 65_536,
                imu_temperature_celsius: 36.53,
            },
            initial_state,
            0.0,
        )
        .unwrap()
    }

    fn simulation(initial_state: ReducedBalanceState) -> ClosedLoopSimulation {
        let reaction_limits = ReactionWheelSpeedLimits::new(80.0, 120.0).unwrap();
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
            .unwrap(),
            frame_evidence: FrameEvidence {
                revision: 1,
                basis: FrameEvidenceBasis::ImportedMeasured,
            },
            drive_encoder_transfer: transfer(),
            reaction_encoder_transfer: transfer(),
            reaction_wheel_limits: reaction_limits,
            sensor_timing_limits: SensorTimingLimits::new(2_000, 3_000, 6_000).unwrap(),
            drive_wheel_radius_m: parameters().drive_wheel_radius_m,
            velocity_target: VelocityTarget {
                forward_velocity_m_per_s: 0.0,
            },
            outer_loop_decimation: 5,
        };
        let velocity_loop = VelocityLoop::new(
            VelocityLoopParameters::new(0.05, 0.01, 0.1, 1.0).unwrap(),
            0.01,
        )
        .unwrap();
        let mut simulation = ClosedLoopSimulation::new(
            world(initial_state),
            config,
            observer(),
            controller(),
            actuators(),
            velocity_loop,
        )
        .unwrap();
        simulation.arm_balance().unwrap();
        simulation
    }

    fn scenario(duration_us: u64, missed_runtime_at_us: Vec<u64>) -> Scenario {
        Scenario {
            id: "synthetic-closed-loop".to_owned(),
            duration_us,
            seed: 0x5357_5002,
            sensor_period_us: 2_000,
            runtime_period_us: 2_000,
            missed_runtime_at_us,
        }
    }

    fn context() -> RunContext {
        let mut context = RunContext::scheduler_only("single-wheel-platform", "test-commit");
        context.production_model_configuration = json!({
            "kind": "synthetic-integration-fixture",
            "inner_hz": 500,
            "outer_hz": 100,
            "path": "production-adapters-estimator-control-runtime-authority"
        });
        context.virtual_physical_truth_configuration = json!({
            "kind": "synthetic-integration-fixture",
            "plant": "stationary-upright-reduced-dynamics",
            "sensor": "device-like-mpu6050-and-encoder"
        });
        context
    }

    #[test]
    fn raw_virtual_sensors_close_through_production_runtime_and_stabilize_upright() {
        let initial = ReducedBalanceState {
            pitch_rad: 0.03,
            roll_rad: -0.025,
            ..ReducedBalanceState::default()
        };
        let mut simulation = simulation(initial);
        let artifacts = run_scenario_with_execution(
            &context(),
            &scenario(1_000_000, vec![]),
            &mut simulation,
        )
        .unwrap();
        let snapshot = simulation.snapshot();

        assert!(snapshot.sensor_samples > 400);
        assert_eq!(snapshot.sensor_samples, snapshot.delivered_observations);
        assert!(snapshot.runtime_steps > 400);
        assert_eq!(snapshot.skipped_unready_observations, 1);
        assert!(snapshot.authorized_commits > 400);
        assert_eq!(snapshot.operating_state, OperatingState::Balancing);
        assert!(snapshot.runtime_faults.is_empty());
        assert_eq!(snapshot.sensor_timing, SensorTimingHealth::Healthy);
        assert!(snapshot.truth.state.pitch_rad.abs() < 0.005);
        assert!(snapshot.truth.state.roll_rad.abs() < 0.005);

        let estimate = snapshot.latest_estimate.unwrap();
        assert!((estimate.pitch_rad - snapshot.truth.state.pitch_rad).abs() < 0.01);
        assert!((estimate.roll_rad - snapshot.truth.state.roll_rad).abs() < 0.01);
        assert!(
            snapshot.truth.state.pitch_rad.abs() < initial.pitch_rad.abs()
                && snapshot.truth.state.roll_rad.abs() < initial.roll_rad.abs()
        );

        let summary: serde_json::Value = serde_json::from_slice(&artifacts.summary_json).unwrap();
        assert_eq!(summary["pass"], true);
    }

    #[test]
    fn missed_runtime_discards_that_observation_and_never_replays_it() {
        let mut simulation = simulation(ReducedBalanceState {
            pitch_rad: 0.01,
            ..ReducedBalanceState::default()
        });
        let scenario = scenario(20_000, vec![10_000]);
        let artifacts =
            run_scenario_with_execution(&context(), &scenario, &mut simulation).unwrap();
        let snapshot = simulation.snapshot();

        assert_eq!(snapshot.missed_runtime_opportunities, 1);
        assert_eq!(snapshot.observations_discarded_on_miss, 1);
        assert_eq!(
            snapshot.runtime_steps + snapshot.skipped_unready_observations,
            scenario.runtime_opportunity_count() - 1
        );

        let summary: serde_json::Value = serde_json::from_slice(&artifacts.summary_json).unwrap();
        assert_eq!(summary["missed_control_opportunities"], 1);
        assert_eq!(summary["pass"], true);
    }
}
