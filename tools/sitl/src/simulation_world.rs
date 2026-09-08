use std::error::Error;
use std::fmt::{Display, Formatter};

use swp_actuation_interface::ActuationSink;
use swp_dynamics_model::{
    ContinuousLinearPlant, PlantParameters, ReducedBalanceState, ReferencePlantInput,
    linearize_stationary_upright,
};
use swp_mpu6050::{AccelRange, GyroRange, STANDARD_GRAVITY_MPS2};
use swp_plant_observation::{
    AcquisitionStatus, MeasurementQuality, RawBatteryObservation, RawEncoderObservation,
    RawImuObservation, RawObservation, TimestampEvidence,
};
use swp_runtime_state::AuthorizedActuation;

use crate::PhysicalTimeAdvance;
use crate::virtual_time::VirtualTime;

const MAX_INTEGRATION_STEP_US: u64 = 1_000;
const TEMPERATURE_OFFSET_C: f32 = 36.53;
const TEMPERATURE_LSB_PER_C: f32 = 340.0;
const COUNTER_MODULUS: i64 = 1 << 16;

/// Explicit physical/sensor configuration for the simulated world.
///
/// There are intentionally no reference-platform defaults here. The repository
/// parameter file still contains unknown physical values, so callers must supply
/// a complete, evidenced or deliberately synthetic configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimulationWorldConfig {
    pub plant: PlantParameters,
    pub accel_range: AccelRange,
    pub gyro_range: GyroRange,
    pub drive_encoder_counts_per_revolution: u32,
    pub reaction_encoder_counts_per_revolution: u32,
    pub imu_temperature_celsius: f32,
}

/// Physical truth retained by the simulation side only.
///
/// Production runtime code must receive `RawObservation`, never this structure.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimulationTruth {
    pub state: ReducedBalanceState,
    pub reaction_wheel_relative_angle_rad: f32,
    pub applied_input: ReferencePlantInput,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SimulationWorldError {
    InvalidPlantParameters,
    InvalidEncoderResolution,
    NonFiniteInitialState,
    NonFiniteReactionWheelAngle,
    NonFiniteTemperature,
    NonFiniteDynamics,
    NonFiniteSensorOutput,
    NonFiniteActuation,
    TimeReversal {
        from_us: u64,
        to_us: u64,
    },
    TimeDiscontinuity {
        expected_us: u64,
        actual_us: u64,
    },
    SampleTimeMismatch {
        world_time_us: u64,
        sample_time_us: u64,
    },
    SampleIndexOverflow,
}

impl Display for SimulationWorldError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidPlantParameters => {
                write!(formatter, "invalid simulation plant parameters")
            }
            Self::InvalidEncoderResolution => {
                write!(formatter, "encoder counts per revolution must be nonzero")
            }
            Self::NonFiniteInitialState => {
                write!(formatter, "simulation initial state is non-finite")
            }
            Self::NonFiniteReactionWheelAngle => {
                write!(formatter, "simulation reaction-wheel angle is non-finite")
            }
            Self::NonFiniteTemperature => {
                write!(formatter, "simulation IMU temperature is non-finite")
            }
            Self::NonFiniteDynamics => write!(formatter, "simulation dynamics became non-finite"),
            Self::NonFiniteSensorOutput => {
                write!(formatter, "simulation sensor output became non-finite")
            }
            Self::NonFiniteActuation => write!(formatter, "authorized actuation is non-finite"),
            Self::TimeReversal { from_us, to_us } => write!(
                formatter,
                "simulation time cannot move backward from {from_us} us to {to_us} us"
            ),
            Self::TimeDiscontinuity {
                expected_us,
                actual_us,
            } => write!(
                formatter,
                "simulation advance started at {actual_us} us but world time is {expected_us} us"
            ),
            Self::SampleTimeMismatch {
                world_time_us,
                sample_time_us,
            } => write!(
                formatter,
                "sensor sample requested at {sample_time_us} us while world time is {world_time_us} us"
            ),
            Self::SampleIndexOverflow => write!(formatter, "simulation sample index exhausted"),
        }
    }
}

impl Error for SimulationWorldError {}

/// Minimal closed physical side of SITL.
///
/// The world currently uses the repository's stationary-upright reduced plant
/// for time evolution, ideal torque application, and deterministic device-like
/// sensing. Sensor frame equals the canonical body frame. Sensor bias, noise,
/// latency, slip, motor electrical dynamics, and battery physics are not
/// invented here.
pub struct SimulationWorld {
    config: SimulationWorldConfig,
    plant: ContinuousLinearPlant,
    truth: SimulationTruth,
    current_time: VirtualTime,
    sample_index: u32,
}

impl SimulationWorld {
    pub fn new(
        config: SimulationWorldConfig,
        initial_state: ReducedBalanceState,
        initial_reaction_wheel_relative_angle_rad: f32,
    ) -> Result<Self, SimulationWorldError> {
        let plant = linearize_stationary_upright(config.plant)
            .ok_or(SimulationWorldError::InvalidPlantParameters)?;

        if config.drive_encoder_counts_per_revolution == 0
            || config.reaction_encoder_counts_per_revolution == 0
        {
            return Err(SimulationWorldError::InvalidEncoderResolution);
        }
        if !state_is_finite(initial_state) {
            return Err(SimulationWorldError::NonFiniteInitialState);
        }
        if !initial_reaction_wheel_relative_angle_rad.is_finite() {
            return Err(SimulationWorldError::NonFiniteReactionWheelAngle);
        }
        if !config.imu_temperature_celsius.is_finite() {
            return Err(SimulationWorldError::NonFiniteTemperature);
        }

        Ok(Self {
            config,
            plant,
            truth: SimulationTruth {
                state: initial_state,
                reaction_wheel_relative_angle_rad: initial_reaction_wheel_relative_angle_rad,
                applied_input: ReferencePlantInput::default(),
            },
            current_time: VirtualTime::ZERO,
            sample_index: 0,
        })
    }

    pub const fn current_time(&self) -> VirtualTime {
        self.current_time
    }

    pub const fn truth(&self) -> SimulationTruth {
        self.truth
    }

    /// Produce one deterministic, zero-latency device-like acquisition batch.
    ///
    /// The virtual MPU6050 frame is the canonical body frame. Encoder counters
    /// wrap exactly as the 16-bit firmware observation type does. Battery
    /// physics is intentionally absent, so the battery channel remains
    /// unavailable rather than fabricating a voltage.
    pub fn sample_raw_observation(
        &mut self,
        at: VirtualTime,
    ) -> Result<RawObservation, SimulationWorldError> {
        if at != self.current_time {
            return Err(SimulationWorldError::SampleTimeMismatch {
                world_time_us: self.current_time.as_micros(),
                sample_time_us: at.as_micros(),
            });
        }

        let derivative = self.derivative(self.state_vector())?;
        let specific_force = body_specific_force(
            self.truth.state,
            derivative[1],
            self.config.plant.gravity_m_per_s2,
        );
        let angular_rate = body_angular_rate(self.truth.state);

        let (accel_raw, accel_saturated) =
            quantize_acceleration(specific_force, self.config.accel_range)?;
        let (gyro_raw, gyro_saturated) =
            quantize_angular_rate(angular_rate, self.config.gyro_range)?;
        let (temperature_raw, temperature_saturated) = quantize_i16(
            (self.config.imu_temperature_celsius - TEMPERATURE_OFFSET_C) * TEMPERATURE_LSB_PER_C,
        )?;

        let mut imu_quality = healthy_quality();
        if accel_saturated || gyro_saturated || temperature_saturated {
            imu_quality |= MeasurementQuality::SATURATED;
        }

        let drive_angle_rad = self.truth.state.forward_position_m
            / self.config.plant.drive_wheel_radius_m
            - self.truth.state.pitch_rad;
        let drive_count = encoder_count(
            drive_angle_rad,
            self.config.drive_encoder_counts_per_revolution,
        )?;
        let reaction_count = encoder_count(
            self.truth.reaction_wheel_relative_angle_rad,
            self.config.reaction_encoder_counts_per_revolution,
        )?;

        let sample_index = self.sample_index;
        self.sample_index = self
            .sample_index
            .checked_add(1)
            .ok_or(SimulationWorldError::SampleIndexOverflow)?;

        let timestamp = TimestampEvidence::Known(at.as_micros());

        Ok(RawObservation {
            sample_index,
            acquisition_started_us: at.as_micros(),
            acquisition_completed_us: at.as_micros(),
            imu: RawImuObservation {
                source_sample_at_us: timestamp,
                read_started_at_us: timestamp,
                read_completed_at_us: timestamp,
                accel_raw,
                temperature_raw,
                gyro_raw,
                quality: imu_quality,
            },
            encoders: [
                RawEncoderObservation {
                    captured_at_us: timestamp,
                    count: drive_count,
                    quality: healthy_quality(),
                },
                RawEncoderObservation {
                    captured_at_us: timestamp,
                    count: reaction_count,
                    quality: healthy_quality(),
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

    fn advance_interval(
        &mut self,
        from: VirtualTime,
        to: VirtualTime,
    ) -> Result<(), SimulationWorldError> {
        if from != self.current_time {
            return Err(SimulationWorldError::TimeDiscontinuity {
                expected_us: self.current_time.as_micros(),
                actual_us: from.as_micros(),
            });
        }
        if to < from {
            return Err(SimulationWorldError::TimeReversal {
                from_us: from.as_micros(),
                to_us: to.as_micros(),
            });
        }

        let mut remaining_us = to.as_micros() - from.as_micros();
        while remaining_us > 0 {
            let step_us = remaining_us.min(MAX_INTEGRATION_STEP_US);
            self.integrate_rk4(step_us as f32 * 1.0e-6)?;
            remaining_us -= step_us;
        }

        self.current_time = to;
        Ok(())
    }

    fn integrate_rk4(&mut self, dt_s: f32) -> Result<(), SimulationWorldError> {
        let x0 = self.state_vector();
        let k1 = self.derivative(x0)?;
        let k2 = self.derivative(add_scaled(x0, k1, 0.5 * dt_s))?;
        let k3 = self.derivative(add_scaled(x0, k2, 0.5 * dt_s))?;
        let k4 = self.derivative(add_scaled(x0, k3, dt_s))?;

        let mut next = x0;
        for index in 0..next.len() {
            next[index] += dt_s / 6.0 * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index]);
        }

        if !next.iter().all(|value| value.is_finite()) {
            return Err(SimulationWorldError::NonFiniteDynamics);
        }

        self.truth.state = ReducedBalanceState {
            forward_position_m: next[0],
            forward_velocity_m_per_s: next[1],
            pitch_rad: next[2],
            pitch_rate_rad_per_s: next[3],
            roll_rad: next[4],
            roll_rate_rad_per_s: next[5],
            reaction_wheel_rate_rad_per_s: next[6],
        };
        self.truth.reaction_wheel_relative_angle_rad = next[7];
        Ok(())
    }

    fn state_vector(&self) -> [f32; 8] {
        let state = self.truth.state.as_vector();
        [
            state[0],
            state[1],
            state[2],
            state[3],
            state[4],
            state[5],
            state[6],
            self.truth.reaction_wheel_relative_angle_rad,
        ]
    }

    fn derivative(&self, state: [f32; 8]) -> Result<[f32; 8], SimulationWorldError> {
        let input = self.truth.applied_input.as_vector();
        let mut derivative = [0.0_f32; 8];

        for row in 0..7 {
            for (column, value) in state[..7].iter().enumerate() {
                derivative[row] += self.plant.a[row][column] * value;
            }
            derivative[row] += self.plant.b[row][0] * input[0];
            derivative[row] += self.plant.b[row][1] * input[1];
        }
        derivative[7] = state[6];

        derivative
            .iter()
            .all(|value| value.is_finite())
            .then_some(derivative)
            .ok_or(SimulationWorldError::NonFiniteDynamics)
    }
}

impl PhysicalTimeAdvance for SimulationWorld {
    fn advance(
        &mut self,
        from: VirtualTime,
        to: VirtualTime,
    ) -> Result<(), Box<dyn Error + 'static>> {
        self.advance_interval(from, to)
            .map_err(|error| Box::new(error) as Box<dyn Error + 'static>)
    }
}

impl ActuationSink for SimulationWorld {
    type Error = SimulationWorldError;

    fn apply_authorized(&mut self, actuation: AuthorizedActuation) -> Result<(), Self::Error> {
        let commands = actuation.commands();
        let input = ReferencePlantInput {
            drive_torque_nm: commands.drive.predicted_torque_nm.0,
            reaction_wheel_torque_nm: commands.reaction.predicted_torque_nm.0,
        };
        if !input.as_vector().iter().all(|value| value.is_finite()) {
            return Err(SimulationWorldError::NonFiniteActuation);
        }
        self.truth.applied_input = input;
        Ok(())
    }

    fn revoke(&mut self) -> Result<(), Self::Error> {
        self.truth.applied_input = ReferencePlantInput::default();
        Ok(())
    }
}

fn state_is_finite(state: ReducedBalanceState) -> bool {
    state.as_vector().iter().all(|value| value.is_finite())
}

fn add_scaled(base: [f32; 8], derivative: [f32; 8], scale: f32) -> [f32; 8] {
    let mut result = base;
    for index in 0..result.len() {
        result[index] += derivative[index] * scale;
    }
    result
}

fn healthy_quality() -> MeasurementQuality {
    MeasurementQuality::AVAILABLE
        | MeasurementQuality::IO_OK
        | MeasurementQuality::TIMING_VALID
        | MeasurementQuality::FRESHNESS_VERIFIED
}

fn body_specific_force(
    state: ReducedBalanceState,
    forward_acceleration_m_per_s2: f32,
    gravity_m_per_s2: f32,
) -> [f32; 3] {
    let (sin_theta, cos_theta) = state.pitch_rad.sin_cos();
    let (sin_phi, cos_phi) = state.roll_rad.sin_cos();

    // R = R_y(theta) R_x(phi). Accelerometer output is specific force.
    let body_x = cos_theta * forward_acceleration_m_per_s2 - sin_theta * gravity_m_per_s2;
    let after_pitch_z = sin_theta * forward_acceleration_m_per_s2 + cos_theta * gravity_m_per_s2;

    [body_x, sin_phi * after_pitch_z, cos_phi * after_pitch_z]
}

fn body_angular_rate(state: ReducedBalanceState) -> [f32; 3] {
    let (sin_phi, cos_phi) = state.roll_rad.sin_cos();
    [
        state.roll_rate_rad_per_s,
        state.pitch_rate_rad_per_s * cos_phi,
        -state.pitch_rate_rad_per_s * sin_phi,
    ]
}

fn quantize_acceleration(
    acceleration_m_per_s2: [f32; 3],
    range: AccelRange,
) -> Result<([i16; 3], bool), SimulationWorldError> {
    let scale = range.lsb_per_g() / STANDARD_GRAVITY_MPS2;
    quantize_vector(acceleration_m_per_s2.map(|value| value * scale))
}

fn quantize_angular_rate(
    angular_rate_rad_per_s: [f32; 3],
    range: GyroRange,
) -> Result<([i16; 3], bool), SimulationWorldError> {
    quantize_vector(angular_rate_rad_per_s.map(|value| value.to_degrees() * range.lsb_per_dps()))
}

fn quantize_vector(values: [f32; 3]) -> Result<([i16; 3], bool), SimulationWorldError> {
    let mut output = [0_i16; 3];
    let mut saturated = false;
    for (index, value) in values.into_iter().enumerate() {
        let (quantized, did_saturate) = quantize_i16(value)?;
        output[index] = quantized;
        saturated |= did_saturate;
    }
    Ok((output, saturated))
}

fn quantize_i16(value: f32) -> Result<(i16, bool), SimulationWorldError> {
    if !value.is_finite() {
        return Err(SimulationWorldError::NonFiniteSensorOutput);
    }

    let rounded = value.round();
    let saturated = rounded < i16::MIN as f32 || rounded > i16::MAX as f32;
    let bounded = rounded.clamp(i16::MIN as f32, i16::MAX as f32);
    Ok((bounded as i16, saturated))
}

fn encoder_count(angle_rad: f32, counts_per_revolution: u32) -> Result<u16, SimulationWorldError> {
    if !angle_rad.is_finite() {
        return Err(SimulationWorldError::NonFiniteSensorOutput);
    }

    let counts = angle_rad / std::f32::consts::TAU * counts_per_revolution as f32;
    if !counts.is_finite() {
        return Err(SimulationWorldError::NonFiniteSensorOutput);
    }

    let wrapped = (counts.round() as i64).rem_euclid(COUNTER_MODULUS);
    Ok(wrapped as u16)
}

#[cfg(test)]
mod tests {
    use super::*;
    use swp_actuator_model::{ActuatorPairCommand, BoundedActuatorCommand};
    use swp_robot_domain::{NormalizedCommand, StateValidity, TorqueNm};
    use swp_runtime_state::{
        AuthorityContext, OperatingState, ReactionWheelAuthority, RuntimeAuthority,
        SensorTimingHealth,
    };

    fn test_parameters() -> PlantParameters {
        // Synthetic test fixture only. These are not reference-platform facts.
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

    fn test_config() -> SimulationWorldConfig {
        SimulationWorldConfig {
            plant: test_parameters(),
            accel_range: AccelRange::G4,
            gyro_range: GyroRange::Dps1000,
            drive_encoder_counts_per_revolution: 1_000,
            reaction_encoder_counts_per_revolution: 1_000,
            imu_temperature_celsius: 36.53,
        }
    }

    fn world(initial_state: ReducedBalanceState) -> SimulationWorld {
        SimulationWorld::new(test_config(), initial_state, 0.0).unwrap()
    }

    fn authorized_actuation(drive_torque_nm: f32, reaction_torque_nm: f32) -> AuthorizedActuation {
        let command = |torque_nm| BoundedActuatorCommand {
            command: NormalizedCommand::new(0.25).unwrap(),
            saturated: false,
            predicted_torque_nm: TorqueNm(torque_nm),
        };
        let outcome = RuntimeAuthority::evaluate(
            AuthorityContext {
                operating_state: OperatingState::Balancing,
                timing: SensorTimingHealth::Healthy,
                estimate_validity: StateValidity::Valid,
                reaction_wheel_authority: ReactionWheelAuthority::Nominal,
            },
            ActuatorPairCommand {
                drive: command(drive_torque_nm),
                reaction: command(reaction_torque_nm),
            },
        );
        outcome.authorized().unwrap()
    }

    #[test]
    fn stationary_upright_generates_device_like_gravity_and_zero_rate() {
        let mut world = world(ReducedBalanceState::default());
        let raw = world.sample_raw_observation(VirtualTime::ZERO).unwrap();

        assert_eq!(raw.sample_index, 0);
        assert_eq!(raw.imu.accel_raw, [0, 0, 8_192]);
        assert_eq!(raw.imu.gyro_raw, [0, 0, 0]);
        assert_eq!(raw.encoders[0].count, 0);
        assert_eq!(raw.encoders[1].count, 0);
        assert!(raw.imu.quality.contains(MeasurementQuality::AVAILABLE));
        assert!(
            raw.imu
                .quality
                .contains(MeasurementQuality::FRESHNESS_VERIFIED)
        );
        assert_eq!(raw.battery.quality, MeasurementQuality::NONE);
    }

    #[test]
    fn pitch_and_roll_sensor_signs_follow_body_frame_contract() {
        let state = ReducedBalanceState {
            pitch_rad: 0.05,
            roll_rad: 0.04,
            ..ReducedBalanceState::default()
        };
        let mut world = world(state);
        let raw = world.sample_raw_observation(VirtualTime::ZERO).unwrap();

        assert!(raw.imu.accel_raw[0] < 0);
        assert!(raw.imu.accel_raw[1] > 0);
    }

    #[test]
    fn open_loop_upright_perturbations_diverge() {
        let initial_pitch = 0.01;
        let initial_roll = -0.01;
        let mut world = world(ReducedBalanceState {
            pitch_rad: initial_pitch,
            roll_rad: initial_roll,
            ..ReducedBalanceState::default()
        });

        world
            .advance_interval(VirtualTime::ZERO, VirtualTime::from_micros(500_000))
            .unwrap();

        let truth = world.truth();
        assert!(truth.state.pitch_rad.abs() > initial_pitch);
        assert!(truth.state.roll_rad.abs() > initial_roll.abs());
    }

    #[test]
    fn authorized_actuation_is_the_only_nonzero_input_boundary() {
        let mut world = world(ReducedBalanceState::default());
        let authorized = authorized_actuation(0.2, -0.1);

        ActuationSink::apply_authorized(&mut world, authorized).unwrap();
        assert_eq!(
            world.truth().applied_input,
            ReferencePlantInput {
                drive_torque_nm: 0.2,
                reaction_wheel_torque_nm: -0.1,
            }
        );

        ActuationSink::revoke(&mut world).unwrap();
        assert_eq!(world.truth().applied_input, ReferencePlantInput::default());
    }

    #[test]
    fn world_rejects_noncontiguous_time_advance_and_off_time_sampling() {
        let mut world = world(ReducedBalanceState::default());

        assert!(matches!(
            world.advance_interval(
                VirtualTime::from_micros(1_000),
                VirtualTime::from_micros(2_000)
            ),
            Err(SimulationWorldError::TimeDiscontinuity { .. })
        ));
        assert!(matches!(
            world.sample_raw_observation(VirtualTime::from_micros(1)),
            Err(SimulationWorldError::SampleTimeMismatch { .. })
        ));
    }
}
