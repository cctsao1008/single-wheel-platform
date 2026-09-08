use std::env;
use std::error::Error;
use std::fs;
use std::io;
use std::path::PathBuf;

use serde::{Deserialize, Serialize};
use swp_actuation_interface::ActuationSink;
use swp_actuator_model::{
    ActuatorPairModel, ActuatorPairOperatingPoint, ActuatorParameters, StaticActuatorModel,
};
use swp_dynamics_model::{PlantParameters, ReducedBalanceState};
use swp_mpu6050::{AccelRange, GyroRange};
use swp_robot_domain::{GeneralizedDemand, StateValidity, TorqueNm};
use swp_runtime_state::{
    AuthorityContext, OperatingState, ReactionWheelAuthority, RuntimeAuthority, SensorTimingHealth,
};
use swp_sitl::PhysicalTimeAdvance;
use swp_sitl::simulation_world::{SimulationWorld, SimulationWorldConfig};
use swp_sitl::virtual_time::VirtualTime;

#[derive(Debug)]
struct Cli {
    fixture: PathBuf,
    output: PathBuf,
}

#[derive(Clone, Copy, Debug, Deserialize)]
struct PlantFixture {
    gravity_m_per_s2: f32,
    body_mass_kg: f32,
    body_com_height_m: f32,
    body_inertia_roll_kg_m2: f32,
    body_inertia_pitch_kg_m2: f32,
    body_inertia_yaw_kg_m2: f32,
    drive_wheel_mass_kg: f32,
    drive_wheel_radius_m: f32,
    drive_wheel_spin_inertia_kg_m2: f32,
    reaction_wheel_mass_kg: f32,
    reaction_wheel_com_height_m: f32,
    reaction_wheel_spin_inertia_kg_m2: f32,
    reaction_wheel_transverse_inertia_kg_m2: f32,
}

impl From<PlantFixture> for PlantParameters {
    fn from(value: PlantFixture) -> Self {
        Self {
            gravity_m_per_s2: value.gravity_m_per_s2,
            body_mass_kg: value.body_mass_kg,
            body_com_height_m: value.body_com_height_m,
            body_inertia_roll_kg_m2: value.body_inertia_roll_kg_m2,
            body_inertia_pitch_kg_m2: value.body_inertia_pitch_kg_m2,
            body_inertia_yaw_kg_m2: value.body_inertia_yaw_kg_m2,
            drive_wheel_mass_kg: value.drive_wheel_mass_kg,
            drive_wheel_radius_m: value.drive_wheel_radius_m,
            drive_wheel_spin_inertia_kg_m2: value.drive_wheel_spin_inertia_kg_m2,
            reaction_wheel_mass_kg: value.reaction_wheel_mass_kg,
            reaction_wheel_com_height_m: value.reaction_wheel_com_height_m,
            reaction_wheel_spin_inertia_kg_m2: value.reaction_wheel_spin_inertia_kg_m2,
            reaction_wheel_transverse_inertia_kg_m2: value.reaction_wheel_transverse_inertia_kg_m2,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
struct InputStep {
    at_us: u64,
    drive_torque_nm: f32,
    reaction_wheel_torque_nm: f32,
}

#[derive(Debug, Deserialize)]
struct Fixture {
    provenance: String,
    sample_period_us: u64,
    duration_us: u64,
    plant: PlantFixture,
    initial_state: [f32; 8],
    input_profile: Vec<InputStep>,
}

#[derive(Debug, Serialize)]
struct TraceSample {
    time_us: u64,
    state: [f32; 8],
    applied_input: [f32; 2],
}

#[derive(Debug, Serialize)]
struct TraceOutput<'a> {
    schema_version: u32,
    provenance: &'a str,
    sample_period_us: u64,
    duration_us: u64,
    samples: Vec<TraceSample>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let cli = parse_cli()?;
    let fixture: Fixture = serde_json::from_str(&fs::read_to_string(&cli.fixture)?)?;
    validate_fixture(&fixture)?;

    let plant: PlantParameters = fixture.plant.into();
    let mut world = SimulationWorld::new(
        SimulationWorldConfig {
            plant,
            // The reference trace does not sample sensors. These explicit values
            // only satisfy SimulationWorld's complete configuration contract.
            accel_range: AccelRange::G2,
            gyro_range: GyroRange::Dps250,
            drive_encoder_counts_per_revolution: 4_096,
            reaction_encoder_counts_per_revolution: 2_048,
            imu_temperature_celsius: 25.0,
        },
        ReducedBalanceState {
            forward_position_m: fixture.initial_state[0],
            forward_velocity_m_per_s: fixture.initial_state[1],
            pitch_rad: fixture.initial_state[2],
            pitch_rate_rad_per_s: fixture.initial_state[3],
            roll_rad: fixture.initial_state[4],
            roll_rate_rad_per_s: fixture.initial_state[5],
            reaction_wheel_rate_rad_per_s: fixture.initial_state[6],
        },
        fixture.initial_state[7],
    )?;
    let actuator_model = correlation_actuator_model()?;

    let sample_count = fixture.duration_us / fixture.sample_period_us + 1;
    let mut samples = Vec::with_capacity(usize::try_from(sample_count)?);
    let mut profile_index = 0_usize;
    let mut at_us = 0_u64;

    loop {
        while profile_index < fixture.input_profile.len()
            && fixture.input_profile[profile_index].at_us == at_us
        {
            apply_physical_input(
                &mut world,
                actuator_model,
                plant,
                fixture.input_profile[profile_index],
            )?;
            profile_index += 1;
        }

        let truth = world.truth();
        let state = truth.state.as_vector();
        samples.push(TraceSample {
            time_us: at_us,
            state: [
                state[0],
                state[1],
                state[2],
                state[3],
                state[4],
                state[5],
                state[6],
                truth.reaction_wheel_relative_angle_rad,
            ],
            applied_input: truth.applied_input.as_vector(),
        });

        if at_us == fixture.duration_us {
            break;
        }

        let next_us = at_us
            .checked_add(fixture.sample_period_us)
            .ok_or_else(|| invalid_data("reference trace time overflow"))?;
        PhysicalTimeAdvance::advance(
            &mut world,
            VirtualTime::from_micros(at_us),
            VirtualTime::from_micros(next_us),
        )?;
        at_us = next_us;
    }

    if profile_index != fixture.input_profile.len() {
        return Err(invalid_data("not all input profile events were consumed").into());
    }

    let output = TraceOutput {
        schema_version: 1,
        provenance: &fixture.provenance,
        sample_period_us: fixture.sample_period_us,
        duration_us: fixture.duration_us,
        samples,
    };
    fs::write(&cli.output, serde_json::to_vec_pretty(&output)?)?;
    println!(
        "wrote SimulationWorld truth trace to {}",
        cli.output.display()
    );
    Ok(())
}

fn correlation_actuator_model() -> Result<ActuatorPairModel, Box<dyn Error>> {
    let parameters = ActuatorParameters::new(1.0, 0.0, 0.0, 0.0, 1.0e-3)
        .ok_or_else(|| invalid_data("invalid synthetic correlation actuator parameters"))?;
    let model = StaticActuatorModel::new(parameters)
        .ok_or_else(|| invalid_data("failed to construct correlation actuator model"))?;
    Ok(ActuatorPairModel {
        drive: model,
        reaction: model,
    })
}

fn apply_physical_input(
    world: &mut SimulationWorld,
    actuator_model: ActuatorPairModel,
    plant: PlantParameters,
    input: InputStep,
) -> Result<(), Box<dyn Error>> {
    let state = world.truth().state;
    let commands = actuator_model
        .command_for_demand(
            GeneralizedDemand {
                drive_wheel_torque: TorqueNm(input.drive_torque_nm),
                reaction_wheel_torque: TorqueNm(input.reaction_wheel_torque_nm),
            },
            ActuatorPairOperatingPoint {
                drive_speed_rad_per_s: state.forward_velocity_m_per_s / plant.drive_wheel_radius_m
                    - state.pitch_rate_rad_per_s,
                reaction_speed_rad_per_s: state.reaction_wheel_rate_rad_per_s,
            },
        )
        .map_err(|error| invalid_data(format!("correlation actuator model failed: {error:?}")))?;

    if commands.drive.saturated || commands.reaction.saturated {
        return Err(invalid_data("correlation input exceeds synthetic actuator authority").into());
    }

    let outcome = RuntimeAuthority::evaluate(
        AuthorityContext {
            operating_state: OperatingState::Balancing,
            timing: SensorTimingHealth::Healthy,
            estimate_validity: StateValidity::Valid,
            reaction_wheel_authority: ReactionWheelAuthority::Nominal,
        },
        commands,
    );
    let authorized = outcome
        .authorized()
        .ok_or_else(|| invalid_data("correlation input was denied by RuntimeAuthority"))?;
    world.apply_authorized(authorized)?;
    Ok(())
}

fn validate_fixture(fixture: &Fixture) -> Result<(), Box<dyn Error>> {
    if fixture.sample_period_us == 0
        || fixture.duration_us % fixture.sample_period_us != 0
        || !fixture.initial_state.iter().all(|value| value.is_finite())
    {
        return Err(invalid_data("invalid correlation timing or initial state").into());
    }

    let mut previous = None;
    for step in &fixture.input_profile {
        if step.at_us > fixture.duration_us
            || step.at_us % fixture.sample_period_us != 0
            || previous.is_some_and(|at_us| step.at_us <= at_us)
            || !step.drive_torque_nm.is_finite()
            || !step.reaction_wheel_torque_nm.is_finite()
        {
            return Err(
                invalid_data("input profile must be finite, ordered, and sample-aligned").into(),
            );
        }
        previous = Some(step.at_us);
    }
    Ok(())
}

fn parse_cli() -> Result<Cli, Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let mut fixture = None;
    let mut output = None;

    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--fixture" => {
                fixture = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| invalid_data("--fixture requires a path"))?,
                ));
            }
            "--output" => {
                output = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| invalid_data("--output requires a path"))?,
                ));
            }
            "--help" | "-h" => {
                println!(
                    "usage: plant_reference_trace --fixture <fixture.json> --output <trace.json>"
                );
                std::process::exit(0);
            }
            other => return Err(invalid_data(format!("unsupported argument: {other}")).into()),
        }
    }

    Ok(Cli {
        fixture: fixture.ok_or_else(|| invalid_data("missing --fixture"))?,
        output: output.ok_or_else(|| invalid_data("missing --output"))?,
    })
}

fn invalid_data(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}
