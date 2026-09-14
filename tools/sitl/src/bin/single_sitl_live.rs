use std::error::Error;
use std::io::{self, Write};

use serde_json::{Value, json};
use swp_dynamics_model::ReducedBalanceState;
use swp_runtime_state::{ActuationAuthority, OperatingState, SensorTimingHealth};
use swp_sitl::scheduler::{EventKind, ScheduledEvent};
use swp_sitl::virtual_time::VirtualTime;
use swp_sitl::{PhysicalTimeAdvance, ScenarioExecution};

#[path = "single_sitl_live/fixture.rs"]
mod fixture;

fn main() {
    if let Err(error) = run() {
        eprintln!("single-sitl-live: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut simulation = fixture::build_live_simulation(fixture::default_initial_state());
    let stdout = io::stdout();
    let mut out = stdout.lock();

    writeln!(
        out,
        "{}",
        json!({
            "type": "meta",
            "schema": 1,
            "dialect": "production-semantic-v1",
            "mode": "closed_loop_production_path",
            "source": {
                "kind": "single-sitl-live",
                "model_class": "synthetic reduced stationary-upright plant",
                "backend": "persistent production Rust SITL semantic path",
                "inner_hz": fixture::LIVE_INNER_HZ,
                "outer_hz": fixture::LIVE_OUTER_HZ,
                "provenance": "explicit synthetic integration fixture; not accepted ONE V2 physical evidence",
                "scope": "simulation evidence only; no physical actuator authority"
            }
        })
    )?;
    out.flush()?;

    let mut previous = VirtualTime::ZERO;
    let mut at = VirtualTime::ZERO;
    let mut insertion_sequence = 0_u64;
    let mut first = true;

    loop {
        if !first {
            simulation.advance(previous, at)?;
        } else {
            dispatch(&mut simulation, at, &mut insertion_sequence, EventKind::ScenarioStart)?;
        }

        dispatch(&mut simulation, at, &mut insertion_sequence, EventKind::SensorSample)?;
        dispatch(
            &mut simulation,
            at,
            &mut insertion_sequence,
            EventKind::ObservationDelivery,
        )?;
        dispatch(
            &mut simulation,
            at,
            &mut insertion_sequence,
            EventKind::ProductionRuntime,
        )?;
        dispatch(
            &mut simulation,
            at,
            &mut insertion_sequence,
            EventKind::ActuationCommit,
        )?;

        let snapshot = simulation.snapshot();
        if snapshot.runtime_steps > 0 {
            if let (Some(estimate), Some(decision)) =
                (snapshot.latest_estimate, snapshot.latest_authority)
            {
                let sample_index = snapshot.sensor_samples.saturating_sub(1);
                writeln!(
                    out,
                    "{}",
                    json!({
                        "type": "sample",
                        "record": {
                            "mode": "closed_loop_production_path",
                            "source": {
                                "kind": "single-sitl-live",
                                "backend": "persistent production Rust SITL semantic path",
                                "provenance": "synthetic integration fixture"
                            },
                            "time_s": at.as_micros() as f64 * 1.0e-6,
                            "raw_device_observation": {
                                "schema": 1,
                                "mapping_id": "sitl-simulation-world-v1",
                                "sample_index": sample_index,
                                "timestamp_us": at.as_micros()
                            },
                            "sitl_evidence_truth": truth_json(snapshot.truth.state, snapshot.truth.reaction_wheel_relative_angle_rad),
                            "production": {
                                "schema": 1,
                                "sample_index": sample_index,
                                "operating_state": operating_state_name(snapshot.operating_state),
                                "runtime_fault_bits": snapshot.runtime_faults.bits(),
                                "sensor_timing": sensor_timing_name(snapshot.sensor_timing),
                                "estimate_validity": Value::Null,
                                "authority": authority_name(decision.authority),
                                "authority_reason_bits": decision.reasons.bits(),
                                "constrained": decision.constrained,
                                "hold_integrator": decision.hold_integrator,
                                "actuation": if decision.closed_loop_authorized() { "apply" } else { "revoke" },
                                "drive_torque_nm": snapshot.truth.applied_input.drive_torque_nm,
                                "reaction_torque_nm": snapshot.truth.applied_input.reaction_wheel_torque_nm,
                                "estimate": state_json(estimate),
                                "reference": state_json(snapshot.reference),
                                "runtime_steps": snapshot.runtime_steps,
                                "skipped_unready_observations": snapshot.skipped_unready_observations
                            },
                            "authorized_drive_torque_nm": snapshot.truth.applied_input.drive_torque_nm,
                            "authorized_reaction_torque_nm": snapshot.truth.applied_input.reaction_wheel_torque_nm
                        }
                    })
                )?;
                out.flush()?;
            }
        }

        first = false;
        previous = at;
        at = at
            .checked_add_micros(fixture::LIVE_PERIOD_US)
            .ok_or_else(|| io::Error::other("live SITL virtual time exhausted"))?;
    }
}

fn dispatch(
    simulation: &mut swp_sitl::closed_loop::ClosedLoopSimulation,
    at: VirtualTime,
    insertion_sequence: &mut u64,
    kind: EventKind,
) -> Result<(), Box<dyn Error>> {
    let event = ScheduledEvent {
        at,
        phase: kind.phase(),
        insertion_sequence: *insertion_sequence,
        kind,
    };
    *insertion_sequence = insertion_sequence
        .checked_add(1)
        .ok_or_else(|| io::Error::other("live SITL insertion sequence exhausted"))?;
    simulation.dispatch_event(event)?;
    Ok(())
}

fn state_json(state: ReducedBalanceState) -> Value {
    json!({
        "forward_position_m": state.forward_position_m,
        "forward_velocity_m_per_s": state.forward_velocity_m_per_s,
        "body_pitch_rad": state.pitch_rad,
        "body_pitch_rate_rad_per_s": state.pitch_rate_rad_per_s,
        "body_roll_rad": state.roll_rad,
        "body_roll_rate_rad_per_s": state.roll_rate_rad_per_s,
        "reaction_rate_rad_per_s": state.reaction_wheel_rate_rad_per_s
    })
}

fn truth_json(state: ReducedBalanceState, reaction_position_rad: f32) -> Value {
    json!({
        "forward_position_m": state.forward_position_m,
        "forward_velocity_m_per_s": state.forward_velocity_m_per_s,
        "body_pitch_rad": state.pitch_rad,
        "body_pitch_rate_rad_per_s": state.pitch_rate_rad_per_s,
        "body_roll_rad": state.roll_rad,
        "body_roll_rate_rad_per_s": state.roll_rate_rad_per_s,
        "reaction_position_rad": reaction_position_rad
    })
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

fn authority_name(authority: ActuationAuthority) -> &'static str {
    match authority {
        ActuationAuthority::Denied => "denied",
        ActuationAuthority::ClosedLoop => "closed_loop",
    }
}
