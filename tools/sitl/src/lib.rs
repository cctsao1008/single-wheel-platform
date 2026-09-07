pub mod evidence;
pub mod scenario;
pub mod scheduler;
pub mod virtual_time;

use evidence::{Manifest, SITL_SCHEMA_VERSION, Summary, TraceRecord, append_json_line, pretty_json};
use scenario::Scenario;
use scheduler::{DeterministicScheduler, SemanticPhase};
use serde_json::json;
use virtual_time::VirtualTime;

#[derive(Debug, PartialEq, Eq)]
pub struct RunArtifacts {
    pub manifest_json: Vec<u8>,
    pub trace_jsonl: Vec<u8>,
    pub summary_json: Vec<u8>,
}

pub fn run_scenario(
    system_identifier: &str,
    git_commit: &str,
    scenario: Scenario,
) -> Result<RunArtifacts, serde_json::Error> {
    assert!(scenario.opportunity_period_us > 0);

    let manifest = Manifest {
        schema_version: SITL_SCHEMA_VERSION,
        system_identifier,
        git_commit,
        scenario: scenario.name,
        seed: scenario.seed,
        duration_us: scenario.duration_us,
        production_model_configuration: json!({"mode": "not-materialized"}),
        virtual_physical_truth_configuration: json!({"mode": "not-materialized"}),
    };

    let mut scheduler = DeterministicScheduler::default();
    let mut at_us = 0_u64;
    loop {
        let at = VirtualTime::from_micros(at_us);
        for phase in [
            SemanticPhase::IntegratePlantTo,
            SemanticPhase::PhysicalOrFaultEvent,
            SemanticPhase::SensorSample,
            SemanticPhase::ObservationDelivery,
            SemanticPhase::ProductionRuntime,
            SemanticPhase::ActuationCommit,
        ] {
            scheduler.schedule(at, phase);
        }

        if scenario.duration_us - at_us < scenario.opportunity_period_us {
            break;
        }
        at_us += scenario.opportunity_period_us;
    }

    let mut trace = Vec::new();
    let mut event_sequence = 0_u64;
    let mut scheduled_control_opportunities = 0_u64;
    let mut admitted_control_opportunities = 0_u64;
    let mut missed_control_opportunities = 0_u64;

    while let Some(event) = scheduler.pop_next() {
        let missed_here = scenario.missed_control_at_us == Some(event.at.as_micros());
        let (record_kind, opportunity_status) = match event.phase {
            SemanticPhase::IntegratePlantTo => ("integrate_plant_to", None),
            SemanticPhase::PhysicalOrFaultEvent => ("physical_or_fault_phase", None),
            SemanticPhase::SensorSample => ("sensor_sample", None),
            SemanticPhase::ObservationDelivery => ("observation_delivery", None),
            SemanticPhase::ProductionRuntime => {
                scheduled_control_opportunities += 1;
                if missed_here {
                    missed_control_opportunities += 1;
                    ("missed_opportunity", Some("missed"))
                } else {
                    admitted_control_opportunities += 1;
                    ("control_opportunity", Some("admitted"))
                }
            }
            SemanticPhase::ActuationCommit => {
                if missed_here {
                    ("actuation_hold", None)
                } else {
                    ("actuation_commit", None)
                }
            }
        };

        append_json_line(
            &mut trace,
            &TraceRecord {
                schema_version: SITL_SCHEMA_VERSION,
                event_sequence,
                virtual_time_us: event.at.as_micros(),
                record_kind,
                semantic_phase: event.phase.as_str(),
                opportunity_status,
            },
        )?;
        event_sequence += 1;
    }

    let pass = scheduled_control_opportunities == scenario.opportunity_count()
        && scheduled_control_opportunities
            == admitted_control_opportunities + missed_control_opportunities;

    let summary = Summary {
        schema_version: SITL_SCHEMA_VERSION,
        scenario: scenario.name,
        pass,
        scheduled_control_opportunities,
        admitted_control_opportunities,
        missed_control_opportunities,
    };

    Ok(RunArtifacts {
        manifest_json: pretty_json(&manifest)?,
        trace_jsonl: trace,
        summary_json: pretty_json(&summary)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_configuration_produces_byte_identical_evidence() {
        let scenario = Scenario::deterministic_baseline();
        let first = run_scenario("single-wheel-platform", "test-commit", scenario).unwrap();
        let second = run_scenario("single-wheel-platform", "test-commit", scenario).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn missed_control_opportunity_is_not_replayed() {
        let scenario = Scenario::deterministic_baseline();
        let artifacts = run_scenario("single-wheel-platform", "test-commit", scenario).unwrap();
        let summary: serde_json::Value = serde_json::from_slice(&artifacts.summary_json).unwrap();

        assert_eq!(summary["scheduled_control_opportunities"], 21);
        assert_eq!(summary["admitted_control_opportunities"], 20);
        assert_eq!(summary["missed_control_opportunities"], 1);
        assert_eq!(summary["pass"], true);

        let trace = String::from_utf8(artifacts.trace_jsonl).unwrap();
        assert_eq!(trace.matches("\"missed_opportunity\"").count(), 1);
        assert_eq!(trace.matches("\"control_opportunity\"").count(), 20);
    }
}
