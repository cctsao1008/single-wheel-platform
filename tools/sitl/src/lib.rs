pub mod evidence;
pub mod scenario;
pub mod scheduler;
pub mod virtual_time;

use std::error::Error;
use std::fmt::{Display, Formatter};

use evidence::{
    Manifest, SITL_SCHEMA_VERSION, Summary, TraceRecord, append_json_line, pretty_json,
};
use scenario::{Scenario, ScenarioError};
use scheduler::{DeterministicScheduler, EventKind, ScheduleError};
use serde_json::{Value, json};
use virtual_time::VirtualTime;

#[derive(Debug, PartialEq, Eq)]
pub struct RunArtifacts {
    pub manifest_json: Vec<u8>,
    pub trace_jsonl: Vec<u8>,
    pub summary_json: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunContext {
    pub system_identifier: String,
    pub git_commit: String,
    pub production_model_configuration: Value,
    pub virtual_physical_truth_configuration: Value,
}

impl RunContext {
    pub fn stage1(system_identifier: impl Into<String>, git_commit: impl Into<String>) -> Self {
        Self {
            system_identifier: system_identifier.into(),
            git_commit: git_commit.into(),
            production_model_configuration: json!({
                "status": "not-materialized",
                "stage": 1
            }),
            virtual_physical_truth_configuration: json!({
                "status": "not-materialized",
                "stage": 1
            }),
        }
    }
}

pub trait PhysicalTimeAdvance {
    fn advance(
        &mut self,
        from: VirtualTime,
        to: VirtualTime,
    ) -> Result<(), Box<dyn Error + 'static>>;
}

#[derive(Debug, Default)]
pub struct NoopPhysicalTimeAdvance;

impl PhysicalTimeAdvance for NoopPhysicalTimeAdvance {
    fn advance(
        &mut self,
        _from: VirtualTime,
        _to: VirtualTime,
    ) -> Result<(), Box<dyn Error + 'static>> {
        Ok(())
    }
}

#[derive(Debug)]
pub enum RunError {
    Scenario(ScenarioError),
    Schedule(ScheduleError),
    Evidence(serde_json::Error),
    PhysicalAdvance(Box<dyn Error + 'static>),
    CounterOverflow,
    VirtualTimeOverflow,
}

impl Display for RunError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Scenario(error) => write!(formatter, "invalid SITL scenario: {error}"),
            Self::Schedule(error) => write!(formatter, "SITL scheduling failed: {error}"),
            Self::Evidence(error) => {
                write!(formatter, "SITL evidence serialization failed: {error}")
            }
            Self::PhysicalAdvance(error) => write!(formatter, "SITL physical advance failed: {error}"),
            Self::CounterOverflow => write!(formatter, "SITL execution counter overflow"),
            Self::VirtualTimeOverflow => write!(formatter, "SITL virtual time overflow"),
        }
    }
}

impl Error for RunError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Scenario(error) => Some(error),
            Self::Schedule(error) => Some(error),
            Self::Evidence(error) => Some(error),
            Self::PhysicalAdvance(error) => Some(error.as_ref()),
            Self::CounterOverflow | Self::VirtualTimeOverflow => None,
        }
    }
}

impl From<ScenarioError> for RunError {
    fn from(error: ScenarioError) -> Self {
        Self::Scenario(error)
    }
}

impl From<ScheduleError> for RunError {
    fn from(error: ScheduleError) -> Self {
        Self::Schedule(error)
    }
}

impl From<serde_json::Error> for RunError {
    fn from(error: serde_json::Error) -> Self {
        Self::Evidence(error)
    }
}

pub fn run_scenario(
    system_identifier: &str,
    git_commit: &str,
    scenario: &Scenario,
) -> Result<RunArtifacts, RunError> {
    let context = RunContext::stage1(system_identifier, git_commit);
    let mut physical_time = NoopPhysicalTimeAdvance;
    run_scenario_with_time_advance(&context, scenario, &mut physical_time)
}

pub fn run_scenario_with_time_advance<A: PhysicalTimeAdvance>(
    context: &RunContext,
    scenario: &Scenario,
    physical_time: &mut A,
) -> Result<RunArtifacts, RunError> {
    scenario.validate()?;

    let manifest = Manifest {
        schema_version: SITL_SCHEMA_VERSION,
        system_identifier: &context.system_identifier,
        git_commit: &context.git_commit,
        scenario: &scenario.id,
        seed: scenario.seed,
        duration_us: scenario.duration_us,
        sensor_period_us: scenario.sensor_period_us,
        runtime_period_us: scenario.runtime_period_us,
        missed_runtime_at_us: &scenario.missed_runtime_at_us,
        production_model_configuration: context.production_model_configuration.clone(),
        virtual_physical_truth_configuration: context.virtual_physical_truth_configuration.clone(),
    };

    let mut scheduler = DeterministicScheduler::default();
    scheduler.schedule(VirtualTime::ZERO, EventKind::ScenarioStart)?;

    schedule_periodic(
        &mut scheduler,
        scenario.duration_us,
        scenario.sensor_period_us,
        |scheduler, at| {
            scheduler.schedule(at, EventKind::SensorSample)?;
            scheduler.schedule(at, EventKind::ObservationDelivery)?;
            Ok(())
        },
    )?;

    schedule_periodic(
        &mut scheduler,
        scenario.duration_us,
        scenario.runtime_period_us,
        |scheduler, at| {
            if scenario.runtime_is_missed(at.as_micros()) {
                scheduler.schedule(at, EventKind::RuntimeOpportunityMissed)?;
            } else {
                scheduler.schedule(at, EventKind::ProductionRuntime)?;
                scheduler.schedule(at, EventKind::ActuationCommit)?;
            }
            Ok(())
        },
    )?;

    let mut trace = Vec::new();
    let mut event_sequence = 0_u64;
    let mut time_slices = 0_u64;
    let mut physical_time_advances = 0_u64;
    let mut scheduled_sensor_samples = 0_u64;
    let mut delivered_observations = 0_u64;
    let mut admitted_control_opportunities = 0_u64;
    let mut missed_control_opportunities = 0_u64;
    let mut actuation_commits = 0_u64;

    while let Some(slice) = scheduler.next_slice() {
        time_slices = time_slices.checked_add(1).ok_or(RunError::CounterOverflow)?;

        if slice.advance.to > slice.advance.from {
            physical_time
                .advance(slice.advance.from, slice.advance.to)
                .map_err(RunError::PhysicalAdvance)?;
            physical_time_advances = physical_time_advances
                .checked_add(1)
                .ok_or(RunError::CounterOverflow)?;
        }

        for event in slice.events {
            let opportunity_status = match event.kind {
                EventKind::ProductionRuntime => {
                    admitted_control_opportunities += 1;
                    Some("admitted")
                }
                EventKind::RuntimeOpportunityMissed => {
                    missed_control_opportunities += 1;
                    Some("missed")
                }
                EventKind::SensorSample => {
                    scheduled_sensor_samples += 1;
                    None
                }
                EventKind::ObservationDelivery => {
                    delivered_observations += 1;
                    None
                }
                EventKind::ActuationCommit => {
                    actuation_commits += 1;
                    None
                }
                EventKind::ScenarioStart => None,
            };

            append_json_line(
                &mut trace,
                &TraceRecord {
                    schema_version: SITL_SCHEMA_VERSION,
                    event_sequence,
                    virtual_time_us: event.at.as_micros(),
                    record_kind: event.kind.record_kind(),
                    semantic_phase: event.phase.as_str(),
                    opportunity_status,
                },
            )?;
            event_sequence = event_sequence
                .checked_add(1)
                .ok_or(RunError::CounterOverflow)?;
        }
    }

    let scheduled_control_opportunities =
        admitted_control_opportunities + missed_control_opportunities;
    let pass = scheduled_sensor_samples == scenario.sensor_sample_count()
        && delivered_observations == scheduled_sensor_samples
        && scheduled_control_opportunities == scenario.runtime_opportunity_count()
        && actuation_commits == admitted_control_opportunities
        && physical_time_advances == time_slices.saturating_sub(1);

    let summary = Summary {
        schema_version: SITL_SCHEMA_VERSION,
        scenario: &scenario.id,
        pass,
        time_slices,
        physical_time_advances,
        scheduled_sensor_samples,
        delivered_observations,
        scheduled_control_opportunities,
        admitted_control_opportunities,
        missed_control_opportunities,
        actuation_commits,
    };

    Ok(RunArtifacts {
        manifest_json: pretty_json(&manifest)?,
        trace_jsonl: trace,
        summary_json: pretty_json(&summary)?,
    })
}

fn schedule_periodic<F>(
    scheduler: &mut DeterministicScheduler,
    duration_us: u64,
    period_us: u64,
    mut schedule: F,
) -> Result<(), RunError>
where
    F: FnMut(&mut DeterministicScheduler, VirtualTime) -> Result<(), ScheduleError>,
{
    let mut at = VirtualTime::ZERO;
    loop {
        schedule(scheduler, at)?;
        let Some(next) = at.checked_add_micros(period_us) else {
            return Err(RunError::VirtualTimeOverflow);
        };
        if next.as_micros() > duration_us {
            break;
        }
        at = next;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        Scenario {
            id: "deterministic-test".to_owned(),
            duration_us: 20_000,
            seed: 0x5357_5001,
            sensor_period_us: 5_000,
            runtime_period_us: 10_000,
            missed_runtime_at_us: vec![10_000],
        }
    }

    #[test]
    fn same_configuration_produces_byte_identical_evidence() {
        let scenario = scenario();
        let first = run_scenario("single-wheel-platform", "test-commit", &scenario).unwrap();
        let second = run_scenario("single-wheel-platform", "test-commit", &scenario).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn separate_sensor_and_runtime_cadences_are_preserved() {
        let scenario = scenario();
        let artifacts = run_scenario("single-wheel-platform", "test-commit", &scenario).unwrap();
        let summary: serde_json::Value = serde_json::from_slice(&artifacts.summary_json).unwrap();

        assert_eq!(summary["time_slices"], 5);
        assert_eq!(summary["physical_time_advances"], 4);
        assert_eq!(summary["scheduled_sensor_samples"], 5);
        assert_eq!(summary["delivered_observations"], 5);
        assert_eq!(summary["scheduled_control_opportunities"], 3);
        assert_eq!(summary["admitted_control_opportunities"], 2);
        assert_eq!(summary["missed_control_opportunities"], 1);
        assert_eq!(summary["actuation_commits"], 2);
        assert_eq!(summary["pass"], true);
    }

    #[test]
    fn missed_control_opportunity_is_not_replayed() {
        let scenario = scenario();
        let artifacts = run_scenario("single-wheel-platform", "test-commit", &scenario).unwrap();
        let records: Vec<serde_json::Value> = artifacts
            .trace_jsonl
            .split(|byte| *byte == b'\n')
            .filter(|line| !line.is_empty())
            .map(|line| serde_json::from_slice(line).unwrap())
            .collect();
        let at_missed_time: Vec<_> = records
            .iter()
            .filter(|record| record["virtual_time_us"] == 10_000)
            .collect();

        assert!(
            at_missed_time
                .iter()
                .any(|record| record["record_kind"] == "missed_opportunity")
        );
        assert!(
            !at_missed_time
                .iter()
                .any(|record| record["record_kind"] == "control_opportunity")
        );
        assert!(
            !at_missed_time
                .iter()
                .any(|record| record["record_kind"] == "actuation_commit")
        );
    }

    #[test]
    fn manifest_keeps_production_and_virtual_truth_provenance_distinct() {
        let scenario = scenario();
        let mut context = RunContext::stage1("single-wheel-platform", "abc123");
        context.production_model_configuration = json!({"model": "production-assumption"});
        context.virtual_physical_truth_configuration = json!({"plant": "simulated-truth"});
        let mut physical_time = NoopPhysicalTimeAdvance;
        let artifacts =
            run_scenario_with_time_advance(&context, &scenario, &mut physical_time).unwrap();
        let manifest: serde_json::Value = serde_json::from_slice(&artifacts.manifest_json).unwrap();

        assert_eq!(manifest["system_identifier"], "single-wheel-platform");
        assert_eq!(manifest["git_commit"], "abc123");
        assert_eq!(manifest["sensor_period_us"], 5_000);
        assert_eq!(manifest["runtime_period_us"], 10_000);
        assert_eq!(
            manifest["production_model_configuration"]["model"],
            "production-assumption"
        );
        assert_eq!(
            manifest["virtual_physical_truth_configuration"]["plant"],
            "simulated-truth"
        );
    }

    #[derive(Default)]
    struct TimeAdvanceProbe {
        intervals: Vec<(u64, u64)>,
    }

    impl PhysicalTimeAdvance for TimeAdvanceProbe {
        fn advance(
            &mut self,
            from: VirtualTime,
            to: VirtualTime,
        ) -> Result<(), Box<dyn Error + 'static>> {
            self.intervals.push((from.as_micros(), to.as_micros()));
            Ok(())
        }
    }

    #[test]
    fn physical_world_advances_between_slices_not_as_a_queued_event() {
        let scenario = scenario();
        let context = RunContext::stage1("single-wheel-platform", "test-commit");
        let mut probe = TimeAdvanceProbe::default();
        let artifacts = run_scenario_with_time_advance(&context, &scenario, &mut probe).unwrap();

        assert_eq!(
            probe.intervals,
            vec![
                (0, 5_000),
                (5_000, 10_000),
                (10_000, 15_000),
                (15_000, 20_000)
            ]
        );
        assert!(
            !String::from_utf8(artifacts.trace_jsonl)
                .unwrap()
                .contains("integrate_plant_to")
        );
    }
}
