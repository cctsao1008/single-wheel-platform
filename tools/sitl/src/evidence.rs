use serde::Serialize;
use serde_json::Value;

pub const SITL_SCHEMA_VERSION: u32 = 1;

#[derive(Debug, Serialize)]
pub struct Manifest<'a> {
    pub schema_version: u32,
    pub system_identifier: &'a str,
    pub git_commit: &'a str,
    pub scenario: &'a str,
    pub seed: u64,
    pub duration_us: u64,
    pub sensor_period_us: u64,
    pub runtime_period_us: u64,
    pub missed_runtime_at_us: &'a [u64],
    pub production_model_configuration: Value,
    pub virtual_physical_truth_configuration: Value,
}

#[derive(Debug, Serialize)]
pub struct TraceRecord<'a> {
    pub schema_version: u32,
    pub event_sequence: u64,
    pub virtual_time_us: u64,
    pub record_kind: &'a str,
    pub semantic_phase: &'a str,
    pub opportunity_status: Option<&'a str>,
}

#[derive(Debug, Serialize)]
pub struct Summary<'a> {
    pub schema_version: u32,
    pub scenario: &'a str,
    pub pass: bool,
    pub scheduled_sensor_samples: u64,
    pub delivered_observations: u64,
    pub scheduled_control_opportunities: u64,
    pub admitted_control_opportunities: u64,
    pub missed_control_opportunities: u64,
    pub actuation_commits: u64,
}

pub fn pretty_json<T: Serialize>(value: &T) -> Result<Vec<u8>, serde_json::Error> {
    let mut bytes = serde_json::to_vec_pretty(value)?;
    bytes.push(b'\n');
    Ok(bytes)
}

pub fn append_json_line<T: Serialize>(
    output: &mut Vec<u8>,
    value: &T,
) -> Result<(), serde_json::Error> {
    serde_json::to_writer(&mut *output, value)?;
    output.push(b'\n');
    Ok(())
}
