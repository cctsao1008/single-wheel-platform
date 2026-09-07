use std::{env, error::Error, fs, io, path::PathBuf};

use swp_sitl::{run_scenario, scenario::Scenario};

fn main() -> Result<(), Box<dyn Error>> {
    let output_dir = parse_output_dir()?;
    let scenario = Scenario::deterministic_baseline();
    let git_commit = env::var("GITHUB_SHA").unwrap_or_else(|_| "unknown".to_owned());
    let artifacts = run_scenario("single-wheel-platform", &git_commit, scenario)?;

    fs::create_dir_all(&output_dir)?;
    fs::write(output_dir.join("manifest.json"), artifacts.manifest_json)?;
    fs::write(output_dir.join("trace.jsonl"), artifacts.trace_jsonl)?;
    fs::write(output_dir.join("summary.json"), artifacts.summary_json)?;

    println!(
        "SITL deterministic skeleton complete: {}",
        output_dir.display()
    );
    Ok(())
}

fn parse_output_dir() -> Result<PathBuf, Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let mut output_dir = PathBuf::from("sitl-output");

    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--output" => {
                let value = args.next().ok_or_else(|| {
                    io::Error::new(io::ErrorKind::InvalidInput, "--output requires a path")
                })?;
                output_dir = PathBuf::from(value);
            }
            other => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("unsupported argument: {other}"),
                )
                .into());
            }
        }
    }

    Ok(output_dir)
}
