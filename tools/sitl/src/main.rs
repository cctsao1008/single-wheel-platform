use std::env;
use std::error::Error;
use std::fs;
use std::io;
use std::path::PathBuf;

use swp_sitl::{run_scenario, scenario::Scenario};

#[derive(Debug)]
struct Cli {
    scenario: PathBuf,
    output: PathBuf,
}

fn main() -> Result<(), Box<dyn Error>> {
    let cli = parse_cli()?;
    let scenario = Scenario::load(&cli.scenario)?;
    let git_commit = env::var("GITHUB_SHA")
        .or_else(|_| env::var("SITL_GIT_COMMIT"))
        .unwrap_or_else(|_| "unknown".to_owned());
    let artifacts = run_scenario("single-wheel-platform", &git_commit, &scenario)?;

    fs::create_dir_all(&cli.output)?;
    fs::write(cli.output.join("manifest.json"), artifacts.manifest_json)?;
    fs::write(cli.output.join("trace.jsonl"), artifacts.trace_jsonl)?;
    fs::write(cli.output.join("summary.json"), artifacts.summary_json)?;

    println!("SITL deterministic execution complete");
    println!("scenario............... {}", scenario.id);
    println!("duration_us............ {}", scenario.duration_us);
    println!("sensor_period_us....... {}", scenario.sensor_period_us);
    println!("runtime_period_us...... {}", scenario.runtime_period_us);
    println!("output................. {}", cli.output.display());
    Ok(())
}

fn parse_cli() -> Result<Cli, Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let mut scenario = None;
    let mut output = None;

    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--scenario" => {
                scenario = Some(PathBuf::from(args.next().ok_or_else(|| {
                    io::Error::new(io::ErrorKind::InvalidInput, "--scenario requires a path")
                })?));
            }
            "--output" => {
                output = Some(PathBuf::from(args.next().ok_or_else(|| {
                    io::Error::new(io::ErrorKind::InvalidInput, "--output requires a path")
                })?));
            }
            "--help" | "-h" => {
                println!("usage: swp-sitl --scenario <scenario.toml> --output <directory>");
                std::process::exit(0);
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

    Ok(Cli {
        scenario: scenario.ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "missing --scenario")
        })?,
        output: output.unwrap_or_else(|| PathBuf::from("sitl-output")),
    })
}
