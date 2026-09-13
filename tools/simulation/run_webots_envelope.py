#!/usr/bin/env python3
"""Run the predeclared #18 Webots disturbance envelope as evidence, not tuning."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess

from analyze_webots_envelope import analyze
from summarize_webots_envelope import summarize
from validate_closed_loop_realization import validate as validate_realization


ROOT = Path(__file__).resolve().parents[2]
SAFE_CASE_ID = re.compile(r"^[A-Za-z0-9_.-]+$")


def run_checked(command: list[str], *, cwd: Path = ROOT) -> None:
    subprocess.run(command, cwd=cwd, check=True)


def git_commit() -> str:
    completed = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def validate_manifest(document: dict) -> None:
    if document.get("schema_version") != 1:
        raise ValueError("unsupported envelope schema")
    provenance = str(document.get("provenance", ""))
    if "synthetic" not in provenance.lower() or "not ONE V2" not in provenance:
        raise ValueError("envelope provenance must remain explicitly synthetic/non-ONE-V2")
    if float(document["control_period_s"]) != 0.002:
        raise ValueError("#18 production-path envelope must preserve the 500 Hz control period")
    if float(document["horizon_s"]) <= 0.0:
        raise ValueError("horizon must be positive")
    cases = document.get("cases")
    if not isinstance(cases, list) or not cases:
        raise ValueError("envelope requires at least one case")
    ids: set[str] = set()
    for case in cases:
        case_id = str(case["id"])
        if not SAFE_CASE_ID.fullmatch(case_id):
            raise ValueError(f"unsafe case id {case_id!r}")
        if case_id in ids:
            raise ValueError(f"duplicate case id {case_id!r}")
        ids.add(case_id)
        axis = case["axis"]
        if axis not in ("none", "pitch", "roll"):
            raise ValueError(f"unsupported disturbance axis {axis!r}")
        pitch = float(case["initial_pitch_rad"])
        roll = float(case["initial_roll_rad"])
        if axis == "pitch" and roll != 0.0:
            raise ValueError(f"pitch case {case_id} has nonzero roll")
        if axis == "roll" and pitch != 0.0:
            raise ValueError(f"roll case {case_id} has nonzero pitch")
        if axis == "none" and (pitch != 0.0 or roll != 0.0):
            raise ValueError("baseline must be upright")


def webots_command(
    *,
    image: str,
    world: str,
    bridge: str,
    trace_repo_path: str,
    duration_s: float,
    initial_pitch_rad: float,
    initial_roll_rad: float,
) -> list[str]:
    return [
        "docker",
        "run",
        "--rm",
        "-e",
        "LIBGL_ALWAYS_SOFTWARE=true",
        "-e",
        "SWP_WEBOTS_CLOSED_LOOP=1",
        "-e",
        f"SWP_WEBOTS_CONTROL_BRIDGE=/workspace/{bridge}",
        "-e",
        f"SWP_WEBOTS_TRACE=/workspace/{trace_repo_path}",
        "-e",
        f"SWP_WEBOTS_DURATION_S={duration_s:.12g}",
        "-e",
        f"SWP_WEBOTS_INITIAL_PITCH_RAD={initial_pitch_rad:.12g}",
        "-e",
        f"SWP_WEBOTS_INITIAL_ROLL_RAD={initial_roll_rad:.12g}",
        "-v",
        f"{ROOT}:/workspace",
        "-w",
        "/workspace",
        image,
        "bash",
        "-lc",
        (
            "set -o pipefail; timeout 15s xvfb-run --auto-servernum webots "
            "--stdout --stderr --batch --mode=fast --no-rendering "
            f"/workspace/{world}"
        ),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("tools/simulation/envelopes/initial-tilt-v1.json"),
    )
    parser.add_argument("--output", type=Path, default=Path("webots-envelope-output"))
    parser.add_argument(
        "--case-id",
        action="append",
        help="run only selected case id(s); intended for local debugging, not canonical CI",
    )
    args = parser.parse_args()

    if args.output.is_absolute():
        parser.error("output directory must be repository-relative for the Docker mount")
    manifest_path = (ROOT / args.manifest).resolve()
    try:
        manifest_path.relative_to(ROOT)
    except ValueError:
        parser.error("manifest must be inside the repository")

    document = json.loads(manifest_path.read_text(encoding="utf-8"))
    validate_manifest(document)
    validate_realization(ROOT / document["realization_fixture"])

    selected_ids = set(args.case_id or [])
    cases = [
        case for case in document["cases"] if not selected_ids or case["id"] in selected_ids
    ]
    unknown_ids = selected_ids - {case["id"] for case in cases}
    if unknown_ids:
        parser.error(f"unknown case id(s): {sorted(unknown_ids)}")

    output = ROOT / args.output
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)

    run_checked(
        [
            "cargo",
            "build",
            "-p",
            "swp-sitl",
            "--bin",
            "webots_production_bridge",
            "--target",
            "x86_64-unknown-linux-gnu",
        ]
    )
    bridge_repo_path = "target/x86_64-unknown-linux-gnu/debug/webots_production_bridge"
    if not (ROOT / bridge_repo_path).is_file():
        raise RuntimeError("production semantic bridge build did not produce an executable")

    image = str(document["backend"]["image"])
    world = str(document["world"])
    horizon_s = float(document["horizon_s"])
    control_period_s = float(document["control_period_s"])
    summaries: list[dict] = []

    # Pull once outside the per-case timeout.  A large Webots image download is
    # infrastructure setup, not part of any disturbance case's physical horizon.
    run_checked(["docker", "pull", image])

    for index, case in enumerate(cases, start=1):
        case_id = str(case["id"])
        case_dir = output / case_id
        case_dir.mkdir()
        trace = case_dir / "trace.jsonl"
        log = case_dir / "webots.log"
        summary_path = case_dir / "summary.json"
        trace_repo_path = trace.relative_to(ROOT).as_posix()

        print(
            f"[{index:02d}/{len(cases):02d}] {case_id}: "
            f"pitch={float(case['initial_pitch_rad']):+.6g} "
            f"roll={float(case['initial_roll_rad']):+.6g}",
            flush=True,
        )
        completed = subprocess.run(
            webots_command(
                image=image,
                world=world,
                bridge=bridge_repo_path,
                trace_repo_path=trace_repo_path,
                duration_s=horizon_s,
                initial_pitch_rad=float(case["initial_pitch_rad"]),
                initial_roll_rad=float(case["initial_roll_rad"]),
            ),
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=30,
        )
        log_text = (completed.stdout or "") + (completed.stderr or "")
        log.write_text(log_text, encoding="utf-8")
        if any(line.startswith("ERROR:") for line in log_text.splitlines()):
            raise RuntimeError(f"Webots reported runtime ERROR for {case_id}")
        if not trace.is_file():
            raise RuntimeError(
                f"Webots produced no evidence trace for {case_id}; exit {completed.returncode}"
            )

        summary = analyze(
            trace,
            case,
            document["classification"],
            control_period_s,
        )
        completed_horizon = float(summary["end_time_s"]) >= horizon_s - control_period_s
        terminated_early = completed.returncode != 0 or not completed_horizon
        summary["simulator_exit_code"] = completed.returncode
        summary["completed_declared_horizon"] = completed_horizon
        summary["terminated_early_after_declared_loss"] = False

        if terminated_early:
            # Once the predeclared loss criterion has already been observed, a
            # later encoder rejection/controller exit is part of the failed
            # trajectory, not permission to discard the counterexample.  A
            # partial trace without declared loss remains an infrastructure
            # failure because its classification would be censored.
            if summary["classification"] != "loss_of_balance":
                raise RuntimeError(
                    f"Webots terminated early for {case_id} before a declared loss; "
                    f"exit {completed.returncode}, end={summary['end_time_s']} s"
                )
            summary["terminated_early_after_declared_loss"] = True

        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        summaries.append(summary)
        suffix = " [early after loss]" if summary["terminated_early_after_declared_loss"] else ""
        print(
            f"    {summary['classification']}{suffix}; "
            f"max_pitch={summary['max_abs_pitch_rad']:.6g} rad, "
            f"max_roll={summary['max_abs_roll_rad']:.6g} rad, "
            f"max_tau={summary['max_abs_authorized_torque_nm']:.6g} N m",
            flush=True,
        )

    commit = git_commit()
    if selected_ids:
        # Debug subsets are useful for reproducing one counterexample but are not
        # authoritative suite summaries because the predeclared grid is incomplete.
        print(json.dumps({"git_commit": commit, "cases": summaries}, indent=2, sort_keys=True))
        return 0

    suite = summarize(document, summaries, commit)
    suite_path = output / "suite-summary.json"
    suite_path.write_text(
        json.dumps(suite, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "git_commit": commit,
                "case_count": suite["case_count"],
                "classification_counts": suite["classification_counts"],
                "boundaries": suite["boundaries"],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
