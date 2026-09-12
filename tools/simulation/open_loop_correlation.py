#!/usr/bin/env python3
"""Run simulator-neutral open-loop correlation against the canonical v2 contract.

The analytical reference and Rust SimulationWorld share the reduced-model parameter
fixture but use independent numeric engines. Webots is compared only after all
backends are projected into common physical observables. Rigid-body differences are
reported rather than tuned away; sign/causal disagreement remains a hard failure.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

from validate_experiment import load_experiment, validate_experiment  # noqa: E402

COMMON_STATE_FIELDS = (
    "forward_position_m",
    "forward_velocity_m_per_s",
    "body_pitch_rad",
    "body_pitch_rate_rad_per_s",
    "body_roll_rad",
    "body_roll_rate_rad_per_s",
    "reaction_position_rad",
    "reaction_rate_rad_per_s",
)
INPUT_FIELDS = ("drive_torque_nm", "reaction_torque_nm")
TRACE_FIELDS = ("time_s", *COMMON_STATE_FIELDS, *INPUT_FIELDS)

# These checks deliberately test causal direction, not model identity. Webots is a
# rigid-body/contact solver, so exact numeric agreement with the reduced model is
# neither expected nor required.
CAUSAL_POLICIES: dict[str, dict[str, Any]] = {
    "synthetic-free-response": {
        "window_s": (0.001, 0.050),
        "fields": ("body_pitch_rad", "body_roll_rad"),
    },
    "synthetic-drive-torque-pulse": {
        "window_s": (0.050, 0.100),
        "fields": ("forward_velocity_m_per_s", "body_pitch_rate_rad_per_s"),
    },
    "synthetic-reaction-torque-pulse": {
        "window_s": (0.050, 0.100),
        "fields": ("body_roll_rate_rad_per_s", "reaction_rate_rad_per_s"),
    },
    "synthetic-combined-small-disturbance": {
        "window_s": (0.050, 0.100),
        "fields": (
            "forward_velocity_m_per_s",
            "body_pitch_rate_rad_per_s",
            "body_roll_rate_rad_per_s",
            "reaction_rate_rad_per_s",
        ),
    },
    "synthetic-zero-input-equilibrium": {
        "equilibrium_max_abs": 1.0e-3,
    },
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _quantity(mapping: dict[str, Any], name: str) -> float:
    return float(mapping[name]["value"])


def materialize_reduced_fixture(experiment: dict[str, Any], root: Path = ROOT) -> dict[str, Any]:
    """Translate v2 common physical semantics into the reduced-model fixture."""
    validate_experiment(experiment)
    parameter_set = experiment["parameter_set"]
    source = root / parameter_set["source"]
    source_fixture = json.loads(source.read_text(encoding="utf-8"))
    if "plant" not in source_fixture:
        raise ValueError(f"parameter source {source} does not contain reduced plant parameters")

    timing = experiment["timing"]
    step_us = int(round(float(timing["step_s"]) * 1_000_000.0))
    duration_us = int(round(float(timing["duration_s"]) * 1_000_000.0))
    if not math.isclose(step_us / 1_000_000.0, float(timing["step_s"]), abs_tol=1e-12):
        raise ValueError("experiment step_s is not representable as whole microseconds")
    if not math.isclose(duration_us / 1_000_000.0, float(timing["duration_s"]), abs_tol=1e-12):
        raise ValueError("experiment duration_s is not representable as whole microseconds")

    state = experiment["initial_state"]
    # Reduced fixture order is [s, s_dot, pitch, pitch_dot, roll, roll_dot,
    # reaction_rate, reaction_relative_angle]. Do not substitute drive-joint angle.
    initial_state = [
        _quantity(state, "forward_position_m"),
        _quantity(state, "forward_velocity_m_per_s"),
        _quantity(state, "body_pitch_rad"),
        _quantity(state, "body_pitch_rate_rad_per_s"),
        _quantity(state, "body_roll_rad"),
        _quantity(state, "body_roll_rate_rad_per_s"),
        _quantity(state, "reaction_rate_rad_per_s"),
        _quantity(state, "reaction_position_rad"),
    ]

    profile: list[dict[str, Any]] = []
    for item in experiment["input_profile"]:
        force = [float(value) for value in item["external_force_body_n"]["value"]]
        if any(abs(value) > 0.0 for value in force):
            raise ValueError("reduced correlation fixture does not model external body force")
        at_us = int(round(_quantity(item, "time_s") * 1_000_000.0))
        if at_us % step_us != 0:
            raise ValueError("input event is not aligned to experiment step")
        profile.append(
            {
                "at_us": at_us,
                "drive_torque_nm": _quantity(item, "drive_torque_nm"),
                "reaction_wheel_torque_nm": _quantity(item, "reaction_torque_nm"),
            }
        )

    return {
        "provenance": f"{parameter_set['provenance']} via {parameter_set['source']}; experiment={experiment['name']}",
        "sample_period_us": step_us,
        "duration_us": duration_us,
        "plant": source_fixture["plant"],
        "initial_state": initial_state,
        "input_profile": profile,
    }


def project_model_samples(samples: list[dict[str, Any]]) -> list[dict[str, float]]:
    projected: list[dict[str, float]] = []
    for sample in samples:
        state = [float(value) for value in sample["state"]]
        applied = [float(value) for value in sample["applied_input"]]
        if len(state) != 8 or len(applied) != 2:
            raise ValueError("model trace must contain eight-state/two-input samples")
        projected.append(
            {
                "time_s": float(sample["time_us"]) * 1.0e-6,
                "forward_position_m": state[0],
                "forward_velocity_m_per_s": state[1],
                "body_pitch_rad": state[2],
                "body_pitch_rate_rad_per_s": state[3],
                "body_roll_rad": state[4],
                "body_roll_rate_rad_per_s": state[5],
                "reaction_position_rad": state[7],
                "reaction_rate_rad_per_s": state[6],
                "drive_torque_nm": applied[0],
                "reaction_torque_nm": applied[1],
            }
        )
    return projected


def load_rust_trace(path: Path) -> list[dict[str, float]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    return project_model_samples(document["samples"])


def load_common_jsonl(path: Path) -> list[dict[str, float]]:
    records: list[dict[str, float]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        value = json.loads(line)
        if set(value) != set(TRACE_FIELDS):
            missing = sorted(set(TRACE_FIELDS) - set(value))
            extra = sorted(set(value) - set(TRACE_FIELDS))
            raise ValueError(f"{path}:{line_number}: trace field mismatch missing={missing} extra={extra}")
        record = {name: float(value[name]) for name in TRACE_FIELDS}
        if not all(math.isfinite(number) for number in record.values()):
            raise ValueError(f"{path}:{line_number}: trace contains non-finite value")
        records.append(record)
    if len(records) < 2:
        raise ValueError(f"{path}: trace must contain at least two records")
    return records


def _by_microsecond(trace: list[dict[str, float]]) -> dict[int, dict[str, float]]:
    result: dict[int, dict[str, float]] = {}
    for record in trace:
        key = int(round(record["time_s"] * 1_000_000.0))
        if key in result:
            raise ValueError(f"duplicate trace timestamp {key} us")
        result[key] = record
    return result


def _sign(value: float, epsilon: float) -> int:
    if value > epsilon:
        return 1
    if value < -epsilon:
        return -1
    return 0


def compare_common_traces(
    experiment_name: str,
    reference: list[dict[str, float]],
    candidate: list[dict[str, float]],
    *,
    strict_error_limit: float | None = None,
) -> dict[str, Any]:
    reference_by_time = _by_microsecond(reference)
    candidate_by_time = _by_microsecond(candidate)
    common_times = sorted(set(reference_by_time) & set(candidate_by_time))
    if len(common_times) < 2:
        raise ValueError("reference/candidate traces have insufficient aligned samples")

    per_field_max = {field: 0.0 for field in COMMON_STATE_FIELDS}
    squared = {field: 0.0 for field in COMMON_STATE_FIELDS}
    input_max = {field: 0.0 for field in INPUT_FIELDS}
    for key in common_times:
        expected = reference_by_time[key]
        actual = candidate_by_time[key]
        for field in COMMON_STATE_FIELDS:
            error = abs(actual[field] - expected[field])
            per_field_max[field] = max(per_field_max[field], error)
            squared[field] += error * error
        for field in INPUT_FIELDS:
            input_max[field] = max(input_max[field], abs(actual[field] - expected[field]))

    per_field_rms = {
        field: math.sqrt(squared[field] / len(common_times)) for field in COMMON_STATE_FIELDS
    }
    overall = max(per_field_max.values())
    input_ok = max(input_max.values()) <= 1.0e-7
    sign_checks: list[dict[str, Any]] = []
    causal_ok = True
    policy = CAUSAL_POLICIES.get(experiment_name)
    if policy is None:
        raise ValueError(f"no causal policy for experiment {experiment_name!r}")

    if "window_s" in policy:
        start_us = int(round(policy["window_s"][0] * 1_000_000.0))
        end_us = int(round(policy["window_s"][1] * 1_000_000.0))
        for field in policy["fields"]:
            if start_us not in reference_by_time or end_us not in reference_by_time:
                raise ValueError(f"causal window is absent from reference trace for {field}")
            if start_us not in candidate_by_time or end_us not in candidate_by_time:
                raise ValueError(f"causal window is absent from candidate trace for {field}")
            reference_delta = reference_by_time[end_us][field] - reference_by_time[start_us][field]
            candidate_delta = candidate_by_time[end_us][field] - candidate_by_time[start_us][field]
            reference_sign = _sign(reference_delta, 1.0e-10)
            candidate_sign = _sign(candidate_delta, 1.0e-8)
            passed = reference_sign != 0 and candidate_sign == reference_sign
            causal_ok = causal_ok and passed
            sign_checks.append(
                {
                    "field": field,
                    "start_s": policy["window_s"][0],
                    "end_s": policy["window_s"][1],
                    "reference_delta": reference_delta,
                    "candidate_delta": candidate_delta,
                    "reference_sign": reference_sign,
                    "candidate_sign": candidate_sign,
                    "pass": passed,
                }
            )
    else:
        maximum = max(abs(candidate_by_time[key][field]) for key in common_times for field in COMMON_STATE_FIELDS)
        causal_ok = maximum <= float(policy["equilibrium_max_abs"])
        sign_checks.append(
            {
                "field": "all_common_states",
                "criterion": f"max_abs <= {policy['equilibrium_max_abs']}",
                "candidate_max_abs": maximum,
                "pass": causal_ok,
            }
        )

    if strict_error_limit is not None:
        status = "pass" if input_ok and causal_ok and overall <= strict_error_limit else "fail"
    elif not input_ok or not causal_ok:
        status = "fail"
    elif overall <= 5.0e-3:
        status = "pass"
    else:
        status = "explainable_difference"

    return {
        "status": status,
        "aligned_samples": len(common_times),
        "aligned_start_s": common_times[0] * 1.0e-6,
        "aligned_end_s": common_times[-1] * 1.0e-6,
        "max_abs_error": overall,
        "strict_error_limit": strict_error_limit,
        "per_field_max_abs_error": per_field_max,
        "per_field_rms_error": per_field_rms,
        "input_max_abs_error": input_max,
        "input_match": input_ok,
        "causal_checks": sign_checks,
        "causal_match": causal_ok,
    }


def _write_jsonl(path: Path, records: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")


def analytical_trace(fixture: dict[str, Any]) -> list[dict[str, float]]:
    model_dir = ROOT / "tools" / "model"
    sys.path.insert(0, str(model_dir))
    from reference_balance import simulate_reference  # type: ignore

    return project_model_samples(simulate_reference(fixture))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", type=Path, required=True)
    parser.add_argument("--materialize-reduced-fixture", type=Path)
    parser.add_argument("--rust-trace", type=Path)
    parser.add_argument("--webots-trace", type=Path)
    parser.add_argument("--analytical-trace", type=Path)
    parser.add_argument("--rust-projected-trace", type=Path)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--rust-max-abs-error", type=float, default=5.0e-6)
    args = parser.parse_args()

    experiment = load_experiment(args.experiment)
    validate_experiment(experiment)
    fixture = materialize_reduced_fixture(experiment)
    if args.materialize_reduced_fixture is not None:
        args.materialize_reduced_fixture.parent.mkdir(parents=True, exist_ok=True)
        args.materialize_reduced_fixture.write_text(
            json.dumps(fixture, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    needs_reference = any(
        value is not None
        for value in (args.rust_trace, args.webots_trace, args.analytical_trace, args.summary)
    )
    if not needs_reference:
        return 0

    reference = analytical_trace(fixture)
    if args.analytical_trace is not None:
        _write_jsonl(args.analytical_trace, reference)

    source_path = ROOT / experiment["parameter_set"]["source"]
    summary: dict[str, Any] = {
        "schema": 1,
        "experiment": experiment["name"],
        "experiment_path": str(args.experiment),
        "experiment_sha256": _sha256(args.experiment),
        "parameter_source": experiment["parameter_set"]["source"],
        "parameter_source_sha256": _sha256(source_path),
        "parameter_provenance": experiment["parameter_set"]["provenance"],
        "reference_backend": "analytical-float64-exact-zoh",
        "comparisons": {},
    }

    if args.rust_trace is not None:
        rust = load_rust_trace(args.rust_trace)
        if args.rust_projected_trace is not None:
            _write_jsonl(args.rust_projected_trace, rust)
        summary["comparisons"]["rust-simulation-world"] = compare_common_traces(
            experiment["name"], reference, rust, strict_error_limit=args.rust_max_abs_error
        )

    if args.webots_trace is not None:
        webots = load_common_jsonl(args.webots_trace)
        summary["comparisons"]["webots"] = compare_common_traces(
            experiment["name"], reference, webots
        )

    if args.summary is not None:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))

    failed = any(value["status"] == "fail" for value in summary["comparisons"].values())
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
