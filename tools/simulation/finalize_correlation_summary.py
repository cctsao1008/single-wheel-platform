#!/usr/bin/env python3
"""Finalize #16 correlation evidence with dimensioned suite-level acceptance policy."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
from typing import Any

from open_loop_correlation import COMMON_STATE_FIELDS, load_common_jsonl

FIELD_UNITS = {
    "forward_position_m": "m",
    "forward_velocity_m_per_s": "m/s",
    "body_pitch_rad": "rad",
    "body_pitch_rate_rad_per_s": "rad/s",
    "body_roll_rad": "rad",
    "body_roll_rate_rad_per_s": "rad/s",
    "reaction_position_rad": "rad",
    "reaction_rate_rad_per_s": "rad/s",
}

EQUILIBRIUM_LIMITS = {
    "forward_position_m": 1.0e-6,
    "forward_velocity_m_per_s": 1.0e-4,
    "body_pitch_rad": 1.0e-4,
    "body_pitch_rate_rad_per_s": 2.0e-3,
    "body_roll_rad": 1.0e-4,
    "body_roll_rate_rad_per_s": 2.0e-3,
    "reaction_position_rad": 1.0e-4,
    "reaction_rate_rad_per_s": 2.0e-3,
}


def repository_commit(root: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def trace_characteristics(trace: list[dict[str, float]]) -> dict[str, dict[str, float | str]]:
    if len(trace) < 2:
        raise ValueError("trace must contain at least two records")
    tail_count = max(1, len(trace) // 5)
    tail = trace[-tail_count:]
    result: dict[str, dict[str, float | str]] = {}
    for field in COMMON_STATE_FIELDS:
        peak = max(trace, key=lambda record: abs(record[field]))
        result[field] = {
            "unit": FIELD_UNITS[field],
            "initial_value": trace[0][field],
            "end_value": trace[-1][field],
            "peak_abs": abs(peak[field]),
            "peak_abs_time_s": peak["time_s"],
            "tail_max_abs": max(abs(record[field]) for record in tail),
        }
    return result


def equilibrium_checks(trace: list[dict[str, float]]) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    for field in COMMON_STATE_FIELDS:
        peak = max(trace, key=lambda record: abs(record[field]))
        maximum = abs(peak[field])
        limit = EQUILIBRIUM_LIMITS[field]
        passed = maximum <= limit
        checks.append(
            {
                "field": field,
                "unit": FIELD_UNITS[field],
                "criterion": "peak_abs <= field_limit",
                "candidate_peak_abs": maximum,
                "candidate_peak_time_s": peak["time_s"],
                "limit": limit,
                "pass": passed,
                "classification": "pass" if passed else "fail",
            }
        )
    return checks


def finalize_summary(
    summary: dict[str, Any],
    *,
    analytical: list[dict[str, float]],
    rust: list[dict[str, float]],
    webots: list[dict[str, float]],
    repository_sha: str | None,
    webots_image: str | None,
) -> dict[str, Any]:
    experiment = str(summary["experiment"])
    comparisons = summary["comparisons"]
    if "rust-simulation-world" not in comparisons or "webots" not in comparisons:
        raise ValueError("summary must contain Rust and Webots comparisons")

    summary["schema"] = 2
    summary["repository_commit"] = repository_sha
    summary["webots_image"] = webots_image
    summary["trace_characteristics"] = {
        "analytical-reference": trace_characteristics(analytical),
        "rust-simulation-world": trace_characteristics(rust),
        "webots": trace_characteristics(webots),
    }

    rust_cmp = comparisons["rust-simulation-world"]
    rust_cmp["acceptance_basis"] = "strict_numeric_and_causal"

    webots_cmp = comparisons["webots"]
    webots_cmp["acceptance_basis"] = "input_match_and_causal_semantics"
    if webots_cmp.get("status") == "explainable_difference" and not webots_cmp.get(
        "explained_differences"
    ):
        webots_cmp["magnitude_difference_reason"] = (
            "rigid-body/contact Webots is not required to numerically equal the reduced "
            "linear plant; magnitude error remains reported while input and causal sign "
            "checks are authoritative"
        )

    if experiment == "synthetic-zero-input-equilibrium":
        checks = equilibrium_checks(webots)
        webots_cmp["legacy_scalar_equilibrium_check"] = webots_cmp.get("causal_checks", [])
        webots_cmp["causal_checks"] = checks
        passed = bool(webots_cmp.get("input_match")) and all(check["pass"] for check in checks)
        webots_cmp["causal_match"] = passed
        webots_cmp["causal_acceptable"] = passed
        webots_cmp["status"] = "pass" if passed else "fail"
        webots_cmp["acceptance_basis"] = "dimensioned_zero_input_equilibrium_envelopes"

    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--analytical-trace", type=Path, required=True)
    parser.add_argument("--rust-projected-trace", type=Path, required=True)
    parser.add_argument("--webots-trace", type=Path, required=True)
    args = parser.parse_args()

    summary = json.loads(args.summary.read_text(encoding="utf-8"))
    root = Path(__file__).resolve().parents[2]
    finalized = finalize_summary(
        summary,
        analytical=load_common_jsonl(args.analytical_trace),
        rust=load_common_jsonl(args.rust_projected_trace),
        webots=load_common_jsonl(args.webots_trace),
        repository_sha=repository_commit(root),
        webots_image=os.environ.get("SWP_WEBOTS_IMAGE"),
    )
    args.summary.write_text(json.dumps(finalized, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(finalized, indent=2, sort_keys=True))
    return 1 if any(
        comparison["status"] == "fail" for comparison in finalized["comparisons"].values()
    ) else 0


if __name__ == "__main__":
    raise SystemExit(main())
