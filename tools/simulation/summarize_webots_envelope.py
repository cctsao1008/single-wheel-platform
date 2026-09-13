#!/usr/bin/env python3
"""Summarize #18 disturbance-case classifications without hiding counterexamples."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


RECOVERY_CLASSES = {"recovered", "saturation_limited_recovery"}
KNOWN_CLASSES = RECOVERY_CLASSES | {"not_recovered_by_horizon", "loss_of_balance"}


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not an object")
    return value


def _case_sign(case: dict[str, Any]) -> str:
    axis = case["axis"]
    if axis == "pitch":
        value = float(case["initial_pitch_rad"])
    elif axis == "roll":
        value = float(case["initial_roll_rad"])
    else:
        return "zero"
    return "positive" if value > 0.0 else "negative"


def _boundary(group: list[dict[str, Any]]) -> dict[str, Any]:
    ordered = sorted(group, key=lambda item: float(item["magnitude_rad"]))
    recovered = [
        float(item["magnitude_rad"])
        for item in ordered
        if item["classification"] in RECOVERY_CLASSES
    ]
    nonrecovered = [
        float(item["magnitude_rad"])
        for item in ordered
        if item["classification"] not in RECOVERY_CLASSES
    ]
    losses = [
        float(item["magnitude_rad"])
        for item in ordered
        if item["classification"] == "loss_of_balance"
    ]
    saturation_recovery = [
        float(item["magnitude_rad"])
        for item in ordered
        if item["classification"] == "saturation_limited_recovery"
    ]

    seen_nonrecovery = False
    monotonic = True
    for item in ordered:
        is_recovery = item["classification"] in RECOVERY_CLASSES
        if not is_recovery:
            seen_nonrecovery = True
        elif seen_nonrecovery:
            monotonic = False

    return {
        "largest_recovered_rad": max(recovered) if recovered else None,
        "smallest_nonrecovered_rad": min(nonrecovered) if nonrecovered else None,
        "smallest_loss_rad": min(losses) if losses else None,
        "smallest_saturation_limited_recovery_rad": (
            min(saturation_recovery) if saturation_recovery else None
        ),
        "recovery_order_monotonic_over_tested_grid": monotonic,
        "tested": [
            {
                "magnitude_rad": float(item["magnitude_rad"]),
                "classification": item["classification"],
            }
            for item in ordered
        ],
    }


def summarize(
    manifest: dict[str, Any],
    summaries: list[dict[str, Any]],
    git_commit: str,
) -> dict[str, Any]:
    case_by_id = {str(item["id"]): item for item in manifest["cases"]}
    if len(summaries) != len(case_by_id):
        raise ValueError(
            f"expected {len(case_by_id)} case summaries, found {len(summaries)}"
        )

    seen: set[str] = set()
    counts = {name: 0 for name in sorted(KNOWN_CLASSES)}
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    baseline: dict[str, Any] | None = None

    for summary in summaries:
        case_id = str(summary["case_id"])
        if case_id in seen or case_id not in case_by_id:
            raise ValueError(f"duplicate/unknown case summary {case_id!r}")
        seen.add(case_id)
        classification = str(summary["classification"])
        if classification not in KNOWN_CLASSES:
            raise ValueError(f"unknown classification {classification!r}")
        counts[classification] += 1

        case = case_by_id[case_id]
        if case["axis"] == "none":
            baseline = summary
        else:
            key = (str(case["axis"]), _case_sign(case))
            grouped.setdefault(key, []).append(summary)

    boundaries = {
        axis: {
            sign: _boundary(grouped.get((axis, sign), []))
            for sign in ("positive", "negative")
        }
        for axis in ("pitch", "roll")
    }

    symmetry: dict[str, list[dict[str, Any]]] = {"pitch": [], "roll": []}
    for axis in ("pitch", "roll"):
        positives = {
            float(item["magnitude_rad"]): item
            for item in grouped.get((axis, "positive"), [])
        }
        negatives = {
            float(item["magnitude_rad"]): item
            for item in grouped.get((axis, "negative"), [])
        }
        for magnitude in sorted(set(positives) & set(negatives)):
            positive = positives[magnitude]
            negative = negatives[magnitude]
            symmetry[axis].append(
                {
                    "magnitude_rad": magnitude,
                    "classification_equal": positive["classification"]
                    == negative["classification"],
                    "positive_classification": positive["classification"],
                    "negative_classification": negative["classification"],
                    "max_abs_pitch_difference_rad": abs(
                        float(positive["max_abs_pitch_rad"])
                        - float(negative["max_abs_pitch_rad"])
                    ),
                    "max_abs_roll_difference_rad": abs(
                        float(positive["max_abs_roll_rad"])
                        - float(negative["max_abs_roll_rad"])
                    ),
                    "max_abs_torque_difference_nm": abs(
                        float(positive["max_abs_authorized_torque_nm"])
                        - float(negative["max_abs_authorized_torque_nm"])
                    ),
                }
            )

    if baseline is None:
        raise ValueError("baseline summary is missing")

    smallest_case_prediction = {}
    for axis in ("pitch", "roll"):
        for sign in ("positive", "negative"):
            group = grouped.get((axis, sign), [])
            if not group:
                continue
            smallest = min(group, key=lambda item: float(item["magnitude_rad"]))
            smallest_case_prediction[f"{axis}_{sign}"] = {
                "case_id": smallest["case_id"],
                "classification": smallest["classification"],
                "recovered": smallest["classification"] in RECOVERY_CLASSES,
            }

    all_finite = all(
        math.isfinite(float(item["max_abs_pitch_rad"]))
        and math.isfinite(float(item["max_abs_roll_rad"]))
        and math.isfinite(float(item["max_abs_authorized_torque_nm"]))
        for item in summaries
    )
    if not all_finite:
        raise ValueError("non-finite summary metric")

    return {
        "schema_version": 1,
        "evidence_status": "complete",
        "suite": manifest["name"],
        "provenance": manifest["provenance"],
        "backend": manifest["backend"],
        "world": manifest["world"],
        "realization_fixture": manifest["realization_fixture"],
        "git_commit": git_commit,
        "case_count": len(summaries),
        "classification_counts": counts,
        "baseline": baseline,
        "boundaries": boundaries,
        "sign_symmetry": symmetry,
        "predeclared_prediction_observation": {
            "baseline_classification": baseline["classification"],
            "smallest_axis_cases": smallest_case_prediction,
        },
        "cases": sorted(summaries, key=lambda item: str(item["case_id"])),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--input-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--git-commit", default="unknown")
    args = parser.parse_args()

    manifest = _load(args.manifest)
    summaries = [
        _load(path)
        for path in sorted(args.input_dir.glob("*/summary.json"))
    ]
    result = summarize(manifest, summaries, args.git_commit)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "case_count": result["case_count"],
        "classification_counts": result["classification_counts"],
        "boundaries": result["boundaries"],
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
