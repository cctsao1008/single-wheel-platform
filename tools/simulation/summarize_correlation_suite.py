#!/usr/bin/env python3
"""Aggregate the five #16 correlation summaries into one machine-readable verdict."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

REQUIRED = (
    ("synthetic-free-response", "synthetic-free-response.summary.json"),
    ("synthetic-drive-torque-pulse", "synthetic-drive-torque-pulse.summary.json"),
    ("synthetic-reaction-torque-pulse", "synthetic-reaction-torque-pulse.summary.json"),
    ("synthetic-combined-small-disturbance", "synthetic-small-angle.summary.json"),
    ("synthetic-zero-input-equilibrium", "synthetic-zero-input-equilibrium.summary.json"),
)
STATUS_ORDER = {"pass": 0, "explainable_difference": 1, "fail": 2}


def build_suite_summary(input_dir: Path) -> dict[str, Any]:
    experiments: dict[str, Any] = {}
    overall_rank = 0

    for expected_name, file_name in REQUIRED:
        path = input_dir / file_name
        if not path.is_file():
            raise ValueError(f"missing required correlation summary: {path}")
        document = json.loads(path.read_text(encoding="utf-8"))
        if document.get("experiment") != expected_name:
            raise ValueError(
                f"{path}: expected experiment {expected_name!r}, "
                f"got {document.get('experiment')!r}"
            )
        comparisons = document.get("comparisons")
        if not isinstance(comparisons, dict) or not comparisons:
            raise ValueError(f"{path}: comparisons must be a non-empty object")

        statuses: dict[str, str] = {}
        for backend, comparison in comparisons.items():
            status = comparison.get("status")
            if status not in STATUS_ORDER:
                raise ValueError(f"{path}: invalid status {status!r} for backend {backend!r}")
            statuses[str(backend)] = str(status)
            overall_rank = max(overall_rank, STATUS_ORDER[status])

        experiments[expected_name] = {
            "summary_file": file_name,
            "summary_schema": document.get("schema"),
            "experiment_sha256": document.get("experiment_sha256"),
            "parameter_source": document.get("parameter_source"),
            "parameter_source_sha256": document.get("parameter_source_sha256"),
            "parameter_provenance": document.get("parameter_provenance"),
            "repository_commit": document.get("repository_commit"),
            "backend_status": statuses,
        }

    overall_status = next(
        status for status, rank in STATUS_ORDER.items() if rank == overall_rank
    )
    return {
        "schema": 1,
        "contract": "single-wheel-platform-cross-backend-open-loop-v1",
        "required_experiment_count": len(REQUIRED),
        "overall_status": overall_status,
        "experiments": experiments,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    summary = build_suite_summary(args.input_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 1 if summary["overall_status"] == "fail" else 0


if __name__ == "__main__":
    raise SystemExit(main())
