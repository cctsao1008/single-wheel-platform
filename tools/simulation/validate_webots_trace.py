#!/usr/bin/env python3
"""Validate the machine-readable Webots common-observable trace."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


FIELDS = (
    "time_s",
    "forward_position_m",
    "forward_velocity_m_per_s",
    "body_pitch_rad",
    "body_pitch_rate_rad_per_s",
    "body_roll_rad",
    "body_roll_rate_rad_per_s",
    "reaction_position_rad",
    "reaction_rate_rad_per_s",
    "drive_torque_nm",
    "reaction_torque_nm",
)


def finite_number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{where}: expected number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{where}: expected finite number")
    return result


def validate_trace(path: Path, minimum_records: int = 2) -> dict[str, float | int]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"line {line_number}: expected JSON object")
            if set(value) != set(FIELDS):
                missing = sorted(set(FIELDS) - set(value))
                extra = sorted(set(value) - set(FIELDS))
                raise ValueError(f"line {line_number}: field mismatch missing={missing} extra={extra}")
            for field in FIELDS:
                finite_number(value[field], f"line {line_number}.{field}")
            records.append(value)

    if len(records) < minimum_records:
        raise ValueError(f"trace contains {len(records)} records; need at least {minimum_records}")

    previous = -math.inf
    for index, record in enumerate(records):
        time_s = float(record["time_s"])
        if time_s <= previous:
            raise ValueError(f"record {index}: time_s must be strictly increasing")
        previous = time_s

    return {
        "records": len(records),
        "start_time_s": float(records[0]["time_s"]),
        "end_time_s": float(records[-1]["time_s"]),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--minimum-records", type=int, default=2)
    args = parser.parse_args()
    summary = validate_trace(args.trace, args.minimum_records)
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
