#!/usr/bin/env python3
"""Validate nominal Webots -> production semantic path evidence for Issue #17."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


class EvidenceError(ValueError):
    pass


def _finite(value, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise EvidenceError(f"{name} is non-finite")
    return value


def validate(path: Path, minimum_records: int) -> dict:
    records = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if len(records) < minimum_records:
        raise EvidenceError(
            f"closed-loop trace has {len(records)} records; expected at least {minimum_records}"
        )

    apply_count = 0
    nonzero_apply_count = 0
    revoke_count = 0
    balancing_count = 0
    first_apply_index = None
    previous_index = None
    previous_timestamp = None
    max_abs_pitch = 0.0
    max_abs_roll = 0.0
    max_abs_authorized_torque = 0.0

    for position, record in enumerate(records):
        if record.get("mode") != "closed_loop_production_path":
            raise EvidenceError(f"record {position} has unexpected mode")

        raw = record["raw_device_observation"]
        production = record["production"]
        truth = record["webots_evidence_truth"]

        if raw.get("schema") != 1 or raw.get("mapping_id") != "webots-body-identity-v1":
            raise EvidenceError(f"record {position} violates the raw-device bridge contract")
        if any(
            name in raw
            for name in (
                "body_pitch_rad",
                "body_roll_rad",
                "forward_position_m",
                "forward_velocity_m_per_s",
            )
        ):
            raise EvidenceError("simulator truth leaked into the production-input object")

        sample_index = int(raw["sample_index"])
        timestamp_us = int(raw["timestamp_us"])
        if production.get("sample_index") != sample_index:
            raise EvidenceError(f"record {position} production sample index mismatch")
        if previous_index is not None and sample_index != previous_index + 1:
            raise EvidenceError(f"sample sequence gap at record {position}")
        if previous_timestamp is not None and timestamp_us - previous_timestamp != 2_000:
            raise EvidenceError(f"production cadence is not exactly 2 ms at record {position}")
        previous_index = sample_index
        previous_timestamp = timestamp_us

        if int(production["runtime_fault_bits"]) != 0:
            raise EvidenceError(
                f"runtime fault bits became nonzero at sample {sample_index}: "
                f"{production['runtime_fault_bits']}"
            )

        actuation = production["actuation"]
        drive_torque = _finite(record["authorized_drive_torque_nm"], "drive torque")
        reaction_torque = _finite(
            record["authorized_reaction_torque_nm"], "reaction torque"
        )
        sample_max_torque = max(abs(drive_torque), abs(reaction_torque))
        max_abs_authorized_torque = max(max_abs_authorized_torque, sample_max_torque)
        if actuation == "revoke":
            revoke_count += 1
            if drive_torque != 0.0 or reaction_torque != 0.0:
                raise EvidenceError("revoked authority produced nonzero Webots torque")
        elif actuation == "apply":
            apply_count += 1
            if sample_max_torque > 1.0e-6:
                nonzero_apply_count += 1
            if first_apply_index is None:
                first_apply_index = position
            if production.get("authority") != "closed_loop":
                raise EvidenceError("applied torque lacks closed-loop authority")
            if not math.isclose(
                drive_torque,
                _finite(production["drive_torque_nm"], "production drive torque"),
                rel_tol=0.0,
                abs_tol=1.0e-9,
            ) or not math.isclose(
                reaction_torque,
                _finite(production["reaction_torque_nm"], "production reaction torque"),
                rel_tol=0.0,
                abs_tol=1.0e-9,
            ):
                raise EvidenceError("Webots torque differs from production bridge output")
        else:
            raise EvidenceError(f"unknown actuation result {actuation!r}")

        if production["operating_state"] == "balancing":
            balancing_count += 1

        pitch = abs(_finite(truth["body_pitch_rad"], "Webots pitch"))
        roll = abs(_finite(truth["body_roll_rad"], "Webots roll"))
        max_abs_pitch = max(max_abs_pitch, pitch)
        max_abs_roll = max(max_abs_roll, roll)
        for name, value in truth.items():
            _finite(value, f"Webots truth {name}")

    if revoke_count < 2:
        raise EvidenceError("startup did not demonstrate fail-closed revocation")
    if first_apply_index is None:
        raise EvidenceError("production authority never reached an applied actuation")
    if first_apply_index < 2:
        raise EvidenceError("actuation was applied before encoder/timing evidence matured")
    if nonzero_apply_count == 0:
        raise EvidenceError("closed-loop evidence never exercised a nonzero authorized torque")
    if balancing_count == 0:
        raise EvidenceError("RuntimeSupervisor never reached Balancing")

    # This is a deliberately loose falsification bound, not a tuning criterion.
    # A small synthetic perturbation should not become a near-horizontal fall in
    # the short nominal evidence window. Tighter performance claims belong to #18.
    if max_abs_pitch >= 0.5 or max_abs_roll >= 0.5:
        raise EvidenceError(
            "nominal small-perturbation run escaped the bounded attitude envelope"
        )

    first_truth = records[0]["webots_evidence_truth"]
    final_truth = records[-1]["webots_evidence_truth"]
    return {
        "records": len(records),
        "revoke_count": revoke_count,
        "apply_count": apply_count,
        "nonzero_apply_count": nonzero_apply_count,
        "first_apply_sample": records[first_apply_index]["production"]["sample_index"],
        "balancing_count": balancing_count,
        "max_abs_authorized_torque_nm": max_abs_authorized_torque,
        "start_pitch_rad": first_truth["body_pitch_rad"],
        "end_pitch_rad": final_truth["body_pitch_rad"],
        "start_roll_rad": first_truth["body_roll_rad"],
        "end_roll_rad": final_truth["body_roll_rad"],
        "max_abs_pitch_rad": max_abs_pitch,
        "max_abs_roll_rad": max_abs_roll,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    parser.add_argument("--minimum-records", type=int, default=10)
    args = parser.parse_args()
    print(json.dumps(validate(args.trace, args.minimum_records), sort_keys=True))


if __name__ == "__main__":
    main()
