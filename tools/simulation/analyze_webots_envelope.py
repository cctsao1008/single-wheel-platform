#!/usr/bin/env python3
"""Classify one #18 Webots production-path disturbance trace.

A robustness result may legitimately be recovery, saturation-limited recovery,
non-recovery by the declared horizon, or loss of balance.  This analyzer fails
only on malformed/causally invalid evidence; it does not turn a physical
counterexample into a CI infrastructure failure.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


class EnvelopeEvidenceError(ValueError):
    pass


def _finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise EnvelopeEvidenceError(f"{name} is non-finite")
    return result


def _load_records(path: Path) -> list[dict[str, Any]]:
    records = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not records:
        raise EnvelopeEvidenceError("trace is empty")
    return records


def analyze(
    path: Path,
    case: dict[str, Any],
    classification: dict[str, Any],
    control_period_s: float,
) -> dict[str, Any]:
    records = _load_records(path)
    expected_period_us = int(round(control_period_s * 1_000_000.0))
    if expected_period_us <= 0:
        raise EnvelopeEvidenceError("control period must be positive")

    settle_angle = _finite(
        classification["settling_angle_abs_rad"], "settling angle band"
    )
    settle_rate = _finite(
        classification["settling_rate_abs_rad_per_s"], "settling rate band"
    )
    settle_dwell = _finite(classification["settling_dwell_s"], "settling dwell")
    hard_attitude = _finite(
        classification["hard_attitude_abs_rad"], "hard attitude bound"
    )
    drive_saturation_bit = int(classification["drive_saturated_reason_bit"])
    reaction_saturation_bit = int(classification["reaction_saturated_reason_bit"])
    saturation_mask = drive_saturation_bit | reaction_saturation_bit

    previous_index: int | None = None
    previous_timestamp: int | None = None
    apply_count = 0
    revoke_count = 0
    constrained_count = 0
    saturation_count = 0
    drive_saturation_count = 0
    reaction_saturation_count = 0
    balancing_count = 0
    fault_bits_union = 0
    first_fault_sample: int | None = None
    first_fault_time_s: float | None = None
    hard_bound_sample: int | None = None
    hard_bound_time_s: float | None = None
    max_abs_pitch = 0.0
    max_abs_roll = 0.0
    max_abs_pitch_rate = 0.0
    max_abs_roll_rate = 0.0
    max_abs_torque = 0.0
    inside_settle_band: list[bool] = []

    for position, record in enumerate(records):
        if record.get("mode") != "closed_loop_production_path":
            raise EnvelopeEvidenceError(f"record {position} has unexpected mode")

        raw = record.get("raw_device_observation")
        production = record.get("production")
        truth = record.get("webots_evidence_truth")
        if not isinstance(raw, dict) or not isinstance(production, dict) or not isinstance(truth, dict):
            raise EnvelopeEvidenceError(f"record {position} is missing evidence sections")

        if raw.get("schema") != 1 or raw.get("mapping_id") != "webots-body-identity-v1":
            raise EnvelopeEvidenceError(f"record {position} violates the raw bridge contract")
        for forbidden in (
            "body_pitch_rad",
            "body_roll_rad",
            "forward_position_m",
            "forward_velocity_m_per_s",
        ):
            if forbidden in raw:
                raise EnvelopeEvidenceError(
                    f"simulator truth field {forbidden} leaked into production input"
                )

        sample_index = int(raw["sample_index"])
        timestamp_us = int(raw["timestamp_us"])
        time_s = _finite(record["time_s"], f"record {position} time")
        if int(production.get("sample_index")) != sample_index:
            raise EnvelopeEvidenceError(f"record {position} production sample mismatch")
        if previous_index is not None and sample_index != previous_index + 1:
            raise EnvelopeEvidenceError(f"sample sequence gap at record {position}")
        if previous_timestamp is not None and timestamp_us - previous_timestamp != expected_period_us:
            raise EnvelopeEvidenceError(
                f"production cadence differs from {expected_period_us} us at record {position}"
            )
        previous_index = sample_index
        previous_timestamp = timestamp_us

        drive_torque = _finite(
            record["authorized_drive_torque_nm"], f"record {position} drive torque"
        )
        reaction_torque = _finite(
            record["authorized_reaction_torque_nm"],
            f"record {position} reaction torque",
        )
        max_abs_torque = max(max_abs_torque, abs(drive_torque), abs(reaction_torque))

        actuation = production.get("actuation")
        if actuation == "revoke":
            revoke_count += 1
            if drive_torque != 0.0 or reaction_torque != 0.0:
                raise EnvelopeEvidenceError("revoked authority produced nonzero Webots torque")
        elif actuation == "apply":
            apply_count += 1
            if production.get("authority") != "closed_loop":
                raise EnvelopeEvidenceError("applied Webots torque lacks closed-loop authority")
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
                raise EnvelopeEvidenceError(
                    "Webots torque differs from production-authorized torque"
                )
        else:
            raise EnvelopeEvidenceError(f"unknown actuation result {actuation!r}")

        reason_bits = int(production.get("authority_reason_bits", 0))
        if bool(production.get("constrained", False)):
            constrained_count += 1
        if reason_bits & saturation_mask:
            saturation_count += 1
        if reason_bits & drive_saturation_bit:
            drive_saturation_count += 1
        if reason_bits & reaction_saturation_bit:
            reaction_saturation_count += 1

        if production.get("operating_state") == "balancing":
            balancing_count += 1

        fault_bits = int(production.get("runtime_fault_bits", 0))
        fault_bits_union |= fault_bits
        if fault_bits != 0 and first_fault_sample is None:
            first_fault_sample = sample_index
            first_fault_time_s = time_s

        pitch = _finite(truth["body_pitch_rad"], f"record {position} pitch")
        roll = _finite(truth["body_roll_rad"], f"record {position} roll")
        pitch_rate = _finite(
            truth["body_pitch_rate_rad_per_s"], f"record {position} pitch rate"
        )
        roll_rate = _finite(
            truth["body_roll_rate_rad_per_s"], f"record {position} roll rate"
        )
        for name, value in truth.items():
            _finite(value, f"record {position} truth {name}")

        max_abs_pitch = max(max_abs_pitch, abs(pitch))
        max_abs_roll = max(max_abs_roll, abs(roll))
        max_abs_pitch_rate = max(max_abs_pitch_rate, abs(pitch_rate))
        max_abs_roll_rate = max(max_abs_roll_rate, abs(roll_rate))

        if (
            hard_bound_sample is None
            and (abs(pitch) >= hard_attitude or abs(roll) >= hard_attitude)
        ):
            hard_bound_sample = sample_index
            hard_bound_time_s = time_s

        inside_settle_band.append(
            abs(pitch) <= settle_angle
            and abs(roll) <= settle_angle
            and abs(pitch_rate) <= settle_rate
            and abs(roll_rate) <= settle_rate
        )

    trailing_inside_count = 0
    for inside in reversed(inside_settle_band):
        if not inside:
            break
        trailing_inside_count += 1
    trailing_inside_s = trailing_inside_count * control_period_s
    recovered = trailing_inside_s + 1.0e-12 >= settle_dwell

    runtime_fault_is_loss = bool(classification.get("runtime_fault_is_loss", True))
    lost = hard_bound_sample is not None or (
        runtime_fault_is_loss and fault_bits_union != 0
    )
    if lost:
        outcome = "loss_of_balance"
        recovery_time_s = None
    elif recovered:
        outcome = (
            "saturation_limited_recovery"
            if saturation_count > 0
            else "recovered"
        )
        recovery_start_index = len(records) - trailing_inside_count
        recovery_time_s = _finite(
            records[recovery_start_index]["time_s"], "recovery start time"
        )
    else:
        outcome = "not_recovered_by_horizon"
        recovery_time_s = None

    first_truth = records[0]["webots_evidence_truth"]
    final_truth = records[-1]["webots_evidence_truth"]
    return {
        "case_id": str(case["id"]),
        "axis": str(case["axis"]),
        "magnitude_rad": _finite(case["magnitude_rad"], "case magnitude"),
        "initial_pitch_rad_declared": _finite(
            case["initial_pitch_rad"], "declared initial pitch"
        ),
        "initial_roll_rad_declared": _finite(
            case["initial_roll_rad"], "declared initial roll"
        ),
        "classification": outcome,
        "records": len(records),
        "start_time_s": _finite(records[0]["time_s"], "start time"),
        "end_time_s": _finite(records[-1]["time_s"], "end time"),
        "apply_count": apply_count,
        "revoke_count": revoke_count,
        "balancing_count": balancing_count,
        "constrained_count": constrained_count,
        "saturation_count": saturation_count,
        "saturation_duration_s": saturation_count * control_period_s,
        "drive_saturation_count": drive_saturation_count,
        "reaction_saturation_count": reaction_saturation_count,
        "runtime_fault_bits_union": fault_bits_union,
        "first_fault_sample": first_fault_sample,
        "first_fault_time_s": first_fault_time_s,
        "hard_bound_sample": hard_bound_sample,
        "hard_bound_time_s": hard_bound_time_s,
        "trailing_settled_duration_s": trailing_inside_s,
        "time_to_recover_s": recovery_time_s,
        "start_pitch_rad": _finite(first_truth["body_pitch_rad"], "start pitch"),
        "end_pitch_rad": _finite(final_truth["body_pitch_rad"], "end pitch"),
        "start_roll_rad": _finite(first_truth["body_roll_rad"], "start roll"),
        "end_roll_rad": _finite(final_truth["body_roll_rad"], "end roll"),
        "max_abs_pitch_rad": max_abs_pitch,
        "max_abs_roll_rad": max_abs_roll,
        "max_abs_pitch_rate_rad_per_s": max_abs_pitch_rate,
        "max_abs_roll_rate_rad_per_s": max_abs_roll_rate,
        "max_abs_authorized_torque_nm": max_abs_torque,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("case_id")
    args = parser.parse_args()

    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    case = next((item for item in manifest["cases"] if item["id"] == args.case_id), None)
    if case is None:
        parser.error(f"unknown case id {args.case_id!r}")
    summary = analyze(
        args.trace,
        case,
        manifest["classification"],
        float(manifest["control_period_s"]),
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
