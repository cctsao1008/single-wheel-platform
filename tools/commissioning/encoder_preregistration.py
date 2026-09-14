#!/usr/bin/env python3
"""Validate and fingerprint pre-capture ONE V2 encoder commissioning hypotheses."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

EXPECTED_BINDINGS = {
    "encoder_1": {
        "installed_role": "ReactionWheel",
        "runtime_counter": "TIM2",
        "mechanical_coordinate": "reaction_wheel_relative_angle",
        "positive_direction_definition": "relative wheel rotation about body +X by the right-hand rule",
    },
    "encoder_2": {
        "installed_role": "DriveWheel",
        "runtime_counter": "TIM4",
        "mechanical_coordinate": "drive_wheel_relative_angle",
        "positive_direction_definition": "relative wheel rotation that produces +X forward rolling when body pitch is held fixed",
    },
}

class PreregistrationError(ValueError):
    pass

def _canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")

def sha256_document(value: Any) -> str:
    return hashlib.sha256(_canonical_bytes(value)).hexdigest()

def _require_bool(mapping: dict[str, Any], key: str, expected: bool) -> None:
    if mapping.get(key) is not expected:
        raise PreregistrationError(f"{key} must be {str(expected).lower()}")

def validate(document: dict[str, Any]) -> dict[str, Any]:
    if document.get("schema") != 1:
        raise PreregistrationError("schema must be 1")
    if document.get("kind") != "encoder-one-revolution-preregistration":
        raise PreregistrationError("kind must be encoder-one-revolution-preregistration")
    if document.get("assembly") != "reference-assembly":
        raise PreregistrationError("assembly must be reference-assembly")

    channel = document.get("encoder_channel")
    if channel not in EXPECTED_BINDINGS:
        raise PreregistrationError("encoder_channel must be encoder_1 or encoder_2")
    expected = EXPECTED_BINDINGS[channel]

    for key in ("installed_role", "runtime_counter", "mechanical_coordinate", "positive_direction_definition"):
        if document.get(key) != expected[key]:
            raise PreregistrationError(f"{channel} {key} does not match the reference-assembly contract")

    safety = document.get("safety")
    if not isinstance(safety, dict):
        raise PreregistrationError("safety must be an object")
    if safety.get("firmware_target") != "observation":
        raise PreregistrationError("firmware_target must be observation")
    _require_bool(safety, "physical_motor_backend_present", False)
    _require_bool(safety, "motor_actuation_allowed", False)
    _require_bool(safety, "manual_rotation_only", True)

    prediction = document.get("prediction")
    if not isinstance(prediction, dict):
        raise PreregistrationError("prediction must be an object")
    _require_bool(prediction, "positive_trial_must_produce_nonzero_net_counts", True)
    _require_bool(prediction, "negative_trial_must_invert_counter_sign", True)
    _require_bool(prediction, "one_revolution_absolute_count_must_repeat", True)

    sign = prediction.get("counter_sign_for_mechanical_positive")
    if sign not in {"increase", "decrease", "unknown"}:
        raise PreregistrationError("counter_sign_for_mechanical_positive must be increase, decrease, or unknown")
    rationale = prediction.get("sign_prediction_basis")
    if not isinstance(rationale, str) or not rationale.strip():
        raise PreregistrationError("sign_prediction_basis must be a non-empty string")
    if sign == "unknown" and "unknown" not in rationale.lower() and "no accepted" not in rationale.lower():
        raise PreregistrationError("unknown sign requires an explicit evidence-gap rationale")

    forbidden = document.get("forbidden_interpretations")
    if not isinstance(forbidden, list) or not forbidden:
        raise PreregistrationError("forbidden_interpretations must be a non-empty list")
    if not all(isinstance(item, str) and item.strip() for item in forbidden):
        raise PreregistrationError("forbidden_interpretations entries must be non-empty strings")

    return {
        "schema": 1,
        "status": "valid_preregistration",
        "assembly": "reference-assembly",
        "encoder_channel": channel,
        "installed_role": expected["installed_role"],
        "runtime_counter": expected["runtime_counter"],
        "mechanical_coordinate": expected["mechanical_coordinate"],
        "counter_sign_hypothesis": sign,
        "preregistration_sha256": sha256_document(document),
    }

def load(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise PreregistrationError("preregistration must be one JSON object")
    return value

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--hash-only", action="store_true", help="print only the canonical preregistration SHA-256")
    args = parser.parse_args()

    try:
        result = validate(load(args.path))
    except (OSError, json.JSONDecodeError, PreregistrationError) as error:
        parser.error(str(error))

    if args.hash_only:
        print(result["preregistration_sha256"])
    else:
        print(json.dumps(result, indent=2, sort_keys=True))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
