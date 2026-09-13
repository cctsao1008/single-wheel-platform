#!/usr/bin/env python3
"""Fail-closed materialization of accepted ONE V2 parameters for simulation.

This tool never invents physical values. It either produces a backend-neutral
accepted-physical parameter fixture from the canonical registry or refuses to
materialize while required evidence remains unknown.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REGISTRY = ROOT / "parameters" / "reference-assembly.json"
CANONICAL_SOURCE = "parameters/reference-assembly.json"
PROFILE = "webots-production-semantic-v1"

MODEL_TOOLS = ROOT / "tools" / "model"
if str(MODEL_TOOLS) not in sys.path:
    sys.path.insert(0, str(MODEL_TOOLS))

from check_parameter_admissibility import (  # noqa: E402
    AdmissibilityError,
    validate_inertia_admissibility,
)


REQUIRED_PATHS = (
    "body.mass_kg",
    "body.com_height_m",
    "body.inertia_roll_kg_m2",
    "body.inertia_pitch_kg_m2",
    "body.inertia_yaw_kg_m2",
    "drive_wheel.radius_m",
    "drive_wheel.mass_kg",
    "drive_wheel.spin_inertia_kg_m2",
    "drive_wheel.encoder_counts_per_revolution",
    "drive_wheel.encoder_positive_sign",
    "drive_wheel.encoder_max_abs_delta_counts_per_sample",
    "reaction_wheel.mass_kg",
    "reaction_wheel.com_height_m",
    "reaction_wheel.spin_inertia_kg_m2",
    "reaction_wheel.transverse_inertia_kg_m2",
    "reaction_wheel.encoder_counts_per_revolution",
    "reaction_wheel.encoder_positive_sign",
    "reaction_wheel.encoder_max_abs_delta_counts_per_sample",
    "imu.forward_x_m",
    "imu.left_y_m",
    "imu.up_z_m",
    "actuator.drive.torque_per_effective_command_nm",
    "actuator.drive.command_deadzone",
    "actuator.drive.viscous_friction_nm_per_rad_s",
    "actuator.drive.coulomb_friction_nm",
    "actuator.drive.delay_s",
    "actuator.reaction.torque_per_effective_command_nm",
    "actuator.reaction.command_deadzone",
    "actuator.reaction.viscous_friction_nm_per_rad_s",
    "actuator.reaction.coulomb_friction_nm",
    "actuator.reaction.delay_s",
)


class PhysicalFixtureError(ValueError):
    pass


class PhysicalFixtureNotReady(PhysicalFixtureError):
    def __init__(self, missing: list[str]):
        self.missing = tuple(missing)
        super().__init__(
            "accepted physical fixture is not ready; missing accepted evidence: "
            + ", ".join(missing)
        )


def _load(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise PhysicalFixtureError(f"{path}: registry must be one JSON object")
    return value


def _leaf(registry: dict[str, Any], path: str) -> dict[str, Any]:
    node: Any = registry
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node:
            raise PhysicalFixtureError(f"{path}: missing registry path")
        node = node[part]
    if not isinstance(node, dict) or set(node) != {"value", "evidence"}:
        raise PhysicalFixtureError(f"{path}: expected exactly {{value, evidence}}")
    evidence = node["evidence"]
    if not isinstance(evidence, str) or not evidence:
        raise PhysicalFixtureError(f"{path}: evidence must be a non-empty string")
    value = node["value"]
    if value is None:
        if evidence != "unknown":
            raise PhysicalFixtureError(
                f"{path}: null accepted value must remain evidence='unknown'"
            )
    elif evidence == "unknown":
        raise PhysicalFixtureError(
            f"{path}: non-null accepted value cannot remain evidence='unknown'"
        )
    return node


def readiness(registry: dict[str, Any]) -> dict[str, Any]:
    if registry.get("schema") != 1:
        raise PhysicalFixtureError("accepted registry schema must be 1")
    if registry.get("assembly") != "reference-assembly":
        raise PhysicalFixtureError("accepted registry assembly must be 'reference-assembly'")

    missing: list[str] = []
    accepted: dict[str, dict[str, Any]] = {}
    for path in REQUIRED_PATHS:
        leaf = _leaf(registry, path)
        if leaf["value"] is None:
            missing.append(path)
        else:
            accepted[path] = {"value": leaf["value"], "evidence": leaf["evidence"]}

    # This gate is reject-only. It validates accepted inertia facts that exist;
    # it never fills unknown values or repairs impossible ones.
    try:
        validate_inertia_admissibility(registry)
    except AdmissibilityError as error:
        raise PhysicalFixtureError(str(error)) from error

    return {
        "schema": 1,
        "profile": PROFILE,
        "provenance": "accepted_physical",
        "ready": not missing,
        "required_count": len(REQUIRED_PATHS),
        "accepted_count": len(accepted),
        "missing_count": len(missing),
        "missing": missing,
    }


def _canonical_parameter_payload(registry: dict[str, Any]) -> dict[str, Any]:
    parameters: dict[str, dict[str, Any]] = {}
    for path in REQUIRED_PATHS:
        leaf = _leaf(registry, path)
        if leaf["value"] is None:
            raise PhysicalFixtureNotReady([path])
        value = leaf["value"]
        if isinstance(value, float) and not math.isfinite(value):
            raise PhysicalFixtureError(f"{path}: accepted numeric value must be finite")
        parameters[path] = {"value": value, "evidence": leaf["evidence"]}
    return {
        "assembly": "reference-assembly",
        "profile": PROFILE,
        "parameters": parameters,
    }


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def materialize(
    registry_path: Path = DEFAULT_REGISTRY,
    *,
    source_label: str = CANONICAL_SOURCE,
) -> dict[str, Any]:
    registry_bytes = registry_path.read_bytes()
    registry = _load(registry_path)
    status = readiness(registry)
    if not status["ready"]:
        raise PhysicalFixtureNotReady(list(status["missing"]))

    payload = _canonical_parameter_payload(registry)
    return {
        "schema": 1,
        "name": "one-v2-accepted-physical-fixture-v1",
        "profile": PROFILE,
        "provenance": "accepted_physical",
        "source": source_label,
        "source_sha256": _sha256_bytes(registry_bytes),
        "parameter_set_sha256": _sha256_bytes(_canonical_json_bytes(payload)),
        "assembly": "reference-assembly",
        "physical_parameters": payload["parameters"],
        "simulation_parameter_policy": (
            "solver, contact, numerical, and other simulator-only settings are not physical "
            "registry values and must remain in a separate backend configuration"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--readiness",
        action="store_true",
        help="report readiness without attempting materialization; returns success even when blocked",
    )
    args = parser.parse_args()

    registry = _load(args.registry)
    if args.readiness:
        print(json.dumps(readiness(registry), indent=2, sort_keys=True))
        return 0

    try:
        fixture = materialize(
            args.registry,
            source_label=(CANONICAL_SOURCE if args.registry.resolve() == DEFAULT_REGISTRY.resolve() else str(args.registry)),
        )
    except PhysicalFixtureNotReady as error:
        print(str(error), file=sys.stderr)
        print(json.dumps(readiness(registry), indent=2, sort_keys=True), file=sys.stderr)
        return 2

    text = json.dumps(fixture, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(text, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
