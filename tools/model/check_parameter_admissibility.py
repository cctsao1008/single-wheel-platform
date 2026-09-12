#!/usr/bin/env python3
"""Reject physically impossible accepted inertia values without inventing missing ones."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
REGISTRY_PATH = ROOT / "parameters" / "reference-assembly.json"

INERTIA_PATHS = (
    "body.inertia_roll_kg_m2",
    "body.inertia_pitch_kg_m2",
    "body.inertia_yaw_kg_m2",
    "drive_wheel.spin_inertia_kg_m2",
    "reaction_wheel.spin_inertia_kg_m2",
    "reaction_wheel.transverse_inertia_kg_m2",
)


class AdmissibilityError(ValueError):
    """Raised when accepted parameters violate the current physical model contract."""


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise AdmissibilityError(f"{path}: expected a JSON object")
    return value


def _leaf(registry: dict[str, Any], path: str) -> dict[str, Any]:
    node: Any = registry
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node:
            raise AdmissibilityError(f"{path}: missing registry field")
        node = node[part]

    if not isinstance(node, dict) or "value" not in node or "evidence" not in node:
        raise AdmissibilityError(f"{path}: expected {{value, evidence}} leaf")
    return node


def _known_inertia(registry: dict[str, Any], path: str) -> float | None:
    value = _leaf(registry, path)["value"]
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AdmissibilityError(f"{path}: accepted inertia must be numeric")

    result = float(value)
    if not math.isfinite(result):
        raise AdmissibilityError(f"{path}: accepted inertia must be finite")
    if result <= 0.0:
        raise AdmissibilityError(f"{path}: accepted inertia must be strictly positive")
    return result


def validate_inertia_admissibility(registry: dict[str, Any]) -> list[str]:
    """Validate only inertia facts that are already accepted in the registry.

    Missing values deliberately remain untested rather than being defaulted. The
    current model treats the body inertia tensor as diagonal in body axes and the
    reaction wheel as axisymmetric, so those assumptions define the admissibility
    checks below.
    """

    values = {path: _known_inertia(registry, path) for path in INERTIA_PATHS}
    checks: list[str] = []

    body_paths = (
        "body.inertia_roll_kg_m2",
        "body.inertia_pitch_kg_m2",
        "body.inertia_yaw_kg_m2",
    )
    body = [values[path] for path in body_paths]
    if all(value is not None for value in body):
        roll, pitch, yaw = (float(value) for value in body)
        inequalities = (
            ("roll", roll, pitch + yaw),
            ("pitch", pitch, roll + yaw),
            ("yaw", yaw, roll + pitch),
        )
        for name, lhs, rhs in inequalities:
            if lhs > rhs:
                raise AdmissibilityError(
                    "body inertia violates rigid-body principal-moment triangle inequality: "
                    f"I_{name}={lhs} > {rhs}"
                )
        checks.append("body principal-moment triangle inequalities")

    reaction_spin = values["reaction_wheel.spin_inertia_kg_m2"]
    reaction_transverse = values["reaction_wheel.transverse_inertia_kg_m2"]
    if reaction_spin is not None and reaction_transverse is not None:
        upper_bound = 2.0 * reaction_transverse
        if reaction_spin > upper_bound:
            raise AdmissibilityError(
                "reaction-wheel inertia violates axisymmetric rigid-body bound: "
                f"I_spin={reaction_spin} > 2*I_transverse={upper_bound}"
            )
        checks.append("reaction-wheel axisymmetric inertia bound")

    known_count = sum(value is not None for value in values.values())
    checks.insert(0, f"{known_count} known inertia values")
    return checks


def main() -> None:
    registry = load_json(REGISTRY_PATH)
    checks = validate_inertia_admissibility(registry)
    print("parameter inertia admissibility OK: " + "; ".join(checks))


if __name__ == "__main__":
    main()
