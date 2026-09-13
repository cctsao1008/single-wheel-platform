#!/usr/bin/env python3
"""Validate the physically realizable Webots fixture used for Issue #17.

The production bridge currently uses a reduced synthetic plant fixture.  Webots
must not counterfeit that fixture with impossible rigid-body inertias.  Instead,
the checked-in high-fidelity realization is allowed to use different component
masses/inertias only when it reproduces the reduced model's governing upright
aggregates and actuator-coordinate quantities within the declared tolerance.

This is synthetic validation evidence only.  It promotes no ONE V2 parameter.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


class RealizationError(ValueError):
    pass


def _positive(value, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise RealizationError(f"{name} must be finite and > 0")
    return value


def _vec3(value, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise RealizationError(f"{name} must contain exactly three values")
    return tuple(_positive(item, f"{name}[{index}]") for index, item in enumerate(value))


def _box_inertia(mass: float, size: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = size
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def _cylinder_inertia(mass: float, radius: float, height: float) -> tuple[float, float]:
    spin = 0.5 * mass * radius * radius
    transverse = mass * (3.0 * radius * radius + height * height) / 12.0
    return spin, transverse


def _principal_inertia_valid(values: tuple[float, float, float], name: str) -> None:
    ixx, iyy, izz = values
    for value in values:
        if not math.isfinite(value) or value <= 0.0:
            raise RealizationError(f"{name} contains a non-positive/non-finite principal inertia")
    tolerance = 1e-15
    if ixx > iyy + izz + tolerance or iyy > ixx + izz + tolerance or izz > ixx + iyy + tolerance:
        raise RealizationError(f"{name} violates the rigid-body triangle inequalities")


def _aggregates(p: dict[str, float]) -> dict[str, float]:
    h = p["body_mass_kg"] * p["body_com_height_m"] + p["reaction_wheel_mass_kg"] * p[
        "reaction_wheel_com_height_m"
    ]
    s = p["body_mass_kg"] * p["body_com_height_m"] ** 2 + p[
        "reaction_wheel_mass_kg"
    ] * p["reaction_wheel_com_height_m"] ** 2
    m_s = (
        p["body_mass_kg"]
        + p["reaction_wheel_mass_kg"]
        + p["drive_wheel_mass_kg"]
        + p["drive_wheel_spin_inertia_kg_m2"] / p["drive_wheel_radius_m"] ** 2
    )
    return {
        "gravitational_first_moment_kg_m": h,
        "vertical_second_moment_kg_m2": s,
        "equivalent_translation_mass_kg": m_s,
        "pitch_inertia_kg_m2": s
        + p["body_inertia_pitch_kg_m2"]
        + p["reaction_wheel_transverse_inertia_kg_m2"],
        "roll_body_inertia_kg_m2": s + p["body_inertia_roll_kg_m2"],
    }


def validate(path: Path) -> dict[str, object]:
    document = json.loads(path.read_text(encoding="utf-8"))
    provenance = str(document.get("provenance", ""))
    if "synthetic" not in provenance.lower() or "not ONE V2" not in provenance:
        raise RealizationError("fixture provenance must explicitly remain synthetic and non-ONE-V2")

    reduced_raw = document.get("reduced_model_parameters")
    realization = document.get("webots_realization")
    contract = document.get("equivalence_contract")
    if not isinstance(reduced_raw, dict) or not isinstance(realization, dict) or not isinstance(contract, dict):
        raise RealizationError("fixture sections are missing")

    reduced = {name: _positive(value, f"reduced_model_parameters.{name}") for name, value in reduced_raw.items()}

    body = realization["body"]
    drive = realization["drive_wheel"]
    reaction = realization["reaction_wheel"]
    body_mass = _positive(body["mass_kg"], "body.mass_kg")
    body_height = _positive(body["com_height_above_drive_axle_m"], "body.com_height_above_drive_axle_m")
    body_size = _vec3(body["box_size_m"], "body.box_size_m")
    body_inertia = _box_inertia(body_mass, body_size)
    _principal_inertia_valid(body_inertia, "body inertia")

    drive_mass = _positive(drive["mass_kg"], "drive_wheel.mass_kg")
    drive_radius = _positive(drive["radius_m"], "drive_wheel.radius_m")
    drive_height = _positive(drive["height_m"], "drive_wheel.height_m")
    drive_spin, drive_transverse = _cylinder_inertia(drive_mass, drive_radius, drive_height)
    _principal_inertia_valid((drive_transverse, drive_transverse, drive_spin), "drive-wheel inertia")

    reaction_mass = _positive(reaction["mass_kg"], "reaction_wheel.mass_kg")
    reaction_height_from_axle = _positive(
        reaction["com_height_above_drive_axle_m"], "reaction_wheel.com_height_above_drive_axle_m"
    )
    reaction_radius = _positive(reaction["radius_m"], "reaction_wheel.radius_m")
    reaction_thickness = _positive(reaction["height_m"], "reaction_wheel.height_m")
    reaction_spin, reaction_transverse = _cylinder_inertia(
        reaction_mass, reaction_radius, reaction_thickness
    )
    _principal_inertia_valid(
        (reaction_transverse, reaction_transverse, reaction_spin), "reaction-wheel inertia"
    )

    accel_height = float(realization["accelerometer_height_above_drive_axle_m"])
    if not math.isfinite(accel_height) or abs(accel_height) > 1e-15:
        raise RealizationError(
            "Issue #17 Webots accelerometer must be located at the reduced-model axle origin"
        )

    realized_parameters = {
        "gravity_m_per_s2": reduced["gravity_m_per_s2"],
        "body_mass_kg": body_mass,
        "body_com_height_m": body_height,
        "body_inertia_roll_kg_m2": body_inertia[0],
        "body_inertia_pitch_kg_m2": body_inertia[1],
        "body_inertia_yaw_kg_m2": body_inertia[2],
        "drive_wheel_mass_kg": drive_mass,
        "drive_wheel_radius_m": drive_radius,
        "drive_wheel_spin_inertia_kg_m2": drive_spin,
        "reaction_wheel_mass_kg": reaction_mass,
        "reaction_wheel_com_height_m": reaction_height_from_axle,
        "reaction_wheel_spin_inertia_kg_m2": reaction_spin,
        "reaction_wheel_transverse_inertia_kg_m2": reaction_transverse,
    }

    reduced_aggregates = _aggregates(reduced)
    realized_aggregates = _aggregates(realized_parameters)
    tolerance = _positive(contract["absolute_tolerance"], "equivalence_contract.absolute_tolerance")
    matched = contract.get("matched_reduced_aggregates")
    if not isinstance(matched, list) or not matched:
        raise RealizationError("equivalence_contract.matched_reduced_aggregates must be non-empty")
    errors: dict[str, float] = {}
    for name in matched:
        if name not in reduced_aggregates or name not in realized_aggregates:
            raise RealizationError(f"unknown aggregate {name!r}")
        error = abs(reduced_aggregates[name] - realized_aggregates[name])
        errors[name] = error
        if error > tolerance:
            raise RealizationError(
                f"aggregate {name} differs by {error:.3e}, tolerance {tolerance:.3e}"
            )

    required_radius = float(contract["matched_drive_wheel_radius_m"])
    if abs(drive_radius - required_radius) > tolerance:
        raise RealizationError("drive-wheel radius does not match the reduced coordinate contract")
    required_reaction_spin = float(contract["matched_reaction_wheel_spin_inertia_kg_m2"])
    if abs(reaction_spin - required_reaction_spin) > tolerance:
        raise RealizationError("reaction-wheel spin inertia does not match the reduced dynamics")

    return {
        "status": "pass",
        "provenance": provenance,
        "aggregate_errors": errors,
        "drive_spin_inertia_kg_m2": drive_spin,
        "reaction_spin_inertia_kg_m2": reaction_spin,
        "reaction_transverse_inertia_kg_m2": reaction_transverse,
        "accelerometer_height_above_drive_axle_m": accel_height,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "fixture",
        nargs="?",
        type=Path,
        default=Path("tools/simulation/fixtures/closed-loop-aggregate-equivalent.json"),
    )
    args = parser.parse_args()
    print(json.dumps(validate(args.fixture), sort_keys=True))


if __name__ == "__main__":
    main()
