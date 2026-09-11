#!/usr/bin/env python3
"""Validate simulator-neutral experiment contracts for host-side evidence."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


ALLOWED_BACKENDS = {"analytical", "rust-simulation-world", "webots", "pybullet-scratch"}
ALLOWED_PROVENANCE = {"synthetic", "accepted_physical"}

EXPECTED_COORDINATES = {
    "body_frame": "+X forward, +Y left, +Z up",
    "body_roll_positive": "right-hand rotation about body +X",
    "body_pitch_positive": "right-hand rotation about body +Y",
    "drive_wheel_positive": "relative rotation that produces +X forward rolling when body pitch is held fixed",
    "reaction_wheel_positive": "relative rotation about body +X by right-hand rule",
}

STATE_UNITS = {
    "body_roll_rad": "rad",
    "body_roll_rate_rad_s": "rad/s",
    "body_pitch_rad": "rad",
    "body_pitch_rate_rad_s": "rad/s",
    "drive_position_rad": "rad",
    "drive_rate_rad_s": "rad/s",
    "reaction_position_rad": "rad",
    "reaction_rate_rad_s": "rad/s",
}

INPUT_UNITS = {
    "time_s": "s",
    "drive_torque_nm": "N*m",
    "reaction_torque_nm": "N*m",
    "external_force_body_n": "N",
}

ALLOWED_OBSERVABLES = {
    "body_roll_rad": "rad",
    "body_roll_rate_rad_s": "rad/s",
    "body_pitch_rad": "rad",
    "body_pitch_rate_rad_s": "rad/s",
    "drive_position_rad": "rad",
    "drive_rate_rad_s": "rad/s",
    "reaction_position_rad": "rad",
    "reaction_rate_rad_s": "rad/s",
    "drive_torque_nm": "N*m",
    "reaction_torque_nm": "N*m",
}


def _fail(message: str) -> None:
    raise ValueError(message)


def _expect_keys(value: dict[str, Any], required: set[str], where: str) -> None:
    missing = required - set(value)
    if missing:
        _fail(f"{where}: missing fields: {', '.join(sorted(missing))}")


def _finite_number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(f"{where}: expected finite number")
    result = float(value)
    if not math.isfinite(result):
        _fail(f"{where}: expected finite number")
    return result


def _validate_quantity(value: Any, expected_unit: str, where: str) -> float:
    if not isinstance(value, dict):
        _fail(f"{where}: expected quantity object")
    _expect_keys(value, {"value", "unit"}, where)
    if value["unit"] != expected_unit:
        _fail(f"{where}: unit must be {expected_unit!r}")
    return _finite_number(value["value"], f"{where}.value")


def validate_experiment(document: dict[str, Any]) -> None:
    if not isinstance(document, dict):
        _fail("experiment must be one JSON object")

    required = {
        "schema",
        "name",
        "backend",
        "parameter_set",
        "coordinates",
        "timing",
        "initial_state",
        "input_profile",
        "observables",
    }
    _expect_keys(document, required, "experiment")

    if document["schema"] != 1:
        _fail("experiment.schema must be 1")
    if not isinstance(document["name"], str) or not document["name"].strip():
        _fail("experiment.name must be a non-empty string")

    backend = document["backend"]
    if not isinstance(backend, dict):
        _fail("backend must be an object")
    _expect_keys(backend, {"contract", "allowed"}, "backend")
    if backend["contract"] != "simulator-neutral-v1":
        _fail("backend.contract must be 'simulator-neutral-v1'")
    allowed = backend["allowed"]
    if not isinstance(allowed, list) or not allowed:
        _fail("backend.allowed must be a non-empty list")
    if len(allowed) != len(set(allowed)):
        _fail("backend.allowed contains duplicates")
    unknown_backends = sorted(set(allowed) - ALLOWED_BACKENDS)
    if unknown_backends:
        _fail(f"backend.allowed contains unsupported backends: {unknown_backends}")

    parameter_set = document["parameter_set"]
    if not isinstance(parameter_set, dict):
        _fail("parameter_set must be an object")
    _expect_keys(parameter_set, {"id", "provenance", "source", "parameters"}, "parameter_set")
    provenance = parameter_set["provenance"]
    if provenance not in ALLOWED_PROVENANCE:
        _fail(f"parameter_set.provenance must be one of {sorted(ALLOWED_PROVENANCE)}")
    if provenance == "accepted_physical" and parameter_set["source"] != "parameters/reference-assembly.json":
        _fail("accepted_physical parameter sets must source parameters/reference-assembly.json")
    if not isinstance(parameter_set["id"], str) or not parameter_set["id"].strip():
        _fail("parameter_set.id must be a non-empty string")
    if not isinstance(parameter_set["source"], str) or not parameter_set["source"].strip():
        _fail("parameter_set.source must be a non-empty string")
    parameters = parameter_set["parameters"]
    if not isinstance(parameters, dict) or not parameters:
        _fail("parameter_set.parameters must be a non-empty object")
    for name, quantity in parameters.items():
        if not isinstance(name, str) or not name.strip():
            _fail("parameter_set.parameters contains an invalid name")
        if not isinstance(quantity, dict):
            _fail(f"parameter_set.parameters.{name}: expected quantity object")
        _expect_keys(quantity, {"value", "unit"}, f"parameter_set.parameters.{name}")
        if not isinstance(quantity["unit"], str) or not quantity["unit"].strip():
            _fail(f"parameter_set.parameters.{name}.unit must be non-empty")
        _finite_number(quantity["value"], f"parameter_set.parameters.{name}.value")

    coordinates = document["coordinates"]
    if coordinates != EXPECTED_COORDINATES:
        _fail("coordinates must exactly match the canonical project sign/axis contract")

    timing = document["timing"]
    if not isinstance(timing, dict):
        _fail("timing must be an object")
    _expect_keys(timing, {"duration_s", "step_s"}, "timing")
    duration_s = _finite_number(timing["duration_s"], "timing.duration_s")
    step_s = _finite_number(timing["step_s"], "timing.step_s")
    if duration_s <= 0.0 or step_s <= 0.0 or step_s > duration_s:
        _fail("timing requires 0 < step_s <= duration_s")

    initial_state = document["initial_state"]
    if not isinstance(initial_state, dict):
        _fail("initial_state must be an object")
    if set(initial_state) != set(STATE_UNITS):
        _fail("initial_state must contain exactly the canonical eight state quantities")
    for name, unit in STATE_UNITS.items():
        _validate_quantity(initial_state[name], unit, f"initial_state.{name}")

    profile = document["input_profile"]
    if not isinstance(profile, list) or not profile:
        _fail("input_profile must be a non-empty list")
    previous_time = -1.0
    for index, item in enumerate(profile):
        where = f"input_profile[{index}]"
        if not isinstance(item, dict):
            _fail(f"{where}: expected object")
        if set(item) != set(INPUT_UNITS):
            _fail(f"{where}: must contain exactly time, two torques, and external force")
        time_s = _validate_quantity(item["time_s"], "s", f"{where}.time_s")
        _validate_quantity(item["drive_torque_nm"], "N*m", f"{where}.drive_torque_nm")
        _validate_quantity(item["reaction_torque_nm"], "N*m", f"{where}.reaction_torque_nm")
        force = item["external_force_body_n"]
        if not isinstance(force, dict):
            _fail(f"{where}.external_force_body_n: expected quantity object")
        _expect_keys(force, {"value", "unit"}, f"{where}.external_force_body_n")
        if force["unit"] != "N":
            _fail(f"{where}.external_force_body_n: unit must be 'N'")
        vector = force["value"]
        if not isinstance(vector, list) or len(vector) != 3:
            _fail(f"{where}.external_force_body_n.value must be [Fx, Fy, Fz]")
        for axis, component in zip("xyz", vector):
            _finite_number(component, f"{where}.external_force_body_n.{axis}")
        if time_s < 0.0 or time_s > duration_s:
            _fail(f"{where}.time_s must be within experiment duration")
        if time_s < previous_time:
            _fail("input_profile times must be nondecreasing")
        previous_time = time_s

    observables = document["observables"]
    if not isinstance(observables, list) or not observables:
        _fail("observables must be a non-empty list")
    names: list[str] = []
    for index, observable in enumerate(observables):
        where = f"observables[{index}]"
        if not isinstance(observable, dict):
            _fail(f"{where}: expected object")
        _expect_keys(observable, {"name", "unit"}, where)
        name = observable["name"]
        if name not in ALLOWED_OBSERVABLES:
            _fail(f"{where}.name is unsupported: {name!r}")
        if observable["unit"] != ALLOWED_OBSERVABLES[name]:
            _fail(f"{where}.unit must be {ALLOWED_OBSERVABLES[name]!r}")
        names.append(name)
    if len(names) != len(set(names)):
        _fail("observables contains duplicate names")


def load_experiment(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        _fail("experiment must be one JSON object")
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", type=Path)
    args = parser.parse_args()
    validate_experiment(load_experiment(args.experiment))
    print(f"simulation experiment contract OK: {args.experiment}")


if __name__ == "__main__":
    main()
