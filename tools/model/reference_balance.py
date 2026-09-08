#!/usr/bin/env python3
"""Independent numeric reference for the reduced stationary-upright plant.

The Rust SITL `SimulationWorld` advances the same physical contract with f32 RK4.
This reference independently rebuilds the continuous A/B matrices from explicit
physical parameters and advances them with a float64 matrix exponential under
zero-order-held torque inputs.  It therefore checks model/sign/integration drift
without sharing the Rust execution engine.

The correlation fixture is deliberately synthetic.  It is not a ONE V2 physical
parameter set and must not be used as physical calibration evidence.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.linalg import expm

STATE_NAMES = (
    "forward_position_m",
    "forward_velocity_m_per_s",
    "pitch_rad",
    "pitch_rate_rad_per_s",
    "roll_rad",
    "roll_rate_rad_per_s",
    "reaction_wheel_rate_rad_per_s",
    "reaction_wheel_relative_angle_rad",
)


@dataclass(frozen=True)
class PlantParameters:
    gravity_m_per_s2: float
    body_mass_kg: float
    body_com_height_m: float
    body_inertia_roll_kg_m2: float
    body_inertia_pitch_kg_m2: float
    body_inertia_yaw_kg_m2: float
    drive_wheel_mass_kg: float
    drive_wheel_radius_m: float
    drive_wheel_spin_inertia_kg_m2: float
    reaction_wheel_mass_kg: float
    reaction_wheel_com_height_m: float
    reaction_wheel_spin_inertia_kg_m2: float
    reaction_wheel_transverse_inertia_kg_m2: float

    @classmethod
    def from_mapping(cls, values: dict[str, Any]) -> "PlantParameters":
        return cls(**{field: float(values[field]) for field in cls.__annotations__})

    def validate(self) -> None:
        values = np.asarray(list(self.__dict__.values()), dtype=np.float64)
        if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
            raise ValueError("plant parameters must be positive and finite")


def upright_state_space(parameters: PlantParameters) -> tuple[np.ndarray, np.ndarray]:
    """Return the continuous eight-state plant used for reference correlation."""
    parameters.validate()
    p = parameters

    h = (
        p.body_mass_kg * p.body_com_height_m
        + p.reaction_wheel_mass_kg * p.reaction_wheel_com_height_m
    )
    second_moment = (
        p.body_mass_kg * p.body_com_height_m**2
        + p.reaction_wheel_mass_kg * p.reaction_wheel_com_height_m**2
    )
    equivalent_mass = (
        p.body_mass_kg
        + p.reaction_wheel_mass_kg
        + p.drive_wheel_mass_kg
        + p.drive_wheel_spin_inertia_kg_m2 / p.drive_wheel_radius_m**2
    )
    pitch_inertia = (
        second_moment
        + p.body_inertia_pitch_kg_m2
        + p.reaction_wheel_transverse_inertia_kg_m2
    )
    roll_body_inertia = second_moment + p.body_inertia_roll_kg_m2
    pitch_determinant = equivalent_mass * pitch_inertia - h**2
    if pitch_determinant <= 0.0:
        raise ValueError("stationary-upright pitch inertia determinant must be positive")

    a = np.zeros((8, 8), dtype=np.float64)
    b = np.zeros((8, 2), dtype=np.float64)

    a[0, 1] = 1.0
    a[1, 2] = -(h**2 * p.gravity_m_per_s2) / pitch_determinant
    a[2, 3] = 1.0
    a[3, 2] = h * equivalent_mass * p.gravity_m_per_s2 / pitch_determinant

    a[4, 5] = 1.0
    a[5, 4] = h * p.gravity_m_per_s2 / roll_body_inertia
    a[6, 4] = -(h * p.gravity_m_per_s2 / roll_body_inertia)
    a[7, 6] = 1.0

    b[1, 0] = (
        pitch_inertia / p.drive_wheel_radius_m + h
    ) / pitch_determinant
    b[3, 0] = -(
        h / p.drive_wheel_radius_m + equivalent_mass
    ) / pitch_determinant
    b[5, 1] = -1.0 / roll_body_inertia
    b[6, 1] = (
        1.0 / p.reaction_wheel_spin_inertia_kg_m2 + 1.0 / roll_body_inertia
    )

    return a, b


def exact_zoh_step(
    a: np.ndarray, b: np.ndarray, dt_s: float
) -> tuple[np.ndarray, np.ndarray]:
    """Compute exact zero-order-hold state/input matrices for one interval."""
    augmented = np.zeros((10, 10), dtype=np.float64)
    augmented[:8, :8] = a
    augmented[:8, 8:] = b
    transition = expm(augmented * dt_s)
    return transition[:8, :8], transition[:8, 8:]


def load_fixture(path: Path) -> dict[str, Any]:
    fixture = json.loads(path.read_text(encoding="utf-8"))
    sample_period_us = int(fixture["sample_period_us"])
    duration_us = int(fixture["duration_us"])
    if sample_period_us <= 0 or duration_us < 0 or duration_us % sample_period_us != 0:
        raise ValueError("duration must be a nonnegative multiple of sample_period_us")

    initial_state = np.asarray(fixture["initial_state"], dtype=np.float64)
    if initial_state.shape != (8,) or not np.all(np.isfinite(initial_state)):
        raise ValueError("initial_state must contain eight finite values")

    previous_at_us = -1
    for step in fixture["input_profile"]:
        at_us = int(step["at_us"])
        torques = np.asarray(
            [step["drive_torque_nm"], step["reaction_wheel_torque_nm"]],
            dtype=np.float64,
        )
        if (
            at_us < 0
            or at_us > duration_us
            or at_us % sample_period_us != 0
            or at_us <= previous_at_us
            or not np.all(np.isfinite(torques))
        ):
            raise ValueError("input_profile must be finite, ordered, and sample-aligned")
        previous_at_us = at_us
    return fixture


def simulate_reference(fixture: dict[str, Any]) -> list[dict[str, Any]]:
    parameters = PlantParameters.from_mapping(fixture["plant"])
    a, b = upright_state_space(parameters)
    sample_period_us = int(fixture["sample_period_us"])
    duration_us = int(fixture["duration_us"])
    a_d, b_d = exact_zoh_step(a, b, sample_period_us * 1.0e-6)

    state = np.asarray(fixture["initial_state"], dtype=np.float64).copy()
    current_input = np.zeros(2, dtype=np.float64)
    profile = fixture["input_profile"]
    profile_index = 0
    samples: list[dict[str, Any]] = []

    for at_us in range(0, duration_us + 1, sample_period_us):
        while profile_index < len(profile) and int(profile[profile_index]["at_us"]) == at_us:
            current_input = np.asarray(
                [
                    profile[profile_index]["drive_torque_nm"],
                    profile[profile_index]["reaction_wheel_torque_nm"],
                ],
                dtype=np.float64,
            )
            profile_index += 1

        samples.append(
            {
                "time_us": at_us,
                "state": state.tolist(),
                "applied_input": current_input.tolist(),
            }
        )
        if at_us != duration_us:
            state = a_d @ state + b_d @ current_input

    if profile_index != len(profile):
        raise ValueError("not all input_profile events were consumed")
    return samples


def compare_rust_trace(
    reference: list[dict[str, Any]], rust_trace_path: Path, max_abs_error: float
) -> dict[str, Any]:
    rust = json.loads(rust_trace_path.read_text(encoding="utf-8"))
    rust_samples = rust["samples"]
    if len(rust_samples) != len(reference):
        raise ValueError("Rust/reference trace lengths differ")

    per_state_max = np.zeros(8, dtype=np.float64)
    for expected, actual in zip(reference, rust_samples, strict=True):
        if int(actual["time_us"]) != int(expected["time_us"]):
            raise ValueError("Rust/reference sample timestamps differ")

        expected_input = np.asarray(expected["applied_input"], dtype=np.float64)
        actual_input = np.asarray(actual["applied_input"], dtype=np.float64)
        if not np.allclose(actual_input, expected_input, rtol=0.0, atol=1.0e-7):
            raise ValueError(f"applied input mismatch at {expected['time_us']} us")

        expected_state = np.asarray(expected["state"], dtype=np.float64)
        actual_state = np.asarray(actual["state"], dtype=np.float64)
        per_state_max = np.maximum(per_state_max, np.abs(actual_state - expected_state))

    overall = float(np.max(per_state_max))
    summary = {
        "pass": overall <= max_abs_error,
        "max_abs_error": overall,
        "max_abs_error_limit": max_abs_error,
        "per_state_max_abs_error": {
            name: float(error) for name, error in zip(STATE_NAMES, per_state_max, strict=True)
        },
        "sample_count": len(reference),
    }
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--rust-trace", type=Path)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--max-abs-error", type=float, default=5.0e-6)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not np.isfinite(args.max_abs_error) or args.max_abs_error <= 0.0:
        raise ValueError("--max-abs-error must be positive and finite")

    fixture = load_fixture(args.fixture)
    reference = simulate_reference(fixture)

    if args.rust_trace is None:
        final = {
            "pass": True,
            "sample_count": len(reference),
            "message": "reference trajectory generated",
        }
    else:
        final = compare_rust_trace(reference, args.rust_trace, args.max_abs_error)

    if args.summary is not None:
        args.summary.write_text(json.dumps(final, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(json.dumps(final, indent=2, sort_keys=True))
    return 0 if final["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
