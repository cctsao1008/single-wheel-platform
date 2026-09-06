#!/usr/bin/env python3
"""Fit the canonical static actuator model from prepared identification data.

CSV columns:
    command,speed_rad_s,torque_nm

Model:
    torque = K_u * effective(command, deadzone)
             - b * speed
             - tau_c * sign(speed)

Deadzone is selected by bounded grid search. Remaining coefficients are solved by
least squares. The script rejects non-physical fits rather than silently taking
absolute values.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def effective_command(command: np.ndarray, deadzone: float) -> np.ndarray:
    magnitude = np.abs(command)
    result = np.zeros_like(command, dtype=float)
    active = magnitude > deadzone
    result[active] = np.sign(command[active]) * (
        (magnitude[active] - deadzone) / (1.0 - deadzone)
    )
    return result


def load_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    command: list[float] = []
    speed: list[float] = []
    torque: list[float] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {"command", "speed_rad_s", "torque_nm"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"CSV must contain {sorted(required)}")
        for row in reader:
            command.append(float(row["command"]))
            speed.append(float(row["speed_rad_s"]))
            torque.append(float(row["torque_nm"]))
    if len(command) < 8:
        raise ValueError("at least 8 identification samples are required")
    arrays = tuple(np.asarray(values, dtype=float) for values in (command, speed, torque))
    if not all(np.isfinite(values).all() for values in arrays):
        raise ValueError("identification data must be finite")
    if np.max(np.abs(arrays[0])) > 1.0:
        raise ValueError("command must be normalized to [-1, 1]")
    return arrays


def fit(
    command: np.ndarray,
    speed: np.ndarray,
    torque: np.ndarray,
    *,
    max_deadzone: float = 0.35,
    deadzone_steps: int = 351,
    sign_epsilon_rad_s: float = 0.5,
) -> dict[str, float]:
    if not 0.0 <= max_deadzone < 1.0:
        raise ValueError("max_deadzone must be in [0, 1)")
    if deadzone_steps < 2:
        raise ValueError("deadzone_steps must be >= 2")
    if sign_epsilon_rad_s <= 0.0:
        raise ValueError("sign epsilon must be positive")

    friction_sign = np.where(
        speed > sign_epsilon_rad_s,
        1.0,
        np.where(speed < -sign_epsilon_rad_s, -1.0, 0.0),
    )

    best: tuple[float, float, np.ndarray] | None = None
    for deadzone in np.linspace(0.0, max_deadzone, deadzone_steps):
        eff = effective_command(command, float(deadzone))
        design = np.column_stack((eff, -speed, -friction_sign))
        coefficients, _, _, _ = np.linalg.lstsq(design, torque, rcond=None)
        ku, viscous, coulomb = coefficients
        if ku <= 0.0 or viscous < 0.0 or coulomb < 0.0:
            continue
        residual = torque - design @ coefficients
        rmse = float(np.sqrt(np.mean(residual * residual)))
        if best is None or rmse < best[0]:
            best = (rmse, float(deadzone), coefficients)

    if best is None:
        raise ValueError("no physically admissible actuator fit found")

    rmse, deadzone, coefficients = best
    ku, viscous, coulomb = (float(value) for value in coefficients)
    return {
        "torque_per_effective_command_nm": ku,
        "command_deadzone": deadzone,
        "viscous_friction_nm_per_rad_s": viscous,
        "coulomb_friction_nm": coulomb,
        "friction_sign_epsilon_rad_s": sign_epsilon_rad_s,
        "fit_rmse_nm": rmse,
        "sample_count": int(command.size),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-deadzone", type=float, default=0.35)
    parser.add_argument("--deadzone-steps", type=int, default=351)
    parser.add_argument("--sign-epsilon-rad-s", type=float, default=0.5)
    args = parser.parse_args()

    result = fit(
        *load_csv(args.csv),
        max_deadzone=args.max_deadzone,
        deadzone_steps=args.deadzone_steps,
        sign_epsilon_rad_s=args.sign_epsilon_rad_s,
    )
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
