#!/usr/bin/env python3
"""Synthesize the 500 Hz upright observer and LQR/LQI design from evidenced parameters."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.linalg import eigvals, expm, solve_discrete_are

STATE_NAMES = [
    "forward_position_m",
    "forward_velocity_m_per_s",
    "pitch_rad",
    "pitch_rate_rad_per_s",
    "roll_rad",
    "roll_rate_rad_per_s",
    "reaction_wheel_rate_rad_per_s",
]
INPUT_NAMES = ["drive_torque_nm", "reaction_wheel_torque_nm"]
MEASUREMENT_NAMES = [
    "accel_x_m_per_s2",
    "accel_y_m_per_s2",
    "accel_z_m_per_s2",
    "gyro_x_rad_per_s",
    "gyro_y_rad_per_s",
    "gyro_z_rad_per_s",
    "drive_encoder_relative_angle_rad",
    "reaction_wheel_relative_rate_rad_per_s",
]

PARAMETER_NAMES = [
    "gravity_m_per_s2",
    "body_mass_kg",
    "body_com_height_m",
    "body_inertia_roll_kg_m2",
    "body_inertia_pitch_kg_m2",
    "body_inertia_yaw_kg_m2",
    "drive_wheel_mass_kg",
    "drive_wheel_radius_m",
    "drive_wheel_spin_inertia_kg_m2",
    "reaction_wheel_mass_kg",
    "reaction_wheel_com_height_m",
    "reaction_wheel_spin_inertia_kg_m2",
    "reaction_wheel_transverse_inertia_kg_m2",
]


class DesignError(ValueError):
    pass


def finite_positive(value: Any, name: str) -> float:
    if not isinstance(value, (int, float)):
        raise DesignError(f"{name} must be a measured/identified number, got {value!r}")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise DesignError(f"{name} must be finite and > 0, got {result!r}")
    return result


def finite_number(value: Any, name: str) -> float:
    if not isinstance(value, (int, float)):
        raise DesignError(f"{name} must be a measured/identified number, got {value!r}")
    result = float(value)
    if not math.isfinite(result):
        raise DesignError(f"{name} must be finite, got {result!r}")
    return result


def positive_vector(value: Any, length: int, name: str) -> np.ndarray:
    if not isinstance(value, list) or len(value) != length:
        raise DesignError(f"{name} must contain exactly {length} values")
    return np.array(
        [finite_positive(item, f"{name}[{index}]") for index, item in enumerate(value)],
        dtype=float,
    )


def matrix(value: Any, rows: int, columns: int, name: str) -> np.ndarray:
    if not isinstance(value, list) or len(value) != rows:
        raise DesignError(f"{name} must contain exactly {rows} rows")
    result = np.zeros((rows, columns), dtype=float)
    for row_index, row in enumerate(value):
        if not isinstance(row, list) or len(row) != columns:
            raise DesignError(f"{name}[{row_index}] must contain exactly {columns} values")
        for column_index, item in enumerate(row):
            result[row_index, column_index] = finite_number(
                item, f"{name}[{row_index}][{column_index}]"
            )
    return result


def load_parameters(config: dict[str, Any]) -> dict[str, float]:
    raw = config.get("plant_parameters")
    if not isinstance(raw, dict):
        raise DesignError("plant_parameters must be an object")
    return {
        name: finite_positive(raw.get(name), f"plant_parameters.{name}")
        for name in PARAMETER_NAMES
    }


def continuous_plant(
    p: dict[str, float],
) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
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
    j_theta = (
        s
        + p["body_inertia_pitch_kg_m2"]
        + p["reaction_wheel_transverse_inertia_kg_m2"]
    )
    j_phi = s + p["body_inertia_roll_kg_m2"]
    delta = m_s * j_theta - h * h
    if not math.isfinite(delta) or delta <= 0.0:
        raise DesignError(f"upright pitch inertia determinant must be > 0, got {delta}")

    g = p["gravity_m_per_s2"]
    r = p["drive_wheel_radius_m"]
    j_r = p["reaction_wheel_spin_inertia_kg_m2"]

    a = np.zeros((7, 7), dtype=float)
    b = np.zeros((7, 2), dtype=float)
    a[0, 1] = 1.0
    a[1, 2] = -(h * h * g) / delta
    a[2, 3] = 1.0
    a[3, 2] = h * m_s * g / delta
    a[4, 5] = 1.0
    a[5, 4] = h * g / j_phi
    a[6, 4] = -(h * g / j_phi)

    b[1, 0] = (j_theta / r + h) / delta
    b[3, 0] = -(h / r + m_s) / delta
    b[5, 1] = -1.0 / j_phi
    b[6, 1] = 1.0 / j_r + 1.0 / j_phi

    aggregates = {
        "gravitational_first_moment_kg_m": h,
        "vertical_second_moment_kg_m2": s,
        "equivalent_translation_mass_kg": m_s,
        "pitch_inertia_kg_m2": j_theta,
        "roll_body_inertia_kg_m2": j_phi,
        "pitch_inertia_determinant_kg2_m2": delta,
    }
    return a, b, aggregates


def zoh_discretize(
    a: np.ndarray, b: np.ndarray, sample_period_s: float
) -> tuple[np.ndarray, np.ndarray]:
    augmented = np.zeros(
        (a.shape[0] + b.shape[1], a.shape[1] + b.shape[1]), dtype=float
    )
    augmented[: a.shape[0], : a.shape[1]] = a
    augmented[: a.shape[0], a.shape[1] :] = b
    transition = expm(augmented * sample_period_s)
    return (
        transition[: a.shape[0], : a.shape[1]],
        transition[: a.shape[0], a.shape[1] :],
    )


def measurement_model(
    p: dict[str, float], a: np.ndarray, b: np.ndarray, config: dict[str, Any]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    placement = config.get("imu_placement_m")
    if not isinstance(placement, dict):
        raise DesignError("imu_placement_m must be an object")
    x_i = finite_number(placement.get("forward_x_m"), "imu_placement_m.forward_x_m")
    y_i = finite_number(placement.get("left_y_m"), "imu_placement_m.left_y_m")
    z_i = finite_number(placement.get("up_z_m"), "imu_placement_m.up_z_m")

    c = np.zeros((8, 7), dtype=float)
    d = np.zeros((8, 2), dtype=float)
    nominal = np.zeros(8, dtype=float)
    g = p["gravity_m_per_s2"]
    r = p["drive_wheel_radius_m"]
    nominal[2] = g

    c[0, :] = a[1, :] + z_i * a[3, :]
    c[1, :] = -z_i * a[5, :]
    c[2, :] = -x_i * a[3, :] + y_i * a[5, :]
    c[0, 2] -= g
    c[1, 4] += g

    d[0, :] = b[1, :] + z_i * b[3, :]
    d[1, :] = -z_i * b[5, :]
    d[2, :] = -x_i * b[3, :] + y_i * b[5, :]

    c[3, 5] = 1.0
    c[4, 3] = 1.0
    c[6, 0] = 1.0 / r
    c[6, 2] = -1.0
    c[7, 6] = 1.0
    return nominal, c, d


def lqr_gain(
    ad: np.ndarray,
    bd: np.ndarray,
    state_scale: np.ndarray,
    input_scale: np.ndarray,
) -> tuple[np.ndarray, float]:
    q = np.diag(1.0 / np.square(state_scale))
    r = np.diag(1.0 / np.square(input_scale))
    p = solve_discrete_are(ad, bd, q, r)
    k = np.linalg.solve(r + bd.T @ p @ bd, bd.T @ p @ ad)
    spectral_radius = float(np.max(np.abs(eigvals(ad - bd @ k))))
    if not math.isfinite(spectral_radius) or spectral_radius >= 1.0:
        raise DesignError(
            f"LQR design is not discrete-time stable; spectral radius={spectral_radius}"
        )
    return k, spectral_radius


def lqi_gain(
    ad: np.ndarray,
    bd: np.ndarray,
    state_scale: np.ndarray,
    input_scale: np.ndarray,
    config: dict[str, Any],
    sample_period_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float] | None:
    raw = config.get("lqi")
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise DesignError("lqi must be null or an object")
    projection = matrix(
        raw.get("integral_projection"), 2, 7, "lqi.integral_projection"
    )
    integral_scale = positive_vector(
        raw.get("integral_scale"), 2, "lqi.integral_scale"
    )

    a_aug = np.block(
        [
            [ad, np.zeros((7, 2), dtype=float)],
            [sample_period_s * projection, np.eye(2, dtype=float)],
        ]
    )
    b_aug = np.vstack([bd, np.zeros((2, 2), dtype=float)])
    q_aug = np.diag(1.0 / np.square(np.concatenate([state_scale, integral_scale])))
    r = np.diag(1.0 / np.square(input_scale))
    p_aug = solve_discrete_are(a_aug, b_aug, q_aug, r)
    k_aug = np.linalg.solve(r + b_aug.T @ p_aug @ b_aug, b_aug.T @ p_aug @ a_aug)
    spectral_radius = float(np.max(np.abs(eigvals(a_aug - b_aug @ k_aug))))
    if not math.isfinite(spectral_radius) or spectral_radius >= 1.0:
        raise DesignError(
            f"LQI design is not discrete-time stable; spectral radius={spectral_radius}"
        )
    return k_aug[:, :7], k_aug[:, 7:], projection, spectral_radius


def observer_gain(
    ad: np.ndarray,
    c: np.ndarray,
    process_std: np.ndarray,
    measurement_std: np.ndarray,
    selected: list[int],
) -> tuple[np.ndarray, float, int]:
    if not selected:
        raise DesignError(
            "observer.required_measurements must contain at least one measurement"
        )
    c_selected = c[selected, :]
    r_selected = np.diag(np.square(measurement_std[selected]))
    q_process = np.diag(np.square(process_std))

    covariance = np.eye(7, dtype=float)
    gain_selected = np.zeros((7, len(selected)), dtype=float)
    iterations = 0
    for iterations in range(1, 20001):
        predicted_covariance = ad @ covariance @ ad.T + q_process
        innovation_covariance = (
            c_selected @ predicted_covariance @ c_selected.T + r_selected
        )
        gain_selected = np.linalg.solve(
            innovation_covariance.T, (predicted_covariance @ c_selected.T).T
        ).T
        next_covariance = (
            np.eye(7) - gain_selected @ c_selected
        ) @ predicted_covariance
        next_covariance = 0.5 * (next_covariance + next_covariance.T)
        if np.max(np.abs(next_covariance - covariance)) < 1.0e-12:
            covariance = next_covariance
            break
        covariance = next_covariance
    else:
        raise DesignError("observer Riccati iteration did not converge")

    full_gain = np.zeros((7, 8), dtype=float)
    for local_index, measurement_index in enumerate(selected):
        full_gain[:, measurement_index] = gain_selected[:, local_index]

    error_transition = (np.eye(7) - gain_selected @ c_selected) @ ad
    spectral_radius = float(np.max(np.abs(eigvals(error_transition))))
    if not math.isfinite(spectral_radius) or spectral_radius >= 1.0:
        raise DesignError(
            "observer design is not asymptotically stable for the selected measurement set; "
            f"spectral radius={spectral_radius}"
        )
    return full_gain, spectral_radius, iterations


def measurement_indices(raw: Any) -> list[int]:
    if not isinstance(raw, list):
        raise DesignError(
            "observer.required_measurements must be a list of measurement names"
        )
    indices: list[int] = []
    for item in raw:
        if item not in MEASUREMENT_NAMES:
            raise DesignError(f"unknown observer measurement {item!r}")
        index = MEASUREMENT_NAMES.index(item)
        if index in indices:
            raise DesignError(f"duplicate observer measurement {item!r}")
        indices.append(index)
    return indices


def rust_array(value: np.ndarray, indent: int = 0) -> str:
    prefix = " " * indent
    if value.ndim == 1:
        return "[" + ", ".join(f"{float(item):.9e}_f32" for item in value) + "]"
    rows = [prefix + "    " + rust_array(row, indent + 4) + "," for row in value]
    return "[\n" + "\n".join(rows) + "\n" + prefix + "]"


def emit_rust(result: dict[str, Any]) -> str:
    ad = np.array(result["discrete_plant"]["a_d"], dtype=float)
    bd = np.array(result["discrete_plant"]["b_d"], dtype=float)
    nominal = np.array(result["measurement_model"]["nominal"], dtype=float)
    c = np.array(result["measurement_model"]["c"], dtype=float)
    d = np.array(result["measurement_model"]["d"], dtype=float)
    k = np.array(result["lqr"]["k"], dtype=float)
    l = np.array(result["observer"]["l"], dtype=float)
    bits = int(result["observer"]["required_measurement_mask_bits"])

    lines = [
        "// Generated by tools/control/synthesize_upright.py from evidenced design inputs.",
        "// Do not hand-edit numeric matrices; regenerate from the canonical design input.",
        f"pub const SAMPLE_PERIOD_S: f32 = {float(result['sample_period_s']):.9e}_f32;",
        f"pub const A_D: [[f32; 7]; 7] = {rust_array(ad)};",
        f"pub const B_D: [[f32; 2]; 7] = {rust_array(bd)};",
        f"pub const Y_NOMINAL: [f32; 8] = {rust_array(nominal)};",
        f"pub const C: [[f32; 7]; 8] = {rust_array(c)};",
        f"pub const D: [[f32; 2]; 8] = {rust_array(d)};",
        f"pub const LQR_K: [[f32; 7]; 2] = {rust_array(k)};",
        f"pub const OBSERVER_L: [[f32; 8]; 7] = {rust_array(l)};",
        f"pub const OBSERVER_REQUIRED_MEASUREMENT_MASK_BITS: u16 = 0x{bits:04x};",
    ]

    if result.get("lqi") is not None:
        state_k = np.array(result["lqi"]["state_k"], dtype=float)
        integral_k = np.array(result["lqi"]["integral_k"], dtype=float)
        projection = np.array(result["lqi"]["integral_projection"], dtype=float)
        lines.extend(
            [
                f"pub const LQI_STATE_K: [[f32; 7]; 2] = {rust_array(state_k)};",
                f"pub const LQI_INTEGRAL_K: [[f32; 2]; 2] = {rust_array(integral_k)};",
                f"pub const LQI_INTEGRAL_PROJECTION: [[f32; 7]; 2] = {rust_array(projection)};",
            ]
        )
    return "\n\n".join(lines) + "\n"


def synthesize(config: dict[str, Any]) -> dict[str, Any]:
    sample_period_s = finite_positive(config.get("sample_period_s"), "sample_period_s")
    p = load_parameters(config)
    a, b, aggregates = continuous_plant(p)
    ad, bd = zoh_discretize(a, b, sample_period_s)
    nominal, c, d = measurement_model(p, a, b, config)

    lqr = config.get("lqr")
    if not isinstance(lqr, dict):
        raise DesignError("lqr must be an object")
    state_scale = positive_vector(lqr.get("state_scale"), 7, "lqr.state_scale")
    input_scale = positive_vector(
        lqr.get("input_scale_nm"), 2, "lqr.input_scale_nm"
    )
    k, lqr_radius = lqr_gain(ad, bd, state_scale, input_scale)

    observer = config.get("observer")
    if not isinstance(observer, dict):
        raise DesignError("observer must be an object")
    process_std = positive_vector(
        observer.get("process_std"), 7, "observer.process_std"
    )
    measurement_std = positive_vector(
        observer.get("measurement_std"), 8, "observer.measurement_std"
    )
    selected = measurement_indices(observer.get("required_measurements"))
    l, observer_radius, observer_iterations = observer_gain(
        ad, c, process_std, measurement_std, selected
    )
    mask_bits = sum(1 << index for index in selected)

    lqi = lqi_gain(ad, bd, state_scale, input_scale, config, sample_period_s)
    lqi_result = None
    if lqi is not None:
        state_k, integral_k, projection, lqi_radius = lqi
        lqi_result = {
            "state_k": state_k.tolist(),
            "integral_k": integral_k.tolist(),
            "integral_projection": projection.tolist(),
            "closed_loop_spectral_radius": lqi_radius,
        }

    return {
        "sample_period_s": sample_period_s,
        "state_order": STATE_NAMES,
        "input_order": INPUT_NAMES,
        "measurement_order": MEASUREMENT_NAMES,
        "upright_aggregates": aggregates,
        "continuous_plant": {"a": a.tolist(), "b": b.tolist()},
        "discrete_plant": {"a_d": ad.tolist(), "b_d": bd.tolist()},
        "measurement_model": {
            "nominal": nominal.tolist(),
            "c": c.tolist(),
            "d": d.tolist(),
        },
        "lqr": {"k": k.tolist(), "closed_loop_spectral_radius": lqr_radius},
        "lqi": lqi_result,
        "observer": {
            "l": l.tolist(),
            "required_measurements": [MEASUREMENT_NAMES[index] for index in selected],
            "required_measurement_mask_bits": mask_bits,
            "error_spectral_radius": observer_radius,
            "riccati_iterations": observer_iterations,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="evidenced JSON design input")
    parser.add_argument(
        "--json-output", type=Path, help="write complete synthesized design JSON"
    )
    parser.add_argument(
        "--rust-output", type=Path, help="write generated Rust matrix constants"
    )
    args = parser.parse_args()

    try:
        config = json.loads(args.config.read_text(encoding="utf-8"))
        if not isinstance(config, dict):
            raise DesignError("top-level design input must be a JSON object")
        result = synthesize(config)
    except (OSError, json.JSONDecodeError, DesignError, np.linalg.LinAlgError) as exc:
        parser.error(str(exc))

    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(encoded, encoding="utf-8")
    else:
        print(encoded, end="")

    if args.rust_output:
        args.rust_output.parent.mkdir(parents=True, exist_ok=True)
        args.rust_output.write_text(emit_rust(result), encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
