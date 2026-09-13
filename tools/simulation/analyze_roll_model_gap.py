#!/usr/bin/env python3
"""Quantify the first roll-model boundary exposed by the #18 Webots baseline.

The diagnostic is deliberately asymmetric about what is allowed to fail CI:
raw gyro sign/unit mapping is a hard bridge contract, while accelerometer and
roll-dynamics residuals are evidence used to localize model-form mismatch. The
latter are reported, not tuned into pass/fail thresholds.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

from webots.bridge_protocol import (
    ACCEL_LSB_PER_G,
    GYRO_LSB_PER_DPS,
    STANDARD_GRAVITY_MPS2,
)


class RollModelGapError(ValueError):
    pass


def _load_jsonl(path: Path) -> list[dict]:
    records: list[dict] = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        if not line.strip():
            continue
        try:
            item = json.loads(line)
        except json.JSONDecodeError as exc:
            raise RollModelGapError(
                f"invalid JSON at {path}:{line_number}: {exc}"
            ) from exc
        if not isinstance(item, dict):
            raise RollModelGapError(f"record {line_number} is not an object")
        records.append(item)
    if len(records) < 3:
        raise RollModelGapError(
            "roll-model diagnostic requires at least three trace records"
        )
    return records


def _finite(value, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise RollModelGapError(f"{name} must be finite")
    return result


def _stats(values: Iterable[float]) -> dict[str, float | int | None]:
    data = [float(value) for value in values]
    if not data:
        return {"count": 0, "rms": None, "max_abs": None, "mean": None}
    return {
        "count": len(data),
        "rms": math.sqrt(sum(value * value for value in data) / len(data)),
        "max_abs": max(abs(value) for value in data),
        "mean": sum(data) / len(data),
    }


def _fixture_terms(path: Path) -> dict[str, float]:
    document = json.loads(path.read_text(encoding="utf-8"))
    provenance = str(document.get("provenance", ""))
    if "synthetic" not in provenance.lower() or "not ONE V2" not in provenance:
        raise RollModelGapError(
            "fixture must remain explicitly synthetic/non-ONE-V2"
        )
    parameters = document.get("reduced_model_parameters")
    if not isinstance(parameters, dict):
        raise RollModelGapError("fixture lacks reduced_model_parameters")
    required = (
        "gravity_m_per_s2",
        "body_mass_kg",
        "body_com_height_m",
        "body_inertia_roll_kg_m2",
        "reaction_wheel_mass_kg",
        "reaction_wheel_com_height_m",
    )
    values = {
        name: _finite(
            parameters.get(name), f"reduced_model_parameters.{name}"
        )
        for name in required
    }
    if any(value <= 0.0 for value in values.values()):
        raise RollModelGapError(
            "required reduced-model parameters must be > 0"
        )
    h = (
        values["body_mass_kg"] * values["body_com_height_m"]
        + values["reaction_wheel_mass_kg"]
        * values["reaction_wheel_com_height_m"]
    )
    s = (
        values["body_mass_kg"] * values["body_com_height_m"] ** 2
        + values["reaction_wheel_mass_kg"]
        * values["reaction_wheel_com_height_m"] ** 2
    )
    j_phi = s + values["body_inertia_roll_kg_m2"]
    return {
        "gravity_m_per_s2": values["gravity_m_per_s2"],
        "gravitational_first_moment_kg_m": h,
        "roll_body_inertia_kg_m2": j_phi,
    }


def analyze(
    trace_path: Path,
    fixture_path: Path,
    early_window_s: float = 0.05,
) -> dict[str, object]:
    if not math.isfinite(early_window_s) or early_window_s <= 0.0:
        raise RollModelGapError(
            "early_window_s must be finite and > 0"
        )
    records = _load_jsonl(trace_path)
    terms = _fixture_terms(fixture_path)
    g = terms["gravity_m_per_s2"]
    h = terms["gravitational_first_moment_kg_m"]
    j_phi = terms["roll_body_inertia_kg_m2"]

    times: list[float] = []
    truth_roll: list[float] = []
    truth_roll_rate: list[float] = []
    gyro_residual: list[float] = []
    accel_y_residual: list[float] = []
    estimator_roll_error: list[float] = []
    estimator_roll_error_early: list[float] = []
    gyro_sign_mismatch_count = 0
    gyro_sign_compared_count = 0

    gyro_step_rad_s = (
        (1.0 / GYRO_LSB_PER_DPS) * math.pi / 180.0
    )
    for index, record in enumerate(records):
        if record.get("mode") != "closed_loop_production_path":
            raise RollModelGapError(
                f"record {index} is not closed-loop production-path evidence"
            )
        time_s = _finite(
            record.get("time_s"), f"record[{index}].time_s"
        )
        if times and time_s <= times[-1]:
            raise RollModelGapError("trace time must increase strictly")
        raw = record.get("raw_device_observation")
        truth = record.get("webots_evidence_truth")
        production = record.get("production")
        if (
            not isinstance(raw, dict)
            or not isinstance(truth, dict)
            or not isinstance(production, dict)
        ):
            raise RollModelGapError(
                f"record {index} lacks required evidence objects"
            )
        if raw.get("mapping_id") != "webots-body-identity-v1":
            raise RollModelGapError(
                f"record {index} does not use canonical Webots mapping"
            )
        gyro_raw = raw.get("gyro_raw")
        accel_raw = raw.get("accel_raw")
        if not isinstance(gyro_raw, list) or len(gyro_raw) != 3:
            raise RollModelGapError(
                f"record {index} gyro_raw must be a 3-vector"
            )
        if not isinstance(accel_raw, list) or len(accel_raw) != 3:
            raise RollModelGapError(
                f"record {index} accel_raw must be a 3-vector"
            )

        phi = _finite(
            truth.get("body_roll_rad"), f"record[{index}].truth.roll"
        )
        phi_dot = _finite(
            truth.get("body_roll_rate_rad_per_s"),
            f"record[{index}].truth.roll_rate",
        )
        decoded_gyro_x = (
            int(gyro_raw[0]) / GYRO_LSB_PER_DPS * math.pi / 180.0
        )
        decoded_accel_y = (
            int(accel_raw[1])
            / ACCEL_LSB_PER_G
            * STANDARD_GRAVITY_MPS2
        )
        predicted_accel_y = g * phi

        times.append(time_s)
        truth_roll.append(phi)
        truth_roll_rate.append(phi_dot)
        gyro_residual.append(decoded_gyro_x - phi_dot)
        accel_y_residual.append(decoded_accel_y - predicted_accel_y)

        if abs(phi_dot) >= gyro_step_rad_s:
            gyro_sign_compared_count += 1
            if (
                decoded_gyro_x == 0.0
                or math.copysign(1.0, decoded_gyro_x)
                != math.copysign(1.0, phi_dot)
            ):
                gyro_sign_mismatch_count += 1

        estimate = production.get("estimate")
        if isinstance(estimate, dict):
            estimate_roll = _finite(
                estimate.get("body_roll_rad"),
                f"record[{index}].estimate.roll",
            )
            error = estimate_roll - phi
            estimator_roll_error.append(error)
            if time_s <= early_window_s + 1.0e-12:
                estimator_roll_error_early.append(error)

    early_indices = [
        index
        for index, time_s in enumerate(times)
        if time_s <= early_window_s + 1.0e-12
    ]
    gyro_early = [gyro_residual[index] for index in early_indices]
    accel_early = [accel_y_residual[index] for index in early_indices]

    roll_dynamics_residual: list[float] = []
    roll_dynamics_residual_early: list[float] = []
    for index in range(1, len(records)):
        dt = times[index] - times[index - 1]
        actual_phi_ddot = (
            truth_roll_rate[index] - truth_roll_rate[index - 1]
        ) / dt
        phi_mid = 0.5 * (
            truth_roll[index] + truth_roll[index - 1]
        )
        # The torque recorded at k-1 is applied by Webots over (t[k-1], t[k]).
        # Using same-sample torque here would introduce a one-period alignment
        # error and could falsely accuse the plant model.
        tau_previous = _finite(
            records[index - 1].get("authorized_reaction_torque_nm"),
            f"record[{index - 1}].authorized_reaction_torque_nm",
        )
        predicted_phi_ddot = (
            h * g / j_phi * phi_mid - tau_previous / j_phi
        )
        residual = actual_phi_ddot - predicted_phi_ddot
        roll_dynamics_residual.append(residual)
        if times[index] <= early_window_s + 1.0e-12:
            roll_dynamics_residual_early.append(residual)

    gyro_tolerance = 0.5 * gyro_step_rad_s + 1.0e-9
    gyro_full_stats = _stats(gyro_residual)
    canonical_gyro = (
        gyro_full_stats["max_abs"] is not None
        and float(gyro_full_stats["max_abs"]) <= gyro_tolerance
        and gyro_sign_mismatch_count == 0
    )

    return {
        "schema_version": 1,
        "provenance": (
            "synthetic model-boundary diagnostic for Issue #25; "
            "not ONE V2 physical evidence"
        ),
        "trace": str(trace_path),
        "fixture": str(fixture_path),
        "records": len(records),
        "end_time_s": times[-1],
        "early_window_s": early_window_s,
        "reduced_roll_terms": terms,
        "gyro_bridge_check": {
            "quantization_step_rad_per_s": gyro_step_rad_s,
            "half_lsb_tolerance_rad_per_s": gyro_tolerance,
            "full": gyro_full_stats,
            "early": _stats(gyro_early),
            "sign_compared_count": gyro_sign_compared_count,
            "sign_mismatch_count": gyro_sign_mismatch_count,
            "canonical_sign_unit_consistent": canonical_gyro,
        },
        "reduced_measurement_residual_accel_y_m_per_s2": {
            "equation": (
                "a_y = g * phi at the declared axle-origin IMU placement"
            ),
            "full": _stats(accel_y_residual),
            "early": _stats(accel_early),
        },
        "reduced_roll_dynamics_residual_rad_per_s2": {
            "equation": (
                "phi_ddot = (H*g/J_phi)*phi - "
                "tau_reaction/J_phi"
            ),
            "torque_alignment": (
                "authorized reaction torque at sample k-1 is paired with "
                "truth acceleration over (t[k-1], t[k])"
            ),
            "full": _stats(roll_dynamics_residual),
            "early": _stats(roll_dynamics_residual_early),
        },
        "estimator_roll_error_rad": {
            "full": _stats(estimator_roll_error),
            "early": _stats(estimator_roll_error_early),
        },
        "interpretation_boundary": (
            "gyro mapping is a hard raw-device contract; measurement/dynamics "
            "residuals are diagnostic evidence, not controller-tuning targets"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument(
        "--fixture",
        type=Path,
        default=Path(
            "tools/simulation/fixtures/closed-loop-aggregate-equivalent.json"
        ),
    )
    parser.add_argument("--early-window-s", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--require-canonical-gyro",
        action="store_true",
        help=(
            "fail if raw gyro-X sign/unit differs from Webots truth by more "
            "than half one bridge LSB"
        ),
    )
    args = parser.parse_args()

    try:
        result = analyze(
            args.trace, args.fixture, args.early_window_s
        )
    except (OSError, json.JSONDecodeError, RollModelGapError) as exc:
        parser.error(str(exc))

    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")

    if (
        args.require_canonical_gyro
        and not result["gyro_bridge_check"][
            "canonical_sign_unit_consistent"
        ]
    ):
        print("canonical Webots gyro sign/unit contract failed", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
