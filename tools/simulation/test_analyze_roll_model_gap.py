from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest

import analyze_roll_model_gap as gap
from webots.bridge_protocol import (
    ACCEL_LSB_PER_G,
    GYRO_LSB_PER_DPS,
    STANDARD_GRAVITY_MPS2,
)


def write_fixture(directory: Path) -> Path:
    fixture = directory / "fixture.json"
    fixture.write_text(
        json.dumps(
            {
                "provenance": "synthetic test fixture; not ONE V2",
                "reduced_model_parameters": {
                    "gravity_m_per_s2": 9.80665,
                    "body_mass_kg": 1.0,
                    "body_com_height_m": 0.1,
                    "body_inertia_roll_kg_m2": 0.01,
                    "reaction_wheel_mass_kg": 0.1,
                    "reaction_wheel_com_height_m": 0.1,
                },
            }
        ),
        encoding="utf-8",
    )
    return fixture


def gyro_code(rate_rad_s: float) -> int:
    return round(
        rate_rad_s * GYRO_LSB_PER_DPS * 180.0 / math.pi
    )


def accel_code(accel_m_per_s2: float) -> int:
    return round(
        accel_m_per_s2
        * ACCEL_LSB_PER_G
        / STANDARD_GRAVITY_MPS2
    )


def record(
    index: int,
    *,
    roll: float,
    rate: float,
    torque: float,
    accel_y: float | None = None,
    gyro_sign: int = 1,
) -> dict:
    if accel_y is None:
        accel_y = STANDARD_GRAVITY_MPS2 * roll
    return {
        "mode": "closed_loop_production_path",
        "time_s": (index + 1) * 0.002,
        "raw_device_observation": {
            "mapping_id": "webots-body-identity-v1",
            "gyro_raw": [gyro_sign * gyro_code(rate), 0, 0],
            "accel_raw": [
                0,
                accel_code(accel_y),
                accel_code(STANDARD_GRAVITY_MPS2),
            ],
        },
        "production": {
            "estimate": {
                "body_roll_rad": roll,
            }
        },
        "authorized_reaction_torque_nm": torque,
        "webots_evidence_truth": {
            "body_roll_rad": roll,
            "body_roll_rate_rad_per_s": rate,
        },
    }


def write_trace(directory: Path, records: list[dict]) -> Path:
    trace = directory / "trace.jsonl"
    trace.write_text(
        "".join(json.dumps(item) + "\n" for item in records),
        encoding="utf-8",
    )
    return trace


class RollModelGapTests(unittest.TestCase):
    def test_canonical_gyro_mapping_is_within_half_lsb(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            fixture = write_fixture(directory)
            records = [
                record(0, roll=0.0, rate=0.01, torque=0.0),
                record(1, roll=0.0, rate=0.02, torque=0.0),
                record(2, roll=0.0, rate=0.03, torque=0.0),
            ]
            result = gap.analyze(
                write_trace(directory, records), fixture
            )
            check = result["gyro_bridge_check"]
            self.assertTrue(check["canonical_sign_unit_consistent"])
            self.assertEqual(check["sign_mismatch_count"], 0)
            self.assertLessEqual(
                check["full"]["max_abs"],
                check["half_lsb_tolerance_rad_per_s"],
            )

    def test_broken_gyro_sign_is_localized_at_raw_bridge_boundary(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            fixture = write_fixture(directory)
            records = [
                record(
                    0,
                    roll=0.0,
                    rate=0.1,
                    torque=0.0,
                    gyro_sign=-1,
                ),
                record(
                    1,
                    roll=0.0,
                    rate=0.1,
                    torque=0.0,
                    gyro_sign=-1,
                ),
                record(
                    2,
                    roll=0.0,
                    rate=0.1,
                    torque=0.0,
                    gyro_sign=-1,
                ),
            ]
            result = gap.analyze(
                write_trace(directory, records), fixture
            )
            self.assertFalse(
                result["gyro_bridge_check"][
                    "canonical_sign_unit_consistent"
                ]
            )
            self.assertGreater(
                result["gyro_bridge_check"]["sign_mismatch_count"], 0
            )

    def test_roll_dynamics_uses_prior_authorized_torque_for_interval(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            fixture = write_fixture(directory)
            # For this fixture J_phi=0.021. With phi=0 and tau=0.021 N m,
            # the reduced equation predicts phi_ddot=-1 rad/s^2. A 2 ms
            # interval therefore changes roll rate by -0.002 rad/s.
            records = [
                record(0, roll=0.0, rate=0.0, torque=0.021),
                record(1, roll=0.0, rate=-0.002, torque=0.0),
                record(2, roll=0.0, rate=-0.002, torque=0.0),
            ]
            result = gap.analyze(
                write_trace(directory, records), fixture
            )
            residual = result[
                "reduced_roll_dynamics_residual_rad_per_s2"
            ]["full"]
            self.assertLess(residual["max_abs"], 1.0e-9)

    def test_measurement_residual_reports_model_gap_without_relabeling_gyro(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            fixture = write_fixture(directory)
            records = [
                record(
                    index,
                    roll=0.01,
                    rate=0.0,
                    torque=0.0,
                    accel_y=1.0,
                )
                for index in range(3)
            ]
            result = gap.analyze(
                write_trace(directory, records), fixture
            )
            self.assertTrue(
                result["gyro_bridge_check"][
                    "canonical_sign_unit_consistent"
                ]
            )
            self.assertGreater(
                result[
                    "reduced_measurement_residual_accel_y_m_per_s2"
                ]["full"]["max_abs"],
                0.8,
            )


if __name__ == "__main__":
    unittest.main()
