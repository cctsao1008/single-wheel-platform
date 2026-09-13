from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import analyze_webots_envelope as envelope


CLASSIFICATION = {
    "settling_dwell_s": 0.004,
    "settling_angle_abs_rad": 0.005,
    "settling_rate_abs_rad_per_s": 0.05,
    "hard_attitude_abs_rad": 0.5,
    "runtime_fault_is_loss": True,
    "drive_saturated_reason_bit": 32,
    "reaction_saturated_reason_bit": 64,
}
CASE = {
    "id": "pitch_p0010",
    "axis": "pitch",
    "magnitude_rad": 0.001,
    "initial_pitch_rad": 0.001,
    "initial_roll_rad": 0.0,
}


def record(
    index: int,
    *,
    pitch: float = 0.001,
    roll: float = 0.0,
    pitch_rate: float = 0.0,
    roll_rate: float = 0.0,
    fault_bits: int = 0,
    reason_bits: int = 0,
    torque: float = 0.01,
) -> dict:
    apply = index >= 2 and fault_bits == 0
    production_torque = torque if apply else 0.0
    return {
        "mode": "closed_loop_production_path",
        "time_s": (index + 1) * 0.002,
        "raw_device_observation": {
            "schema": 1,
            "mapping_id": "webots-body-identity-v1",
            "sample_index": index,
            "timestamp_us": (index + 1) * 2_000,
        },
        "production": {
            "sample_index": index,
            "runtime_fault_bits": fault_bits,
            "operating_state": "balancing" if index >= 1 else "capture_window",
            "authority": "closed_loop" if apply else "denied",
            "authority_reason_bits": reason_bits,
            "constrained": bool(reason_bits),
            "actuation": "apply" if apply else "revoke",
            "drive_torque_nm": production_torque,
            "reaction_torque_nm": 0.0,
        },
        "authorized_drive_torque_nm": production_torque,
        "authorized_reaction_torque_nm": 0.0,
        "webots_evidence_truth": {
            "forward_position_m": 0.0,
            "forward_velocity_m_per_s": 0.0,
            "body_pitch_rad": pitch,
            "body_roll_rad": roll,
            "body_yaw_rad": 0.0,
            "body_pitch_rate_rad_per_s": pitch_rate,
            "body_roll_rate_rad_per_s": roll_rate,
            "body_yaw_rate_rad_per_s": 0.0,
            "reaction_position_rad": 0.0,
        },
    }


def write_trace(records: list[dict]) -> Path:
    directory = tempfile.TemporaryDirectory()
    path = Path(directory.name) / "trace.jsonl"
    path.write_text(
        "".join(json.dumps(item) + "\n" for item in records), encoding="utf-8"
    )
    path._temporary_directory = directory  # type: ignore[attr-defined]
    return path


class EnvelopeAnalyzerTests(unittest.TestCase):
    def test_recovered_case_is_classified_without_saturation(self) -> None:
        path = write_trace([record(index) for index in range(5)])
        summary = envelope.analyze(path, CASE, CLASSIFICATION, 0.002)
        self.assertEqual(summary["classification"], "recovered")
        self.assertEqual(summary["saturation_count"], 0)

    def test_saturation_limited_recovery_is_distinct(self) -> None:
        records = [record(index, reason_bits=32 if index == 3 else 0) for index in range(5)]
        path = write_trace(records)
        summary = envelope.analyze(path, CASE, CLASSIFICATION, 0.002)
        self.assertEqual(summary["classification"], "saturation_limited_recovery")
        self.assertEqual(summary["drive_saturation_count"], 1)

    def test_runtime_fault_is_recorded_as_loss_not_evidence_error(self) -> None:
        records = [record(index, fault_bits=8 if index >= 3 else 0) for index in range(5)]
        path = write_trace(records)
        summary = envelope.analyze(path, CASE, CLASSIFICATION, 0.002)
        self.assertEqual(summary["classification"], "loss_of_balance")
        self.assertEqual(summary["first_fault_sample"], 3)

    def test_hard_attitude_excursion_is_loss(self) -> None:
        records = [record(index, pitch=0.6 if index == 3 else 0.001) for index in range(5)]
        path = write_trace(records)
        summary = envelope.analyze(path, CASE, CLASSIFICATION, 0.002)
        self.assertEqual(summary["classification"], "loss_of_balance")
        self.assertEqual(summary["hard_bound_sample"], 3)

    def test_nonsettled_but_fault_free_case_is_not_recovered_by_horizon(self) -> None:
        records = [record(index, pitch=0.02) for index in range(5)]
        path = write_trace(records)
        summary = envelope.analyze(path, CASE, CLASSIFICATION, 0.002)
        self.assertEqual(summary["classification"], "not_recovered_by_horizon")

    def test_truth_leak_into_raw_object_fails_evidence_contract(self) -> None:
        records = [record(index) for index in range(5)]
        records[0]["raw_device_observation"]["body_pitch_rad"] = 0.001
        path = write_trace(records)
        with self.assertRaises(envelope.EnvelopeEvidenceError):
            envelope.analyze(path, CASE, CLASSIFICATION, 0.002)


if __name__ == "__main__":
    unittest.main()
