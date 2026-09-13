from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from validate_webots_closed_loop import EvidenceError, validate


def record(index: int, actuation: str, state: str, authority):
    drive = 0.0 if actuation == "revoke" else 0.01
    reaction = 0.0 if actuation == "revoke" else -0.01
    return {
        "mode": "closed_loop_production_path",
        "time_s": 0.002 * (index + 1),
        "raw_device_observation": {
            "schema": 1,
            "mapping_id": "webots-body-identity-v1",
            "sample_index": index,
            "timestamp_us": 2_000 * (index + 1),
            "accel_raw": [0, 0, 8192],
            "temperature_raw": 0,
            "gyro_raw": [0, 0, 0],
            "drive_encoder_count": 0,
            "reaction_encoder_count": 0,
        },
        "production": {
            "schema": 1,
            "sample_index": index,
            "operating_state": state,
            "runtime_fault_bits": 0,
            "sensor_timing": "healthy" if index else "startup",
            "estimate_validity": "valid" if index else None,
            "authority": authority,
            "authority_reason_bits": 0,
            "constrained": False,
            "hold_integrator": actuation == "revoke",
            "actuation": actuation,
            "drive_torque_nm": drive,
            "reaction_torque_nm": reaction,
            "estimate": None,
            "reference": {},
            "runtime_steps": max(0, index),
            "skipped_unready_observations": 1,
        },
        "authorized_drive_torque_nm": drive,
        "authorized_reaction_torque_nm": reaction,
        "webots_evidence_truth": {
            "forward_position_m": 0.0,
            "forward_velocity_m_per_s": 0.0,
            "body_roll_rad": -0.005 + 0.0001 * index,
            "body_pitch_rad": 0.005 - 0.0001 * index,
            "body_yaw_rad": 0.0,
            "body_roll_rate_rad_per_s": 0.0,
            "body_pitch_rate_rad_per_s": 0.0,
            "body_yaw_rate_rad_per_s": 0.0,
            "reaction_position_rad": 0.0,
        },
    }


class WebotsClosedLoopEvidenceTests(unittest.TestCase):
    def write_trace(self, rows):
        directory = tempfile.TemporaryDirectory()
        path = Path(directory.name) / "trace.jsonl"
        path.write_text(
            "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
        )
        self.addCleanup(directory.cleanup)
        return path

    def test_nominal_authority_sequence_passes(self):
        rows = [
            record(0, "revoke", "capture_window", None),
            record(1, "revoke", "balancing", "denied"),
        ] + [record(index, "apply", "balancing", "closed_loop") for index in range(2, 8)]
        summary = validate(self.write_trace(rows), minimum_records=8)
        self.assertEqual(summary["first_apply_sample"], 2)
        self.assertEqual(summary["apply_count"], 6)

    def test_revoke_with_nonzero_torque_fails(self):
        rows = [
            record(0, "revoke", "capture_window", None),
            record(1, "revoke", "balancing", "denied"),
            record(2, "apply", "balancing", "closed_loop"),
        ]
        rows[1]["authorized_drive_torque_nm"] = 0.01
        with self.assertRaises(EvidenceError):
            validate(self.write_trace(rows), minimum_records=3)

    def test_truth_field_in_raw_input_fails(self):
        rows = [
            record(0, "revoke", "capture_window", None),
            record(1, "revoke", "balancing", "denied"),
            record(2, "apply", "balancing", "closed_loop"),
        ]
        rows[0]["raw_device_observation"]["body_pitch_rad"] = 0.005
        with self.assertRaises(EvidenceError):
            validate(self.write_trace(rows), minimum_records=3)

    def test_sample_replay_or_gap_fails(self):
        rows = [
            record(0, "revoke", "capture_window", None),
            record(1, "revoke", "balancing", "denied"),
            record(3, "apply", "balancing", "closed_loop"),
        ]
        with self.assertRaises(EvidenceError):
            validate(self.write_trace(rows), minimum_records=3)


if __name__ == "__main__":
    unittest.main()
