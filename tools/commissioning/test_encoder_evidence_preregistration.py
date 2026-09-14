import importlib.util
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).with_name("encoder_evidence.py")
SPEC = importlib.util.spec_from_file_location("encoder_evidence", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def row(sequence, encoder_1, encoder_2, timestamp_us, dropped=0, quality=0x000B):
    return {
        "sequence": sequence,
        "encoder_1_count": encoder_1,
        "encoder_1_quality": quality,
        "encoder_1_captured_at_us": timestamp_us,
        "encoder_2_count": encoder_2,
        "encoder_2_quality": quality,
        "encoder_2_captured_at_us": timestamp_us,
        "dropped_records": dropped,
    }


class EncoderEvidencePreregistrationTests(unittest.TestCase):
    def test_unknown_sign_preregistration_is_preserved_without_biasing_candidate(self):
        rows = [
            row(10, 0, 65530, 0),
            row(11, 0, 2, 10_000),
            row(12, 0, 10, 20_000),
            row(13, 0, 2, 30_000),
            row(14, 0, 65530, 40_000),
        ]
        plan = {
            "schema": 1,
            "assembly": "reference-assembly",
            "encoder_channel": "encoder_2",
            "mechanical_coordinate": "drive_wheel_relative_angle",
            "positive_direction_definition": "relative wheel rotation that produces +X forward rolling when body pitch is held fixed",
            "trials": [
                {"name": "positive-1", "direction": "positive", "start_sequence": 10, "end_sequence": 12},
                {"name": "negative-1", "direction": "negative", "start_sequence": 12, "end_sequence": 14},
            ],
        }
        preregistration = {
            "schema": 1,
            "status": "valid_preregistration",
            "assembly": "reference-assembly",
            "encoder_channel": "encoder_2",
            "installed_role": "DriveWheel",
            "runtime_counter": "TIM4",
            "mechanical_coordinate": "drive_wheel_relative_angle",
            "counter_sign_hypothesis": "unknown",
            "preregistration_sha256": "a" * 64,
        }

        result = MODULE.analyze_rows(rows, plan, "capture", preregistration)
        self.assertEqual(result["analysis"]["counter_sign_for_mechanical_positive"], 1)
        self.assertEqual(result["analysis"]["counts_per_mechanical_revolution_candidate"], 16)
        self.assertEqual(result["prediction_comparison"]["status"], "not_prejudged")
        self.assertEqual(result["preregistration"]["preregistration_sha256"], "a" * 64)

    def test_explicit_wrong_sign_hypothesis_is_reported_as_counterexample(self):
        rows = [
            row(1, 0, 100, 0),
            row(2, 0, 110, 10_000),
            row(3, 0, 120, 20_000),
            row(4, 0, 110, 30_000),
            row(5, 0, 100, 40_000),
        ]
        plan = {
            "schema": 1,
            "encoder_channel": "encoder_2",
            "mechanical_coordinate": "drive_wheel_relative_angle",
            "trials": [
                {"direction": "positive", "start_sequence": 1, "end_sequence": 3},
                {"direction": "negative", "start_sequence": 3, "end_sequence": 5},
            ],
        }
        preregistration = {
            "encoder_channel": "encoder_2",
            "mechanical_coordinate": "drive_wheel_relative_angle",
            "counter_sign_hypothesis": "decrease",
        }

        result = MODULE.analyze_rows(rows, plan, "capture", preregistration)
        self.assertEqual(result["analysis"]["status"], "consistent_bidirectional_one_revolution_evidence")
        self.assertEqual(result["prediction_comparison"]["status"], "counterexample")

    def test_rejects_preregistration_for_wrong_encoder(self):
        rows = [row(1, 0, 0, 0), row(2, 0, 5, 10_000)]
        plan = {
            "schema": 1,
            "encoder_channel": "encoder_2",
            "mechanical_coordinate": "drive_wheel_relative_angle",
            "trials": [{"direction": "positive", "start_sequence": 1, "end_sequence": 2}],
        }
        preregistration = {
            "encoder_channel": "encoder_1",
            "mechanical_coordinate": "reaction_wheel_relative_angle",
            "counter_sign_hypothesis": "unknown",
        }
        with self.assertRaisesRegex(MODULE.EvidenceError, "encoder_channel"):
            MODULE.analyze_rows(rows, plan, "capture", preregistration)


if __name__ == "__main__":
    unittest.main()
