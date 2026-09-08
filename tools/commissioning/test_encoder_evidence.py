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


class EncoderEvidenceTests(unittest.TestCase):
    def test_signed_delta_handles_u16_wrap(self):
        self.assertEqual(MODULE.signed_u16_delta(65534, 1), 3)
        self.assertEqual(MODULE.signed_u16_delta(1, 65534), -3)

    def test_bidirectional_equal_revolutions_produce_candidate(self):
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
            "positive_direction_definition": "forward rolling direction",
            "trials": [
                {"name": "positive-1", "direction": "positive", "start_sequence": 10, "end_sequence": 12},
                {"name": "negative-1", "direction": "negative", "start_sequence": 12, "end_sequence": 14},
            ],
        }
        result = MODULE.analyze_rows(rows, plan, "abc")
        self.assertEqual(result["analysis"]["counter_sign_for_mechanical_positive"], 1)
        self.assertEqual(result["analysis"]["counts_per_mechanical_revolution_candidate"], 16)
        self.assertEqual(result["analysis"]["status"], "consistent_bidirectional_one_revolution_evidence")

    def test_mismatched_revolutions_do_not_promote_candidate(self):
        rows = [
            row(1, 0, 100, 0),
            row(2, 0, 110, 10_000),
            row(3, 0, 120, 20_000),
            row(4, 0, 111, 30_000),
            row(5, 0, 101, 40_000),
        ]
        plan = {
            "schema": 1,
            "encoder_channel": "encoder_2",
            "trials": [
                {"direction": "positive", "start_sequence": 1, "end_sequence": 3},
                {"direction": "negative", "start_sequence": 3, "end_sequence": 5},
            ],
        }
        result = MODULE.analyze_rows(rows, plan, "abc")
        self.assertIsNone(result["analysis"]["counts_per_mechanical_revolution_candidate"])
        self.assertEqual(result["analysis"]["status"], "inconsistent_or_incomplete_evidence")

    def test_sequence_gap_is_rejected(self):
        rows = [row(1, 0, 0, 0), row(3, 0, 5, 10_000)]
        plan = {
            "schema": 1,
            "encoder_channel": "encoder_2",
            "trials": [{"direction": "positive", "start_sequence": 1, "end_sequence": 3}],
        }
        with self.assertRaises(MODULE.EvidenceError):
            MODULE.analyze_rows(rows, plan, "abc")

    def test_dropped_record_event_is_rejected(self):
        rows = [row(1, 0, 0, 0, dropped=0), row(2, 0, 5, 10_000, dropped=1)]
        plan = {
            "schema": 1,
            "encoder_channel": "encoder_2",
            "trials": [{"direction": "positive", "start_sequence": 1, "end_sequence": 2}],
        }
        with self.assertRaises(MODULE.EvidenceError):
            MODULE.analyze_rows(rows, plan, "abc")


if __name__ == "__main__":
    unittest.main()