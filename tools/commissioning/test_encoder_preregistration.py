import copy
import importlib.util
import json
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).with_name("encoder_preregistration.py")
SPEC = importlib.util.spec_from_file_location("encoder_preregistration", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

PLANS = Path(__file__).with_name("plans")


class EncoderPreregistrationTests(unittest.TestCase):
    def load(self, name):
        return json.loads((PLANS / name).read_text(encoding="utf-8"))

    def test_committed_drive_preregistration_is_valid(self):
        result = MODULE.validate(self.load("drive-encoder-one-rev.preregistration.json"))
        self.assertEqual(result["encoder_channel"], "encoder_2")
        self.assertEqual(result["runtime_counter"], "TIM4")
        self.assertEqual(result["counter_sign_hypothesis"], "unknown")
        self.assertEqual(len(result["preregistration_sha256"]), 64)

    def test_committed_reaction_preregistration_is_valid(self):
        result = MODULE.validate(self.load("reaction-encoder-one-rev.preregistration.json"))
        self.assertEqual(result["encoder_channel"], "encoder_1")
        self.assertEqual(result["runtime_counter"], "TIM2")
        self.assertEqual(result["counter_sign_hypothesis"], "unknown")

    def test_rejects_swapped_mechanical_coordinate(self):
        document = self.load("drive-encoder-one-rev.preregistration.json")
        broken = copy.deepcopy(document)
        broken["mechanical_coordinate"] = "reaction_wheel_relative_angle"
        with self.assertRaisesRegex(MODULE.PreregistrationError, "mechanical_coordinate"):
            MODULE.validate(broken)

    def test_rejects_actuating_plan(self):
        document = self.load("reaction-encoder-one-rev.preregistration.json")
        broken = copy.deepcopy(document)
        broken["safety"]["motor_actuation_allowed"] = True
        with self.assertRaisesRegex(MODULE.PreregistrationError, "motor_actuation_allowed"):
            MODULE.validate(broken)

    def test_unknown_sign_requires_evidence_gap_rationale(self):
        document = self.load("drive-encoder-one-rev.preregistration.json")
        broken = copy.deepcopy(document)
        broken["prediction"]["sign_prediction_basis"] = "because"
        with self.assertRaisesRegex(MODULE.PreregistrationError, "evidence-gap"):
            MODULE.validate(broken)

    def test_hash_is_stable_for_semantically_identical_json(self):
        document = self.load("drive-encoder-one-rev.preregistration.json")
        reordered = dict(reversed(list(document.items())))
        self.assertEqual(MODULE.sha256_document(document), MODULE.sha256_document(reordered))


if __name__ == "__main__":
    unittest.main()
