from __future__ import annotations

import copy
import json
from pathlib import Path
import unittest

from validate_experiment import validate_experiment


HERE = Path(__file__).resolve().parent
FIXTURE = HERE / "experiments" / "synthetic-small-angle.json"


class ExperimentContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.document = json.loads(FIXTURE.read_text(encoding="utf-8"))

    def test_committed_synthetic_fixture_is_valid(self) -> None:
        validate_experiment(self.document)

    def test_committed_fixture_matches_canonical_synthetic_perturbation(self) -> None:
        state = self.document["initial_state"]
        self.assertEqual(state["forward_position_m"]["value"], 0.0)
        self.assertEqual(state["forward_velocity_m_per_s"]["value"], 0.0)
        self.assertEqual(state["body_pitch_rad"]["value"], 0.025)
        self.assertEqual(state["body_roll_rad"]["value"], -0.02)
        self.assertNotIn("drive_position_rad", state)
        self.assertNotIn("drive_rate_rad_s", state)

    def test_rejects_obsolete_drive_joint_as_reduced_state(self) -> None:
        broken = copy.deepcopy(self.document)
        del broken["initial_state"]["forward_position_m"]
        del broken["initial_state"]["forward_velocity_m_per_s"]
        broken["initial_state"]["drive_position_rad"] = {"value": 0.0, "unit": "rad"}
        broken["initial_state"]["drive_rate_rad_s"] = {"value": 0.0, "unit": "rad/s"}
        with self.assertRaisesRegex(ValueError, "canonical eight physical state"):
            validate_experiment(broken)

    def test_rejects_old_schema(self) -> None:
        broken = copy.deepcopy(self.document)
        broken["schema"] = 1
        broken["backend"]["contract"] = "simulator-neutral-v1"
        with self.assertRaisesRegex(ValueError, "schema must be 2"):
            validate_experiment(broken)

    def test_rejects_unclassified_parameter_provenance(self) -> None:
        broken = copy.deepcopy(self.document)
        broken["parameter_set"]["provenance"] = "probably-real"
        with self.assertRaisesRegex(ValueError, "provenance"):
            validate_experiment(broken)

    def test_accepted_physical_must_use_canonical_registry(self) -> None:
        broken = copy.deepcopy(self.document)
        broken["parameter_set"]["provenance"] = "accepted_physical"
        broken["parameter_set"]["source"] = "some-convenient-values.json"
        with self.assertRaisesRegex(ValueError, "reference-assembly"):
            validate_experiment(broken)

    def test_rejects_wrong_physical_sign_contract(self) -> None:
        broken = copy.deepcopy(self.document)
        broken["coordinates"]["drive_wheel_positive"] = "whatever makes the controller stable"
        with self.assertRaisesRegex(ValueError, "coordinate"):
            validate_experiment(broken)

    def test_rejects_missing_units(self) -> None:
        broken = copy.deepcopy(self.document)
        del broken["initial_state"]["body_pitch_rad"]["unit"]
        with self.assertRaisesRegex(ValueError, "unit"):
            validate_experiment(broken)

    def test_rejects_non_monotonic_input_time(self) -> None:
        broken = copy.deepcopy(self.document)
        broken["input_profile"][2]["time_s"]["value"] = 0.01
        with self.assertRaisesRegex(ValueError, "nondecreasing"):
            validate_experiment(broken)


if __name__ == "__main__":
    unittest.main()
