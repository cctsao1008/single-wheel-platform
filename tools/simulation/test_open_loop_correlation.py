from __future__ import annotations

import json
from pathlib import Path
import unittest

from open_loop_correlation import materialize_reduced_fixture, project_model_samples
from validate_experiment import validate_experiment


HERE = Path(__file__).resolve().parent
EXPERIMENTS = (
    "synthetic-free-response.json",
    "synthetic-drive-torque-pulse.json",
    "synthetic-reaction-torque-pulse.json",
    "synthetic-small-angle.json",
    "synthetic-zero-input-equilibrium.json",
)


class OpenLoopCorrelationTests(unittest.TestCase):
    def test_required_experiment_suite_validates(self) -> None:
        for name in EXPERIMENTS:
            with self.subTest(name=name):
                document = json.loads((HERE / "experiments" / name).read_text(encoding="utf-8"))
                validate_experiment(document)

    def test_materializer_preserves_common_state_semantics(self) -> None:
        document = json.loads(
            (HERE / "experiments" / "synthetic-small-angle.json").read_text(encoding="utf-8")
        )
        fixture = materialize_reduced_fixture(document)
        self.assertEqual(
            fixture["initial_state"],
            [0.0, 0.0, 0.025, 0.0, -0.02, 0.0, 0.0, 0.0],
        )
        self.assertNotIn("drive_position_rad", document["initial_state"])
        self.assertEqual(fixture["sample_period_us"], 1000)
        self.assertEqual(fixture["duration_us"], 250000)

    def test_projection_swaps_reaction_rate_phase_into_common_names(self) -> None:
        projected = project_model_samples(
            [
                {
                    "time_us": 1000,
                    "state": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0],
                    "applied_input": [9.0, 10.0],
                }
            ]
        )[0]
        self.assertEqual(projected["reaction_rate_rad_per_s"], 7.0)
        self.assertEqual(projected["reaction_position_rad"], 8.0)
        self.assertEqual(projected["forward_position_m"], 1.0)
        self.assertEqual(projected["drive_torque_nm"], 9.0)


if __name__ == "__main__":
    unittest.main()
