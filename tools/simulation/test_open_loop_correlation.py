from __future__ import annotations

import json
import math
from pathlib import Path
import unittest

from open_loop_correlation import (
    COMMON_STATE_FIELDS,
    compare_common_traces,
    materialize_reduced_fixture,
    project_model_samples,
)
from validate_experiment import validate_experiment


HERE = Path(__file__).resolve().parent
EXPERIMENTS = (
    "synthetic-free-response.json",
    "synthetic-drive-torque-pulse.json",
    "synthetic-reaction-torque-pulse.json",
    "synthetic-small-angle.json",
    "synthetic-zero-input-equilibrium.json",
)
RIGIDBODY_SOURCE = "tools/simulation/fixtures/synthetic-rigidbody-correlation.json"


class OpenLoopCorrelationTests(unittest.TestCase):
    def test_required_experiment_suite_validates_and_uses_one_parameter_source(self) -> None:
        for name in EXPERIMENTS:
            with self.subTest(name=name):
                document = json.loads((HERE / "experiments" / name).read_text(encoding="utf-8"))
                validate_experiment(document)
                self.assertEqual(document["parameter_set"]["source"], RIGIDBODY_SOURCE)
                self.assertEqual(document["parameter_set"]["provenance"], "synthetic")

    def test_rigidbody_fixture_matches_committed_box_and_cylinder_geometry(self) -> None:
        fixture = json.loads(
            (HERE / "fixtures" / "synthetic-rigidbody-correlation.json").read_text(
                encoding="utf-8"
            )
        )
        plant = fixture["plant"]
        geometry = fixture["webots_geometry"]

        x, y, z = geometry["body_size_m"]
        body_mass = plant["body_mass_kg"]
        self.assertTrue(
            math.isclose(
                plant["body_inertia_roll_kg_m2"],
                body_mass * (y * y + z * z) / 12.0,
                rel_tol=0.0,
                abs_tol=1.0e-15,
            )
        )
        self.assertTrue(
            math.isclose(
                plant["body_inertia_pitch_kg_m2"],
                body_mass * (x * x + z * z) / 12.0,
                rel_tol=0.0,
                abs_tol=1.0e-15,
            )
        )
        self.assertTrue(
            math.isclose(
                plant["body_inertia_yaw_kg_m2"],
                body_mass * (x * x + y * y) / 12.0,
                rel_tol=0.0,
                abs_tol=1.0e-15,
            )
        )

        drive_mass = plant["drive_wheel_mass_kg"]
        drive_radius = plant["drive_wheel_radius_m"]
        self.assertTrue(
            math.isclose(
                plant["drive_wheel_spin_inertia_kg_m2"],
                0.5 * drive_mass * drive_radius * drive_radius,
                rel_tol=0.0,
                abs_tol=1.0e-15,
            )
        )

        reaction_mass = plant["reaction_wheel_mass_kg"]
        reaction_radius = geometry["reaction_wheel_radius_m"]
        reaction_width = geometry["reaction_wheel_width_m"]
        self.assertTrue(
            math.isclose(
                plant["reaction_wheel_spin_inertia_kg_m2"],
                0.5 * reaction_mass * reaction_radius * reaction_radius,
                rel_tol=0.0,
                abs_tol=1.0e-15,
            )
        )
        self.assertTrue(
            math.isclose(
                plant["reaction_wheel_transverse_inertia_kg_m2"],
                reaction_mass
                * (3.0 * reaction_radius * reaction_radius + reaction_width * reaction_width)
                / 12.0,
                rel_tol=0.0,
                abs_tol=1.0e-15,
            )
        )
        self.assertTrue(
            math.isclose(
                plant["reaction_wheel_com_height_m"],
                geometry["reaction_axle_body_z_m"] - geometry["drive_axle_body_z_m"],
                rel_tol=0.0,
                abs_tol=1.0e-15,
            )
        )

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

    def test_finite_width_free_roll_mismatch_is_preserved_as_explainable(self) -> None:
        def sample(time_s: float, pitch: float, roll: float) -> dict[str, float]:
            record = {field: 0.0 for field in COMMON_STATE_FIELDS}
            record.update(
                {
                    "time_s": time_s,
                    "body_pitch_rad": pitch,
                    "body_roll_rad": roll,
                    "drive_torque_nm": 0.0,
                    "reaction_torque_nm": 0.0,
                }
            )
            return record

        reference = [
            sample(0.001, 0.0250, -0.0200),
            sample(0.050, 0.0270, -0.0210),
        ]
        webots_like = [
            sample(0.001, 0.0250, -0.0200),
            sample(0.050, 0.0300, -0.0170),
        ]
        result = compare_common_traces(
            "synthetic-free-response", reference, webots_like
        )
        self.assertEqual(result["status"], "explainable_difference")
        self.assertFalse(result["causal_match"])
        self.assertTrue(result["causal_acceptable"])
        self.assertEqual(result["explained_differences"][0]["field"], "body_roll_rad")


if __name__ == "__main__":
    unittest.main()
