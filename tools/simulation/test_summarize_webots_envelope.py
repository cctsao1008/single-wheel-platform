from __future__ import annotations

import unittest

import summarize_webots_envelope as summary


class EnvelopeSummaryTests(unittest.TestCase):
    def manifest(self) -> dict:
        return {
            "name": "test-envelope",
            "provenance": "synthetic test; not ONE V2",
            "backend": {"name": "Webots", "version": "test", "image": "pinned"},
            "world": "world.wbt",
            "realization_fixture": "fixture.json",
            "cases": [
                {"id": "baseline", "axis": "none", "magnitude_rad": 0.0, "initial_pitch_rad": 0.0, "initial_roll_rad": 0.0},
                {"id": "pitch_p1", "axis": "pitch", "magnitude_rad": 0.01, "initial_pitch_rad": 0.01, "initial_roll_rad": 0.0},
                {"id": "pitch_n1", "axis": "pitch", "magnitude_rad": 0.01, "initial_pitch_rad": -0.01, "initial_roll_rad": 0.0},
                {"id": "pitch_p2", "axis": "pitch", "magnitude_rad": 0.02, "initial_pitch_rad": 0.02, "initial_roll_rad": 0.0},
                {"id": "pitch_n2", "axis": "pitch", "magnitude_rad": 0.02, "initial_pitch_rad": -0.02, "initial_roll_rad": 0.0},
            ],
        }

    def item(self, case_id: str, classification: str, magnitude: float) -> dict:
        return {
            "case_id": case_id,
            "classification": classification,
            "magnitude_rad": magnitude,
            "max_abs_pitch_rad": magnitude,
            "max_abs_roll_rad": 0.0,
            "max_abs_authorized_torque_nm": magnitude,
        }

    def test_summary_preserves_counterexample_boundary(self) -> None:
        summaries = [
            self.item("baseline", "recovered", 0.0),
            self.item("pitch_p1", "recovered", 0.01),
            self.item("pitch_n1", "recovered", 0.01),
            self.item("pitch_p2", "loss_of_balance", 0.02),
            self.item("pitch_n2", "not_recovered_by_horizon", 0.02),
        ]
        result = summary.summarize(self.manifest(), summaries, "abc123")
        positive = result["boundaries"]["pitch"]["positive"]
        negative = result["boundaries"]["pitch"]["negative"]
        self.assertEqual(positive["largest_recovered_rad"], 0.01)
        self.assertEqual(positive["smallest_loss_rad"], 0.02)
        self.assertEqual(negative["smallest_nonrecovered_rad"], 0.02)
        self.assertEqual(result["classification_counts"]["loss_of_balance"], 1)

    def test_recovery_after_a_failed_smaller_case_is_flagged_nonmonotonic(self) -> None:
        manifest = self.manifest()
        summaries = [
            self.item("baseline", "recovered", 0.0),
            self.item("pitch_p1", "loss_of_balance", 0.01),
            self.item("pitch_n1", "recovered", 0.01),
            self.item("pitch_p2", "recovered", 0.02),
            self.item("pitch_n2", "recovered", 0.02),
        ]
        result = summary.summarize(manifest, summaries, "abc123")
        self.assertFalse(
            result["boundaries"]["pitch"]["positive"]
            ["recovery_order_monotonic_over_tested_grid"]
        )


if __name__ == "__main__":
    unittest.main()
