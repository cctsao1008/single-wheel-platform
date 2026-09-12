from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from finalize_correlation_summary import (
    EQUILIBRIUM_LIMITS,
    equilibrium_checks,
    finalize_summary,
)
from summarize_correlation_suite import REQUIRED, build_suite_summary


STATE_FIELDS = (
    "forward_position_m",
    "forward_velocity_m_per_s",
    "body_pitch_rad",
    "body_pitch_rate_rad_per_s",
    "body_roll_rad",
    "body_roll_rate_rad_per_s",
    "reaction_position_rad",
    "reaction_rate_rad_per_s",
)


def sample(time_s: float, **values: float) -> dict[str, float]:
    record = {field: 0.0 for field in STATE_FIELDS}
    record.update(
        {
            "time_s": time_s,
            "drive_torque_nm": 0.0,
            "reaction_torque_nm": 0.0,
        }
    )
    record.update(values)
    return record


def base_summary(experiment: str, webots_status: str = "fail") -> dict:
    return {
        "schema": 1,
        "experiment": experiment,
        "experiment_sha256": "experiment-sha",
        "parameter_source": "tools/simulation/fixtures/synthetic-rigidbody-correlation.json",
        "parameter_source_sha256": "parameter-sha",
        "parameter_provenance": "synthetic",
        "comparisons": {
            "rust-simulation-world": {
                "status": "pass",
                "input_match": True,
                "causal_match": True,
                "causal_acceptable": True,
                "causal_checks": [],
                "explained_differences": [],
            },
            "webots": {
                "status": webots_status,
                "input_match": True,
                "causal_match": False,
                "causal_acceptable": False,
                "causal_checks": [{"field": "all_common_states", "pass": False}],
                "explained_differences": [],
            },
        },
    }


class CorrelationEvidencePolicyTests(unittest.TestCase):
    def test_equilibrium_checks_use_dimensioned_per_field_limits(self) -> None:
        bounded = [
            sample(0.001),
            sample(
                0.100,
                body_roll_rad=0.5 * EQUILIBRIUM_LIMITS["body_roll_rad"],
                body_roll_rate_rad_per_s=0.75
                * EQUILIBRIUM_LIMITS["body_roll_rate_rad_per_s"],
            ),
        ]
        checks = equilibrium_checks(bounded)
        self.assertTrue(all(check["pass"] for check in checks))

        bounded[-1]["body_roll_rad"] = 1.01 * EQUILIBRIUM_LIMITS["body_roll_rad"]
        checks = equilibrium_checks(bounded)
        roll = next(check for check in checks if check["field"] == "body_roll_rad")
        self.assertFalse(roll["pass"])
        self.assertEqual(roll["unit"], "rad")

    def test_zero_input_finalizer_supersedes_legacy_scalar_check_explicitly(self) -> None:
        trace = [
            sample(0.001),
            sample(
                0.100,
                body_roll_rad=3.0e-5,
                body_roll_rate_rad_per_s=1.2e-3,
                reaction_position_rad=-3.0e-5,
                reaction_rate_rad_per_s=-1.2e-3,
            ),
        ]
        summary = finalize_summary(
            base_summary("synthetic-zero-input-equilibrium"),
            analytical=[sample(0.001), sample(0.100)],
            rust=[sample(0.001), sample(0.100)],
            webots=trace,
            repository_sha="deadbeef",
            webots_image="pinned-image",
        )
        comparison = summary["comparisons"]["webots"]
        self.assertEqual(comparison["status"], "pass")
        self.assertEqual(
            comparison["acceptance_basis"],
            "dimensioned_zero_input_equilibrium_envelopes",
        )
        self.assertIn("legacy_scalar_equilibrium_check", comparison)
        self.assertEqual(summary["schema"], 2)
        self.assertEqual(summary["repository_commit"], "deadbeef")

    def test_suite_summary_preserves_explainable_difference_as_non_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for experiment, file_name in REQUIRED:
                webots_status = (
                    "explainable_difference"
                    if experiment == "synthetic-free-response"
                    else "pass"
                )
                document = base_summary(experiment, webots_status=webots_status)
                document["comparisons"]["webots"]["causal_acceptable"] = True
                (root / file_name).write_text(json.dumps(document), encoding="utf-8")

            suite = build_suite_summary(root)
            self.assertEqual(suite["required_experiment_count"], 5)
            self.assertEqual(suite["overall_status"], "explainable_difference")


if __name__ == "__main__":
    unittest.main()
