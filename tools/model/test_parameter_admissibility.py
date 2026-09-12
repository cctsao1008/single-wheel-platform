#!/usr/bin/env python3

import unittest

from check_parameter_admissibility import AdmissibilityError, validate_inertia_admissibility


def leaf(value):
    return {
        "value": value,
        "evidence": "unknown" if value is None else "measured",
    }


def registry(
    *,
    body=(None, None, None),
    drive_spin=None,
    reaction=(None, None),
):
    return {
        "body": {
            "inertia_roll_kg_m2": leaf(body[0]),
            "inertia_pitch_kg_m2": leaf(body[1]),
            "inertia_yaw_kg_m2": leaf(body[2]),
        },
        "drive_wheel": {
            "spin_inertia_kg_m2": leaf(drive_spin),
        },
        "reaction_wheel": {
            "spin_inertia_kg_m2": leaf(reaction[0]),
            "transverse_inertia_kg_m2": leaf(reaction[1]),
        },
    }


class ParameterAdmissibilityTests(unittest.TestCase):
    def test_unknown_inertias_remain_allowed(self):
        checks = validate_inertia_admissibility(registry())
        self.assertEqual(checks, ["0 known inertia values"])

    def test_partial_evidence_is_checked_but_not_completed_by_guessing(self):
        checks = validate_inertia_admissibility(
            registry(body=(0.010, 0.012, None), drive_spin=0.001)
        )
        self.assertEqual(checks, ["3 known inertia values"])

    def test_physically_admissible_complete_set_passes(self):
        checks = validate_inertia_admissibility(
            registry(
                body=(0.010, 0.012, 0.015),
                drive_spin=0.001,
                reaction=(0.002, 0.0015),
            )
        )
        self.assertEqual(
            checks,
            [
                "6 known inertia values",
                "body principal-moment triangle inequalities",
                "reaction-wheel axisymmetric inertia bound",
            ],
        )

    def test_impossible_body_principal_moments_are_rejected(self):
        with self.assertRaisesRegex(AdmissibilityError, "principal-moment triangle"):
            validate_inertia_admissibility(registry(body=(0.030, 0.010, 0.010)))

    def test_impossible_axisymmetric_reaction_wheel_is_rejected(self):
        with self.assertRaisesRegex(AdmissibilityError, "axisymmetric rigid-body bound"):
            validate_inertia_admissibility(registry(reaction=(0.0031, 0.0015)))

    def test_nonpositive_known_inertia_is_rejected(self):
        with self.assertRaisesRegex(AdmissibilityError, "strictly positive"):
            validate_inertia_admissibility(registry(drive_spin=0.0))

    def test_nonfinite_known_inertia_is_rejected(self):
        with self.assertRaisesRegex(AdmissibilityError, "must be finite"):
            validate_inertia_admissibility(registry(drive_spin=float("nan")))


if __name__ == "__main__":
    unittest.main()
