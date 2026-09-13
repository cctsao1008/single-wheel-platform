from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import materialize_physical_fixture as physical


def leaf(value, evidence="measured"):
    return {
        "value": value,
        "evidence": "unknown" if value is None else evidence,
    }


def complete_registry():
    return {
        "schema": 1,
        "assembly": "reference-assembly",
        "body": {
            "mass_kg": leaf(1.0),
            "com_height_m": leaf(0.10),
            "inertia_roll_kg_m2": leaf(0.010),
            "inertia_pitch_kg_m2": leaf(0.012),
            "inertia_yaw_kg_m2": leaf(0.015),
        },
        "drive_wheel": {
            "radius_m": leaf(0.05),
            "mass_kg": leaf(0.20),
            "spin_inertia_kg_m2": leaf(0.0003),
            "encoder_counts_per_revolution": leaf(65536, "datasheet"),
            "encoder_positive_sign": leaf(1),
            "encoder_max_abs_delta_counts_per_sample": leaf(20000),
        },
        "reaction_wheel": {
            "mass_kg": leaf(0.10),
            "com_height_m": leaf(0.10),
            "spin_inertia_kg_m2": leaf(0.0010),
            "transverse_inertia_kg_m2": leaf(0.0006),
            "encoder_counts_per_revolution": leaf(65536, "datasheet"),
            "encoder_positive_sign": leaf(1),
            "encoder_max_abs_delta_counts_per_sample": leaf(20000),
        },
        "imu": {
            "forward_x_m": leaf(0.0),
            "left_y_m": leaf(0.0),
            "up_z_m": leaf(0.0),
        },
        "actuator": {
            "drive": {
                "torque_per_effective_command_nm": leaf(1.0, "identified"),
                "command_deadzone": leaf(0.0, "identified"),
                "viscous_friction_nm_per_rad_s": leaf(0.0, "identified"),
                "coulomb_friction_nm": leaf(0.0, "identified"),
                "delay_s": leaf(0.0, "identified"),
            },
            "reaction": {
                "torque_per_effective_command_nm": leaf(1.0, "identified"),
                "command_deadzone": leaf(0.0, "identified"),
                "viscous_friction_nm_per_rad_s": leaf(0.0, "identified"),
                "coulomb_friction_nm": leaf(0.0, "identified"),
                "delay_s": leaf(0.0, "identified"),
            },
        },
    }


class PhysicalFixtureMaterializationTests(unittest.TestCase):
    def test_complete_accepted_registry_materializes_without_defaults(self):
        registry = complete_registry()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference-assembly.json"
            path.write_text(json.dumps(registry, sort_keys=True), encoding="utf-8")
            fixture = physical.materialize(path, source_label="parameters/reference-assembly.json")

        self.assertEqual(fixture["provenance"], "accepted_physical")
        self.assertEqual(fixture["profile"], physical.PROFILE)
        self.assertEqual(
            set(fixture["physical_parameters"]),
            set(physical.REQUIRED_PATHS),
        )
        self.assertEqual(
            fixture["physical_parameters"]["drive_wheel.radius_m"],
            {"value": 0.05, "evidence": "measured"},
        )
        self.assertNotIn("contact_friction", fixture["physical_parameters"])
        self.assertNotIn("solver", fixture["physical_parameters"])
        self.assertEqual(len(fixture["source_sha256"]), 64)
        self.assertEqual(len(fixture["parameter_set_sha256"]), 64)

    def test_one_unknown_required_value_blocks_materialization(self):
        registry = complete_registry()
        registry["drive_wheel"]["radius_m"] = leaf(None)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference-assembly.json"
            path.write_text(json.dumps(registry), encoding="utf-8")
            with self.assertRaises(physical.PhysicalFixtureNotReady) as raised:
                physical.materialize(path)
        self.assertEqual(raised.exception.missing, ("drive_wheel.radius_m",))

    def test_readiness_reports_all_unknowns_without_filling_them(self):
        registry = complete_registry()
        registry["body"]["mass_kg"] = leaf(None)
        registry["imu"]["up_z_m"] = leaf(None)
        status = physical.readiness(registry)
        self.assertFalse(status["ready"])
        self.assertEqual(status["missing_count"], 2)
        self.assertEqual(status["accepted_count"], len(physical.REQUIRED_PATHS) - 2)
        self.assertEqual(status["missing"], ["body.mass_kg", "imu.up_z_m"])
        self.assertIsNone(registry["body"]["mass_kg"]["value"])
        self.assertIsNone(registry["imu"]["up_z_m"]["value"])

    def test_non_null_value_cannot_claim_unknown_evidence(self):
        registry = complete_registry()
        registry["body"]["mass_kg"] = {"value": 1.0, "evidence": "unknown"}
        with self.assertRaisesRegex(physical.PhysicalFixtureError, "cannot remain evidence='unknown'"):
            physical.readiness(registry)

    def test_physically_impossible_accepted_inertia_is_rejected(self):
        registry = complete_registry()
        registry["body"]["inertia_roll_kg_m2"] = leaf(0.040)
        with self.assertRaisesRegex(physical.PhysicalFixtureError, "principal-moment triangle"):
            physical.readiness(registry)

    def test_repository_registry_readiness_matches_its_null_evidence(self):
        registry = json.loads(physical.DEFAULT_REGISTRY.read_text(encoding="utf-8"))
        status = physical.readiness(registry)
        for path in status["missing"]:
            node = registry
            for part in path.split("."):
                node = node[part]
            self.assertIsNone(node["value"])
            self.assertEqual(node["evidence"], "unknown")


if __name__ == "__main__":
    unittest.main()
