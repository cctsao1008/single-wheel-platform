from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import validate_closed_loop_realization as realization


HERE = Path(__file__).resolve().parent
FIXTURE = HERE / "fixtures" / "closed-loop-aggregate-equivalent.json"


class ClosedLoopRealizationTests(unittest.TestCase):
    def test_checked_in_realization_is_physical_and_aggregate_equivalent(self) -> None:
        summary = realization.validate(FIXTURE)
        self.assertEqual(summary["status"], "pass")
        self.assertEqual(summary["accelerometer_height_above_drive_axle_m"], 0.0)
        self.assertLess(
            max(summary["aggregate_errors"].values(), default=0.0),
            1e-9,
        )

    def test_hidden_accelerometer_offset_is_rejected(self) -> None:
        document = json.loads(FIXTURE.read_text(encoding="utf-8"))
        document["webots_realization"]["accelerometer_height_above_drive_axle_m"] = 0.03
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaises(realization.RealizationError):
                realization.validate(path)

    def test_non_equivalent_drive_mass_is_rejected(self) -> None:
        document = json.loads(FIXTURE.read_text(encoding="utf-8"))
        document["webots_realization"]["drive_wheel"]["mass_kg"] = 0.1
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaises(realization.RealizationError):
                realization.validate(path)


if __name__ == "__main__":
    unittest.main()
