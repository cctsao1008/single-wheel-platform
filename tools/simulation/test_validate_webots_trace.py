from __future__ import annotations

import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from validate_webots_trace import FIELDS, validate_trace


def record(time_s: float) -> dict[str, float]:
    return {field: (time_s if field == "time_s" else 0.0) for field in FIELDS}


class WebotsTraceTests(unittest.TestCase):
    def write_trace(self, records: list[dict[str, float]]) -> Path:
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "trace.jsonl"
        path.write_text(
            "".join(json.dumps(item) + "\n" for item in records),
            encoding="utf-8",
        )
        return path

    def test_accepts_minimal_well_formed_trace(self) -> None:
        summary = validate_trace(self.write_trace([record(0.001), record(0.002)]))
        self.assertEqual(summary["records"], 2)
        self.assertEqual(summary["end_time_s"], 0.002)

    def test_rejects_non_monotonic_time(self) -> None:
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            validate_trace(self.write_trace([record(0.001), record(0.001)]))

    def test_rejects_missing_field(self) -> None:
        broken = record(0.001)
        del broken["body_pitch_rad"]
        with self.assertRaisesRegex(ValueError, "field mismatch"):
            validate_trace(self.write_trace([broken, record(0.002)]))


if __name__ == "__main__":
    unittest.main()
