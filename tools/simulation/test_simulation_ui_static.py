import json
import re
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
UI = HERE / "ui"
APP = UI / "app.js"
INDEX = UI / "index.html"
STYLE = UI / "style.css"
COMMON_SAMPLE = UI / "sample-trace.jsonl"
CLOSED_LOOP_SAMPLE = UI / "sample-closed-loop-trace.jsonl"

EXPECTED_FIELDS = (
    "time_s",
    "forward_position_m",
    "forward_velocity_m_per_s",
    "body_pitch_rad",
    "body_pitch_rate_rad_per_s",
    "body_roll_rad",
    "body_roll_rate_rad_per_s",
    "reaction_position_rad",
    "reaction_rate_rad_per_s",
    "drive_torque_nm",
    "reaction_torque_nm",
)

REQUIRED_CONSOLE_IDS = (
    "modelViewport",
    "traceStatus",
    "traceDialect",
    "eventLog",
    "traceFile",
    "scrubber",
    "playPause",
    "provenanceBadge",
    "comparisonPanel",
    "referencePanel",
    "runtimePanel",
    "authoritySummary",
)


def load_jsonl(path: Path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


class SimulationUiStaticTests(unittest.TestCase):
    def test_viewer_assets_exist(self):
        for path in (
            INDEX,
            APP,
            STYLE,
            UI / "README.md",
            UI / "CONSOLE_LAYOUT.md",
            COMMON_SAMPLE,
            CLOSED_LOOP_SAMPLE,
        ):
            self.assertTrue(path.is_file(), path)

    def test_ui_common_trace_fields_match_contract(self):
        source = APP.read_text(encoding="utf-8")
        match = re.search(r"const COMMON_TRACE_FIELDS = \[(.*?)\];", source, re.DOTALL)
        self.assertIsNotNone(match)
        fields = tuple(re.findall(r'"([a-z0-9_]+)"', match.group(1)))
        self.assertEqual(fields, EXPECTED_FIELDS)

    def test_closed_loop_dialect_is_explicit(self):
        source = APP.read_text(encoding="utf-8")
        self.assertIn('const CLOSED_LOOP_MODE = "closed_loop_production_path";', source)
        self.assertIn('"production-semantic-v1"', source)
        self.assertIn("webots_evidence_truth", source)
        self.assertIn("production.estimate", source)
        self.assertIn("production.reference", source)
        self.assertIn("production.authority", source)
        self.assertIn("revoked authority carries nonzero applied torque", source)
        self.assertIn("applies torque without closed_loop authority", source)

    def test_closed_loop_sample_preserves_provenance_layers(self):
        records = load_jsonl(CLOSED_LOOP_SAMPLE)
        self.assertGreaterEqual(len(records), 3)
        self.assertTrue(all(record["mode"] == "closed_loop_production_path" for record in records))
        self.assertEqual(
            [record["raw_device_observation"]["sample_index"] for record in records],
            list(range(len(records))),
        )
        self.assertTrue(any(record["production"]["actuation"] == "revoke" for record in records))
        self.assertTrue(any(record["production"]["actuation"] == "apply" for record in records))
        for record in records:
            self.assertIn("webots_evidence_truth", record)
            self.assertIn("production", record)
            self.assertIn("reference", record["production"])
            if record["production"]["actuation"] == "apply":
                self.assertEqual(record["production"]["authority"], "closed_loop")
                self.assertEqual(
                    record["authorized_drive_torque_nm"],
                    record["production"]["drive_torque_nm"],
                )
                self.assertEqual(
                    record["authorized_reaction_torque_nm"],
                    record["production"]["reaction_torque_nm"],
                )

    def test_console_structure_is_present(self):
        source = INDEX.read_text(encoding="utf-8")
        for element_id in REQUIRED_CONSOLE_IDS:
            self.assertRegex(source, rf'id="{re.escape(element_id)}"')
        self.assertIn("Single Control & Evidence Console", source)
        self.assertIn("Truth vs estimate", source)
        self.assertIn("Production runtime", source)
        self.assertIn("UI-derived truth minus estimate", source)

    def test_ui_is_observer_only(self):
        source = (INDEX.read_text(encoding="utf-8") + APP.read_text(encoding="utf-8")).lower()
        self.assertIn("observer only", source)
        self.assertNotIn("fetch(", source)
        self.assertNotIn("websocket", source)
        self.assertNotIn("authorizedactuation(", source)


if __name__ == "__main__":
    unittest.main()
