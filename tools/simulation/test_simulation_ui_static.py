import re
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
UI = HERE / "ui"
APP = UI / "app.js"
INDEX = UI / "index.html"
STYLE = UI / "style.css"

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


class SimulationUiStaticTests(unittest.TestCase):
    def test_viewer_assets_exist(self):
        for path in (INDEX, APP, STYLE, UI / "README.md"):
            self.assertTrue(path.is_file(), path)

    def test_ui_common_trace_fields_match_contract(self):
        source = APP.read_text(encoding="utf-8")
        match = re.search(r"const COMMON_TRACE_FIELDS = \[(.*?)\];", source, re.DOTALL)
        self.assertIsNotNone(match)
        fields = tuple(re.findall(r'"([a-z0-9_]+)"', match.group(1)))
        self.assertEqual(fields, EXPECTED_FIELDS)

    def test_ui_is_observer_only(self):
        source = (INDEX.read_text(encoding="utf-8") + APP.read_text(encoding="utf-8")).lower()
        self.assertIn("observer only", source)
        self.assertNotIn("fetch(", source)
        self.assertNotIn("websocket", source)
        self.assertNotIn("authorizedactuation(", source)


if __name__ == "__main__":
    unittest.main()
