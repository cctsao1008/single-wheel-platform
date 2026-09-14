import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
UI = HERE / "ui"
MODEL = UI / "model3d.js"
APP = UI / "app.js"
LIVE = UI / "live.js"


class SimulationUiSpatialGhostTests(unittest.TestCase):
    def setUp(self):
        self.source = MODEL.read_text(encoding="utf-8")

    def test_ghost_comes_only_from_explicit_production_estimate(self):
        self.assertIn('record?.dialect !== "production-semantic-v1"', self.source)
        self.assertIn("const estimate = record.production?.estimate;", self.source)
        self.assertIn("if (!estimate) return null;", self.source)
        self.assertIn("estimate.body_pitch_rad", self.source)
        self.assertIn("estimate.body_roll_rad", self.source)
        self.assertNotIn("estimate.reaction_position_rad", self.source)

    def test_ghost_uses_same_physical_sign_transform_as_truth(self):
        self.assertIn("return rotateY(rotateX(v, roll), pitch);", self.source)
        self.assertIn("const vertices = bodyVertices(pitch, roll);", self.source)
        self.assertIn("The estimate ghost uses this same transform", self.source)
        self.assertIn("never receives a visual-only sign fix", self.source)

    def test_ghost_is_attitude_only_at_same_schematic_origin(self):
        self.assertIn("Estimate ghost is attitude-only", self.source)
        self.assertIn("no estimated translation is synthesized", self.source)
        self.assertIn('origin: "same schematic axle/origin as truth; no estimated translation synthesized"', self.source)
        self.assertNotIn("estimate.forward_position_m", self.source)
        self.assertNotIn("estimate.forward_velocity_m_per_s", self.source)

    def test_reaction_phase_remains_truth_common_only(self):
        self.assertIn("Reaction phase is truth/common evidence only", self.source)
        self.assertIn("production estimate does not carry this cyclic coordinate", self.source)
        self.assertIn("truth/common only", self.source)
        self.assertIn('reactionPhase: "not estimated; truth/common only"', self.source)

    def test_absent_estimate_means_absent_ghost(self):
        self.assertIn("const estimate = estimateAttitude(record);", self.source)
        self.assertIn("if (estimateGhostEnabled && estimate)", self.source)
        self.assertIn("estimate ghost: unavailable", self.source)
        self.assertNotIn("estimate ??", self.source)

    def test_toggle_is_presentation_only(self):
        self.assertIn('id="spatialEstimateGhost"', self.source)
        self.assertIn("estimateGhostEnabled = event.target.checked;", self.source)
        self.assertIn("draw(latestRecord);", self.source)
        lowered = self.source.lower()
        self.assertNotIn("fetch(", lowered)
        self.assertNotIn("eventsource", lowered)
        self.assertNotIn("websocket", lowered)
        self.assertNotIn("authorizedactuation(", lowered)
        self.assertNotIn("closedloopsimulation", lowered)

    def test_replay_and_live_still_share_render_record(self):
        app = APP.read_text(encoding="utf-8")
        live = LIVE.read_text(encoding="utf-8")
        self.assertIn("const baseRenderRecord = renderRecord;", self.source)
        self.assertIn("renderRecord = function spatialAwareRenderRecord(record)", self.source)
        self.assertIn("renderRecord(state.trace[state.index]);", app)
        self.assertIn("renderRecord(normalized);", live)


if __name__ == "__main__":
    unittest.main()
