import re
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
UI = HERE / "ui"
INDEX = UI / "index.html"
APP = UI / "app.js"
LIVE = UI / "live.js"
MODEL = UI / "model3d.js"
STYLE = UI / "model3d.css"


class SimulationUiSpatialTests(unittest.TestCase):
    def test_spatial_assets_are_local_and_wired(self):
        self.assertTrue(MODEL.is_file())
        self.assertTrue(STYLE.is_file())
        index = INDEX.read_text(encoding="utf-8")
        self.assertIn('<link rel="stylesheet" href="model3d.css">', index)
        self.assertIn('<script src="model3d.js"></script>', index)
        self.assertNotIn("three.js", index.lower())
        self.assertNotIn("cdn", index.lower())

    def test_spatial_panel_declares_schematic_not_physical_geometry(self):
        index = INDEX.read_text(encoding="utf-8")
        self.assertIn('id="spatialPanel"', index)
        self.assertIn('id="spatialCanvas"', index)
        self.assertIn("SCHEMATIC GEOMETRY", index)
        self.assertIn("no ONE V2 physical dimensions are claimed here", index)
        self.assertIn("Drive-wheel phase is intentionally not inferred", index)
        self.assertIn("Camera presets change presentation only", index)

    def test_coordinate_contract_is_explicit_and_right_handed(self):
        source = MODEL.read_text(encoding="utf-8")
        self.assertIn("+X forward, +Y left, +Z up", source)
        self.assertIn("right-hand rotation about +Y", source)
        self.assertIn("right-hand rotation about +X", source)
        self.assertIn("right-hand rotation about body +X", source)
        self.assertIn("return rotateY(rotateX(v, roll), pitch);", source)

        # +roll about +X: +Y moves toward +Z for a small positive angle.
        self.assertRegex(
            source,
            r"return \[v\[0\], v\[1\] \* c - v\[2\] \* s, v\[1\] \* s \+ v\[2\] \* c\];",
        )
        # +pitch about +Y: +Z moves toward +X for a small positive angle.
        self.assertRegex(
            source,
            r"return \[v\[0\] \* c \+ v\[2\] \* s, v\[1\], -v\[0\] \* s \+ v\[2\] \* c\];",
        )

    def test_reaction_phase_is_carried_or_unavailable_not_fabricated(self):
        source = MODEL.read_text(encoding="utf-8")
        index = INDEX.read_text(encoding="utf-8")
        self.assertIn(
            "const reactionPhase = Number.isFinite(visual.reaction_position_rad) ? visual.reaction_position_rad : null;",
            source,
        )
        self.assertIn('reactionPhase === null', source)
        self.assertIn('"unavailable"', source)
        self.assertIn('id="spatialPhase">unavailable</strong>', index)

    def test_forward_position_moves_reference_grid_not_drive_phase(self):
        source = MODEL.read_text(encoding="utf-8")
        self.assertIn("drawGround(basis, width, height, forward);", source)
        self.assertIn("const offset = ((forwardPosition % spacing) + spacing) % spacing;", source)
        self.assertNotRegex(source, r"forward(?:Position)?\s*/\s*(?:drive|wheel|radius)")
        self.assertNotIn("drivePhase", source)
        self.assertNotIn("drive_phase", source)

    def test_spatial_renderer_is_observer_only_presentation(self):
        source = MODEL.read_text(encoding="utf-8")
        lowered = source.lower()
        self.assertNotIn("fetch(", lowered)
        self.assertNotIn("eventsource", lowered)
        self.assertNotIn("websocket", lowered)
        self.assertNotIn("authorizedactuation(", lowered)
        self.assertNotIn("closedloopsimulation", lowered)
        self.assertNotIn("integration step", lowered)
        self.assertNotIn("requestanimationframe", lowered)

    def test_replay_and_live_share_the_same_render_record_boundary(self):
        model = MODEL.read_text(encoding="utf-8")
        app = APP.read_text(encoding="utf-8")
        live = LIVE.read_text(encoding="utf-8")
        self.assertIn("const baseRenderRecord = renderRecord;", model)
        self.assertIn("renderRecord = function spatialAwareRenderRecord(record)", model)
        self.assertIn("renderRecord(state.trace[state.index]);", app)
        self.assertIn("renderRecord(normalized);", live)

    def test_camera_presets_are_presentation_only(self):
        source = MODEL.read_text(encoding="utf-8")
        index = INDEX.read_text(encoding="utf-8")
        for preset in ("iso", "side", "front"):
            self.assertIn(f'{preset}: {{ eye:', source)
            self.assertIn(f'data-camera-preset="{preset}"', index)
        self.assertIn("cameraPreset = preset;", source)
        self.assertIn("draw(latestRecord);", source)


if __name__ == "__main__":
    unittest.main()
