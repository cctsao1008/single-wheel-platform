import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
UI = HERE / "ui"
MODEL = UI / "model3d.js"
STYLE = UI / "model3d.css"
LAYOUT = UI / "CONSOLE_LAYOUT.md"


class SimulationUiSpatialCameraTests(unittest.TestCase):
    def setUp(self):
        self.source = MODEL.read_text(encoding="utf-8")
        self.style = STYLE.read_text(encoding="utf-8")
        self.layout = LAYOUT.read_text(encoding="utf-8")

    def test_camera_has_declared_bounded_radius_and_elevation(self):
        self.assertIn("const CAMERA_MIN_RADIUS = 1.8;", self.source)
        self.assertIn("const CAMERA_MAX_RADIUS = 7.5;", self.source)
        self.assertIn("const CAMERA_MIN_ELEVATION_RAD = 0.08;", self.source)
        self.assertIn("const CAMERA_MAX_ELEVATION_RAD = 1.45;", self.source)
        self.assertIn("clamp(radius, CAMERA_MIN_RADIUS, CAMERA_MAX_RADIUS)", self.source)
        self.assertIn("clamp(elevation, CAMERA_MIN_ELEVATION_RAD, CAMERA_MAX_ELEVATION_RAD)", self.source)

    def test_pointer_drag_orbits_camera_and_only_redraws_existing_record(self):
        self.assertIn('canvas.addEventListener("pointerdown", beginOrbit);', self.source)
        self.assertIn('canvas.addEventListener("pointermove", moveOrbit);', self.source)
        self.assertIn('canvas.addEventListener("pointerup", endOrbit);', self.source)
        self.assertIn('canvas.addEventListener("pointercancel", endOrbit);', self.source)
        self.assertIn("const spherical = cameraSpherical();", self.source)
        self.assertIn("markCameraCustom();", self.source)
        self.assertIn("setCameraSpherical(", self.source)
        self.assertIn("draw(latestRecord);", self.source)
        self.assertNotIn("state.index", self.source)
        self.assertNotIn("state.trace", self.source)
        self.assertNotIn("simulation.advance", self.source)

    def test_wheel_zoom_is_presentation_only_and_non_passive(self):
        self.assertIn('canvas.addEventListener("wheel", zoomCamera, { passive: false });', self.source)
        self.assertIn("event.preventDefault();", self.source)
        self.assertIn("Math.exp(event.deltaY * CAMERA_ZOOM_SENSITIVITY)", self.source)
        self.assertIn("clamp(spherical.radius * factor, CAMERA_MIN_RADIUS, CAMERA_MAX_RADIUS)", self.source)
        self.assertNotIn("forward_position_m =", self.source)
        self.assertNotIn("body_pitch_rad =", self.source)
        self.assertNotIn("body_roll_rad =", self.source)

    def test_presets_and_reset_are_deterministic_camera_operations(self):
        for preset in ("iso", "side", "front"):
            self.assertIn(f"{preset}: {{ eye:", self.source)
        self.assertIn("cameraState = cloneCamera(cameraPresets[preset]);", self.source)
        self.assertIn('reset.id = "spatialCameraReset";', self.source)
        self.assertIn('reset.dataset.cameraReset = "iso";', self.source)
        self.assertIn('reset.addEventListener("click", () => selectCameraPreset("iso"));', self.source)
        self.assertIn('resetCamera: () => selectCameraPreset("iso")', self.source)

    def test_camera_state_is_visible_but_not_evidence(self):
        self.assertIn('status.id = "spatialCameraState";', self.source)
        self.assertIn("className = \"spatial-camera-status\"", self.source)
        self.assertIn('scope: "presentation only"', self.source)
        self.assertIn('evidenceMutation: "none"', self.source)
        self.assertIn("Camera state is independent presentation state", self.layout)
        self.assertIn("Moving the camera is not moving the robot", self.layout)

    def test_canvas_interaction_is_local_and_has_no_control_route(self):
        lowered = self.source.lower()
        self.assertNotIn("fetch(", lowered)
        self.assertNotIn("eventsource", lowered)
        self.assertNotIn("websocket", lowered)
        self.assertNotIn("authorizedactuation(", lowered)
        self.assertNotIn("closedloopsimulation", lowered)
        self.assertIn("cursor: grab", self.style)
        self.assertIn("touch-action: none", self.style)
        self.assertIn("cursor: grabbing", self.style)

    def test_durable_layout_records_spatial_provenance_boundaries(self):
        self.assertIn("## Spatial projection boundary", self.layout)
        self.assertIn("solid truth/common body", self.layout)
        self.assertIn("optional cyan estimate-attitude ghost", self.layout)
        self.assertIn("## Spatial projection boundary", self.layout)
        self.assertIn("### Camera interaction boundary", self.layout)
        self.assertIn("does not infer a drive-wheel phase", self.layout)
        self.assertIn("does not synthesize estimated 3-D translation or reaction phase", self.layout)


if __name__ == "__main__":
    unittest.main()
