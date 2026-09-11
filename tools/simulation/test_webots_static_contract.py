from __future__ import annotations

from pathlib import Path
import unittest


HERE = Path(__file__).resolve().parent
WORLD = HERE / "webots" / "worlds" / "single_wheel_synthetic.wbt"
CONTROLLER = HERE / "webots" / "controllers" / "swp_trace" / "swp_trace.py"


class WebotsStaticContractTests(unittest.TestCase):
    def test_world_keeps_project_axis_and_device_contract(self) -> None:
        text = WORLD.read_text(encoding="utf-8")
        for fragment in (
            'axis 0 1 0',
            'name "drive_motor"',
            'name "drive_encoder"',
            'axis 1 0 0',
            'name "reaction_motor"',
            'name "reaction_encoder"',
            'name "body_imu"',
            'name "body_gyro"',
            'contactMaterial "drive_tire"',
        ):
            self.assertIn(fragment, text)

    def test_controller_uses_direct_torque_and_expected_devices(self) -> None:
        text = CONTROLLER.read_text(encoding="utf-8")
        for fragment in (
            'getDevice("body_imu")',
            'getDevice("body_gyro")',
            'getDevice("drive_encoder")',
            'getDevice("reaction_encoder")',
            'drive_motor.setTorque(DRIVE_TORQUE_NM)',
            'reaction_motor.setTorque(REACTION_TORQUE_NM)',
        ):
            self.assertIn(fragment, text)


if __name__ == "__main__":
    unittest.main()
