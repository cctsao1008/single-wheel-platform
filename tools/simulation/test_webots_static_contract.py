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

    def test_controller_uses_direct_torque_and_common_physical_observables(self) -> None:
        text = CONTROLLER.read_text(encoding="utf-8")
        for fragment in (
            'getDevice("body_imu")',
            'getDevice("body_gyro")',
            'getDevice("reaction_encoder")',
            'getDevice("drive_motor")',
            'getDevice("reaction_motor")',
            'drive_motor.setTorque(drive_torque)',
            'reaction_motor.setTorque(reaction_torque)',
            'position = body_node.getPosition()',
            'velocity = body_node.getVelocity()',
            '"forward_position_m": position[0]',
            '"forward_velocity_m_per_s": velocity[0]',
        ):
            self.assertIn(fragment, text)

        # The world may retain a drive encoder for diagnostics, but the v2 common
        # projection must not read drive-joint angle as if it were translation s.
        self.assertNotIn('getDevice("drive_encoder")', text)
        self.assertNotIn('"drive_position_rad"', text)
        self.assertNotIn('"drive_rate_rad_s"', text)


if __name__ == "__main__":
    unittest.main()
