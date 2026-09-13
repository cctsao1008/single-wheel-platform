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
            'name "body_accel"',
            'name "body_gyro"',
            'contactMaterial "drive_tire"',
        ):
            self.assertIn(fragment, text)

    def test_controller_separates_raw_device_evidence_from_common_truth(self) -> None:
        text = CONTROLLER.read_text(encoding="utf-8")
        for fragment in (
            'getDevice("body_imu")',
            'getDevice("body_accel")',
            'getDevice("body_gyro")',
            'getDevice("drive_encoder")',
            'getDevice("reaction_encoder")',
            'getDevice("drive_motor")',
            'getDevice("reaction_motor")',
            'encode_raw_sample(',
            'drive_encoder_rad=drive_encoder.getValue()',
            'drive_motor.setTorque(drive_torque)',
            'reaction_motor.setTorque(reaction_torque)',
            'position = body_node.getPosition()',
            'velocity = body_node.getVelocity()',
            '"forward_position_m": position[0]',
            '"forward_velocity_m_per_s": velocity[0]',
        ):
            self.assertIn(fragment, text)

        # Closed-loop production input legitimately includes the physical drive
        # encoder. It still must never masquerade as reduced-model translation s
        # in the common/open-loop observable projection.
        self.assertNotIn('"drive_position_rad"', text)
        self.assertNotIn('"drive_rate_rad_s"', text)

    def test_only_production_bridge_response_changes_closed_loop_torque(self) -> None:
        text = CONTROLLER.read_text(encoding="utf-8")
        self.assertIn('response = bridge_step(bridge, raw_sample)', text)
        self.assertIn('if response["actuation"] == "apply":', text)
        self.assertIn('elif response["actuation"] == "revoke":', text)
        self.assertIn('"webots_evidence_truth": truth_record(', text)


if __name__ == "__main__":
    unittest.main()
