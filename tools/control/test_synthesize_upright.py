from __future__ import annotations

import unittest

import numpy as np

import synthesize_upright as synthesis


class ControlSynthesisTests(unittest.TestCase):
    def fixture(self) -> dict[str, object]:
        # Numerical fixture only. These values are not reference-platform facts.
        return {
            "sample_period_s": 0.002,
            "plant_parameters": {
                "gravity_m_per_s2": 9.80665,
                "body_mass_kg": 1.0,
                "body_com_height_m": 0.1,
                "body_inertia_roll_kg_m2": 0.01,
                "body_inertia_pitch_kg_m2": 0.01,
                "body_inertia_yaw_kg_m2": 0.01,
                "drive_wheel_mass_kg": 0.1,
                "drive_wheel_radius_m": 0.05,
                "drive_wheel_spin_inertia_kg_m2": 0.001,
                "reaction_wheel_mass_kg": 0.1,
                "reaction_wheel_com_height_m": 0.1,
                "reaction_wheel_spin_inertia_kg_m2": 0.001,
                "reaction_wheel_transverse_inertia_kg_m2": 0.0005,
            },
            "imu_placement_m": {
                "forward_x_m": 0.0,
                "left_y_m": 0.0,
                "up_z_m": 0.03,
            },
            "lqr": {
                "state_scale": [1.0, 1.0, 0.2, 1.0, 0.2, 1.0, 100.0],
                "input_scale_nm": [1.0, 1.0],
            },
            "observer": {
                "process_std": [0.001, 0.01, 0.001, 0.01, 0.001, 0.01, 0.1],
                "measurement_std": [
                    0.2,
                    0.2,
                    0.2,
                    0.01,
                    0.01,
                    0.01,
                    0.001,
                    0.1,
                ],
                "required_measurements": [
                    "accel_x_m_per_s2",
                    "accel_y_m_per_s2",
                    "gyro_x_rad_per_s",
                    "gyro_y_rad_per_s",
                    "drive_encoder_relative_angle_rad",
                    "reaction_wheel_relative_rate_rad_per_s",
                ],
            },
            "lqi": None,
        }

    def test_exact_zoh_is_not_forward_euler(self) -> None:
        config = self.fixture()
        parameters = synthesis.load_parameters(config)
        a, b, _ = synthesis.continuous_plant(parameters)
        sample_period_s = float(config["sample_period_s"])
        ad, bd = synthesis.zoh_discretize(a, b, sample_period_s)

        forward_euler_a = np.eye(7) + sample_period_s * a
        forward_euler_b = sample_period_s * b
        self.assertGreater(float(np.max(np.abs(ad - forward_euler_a))), 1.0e-10)
        self.assertGreater(float(np.max(np.abs(bd - forward_euler_b))), 1.0e-10)

    def test_synthesized_lqr_and_observer_are_discrete_time_stable(self) -> None:
        result = synthesis.synthesize(self.fixture())
        self.assertLess(result["lqr"]["closed_loop_spectral_radius"], 1.0)
        self.assertLess(result["observer"]["error_spectral_radius"], 1.0)
        self.assertEqual(result["sample_period_s"], 0.002)
        self.assertEqual(len(result["lqr"]["k"]), 2)
        self.assertEqual(len(result["observer"]["l"]), 7)

    def test_missing_physical_parameter_is_rejected_instead_of_guessed(self) -> None:
        config = self.fixture()
        config["plant_parameters"]["body_mass_kg"] = None
        with self.assertRaises(synthesis.DesignError):
            synthesis.synthesize(config)

    def test_generated_rust_contains_fixed_runtime_matrices(self) -> None:
        result = synthesis.synthesize(self.fixture())
        rust = synthesis.emit_rust(result)
        self.assertIn("pub const A_D: [[f32; 7]; 7]", rust)
        self.assertIn("pub const B_D: [[f32; 2]; 7]", rust)
        self.assertIn("pub const LQR_K: [[f32; 7]; 2]", rust)
        self.assertIn("pub const OBSERVER_L: [[f32; 8]; 7]", rust)


if __name__ == "__main__":
    unittest.main()
