import unittest

import numpy as np

from identify_static import effective_command, fit


class ActuatorIdentificationTest(unittest.TestCase):
    def test_recovers_synthetic_static_model(self):
        command = np.linspace(-1.0, 1.0, 81)
        speed = np.linspace(-25.0, 25.0, 81)
        deadzone = 0.12
        ku = 0.24
        viscous = 0.0015
        coulomb = 0.009
        sign = np.where(speed > 0.5, 1.0, np.where(speed < -0.5, -1.0, 0.0))
        torque = ku * effective_command(command, deadzone) - viscous * speed - coulomb * sign

        result = fit(command, speed, torque, max_deadzone=0.2, deadzone_steps=201)

        self.assertAlmostEqual(result["command_deadzone"], deadzone, places=3)
        self.assertAlmostEqual(result["torque_per_effective_command_nm"], ku, places=5)
        self.assertAlmostEqual(result["viscous_friction_nm_per_rad_s"], viscous, places=5)
        self.assertAlmostEqual(result["coulomb_friction_nm"], coulomb, places=5)
        self.assertLess(result["fit_rmse_nm"], 1.0e-10)


if __name__ == "__main__":
    unittest.main()
