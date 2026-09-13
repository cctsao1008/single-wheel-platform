import math
import unittest

from webots.bridge_protocol import (
    AxisMapping,
    BridgeContractError,
    STANDARD_GRAVITY_MPS2,
    encode_raw_sample,
)


class WebotsBridgeProtocolTests(unittest.TestCase):
    def test_stationary_upright_matches_mpu6050_fixture(self):
        sample = encode_raw_sample(
            sample_index=0,
            timestamp_us=2_000,
            accel_m_per_s2=[0.0, 0.0, STANDARD_GRAVITY_MPS2],
            gyro_rad_per_s=[0.0, 0.0, 0.0],
            drive_encoder_rad=0.0,
            reaction_encoder_rad=0.0,
        )
        self.assertEqual(sample["accel_raw"], [0, 0, 8192])
        self.assertEqual(sample["gyro_raw"], [0, 0, 0])
        self.assertEqual(sample["drive_encoder_count"], 0)
        self.assertEqual(sample["reaction_encoder_count"], 0)

    def test_project_physical_signs_survive_quantization(self):
        sample = encode_raw_sample(
            sample_index=1,
            timestamp_us=4_000,
            accel_m_per_s2=[-0.5, 0.25, STANDARD_GRAVITY_MPS2],
            gyro_rad_per_s=[0.1, 0.5, -0.2],
            drive_encoder_rad=math.pi / 2.0,
            reaction_encoder_rad=-math.pi / 2.0,
        )
        self.assertLess(sample["accel_raw"][0], 0)
        self.assertGreater(sample["accel_raw"][1], 0)
        self.assertGreater(sample["gyro_raw"][0], 0)
        self.assertGreater(sample["gyro_raw"][1], 0)
        self.assertLess(sample["gyro_raw"][2], 0)
        self.assertEqual(sample["drive_encoder_count"], 16_384)
        self.assertEqual(sample["reaction_encoder_count"], 49_152)

    def test_one_revolution_wraps_without_changing_positive_direction(self):
        sample = encode_raw_sample(
            sample_index=2,
            timestamp_us=6_000,
            accel_m_per_s2=[0.0, 0.0, STANDARD_GRAVITY_MPS2],
            gyro_rad_per_s=[0.0, 0.0, 0.0],
            drive_encoder_rad=math.tau,
            reaction_encoder_rad=math.tau,
        )
        self.assertEqual(sample["drive_encoder_count"], 0)
        self.assertEqual(sample["reaction_encoder_count"], 0)

    def test_deliberately_broken_pitch_sign_mapping_is_rejected(self):
        broken = AxisMapping(gyro_signs=(1, -1, 1))
        with self.assertRaises(BridgeContractError):
            encode_raw_sample(
                sample_index=3,
                timestamp_us=8_000,
                accel_m_per_s2=[0.0, 0.0, STANDARD_GRAVITY_MPS2],
                gyro_rad_per_s=[0.0, 0.5, 0.0],
                drive_encoder_rad=0.0,
                reaction_encoder_rad=0.0,
                mapping=broken,
            )

    def test_nonfinite_device_evidence_fails_closed(self):
        with self.assertRaises(BridgeContractError):
            encode_raw_sample(
                sample_index=4,
                timestamp_us=10_000,
                accel_m_per_s2=[0.0, float("nan"), STANDARD_GRAVITY_MPS2],
                gyro_rad_per_s=[0.0, 0.0, 0.0],
                drive_encoder_rad=0.0,
                reaction_encoder_rad=0.0,
            )


if __name__ == "__main__":
    unittest.main()
