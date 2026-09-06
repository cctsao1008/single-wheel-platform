import importlib.util
import struct
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).with_name("decode_runtime_observation.py")
spec = importlib.util.spec_from_file_location("decode_runtime_observation", MODULE_PATH)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(mod)


class RuntimeObservationDecoderTests(unittest.TestCase):
    def make_record(self) -> bytes:
        out = bytearray(mod.RECORD_LEN)
        struct.pack_into("<2sBBH", out, 0, mod.MAGIC, mod.VERSION, mod.KIND_RUNTIME_OBSERVATION, mod.PAYLOAD_LEN)
        struct.pack_into("<I", out, 6, 42)
        struct.pack_into("<Q", out, 10, 123456)
        for i in range(7):
            struct.pack_into("<f", out, 18 + i * 4, float(i + 1))
            struct.pack_into("<f", out, 46 + i * 4, float(-(i + 1)))
        struct.pack_into("<ffffff", out, 74, 0.2, -0.3, 0.4, -0.5, 0.19, -0.28)
        out[98] = 4
        out[99] = 1
        out[100] = 1
        out[101] = 1
        struct.pack_into("<HHHHf", out, 102, 0x12, 0x20, 0x3, 5, 0.75)
        struct.pack_into("<H", out, mod.CRC_OFFSET, mod.crc16_ccitt_false(out[: mod.CRC_OFFSET]))
        return bytes(out)

    def test_decode(self):
        row = mod.decode_record(self.make_record())
        self.assertIsNotNone(row)
        assert row is not None
        self.assertEqual(row["sample_index"], 42)
        self.assertEqual(row["operating_state"], "Balancing")
        self.assertEqual(row["authorized"], 1)
        self.assertEqual(row["drive_saturated"], 1)
        self.assertEqual(row["reaction_saturated"], 0)
        self.assertAlmostEqual(row["outer_target_velocity_m_per_s"], 0.75)

    def test_crc_rejects_corruption(self):
        record = bytearray(self.make_record())
        record[22] ^= 0x40
        self.assertIsNone(mod.decode_record(bytes(record)))


if __name__ == "__main__":
    unittest.main()
