import io
import struct
import unittest

import decode_control_profile as profile


def build_record(kind: int = profile.KIND_CONTROL_PROFILE) -> bytes:
    out = bytearray(profile.RECORD_LEN)
    struct.pack_into("<2sBBH", out, 0, profile.MAGIC, profile.VERSION, kind, profile.PAYLOAD_LEN)
    struct.pack_into("<I", out, 6, 42)
    struct.pack_into("<Q", out, 10, 123_456)
    for offset, value in [
        (18, 7_200),
        (22, 144),
        (26, 3_600),
        (30, 10_800),
        (34, 1_440),
        (38, 2_160),
        (42, 72_000),
        (46, 80_000),
        (50, 90_000),
        (54, 144_000),
        (58, 2),
        (62, 72_000_000),
    ]:
        struct.pack_into("<I", out, offset, value)
    struct.pack_into("<H", out, 66, 0x12)
    status = (
        profile.STATUS_SYNTHETIC_NUMERICS
        | profile.STATUS_MOTOR_PERIPHERALS_ABSENT
        | profile.STATUS_ESTIMATOR_OK
    )
    struct.pack_into("<H", out, 68, status)
    struct.pack_into("<H", out, 70, 3)
    struct.pack_into("<H", out, profile.CRC_OFFSET, profile.crc16_ccitt_false(out[: profile.CRC_OFFSET]))
    return bytes(out)


class DecodeControlProfileTests(unittest.TestCase):
    def test_decode_exposes_cycles_microseconds_and_headroom(self) -> None:
        row = profile.decode_record(build_record())
        self.assertIsNotNone(row)
        assert row is not None
        self.assertEqual(row["sequence"], 42)
        self.assertAlmostEqual(row["critical_path_us"], 1000.0)
        self.assertAlmostEqual(row["deadline_us"], 2000.0)
        self.assertAlmostEqual(row["headroom_percent"], 50.0)
        self.assertEqual(row["synthetic_numerics"], 1)
        self.assertEqual(row["motor_peripherals_absent"], 1)
        self.assertEqual(row["estimator_ok"], 1)
        self.assertEqual(row["dropped_records"], 3)

    def test_stream_skips_other_valid_sw_record_kinds_atomically(self) -> None:
        other = build_record(kind=1)
        wanted = build_record()
        rows = list(profile.records(io.BytesIO(b"junk" + other + wanted)))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["sequence"], 42)

    def test_crc_failure_is_rejected(self) -> None:
        broken = bytearray(build_record())
        broken[30] ^= 0x55
        self.assertIsNone(profile.decode_record(bytes(broken)))
        self.assertEqual(list(profile.records(io.BytesIO(bytes(broken)))), [])


if __name__ == "__main__":
    unittest.main()
