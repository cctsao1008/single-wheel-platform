from __future__ import annotations

import io
import struct
import unittest

import decode


def make_record() -> bytes:
    record = bytearray(decode.RECORD_LEN)
    struct.pack_into(
        "<2sBBH",
        record,
        0,
        decode.MAGIC,
        decode.VERSION,
        decode.KIND_RAW_OBSERVATION,
        decode.PAYLOAD_LEN,
    )
    base = 1_000_000
    struct.pack_into("<I", record, 6, 7)
    struct.pack_into("<Q", record, 10, base)
    struct.pack_into("<I", record, 18, 1750)
    struct.pack_into("<i", record, 22, decode.UNKNOWN_SAMPLE_OFFSET_US)
    struct.pack_into("<I", record, 26, 40)
    struct.pack_into("<I", record, 30, 1400)
    struct.pack_into("<hhhhhhh", record, 34, -1, 2, -3, 4, -5, 6, -7)
    struct.pack_into("<H", record, 48, 0x0003)
    struct.pack_into("<IHH", record, 50, 1420, 100, 0x000B)
    struct.pack_into("<IHH", record, 58, 1430, 200, 0x000B)
    struct.pack_into("<IHHHH", record, 66, 1700, 3000, 0x0003, 0x0007, 9)
    crc = decode.crc16_ccitt_false(record[: decode.CRC_OFFSET])
    struct.pack_into("<H", record, decode.CRC_OFFSET, crc)
    return bytes(record)


class DecoderTests(unittest.TestCase):
    def test_record_preserves_unknown_imu_sample_time(self) -> None:
        row = decode.decode_record(make_record())
        self.assertIsNotNone(row)
        assert row is not None
        self.assertIsNone(row["imu_sample_time_us"])
        self.assertEqual(row["imu_read_started_us"], 1_000_040)
        self.assertEqual(row["encoder_1_captured_at_us"], 1_001_420)
        self.assertEqual(row["encoder_2_count"], 200)
        self.assertEqual(row["battery_adc_raw"], 3000)
        self.assertEqual(row["dropped_records"], 9)

    def test_stream_resynchronizes_after_corruption(self) -> None:
        good = make_record()
        bad = bytearray(good)
        bad[40] ^= 0x01
        rows = list(decode.records(io.BytesIO(b"noise" + bytes(bad) + good)))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["sequence"], 7)

    def test_crc_rejects_corruption(self) -> None:
        bad = bytearray(make_record())
        bad[70] ^= 0x01
        self.assertIsNone(decode.decode_record(bytes(bad)))


if __name__ == "__main__":
    unittest.main()
