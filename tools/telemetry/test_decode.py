from __future__ import annotations

import io
import struct
import unittest

import decode


def make_frame(kind: int, payload: bytes) -> bytes:
    header = struct.pack("<2sBBH", decode.MAGIC, decode.VERSION, kind, len(payload))
    body = header + payload
    return body + struct.pack("<H", decode.crc16_ccitt_false(body))


class DecoderTests(unittest.TestCase):
    def test_sensor_snapshot(self) -> None:
        payload = decode.SENSOR_SNAPSHOT_PAYLOAD.pack(
            7,
            123456,
            -1,
            2,
            -3,
            4,
            -5,
            6,
            -7,
            100,
            200,
            3000,
            0x007F,
            9,
        )
        row = decode.decode_frame(make_frame(decode.KIND_SENSOR_SNAPSHOT, payload))
        self.assertIsNotNone(row)
        assert row is not None
        self.assertEqual(row["encoder_1_count"], 100)
        self.assertEqual(row["encoder_2_count"], 200)
        self.assertEqual(row["battery_adc_raw"], 3000)

    def test_legacy_raw_imu_frame_is_still_decoded(self) -> None:
        payload = decode.RAW_IMU_PAYLOAD.pack(
            8,
            654321,
            1,
            2,
            3,
            4,
            5,
            6,
            7,
            0x000F,
            10,
        )
        row = decode.decode_frame(make_frame(decode.KIND_RAW_IMU, payload))
        self.assertIsNotNone(row)
        assert row is not None
        self.assertEqual(row["encoder_1_count"], 0)
        self.assertEqual(row["battery_adc_raw"], 0)

    def test_stream_resynchronizes_after_corruption(self) -> None:
        payload = decode.SENSOR_SNAPSHOT_PAYLOAD.pack(
            9,
            111,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            1,
            2,
            3,
            0x007F,
            0,
        )
        good = make_frame(decode.KIND_SENSOR_SNAPSHOT, payload)
        bad = bytearray(good)
        bad[20] ^= 0x01
        rows = list(decode.frames(io.BytesIO(b"noise" + bytes(bad) + good)))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["sequence"], 9)


if __name__ == "__main__":
    unittest.main()
