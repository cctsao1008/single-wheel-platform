#!/usr/bin/env python3
"""Decode Self-Balancing Single-Wheel Platform telemetry to CSV."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from pathlib import Path
from typing import BinaryIO, Iterator

MAGIC = b"SW"
VERSION = 1
KIND_RAW_IMU = 1
KIND_SENSOR_SNAPSHOT = 2
HEADER_LEN = 6
CRC_LEN = 2
EXPECTED_PAYLOAD_LEN = {
    KIND_RAW_IMU: 30,
    KIND_SENSOR_SNAPSHOT: 36,
}
RAW_IMU_PAYLOAD = struct.Struct("<IQhhhhhhhHH")
SENSOR_SNAPSHOT_PAYLOAD = struct.Struct("<IQhhhhhhhHHHHH")


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def decode_frame(frame: bytes) -> dict[str, int] | None:
    if len(frame) < HEADER_LEN + CRC_LEN:
        return None
    magic, version, kind, payload_len = struct.unpack_from("<2sBBH", frame)
    if magic != MAGIC or version != VERSION:
        return None
    if EXPECTED_PAYLOAD_LEN.get(kind) != payload_len:
        return None
    if len(frame) != HEADER_LEN + payload_len + CRC_LEN:
        return None

    crc_offset = len(frame) - CRC_LEN
    crc = struct.unpack_from("<H", frame, crc_offset)[0]
    if crc16_ccitt_false(frame[:crc_offset]) != crc:
        return None

    payload = frame[HEADER_LEN:crc_offset]
    row: dict[str, int] = {
        "kind": kind,
        "encoder_1_count": 0,
        "encoder_2_count": 0,
        "battery_adc_raw": 0,
        "crc": crc,
    }

    if kind == KIND_RAW_IMU:
        values = RAW_IMU_PAYLOAD.unpack(payload)
        (
            row["sequence"],
            row["timestamp_us"],
            row["accel_x_raw"],
            row["accel_y_raw"],
            row["accel_z_raw"],
            row["temperature_raw"],
            row["gyro_x_raw"],
            row["gyro_y_raw"],
            row["gyro_z_raw"],
            row["status"],
            row["dropped_frames"],
        ) = values
    else:
        values = SENSOR_SNAPSHOT_PAYLOAD.unpack(payload)
        (
            row["sequence"],
            row["timestamp_us"],
            row["accel_x_raw"],
            row["accel_y_raw"],
            row["accel_z_raw"],
            row["temperature_raw"],
            row["gyro_x_raw"],
            row["gyro_y_raw"],
            row["gyro_z_raw"],
            row["encoder_1_count"],
            row["encoder_2_count"],
            row["battery_adc_raw"],
            row["status"],
            row["dropped_frames"],
        ) = values
    return row


def frames(stream: BinaryIO) -> Iterator[dict[str, int]]:
    buffer = bytearray()
    while True:
        chunk = stream.read(4096)
        if chunk:
            buffer.extend(chunk)
        elif not buffer:
            return

        while True:
            start = buffer.find(MAGIC)
            if start < 0:
                if len(buffer) > 1:
                    del buffer[:-1]
                break
            if start:
                del buffer[:start]
            if len(buffer) < HEADER_LEN:
                break

            _, version, kind, payload_len = struct.unpack_from("<2sBBH", buffer)
            expected_payload_len = EXPECTED_PAYLOAD_LEN.get(kind)
            if version != VERSION or expected_payload_len != payload_len:
                del buffer[0]
                continue

            frame_len = HEADER_LEN + payload_len + CRC_LEN
            if len(buffer) < frame_len:
                break

            candidate = bytes(buffer[:frame_len])
            row = decode_frame(candidate)
            if row is not None:
                yield row
                del buffer[:frame_len]
            else:
                del buffer[0]

        if not chunk:
            return


def open_input(path: str) -> BinaryIO:
    if path == "-":
        return sys.stdin.buffer
    return Path(path).open("rb")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="binary capture file, or - for stdin")
    args = parser.parse_args()

    fieldnames = [
        "kind",
        "sequence",
        "timestamp_us",
        "accel_x_raw",
        "accel_y_raw",
        "accel_z_raw",
        "temperature_raw",
        "gyro_x_raw",
        "gyro_y_raw",
        "gyro_z_raw",
        "encoder_1_count",
        "encoder_2_count",
        "battery_adc_raw",
        "status",
        "dropped_frames",
        "crc",
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()

    stream = open_input(args.input)
    try:
        for row in frames(stream):
            writer.writerow(row)
    finally:
        if stream is not sys.stdin.buffer:
            stream.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
