#!/usr/bin/env python3
"""Decode Self-Balancing Single-Wheel Platform raw IMU telemetry to CSV."""

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
PAYLOAD_LEN = 30
FRAME_LEN = 38
HEADER_AND_PAYLOAD_LEN = 36
FRAME_STRUCT = struct.Struct("<2sBBHIQhhhhhhhHHH")


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def valid_frame(frame: bytes) -> bool:
    if len(frame) != FRAME_LEN:
        return False
    if frame[:2] != MAGIC or frame[2] != VERSION or frame[3] != KIND_RAW_IMU:
        return False
    if struct.unpack_from("<H", frame, 4)[0] != PAYLOAD_LEN:
        return False
    expected = struct.unpack_from("<H", frame, HEADER_AND_PAYLOAD_LEN)[0]
    return crc16_ccitt_false(frame[:HEADER_AND_PAYLOAD_LEN]) == expected


def frames(stream: BinaryIO) -> Iterator[tuple[int, ...]]:
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
            if len(buffer) < FRAME_LEN:
                break

            candidate = bytes(buffer[:FRAME_LEN])
            if valid_frame(candidate):
                values = FRAME_STRUCT.unpack(candidate)
                yield values[4:]
                del buffer[:FRAME_LEN]
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
        "sequence",
        "timestamp_us",
        "accel_x_raw",
        "accel_y_raw",
        "accel_z_raw",
        "temperature_raw",
        "gyro_x_raw",
        "gyro_y_raw",
        "gyro_z_raw",
        "status",
        "dropped_frames",
        "crc",
    ]
    writer = csv.writer(sys.stdout, lineterminator="\n")
    writer.writerow(fieldnames)

    stream = open_input(args.input)
    try:
        for values in frames(stream):
            writer.writerow(values)
    finally:
        if stream is not sys.stdin.buffer:
            stream.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
