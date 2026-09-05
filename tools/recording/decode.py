#!/usr/bin/env python3
"""Decode canonical raw-observation records to CSV."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from pathlib import Path
from typing import BinaryIO, Iterator

MAGIC = b"SW"
VERSION = 1
KIND_RAW_OBSERVATION = 1
HEADER_LEN = 6
PAYLOAD_LEN = 72
RECORD_LEN = 80
CRC_OFFSET = RECORD_LEN - 2
UNKNOWN_OFFSET_US = 0xFFFFFFFF
UNKNOWN_SAMPLE_OFFSET_US = -0x80000000

ACQ_BUS_READY = 1 << 0
ACQ_IMU_PRESENT = 1 << 1
ACQ_IMU_CONFIGURED = 1 << 2
ACQ_IMU_DATA_READY_IRQ_ENABLED = 1 << 3
ACQ_IMU_DATA_READY_SEEN = 1 << 4
ACQ_IMU_TIMING_HEALTHY = 1 << 5
ACQ_IMU_TIMING_LATE = 1 << 6
ACQ_IMU_TIMING_TIMEOUT = 1 << 7


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def _u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def _i32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<i", data, offset)[0]


def _u64(data: bytes, offset: int) -> int:
    return struct.unpack_from("<Q", data, offset)[0]


def _i16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def _decode_offset(base: int, offset: int) -> int | None:
    return None if offset == UNKNOWN_OFFSET_US else base + offset


def _decode_sample_offset(base: int, offset: int) -> int | None:
    return None if offset == UNKNOWN_SAMPLE_OFFSET_US else base + offset


def _flag(bits: int, mask: int) -> int:
    return 1 if bits & mask else 0


def imu_timing_label(status: int) -> str:
    if status & ACQ_IMU_TIMING_TIMEOUT:
        return "TIMEOUT"
    if status & ACQ_IMU_TIMING_LATE:
        return "LATE"
    if status & ACQ_IMU_TIMING_HEALTHY:
        return "OK"
    return "STARTUP"


def decode_record(record: bytes) -> dict[str, int | None] | None:
    if len(record) != RECORD_LEN:
        return None
    magic, version, kind, payload_len = struct.unpack_from("<2sBBH", record)
    if (
        magic != MAGIC
        or version != VERSION
        or kind != KIND_RAW_OBSERVATION
        or payload_len != PAYLOAD_LEN
    ):
        return None
    crc = _u16(record, CRC_OFFSET)
    if crc16_ccitt_false(record[:CRC_OFFSET]) != crc:
        return None

    base = _u64(record, 10)
    acquisition_duration_us = _u32(record, 18)
    if acquisition_duration_us == UNKNOWN_OFFSET_US:
        return None
    acquisition_status = _u16(record, 74)

    return {
        "sequence": _u32(record, 6),
        "acquisition_started_us": base,
        "acquisition_completed_us": base + acquisition_duration_us,
        "acquisition_duration_us": acquisition_duration_us,
        "imu_sample_time_us": _decode_sample_offset(base, _i32(record, 22)),
        "imu_read_started_us": _decode_offset(base, _u32(record, 26)),
        "imu_read_completed_us": _decode_offset(base, _u32(record, 30)),
        "accel_x_raw": _i16(record, 34),
        "accel_y_raw": _i16(record, 36),
        "accel_z_raw": _i16(record, 38),
        "temperature_raw": _i16(record, 40),
        "gyro_x_raw": _i16(record, 42),
        "gyro_y_raw": _i16(record, 44),
        "gyro_z_raw": _i16(record, 46),
        "imu_quality": _u16(record, 48),
        "encoder_1_captured_at_us": _decode_offset(base, _u32(record, 50)),
        "encoder_1_count": _u16(record, 54),
        "encoder_1_quality": _u16(record, 56),
        "encoder_2_captured_at_us": _decode_offset(base, _u32(record, 58)),
        "encoder_2_count": _u16(record, 62),
        "encoder_2_quality": _u16(record, 64),
        "battery_read_completed_us": _decode_offset(base, _u32(record, 66)),
        "battery_adc_raw": _u16(record, 70),
        "battery_quality": _u16(record, 72),
        "acquisition_status": acquisition_status,
        "imu_data_ready_seen": _flag(acquisition_status, ACQ_IMU_DATA_READY_SEEN),
        "imu_timing_healthy": _flag(acquisition_status, ACQ_IMU_TIMING_HEALTHY),
        "imu_timing_late": _flag(acquisition_status, ACQ_IMU_TIMING_LATE),
        "imu_timing_timeout": _flag(acquisition_status, ACQ_IMU_TIMING_TIMEOUT),
        "dropped_records": _u16(record, 76),
        "crc": crc,
    }


def records(stream: BinaryIO) -> Iterator[dict[str, int | None]]:
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
            if version != VERSION or kind != KIND_RAW_OBSERVATION or payload_len != PAYLOAD_LEN:
                del buffer[0]
                continue
            if len(buffer) < RECORD_LEN:
                break

            candidate = bytes(buffer[:RECORD_LEN])
            row = decode_record(candidate)
            if row is not None:
                yield row
                del buffer[:RECORD_LEN]
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
    parser.add_argument("input", help="binary observation-record file, or - for stdin")
    args = parser.parse_args()

    fieldnames = [
        "sequence",
        "acquisition_started_us",
        "acquisition_completed_us",
        "acquisition_duration_us",
        "imu_sample_time_us",
        "imu_read_started_us",
        "imu_read_completed_us",
        "accel_x_raw",
        "accel_y_raw",
        "accel_z_raw",
        "temperature_raw",
        "gyro_x_raw",
        "gyro_y_raw",
        "gyro_z_raw",
        "imu_quality",
        "encoder_1_captured_at_us",
        "encoder_1_count",
        "encoder_1_quality",
        "encoder_2_captured_at_us",
        "encoder_2_count",
        "encoder_2_quality",
        "battery_read_completed_us",
        "battery_adc_raw",
        "battery_quality",
        "acquisition_status",
        "imu_data_ready_seen",
        "imu_timing_healthy",
        "imu_timing_late",
        "imu_timing_timeout",
        "dropped_records",
        "crc",
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()

    stream = open_input(args.input)
    try:
        for row in records(stream):
            writer.writerow(row)
    finally:
        if stream is not sys.stdin.buffer:
            stream.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
