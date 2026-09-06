#!/usr/bin/env python3
"""Decode live shadow-control timing records to CSV."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from pathlib import Path
from typing import BinaryIO, Iterator

MAGIC = b"SW"
VERSION = 1
KIND_CONTROL_PROFILE = 2
HEADER_LEN = 6
PAYLOAD_LEN = 72
RECORD_LEN = 80
CRC_OFFSET = RECORD_LEN - 2

STATUS_SYNTHETIC_NUMERICS = 1 << 0
STATUS_MOTOR_PERIPHERALS_ABSENT = 1 << 1
STATUS_IMU_IO_OK = 1 << 2
STATUS_TIMING_HEALTHY = 1 << 3
STATUS_SEMANTIC_PROJECTION_READY = 1 << 4
STATUS_ESTIMATOR_OK = 1 << 5
STATUS_FEEDBACK_OK = 1 << 6
STATUS_AUTHORITY_EVALUATED = 1 << 7
STATUS_AUTHORIZED_TOKEN_DROPPED = 1 << 8
STATUS_CRITICAL_PATH_OVERRUN = 1 << 9


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


def _u64(data: bytes, offset: int) -> int:
    return struct.unpack_from("<Q", data, offset)[0]


def _flag(bits: int, mask: int) -> int:
    return 1 if bits & mask else 0


def _cycles_to_us(cycles: int, cpu_hz: int) -> float:
    return float("nan") if cpu_hz <= 0 else cycles * 1_000_000.0 / cpu_hz


def decode_record(record: bytes) -> dict[str, int | float] | None:
    if len(record) != RECORD_LEN:
        return None
    magic, version, kind, payload_len = struct.unpack_from("<2sBBH", record)
    if (
        magic != MAGIC
        or version != VERSION
        or kind != KIND_CONTROL_PROFILE
        or payload_len != PAYLOAD_LEN
    ):
        return None
    stored_crc = _u16(record, CRC_OFFSET)
    if crc16_ccitt_false(record[:CRC_OFFSET]) != stored_crc:
        return None

    cpu_hz = _u32(record, 62)
    critical = _u32(record, 42)
    deadline = _u32(record, 54)
    status = _u16(record, 68)
    headroom_percent = (
        float("nan") if deadline <= 0 else 100.0 * (1.0 - critical / deadline)
    )

    return {
        "sequence": _u32(record, 6),
        "event_started_us": _u64(record, 10),
        "imu_read_cycles": _u32(record, 18),
        "encoder_snapshot_cycles": _u32(record, 22),
        "semantic_projection_cycles": _u32(record, 26),
        "estimator_cycles": _u32(record, 30),
        "feedback_cycles": _u32(record, 34),
        "actuator_authority_cycles": _u32(record, 38),
        "critical_path_cycles": critical,
        "window_max_critical_path_cycles": _u32(record, 46),
        "boot_max_critical_path_cycles": _u32(record, 50),
        "deadline_cycles": deadline,
        "overrun_count": _u32(record, 58),
        "cpu_hz": cpu_hz,
        "authority_reasons": _u16(record, 66),
        "status": status,
        "dropped_records": _u16(record, 70),
        "imu_read_us": _cycles_to_us(_u32(record, 18), cpu_hz),
        "encoder_snapshot_us": _cycles_to_us(_u32(record, 22), cpu_hz),
        "semantic_projection_us": _cycles_to_us(_u32(record, 26), cpu_hz),
        "estimator_us": _cycles_to_us(_u32(record, 30), cpu_hz),
        "feedback_us": _cycles_to_us(_u32(record, 34), cpu_hz),
        "actuator_authority_us": _cycles_to_us(_u32(record, 38), cpu_hz),
        "critical_path_us": _cycles_to_us(critical, cpu_hz),
        "window_max_critical_path_us": _cycles_to_us(_u32(record, 46), cpu_hz),
        "boot_max_critical_path_us": _cycles_to_us(_u32(record, 50), cpu_hz),
        "deadline_us": _cycles_to_us(deadline, cpu_hz),
        "headroom_percent": headroom_percent,
        "synthetic_numerics": _flag(status, STATUS_SYNTHETIC_NUMERICS),
        "motor_peripherals_absent": _flag(status, STATUS_MOTOR_PERIPHERALS_ABSENT),
        "imu_io_ok": _flag(status, STATUS_IMU_IO_OK),
        "timing_healthy": _flag(status, STATUS_TIMING_HEALTHY),
        "semantic_ready": _flag(status, STATUS_SEMANTIC_PROJECTION_READY),
        "estimator_ok": _flag(status, STATUS_ESTIMATOR_OK),
        "feedback_ok": _flag(status, STATUS_FEEDBACK_OK),
        "authority_evaluated": _flag(status, STATUS_AUTHORITY_EVALUATED),
        "authorized_token_dropped": _flag(status, STATUS_AUTHORIZED_TOKEN_DROPPED),
        "critical_path_overrun": _flag(status, STATUS_CRITICAL_PATH_OVERRUN),
        "crc": stored_crc,
    }


def records(stream: BinaryIO) -> Iterator[dict[str, int | float]]:
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
            if version != VERSION or payload_len != PAYLOAD_LEN:
                del buffer[0]
                continue
            if len(buffer) < RECORD_LEN:
                break

            candidate = bytes(buffer[:RECORD_LEN])
            if crc16_ccitt_false(candidate[:CRC_OFFSET]) != _u16(candidate, CRC_OFFSET):
                del buffer[0]
                continue

            if kind == KIND_CONTROL_PROFILE:
                row = decode_record(candidate)
                if row is not None:
                    yield row
            # Other valid fixed-size SW v1 records are skipped atomically.
            del buffer[:RECORD_LEN]

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
        "event_started_us",
        "imu_read_cycles",
        "encoder_snapshot_cycles",
        "semantic_projection_cycles",
        "estimator_cycles",
        "feedback_cycles",
        "actuator_authority_cycles",
        "critical_path_cycles",
        "window_max_critical_path_cycles",
        "boot_max_critical_path_cycles",
        "deadline_cycles",
        "overrun_count",
        "cpu_hz",
        "authority_reasons",
        "status",
        "dropped_records",
        "imu_read_us",
        "encoder_snapshot_us",
        "semantic_projection_us",
        "estimator_us",
        "feedback_us",
        "actuator_authority_us",
        "critical_path_us",
        "window_max_critical_path_us",
        "boot_max_critical_path_us",
        "deadline_us",
        "headroom_percent",
        "synthetic_numerics",
        "motor_peripherals_absent",
        "imu_io_ok",
        "timing_healthy",
        "semantic_ready",
        "estimator_ok",
        "feedback_ok",
        "authority_evaluated",
        "authorized_token_dropped",
        "critical_path_overrun",
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
