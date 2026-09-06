#!/usr/bin/env python3
"""Decode canonical 100 Hz runtime-observation records to CSV."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from pathlib import Path
from typing import BinaryIO, Iterator

MAGIC = b"SW"
VERSION = 1
KIND_RUNTIME_OBSERVATION = 3
PAYLOAD_LEN = 120
RECORD_LEN = 128
CRC_OFFSET = RECORD_LEN - 2
STATE_NAMES = [
    "forward_position_m",
    "forward_velocity_m_per_s",
    "pitch_rad",
    "pitch_rate_rad_per_s",
    "roll_rad",
    "roll_rate_rad_per_s",
    "reaction_wheel_rate_rad_per_s",
]
OPERATING_STATES = {0: "Boot", 1: "HardwareCheck", 2: "Standby", 3: "CaptureWindow", 4: "Balancing", 5: "MomentumLimited", 6: "Fault"}
TIMING_STATES = {0: "Startup", 1: "Healthy", 2: "Late", 3: "Timeout"}
VALIDITY_STATES = {0: "Invalid", 1: "Valid"}
WATCHDOG_STATES = {0: "Startup", 1: "Healthy", 2: "Timeout"}


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


def _f32(data: bytes, offset: int) -> float:
    return struct.unpack_from("<f", data, offset)[0]


def decode_record(record: bytes) -> dict[str, int | float | str] | None:
    if len(record) != RECORD_LEN:
        return None
    magic, version, kind, payload_len = struct.unpack_from("<2sBBH", record)
    if magic != MAGIC or version != VERSION or kind != KIND_RUNTIME_OBSERVATION or payload_len != PAYLOAD_LEN:
        return None
    if crc16_ccitt_false(record[:CRC_OFFSET]) != _u16(record, CRC_OFFSET):
        return None
    operating = OPERATING_STATES.get(record[98])
    timing = TIMING_STATES.get(record[99])
    validity = VALIDITY_STATES.get(record[100])
    watchdog = WATCHDOG_STATES.get(record[101])
    if None in (operating, timing, validity, watchdog):
        return None
    flags = _u16(record, 106)
    row: dict[str, int | float | str] = {
        "sample_index": _u32(record, 6),
        "timestamp_us": _u64(record, 10),
        "drive_demand_nm": _f32(record, 74),
        "reaction_demand_nm": _f32(record, 78),
        "drive_command": _f32(record, 82),
        "reaction_command": _f32(record, 86),
        "drive_predicted_torque_nm": _f32(record, 90),
        "reaction_predicted_torque_nm": _f32(record, 94),
        "operating_state": operating,
        "timing": timing,
        "estimate_validity": validity,
        "watchdog": watchdog,
        "authority_reasons": _u16(record, 102),
        "runtime_faults": _u16(record, 104),
        "authorized": 1 if flags & 0x1 else 0,
        "drive_saturated": 1 if flags & 0x2 else 0,
        "reaction_saturated": 1 if flags & 0x4 else 0,
        "dropped_records": _u16(record, 108),
        "outer_target_velocity_m_per_s": _f32(record, 110),
        "crc": _u16(record, CRC_OFFSET),
    }
    for index, name in enumerate(STATE_NAMES):
        row[f"estimated_{name}"] = _f32(record, 18 + index * 4)
        row[f"reference_{name}"] = _f32(record, 46 + index * 4)
    return row


def records(stream: BinaryIO) -> Iterator[dict[str, int | float | str]]:
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
            if len(buffer) < 6:
                break
            _, version, kind, payload_len = struct.unpack_from("<2sBBH", buffer)
            if version != VERSION or kind != KIND_RUNTIME_OBSERVATION or payload_len != PAYLOAD_LEN:
                del buffer[0]
                continue
            if len(buffer) < RECORD_LEN:
                break
            candidate = bytes(buffer[:RECORD_LEN])
            row = decode_record(candidate)
            if row is None:
                del buffer[0]
                continue
            yield row
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
    fieldnames = ["sample_index", "timestamp_us"]
    for prefix in ("estimated", "reference"):
        fieldnames.extend(f"{prefix}_{name}" for name in STATE_NAMES)
    fieldnames.extend([
        "drive_demand_nm", "reaction_demand_nm", "drive_command", "reaction_command",
        "drive_predicted_torque_nm", "reaction_predicted_torque_nm", "operating_state", "timing",
        "estimate_validity", "watchdog", "authority_reasons", "runtime_faults", "authorized",
        "drive_saturated", "reaction_saturated", "dropped_records", "outer_target_velocity_m_per_s", "crc",
    ])
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
