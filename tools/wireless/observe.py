#!/usr/bin/env python3
"""Capture and observe canonical records from the ECB02S2 BLE link."""

from __future__ import annotations

import argparse
import asyncio
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

RECORDING_DIR = Path(__file__).resolve().parents[1] / "recording"
sys.path.insert(0, str(RECORDING_DIR))
import decode as record_decode  # noqa: E402


class StreamDecoder:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.invalid_records = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[dict[str, int | None]]:
        self.buffer.extend(data)
        rows: list[dict[str, int | None]] = []

        while True:
            start = self.buffer.find(record_decode.MAGIC)
            if start < 0:
                if len(self.buffer) > 1:
                    self.discarded_bytes += len(self.buffer) - 1
                    del self.buffer[:-1]
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]

            if len(self.buffer) < record_decode.HEADER_LEN:
                break

            version = self.buffer[2]
            kind = self.buffer[3]
            payload_len = int.from_bytes(self.buffer[4:6], "little")
            if (
                version != record_decode.VERSION
                or kind != record_decode.KIND_RAW_OBSERVATION
                or payload_len != record_decode.PAYLOAD_LEN
            ):
                self.invalid_records += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue

            if len(self.buffer) < record_decode.RECORD_LEN:
                break

            candidate = bytes(self.buffer[: record_decode.RECORD_LEN])
            stored_crc = int.from_bytes(candidate[-2:], "little")
            computed_crc = record_decode.crc16_ccitt_false(candidate[:-2])
            if stored_crc != computed_crc:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue

            row = record_decode.decode_record(candidate)
            if row is None:
                self.invalid_records += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue

            rows.append(row)
            del self.buffer[: record_decode.RECORD_LEN]

        return rows


@dataclass
class LinkStats:
    rx_bytes: int = 0
    frames: int = 0
    sequence_gaps: int = 0
    callback_chunks: int = 0
    last_sequence: int | None = None
    device_drops: int = 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan", action="store_true", help="list visible BLE devices and exit")
    parser.add_argument("--name", default="ECB02", help="substring used to select the BLE device")
    parser.add_argument("--address", help="BLE address/identifier; overrides --name")
    parser.add_argument("--notify-uuid", help="explicit notification characteristic UUID")
    parser.add_argument("--scan-timeout", type=float, default=5.0)
    parser.add_argument("--duration", type=float, help="capture duration in seconds")
    parser.add_argument("--display-hz", type=float, default=2.0)
    parser.add_argument("--output", help="raw binary capture path")
    return parser


def default_capture_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return Path(f"swp-{stamp}.bin")


async def discover_device(args: argparse.Namespace, BleakScanner: Any) -> Any:
    devices = await BleakScanner.discover(timeout=args.scan_timeout)
    if args.scan:
        for device in devices:
            print(f"{device.address}  {device.name or '<unnamed>'}")
        return None

    if args.address:
        for device in devices:
            if device.address.lower() == args.address.lower():
                return device
        raise RuntimeError(f"BLE device {args.address!r} was not found")

    matches = [device for device in devices if args.name.lower() in (device.name or "").lower()]
    if not matches:
        visible = "\n".join(
            f"  {device.address}  {device.name or '<unnamed>'}" for device in devices
        )
        raise RuntimeError(
            f"no BLE device name contains {args.name!r}; visible devices:\n{visible or '  <none>'}"
        )
    if len(matches) > 1:
        choices = "\n".join(f"  {device.address}  {device.name or '<unnamed>'}" for device in matches)
        raise RuntimeError(f"multiple devices match {args.name!r}; use --address:\n{choices}")
    return matches[0]


def notification_characteristic(client: Any, explicit_uuid: str | None) -> Any:
    candidates = []
    for service in client.services:
        for characteristic in service.characteristics:
            properties = {item.lower() for item in characteristic.properties}
            if "notify" in properties or "indicate" in properties:
                candidates.append(characteristic)

    if explicit_uuid:
        target = explicit_uuid.lower()
        for characteristic in candidates:
            if characteristic.uuid.lower() == target:
                return characteristic
        available = "\n".join(f"  {characteristic.uuid}" for characteristic in candidates)
        raise RuntimeError(
            f"notify characteristic {explicit_uuid!r} was not found; candidates:\n"
            f"{available or '  <none>'}"
        )

    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise RuntimeError("connected device exposes no notify/indicate characteristic")

    available = "\n".join(
        f"  {characteristic.uuid}  properties={','.join(characteristic.properties)}"
        for characteristic in candidates
    )
    raise RuntimeError(
        "multiple notify characteristics are available; pass --notify-uuid:\n" + available
    )


def update_sequence(stats: LinkStats, sequence: int) -> None:
    if stats.last_sequence is not None:
        expected = (stats.last_sequence + 1) & 0xFFFFFFFF
        if sequence != expected:
            stats.sequence_gaps += (sequence - expected) & 0xFFFFFFFF
    stats.last_sequence = sequence


def frame_rate(frame_times: deque[float]) -> float:
    if len(frame_times) < 2:
        return 0.0
    span = frame_times[-1] - frame_times[0]
    return 0.0 if span <= 0.0 else (len(frame_times) - 1) / span


def status_line(
    row: dict[str, int | None],
    stats: LinkStats,
    decoder: StreamDecoder,
    frame_times: deque[float],
) -> str:
    timing = record_decode.imu_timing_label(int(row["acquisition_status"]))
    return (
        f"seq={int(row['sequence']):10d} "
        f"rate={frame_rate(frame_times):6.2f}Hz "
        f"timing={timing:7s} "
        f"gaps={stats.sequence_gaps:5d} "
        f"crc={decoder.crc_errors:4d} "
        f"drops={stats.device_drops:5d} "
        f"acc=({int(row['accel_x_raw']):6d},{int(row['accel_y_raw']):6d},{int(row['accel_z_raw']):6d}) "
        f"gyro=({int(row['gyro_x_raw']):6d},{int(row['gyro_y_raw']):6d},{int(row['gyro_z_raw']):6d}) "
        f"enc=({int(row['encoder_1_count']):5d},{int(row['encoder_2_count']):5d}) "
        f"bat={int(row['battery_adc_raw']):4d}"
    )


async def capture(args: argparse.Namespace, BleakClient: Any, BleakScanner: Any) -> int:
    device = await discover_device(args, BleakScanner)
    if args.scan:
        return 0

    output = Path(args.output) if args.output else default_capture_path()
    output.parent.mkdir(parents=True, exist_ok=True)

    queue: asyncio.Queue[bytes] = asyncio.Queue()
    stats = LinkStats()
    decoder = StreamDecoder()
    frame_times: deque[float] = deque(maxlen=200)
    display_period = 1.0 / max(args.display_hz, 0.1)
    next_display = time.monotonic()
    started = time.monotonic()

    async with BleakClient(device) as client:
        characteristic = notification_characteristic(client, args.notify_uuid)
        print(f"device       {device.name or '<unnamed>'} ({device.address})")
        print(f"notify UUID  {characteristic.uuid}")
        print(f"capture      {output}")

        def on_notify(_: Any, data: bytearray) -> None:
            stats.callback_chunks += 1
            queue.put_nowait(bytes(data))

        await client.start_notify(characteristic, on_notify)
        try:
            with output.open("wb") as raw:
                last_row: dict[str, int | None] | None = None
                while True:
                    if args.duration is not None and time.monotonic() - started >= args.duration:
                        break

                    try:
                        chunk = await asyncio.wait_for(queue.get(), timeout=0.25)
                    except TimeoutError:
                        if not client.is_connected:
                            raise RuntimeError("BLE connection closed")
                        continue

                    raw.write(chunk)
                    stats.rx_bytes += len(chunk)
                    for row in decoder.feed(chunk):
                        now = time.monotonic()
                        frame_times.append(now)
                        stats.frames += 1
                        sequence = int(row["sequence"])
                        update_sequence(stats, sequence)
                        stats.device_drops = int(row["dropped_records"])
                        last_row = row

                    now = time.monotonic()
                    if last_row is not None and now >= next_display:
                        print("\r" + status_line(last_row, stats, decoder, frame_times), end="", flush=True)
                        next_display = now + display_period
        finally:
            await client.stop_notify(characteristic)

    print()
    print(
        f"frames={stats.frames} bytes={stats.rx_bytes} gaps={stats.sequence_gaps} "
        f"crc_errors={decoder.crc_errors} invalid={decoder.invalid_records} "
        f"device_drops={stats.device_drops} discarded_bytes={decoder.discarded_bytes}"
    )
    return 0


def main() -> int:
    args = build_parser().parse_args()
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print(
            "bleak is required: python -m pip install -r tools/wireless/requirements.txt",
            file=sys.stderr,
        )
        return 2

    try:
        return asyncio.run(capture(args, BleakClient, BleakScanner))
    except KeyboardInterrupt:
        print()
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
