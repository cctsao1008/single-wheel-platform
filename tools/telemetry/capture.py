#!/usr/bin/env python3
"""Capture binary telemetry from a serial port without interpreting it."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: py -m pip install pyserial") from exc


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="serial port, for example COM5 or /dev/ttyUSB0")
    parser.add_argument("output", help="binary output file")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    path = Path(args.output)
    with serial.Serial(args.port, args.baud, timeout=1) as source, path.open("wb") as target:
        print(f"capturing {args.port} @ {args.baud} -> {path}", file=sys.stderr)
        try:
            while True:
                chunk = source.read(4096)
                if chunk:
                    target.write(chunk)
                    target.flush()
        except KeyboardInterrupt:
            print("capture stopped", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
