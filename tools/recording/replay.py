#!/usr/bin/env python3
"""Replay canonical observation records as deterministic JSON lines.

Replay timing comes from the recorded measurement metadata. Host wall-clock
execution speed is intentionally irrelevant.
"""

from __future__ import annotations

import argparse
import json
import sys

import decode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="binary observation-record file, or - for stdin")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="fail on sequence gaps or backwards acquisition time",
    )
    args = parser.parse_args()

    previous_sequence: int | None = None
    previous_time: int | None = None
    stream = decode.open_input(args.input)
    try:
        for row in decode.records(stream):
            sequence = int(row["sequence"])
            started = int(row["acquisition_started_us"])
            if previous_sequence is not None and sequence != (previous_sequence + 1) & 0xFFFFFFFF:
                message = f"sequence gap: previous={previous_sequence} current={sequence}"
                if args.strict:
                    print(message, file=sys.stderr)
                    return 2
                print(f"warning: {message}", file=sys.stderr)
            if previous_time is not None and started < previous_time:
                message = f"time moved backwards: previous={previous_time} current={started}"
                if args.strict:
                    print(message, file=sys.stderr)
                    return 3
                print(f"warning: {message}", file=sys.stderr)

            print(json.dumps(row, separators=(",", ":")))
            previous_sequence = sequence
            previous_time = started
    finally:
        if stream is not sys.stdin.buffer:
            stream.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
