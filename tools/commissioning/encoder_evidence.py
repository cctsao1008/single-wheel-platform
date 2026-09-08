#!/usr/bin/env python3
"""Build reproducible encoder one-revolution evidence from raw observation captures."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Iterable

COUNTER_MODULUS = 1 << 16
COUNTER_HALF_RANGE = 1 << 15
QUALITY_AVAILABLE = 1 << 0
QUALITY_IO_OK = 1 << 1
QUALITY_IO_ERROR = 1 << 2
QUALITY_TIMING_VALID = 1 << 3
QUALITY_STALE = 1 << 6
REQUIRED_ENCODER_QUALITY = QUALITY_AVAILABLE | QUALITY_IO_OK | QUALITY_TIMING_VALID
REJECTED_ENCODER_QUALITY = QUALITY_IO_ERROR | QUALITY_STALE


class EvidenceError(ValueError):
    pass


def signed_u16_delta(previous: int, current: int) -> int:
    raw = (current - previous) & 0xFFFF
    if raw == COUNTER_HALF_RANGE:
        raise EvidenceError("ambiguous 16-bit encoder delta of exactly half the counter range")
    return raw if raw < COUNTER_HALF_RANGE else raw - COUNTER_MODULUS


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sequence_next(value: int) -> int:
    return (value + 1) & 0xFFFFFFFF


def _trial_rows(rows: list[dict], start_sequence: int, end_sequence: int) -> list[dict]:
    start_index = next((i for i, row in enumerate(rows) if row["sequence"] == start_sequence), None)
    end_index = next((i for i, row in enumerate(rows) if row["sequence"] == end_sequence), None)
    if start_index is None:
        raise EvidenceError(f"start sequence {start_sequence} is not present in the capture")
    if end_index is None:
        raise EvidenceError(f"end sequence {end_sequence} is not present in the capture")
    if end_index <= start_index:
        raise EvidenceError("trial end sequence must occur after its start sequence in the capture")
    return rows[start_index : end_index + 1]


def _analyze_trial(rows: list[dict], encoder_index: int, trial: dict) -> dict:
    direction = trial.get("direction")
    if direction not in {"positive", "negative"}:
        raise EvidenceError("trial direction must be 'positive' or 'negative'")

    selected = _trial_rows(rows, int(trial["start_sequence"]), int(trial["end_sequence"]))
    count_key = f"encoder_{encoder_index}_count"
    quality_key = f"encoder_{encoder_index}_quality"
    time_key = f"encoder_{encoder_index}_captured_at_us"

    dropped_baseline = selected[0]["dropped_records"]
    steps: list[int] = []
    for previous, current in zip(selected, selected[1:]):
        if current["sequence"] != _sequence_next(previous["sequence"]):
            raise EvidenceError(
                f"trial {trial.get('name', '<unnamed>')} contains a sequence gap: "
                f"{previous['sequence']} -> {current['sequence']}"
            )
        if current["dropped_records"] != dropped_baseline or previous["dropped_records"] != dropped_baseline:
            raise EvidenceError(f"trial {trial.get('name', '<unnamed>')} crosses a dropped-record event")
        for row in (previous, current):
            quality = int(row[quality_key])
            if quality & REQUIRED_ENCODER_QUALITY != REQUIRED_ENCODER_QUALITY:
                raise EvidenceError(
                    f"trial {trial.get('name', '<unnamed>')} has insufficient encoder quality "
                    f"0x{quality:04x}"
                )
            if quality & REJECTED_ENCODER_QUALITY:
                raise EvidenceError(
                    f"trial {trial.get('name', '<unnamed>')} has rejected encoder quality "
                    f"0x{quality:04x}"
                )
        steps.append(signed_u16_delta(int(previous[count_key]), int(current[count_key])))

    start_time = selected[0][time_key]
    end_time = selected[-1][time_key]
    if start_time is None or end_time is None:
        raise EvidenceError(f"trial {trial.get('name', '<unnamed>')} lacks encoder capture timestamps")

    net_counts = sum(steps)
    return {
        "name": trial.get("name", f"{direction}-{trial['start_sequence']}-{trial['end_sequence']}"),
        "direction": direction,
        "start_sequence": int(trial["start_sequence"]),
        "end_sequence": int(trial["end_sequence"]),
        "sample_count": len(selected),
        "start_counter": int(selected[0][count_key]),
        "end_counter": int(selected[-1][count_key]),
        "net_counts": net_counts,
        "absolute_counts": abs(net_counts),
        "observed_max_abs_step_counts": max((abs(step) for step in steps), default=0),
        "duration_us": int(end_time) - int(start_time),
        "dropped_records": int(dropped_baseline),
    }


def analyze_rows(rows: Iterable[dict], plan: dict, capture_sha256: str) -> dict:
    rows = list(rows)
    if not rows:
        raise EvidenceError("capture contains no valid raw-observation records")
    if int(plan.get("schema", 0)) != 1:
        raise EvidenceError("unsupported commissioning-plan schema")

    channel = plan.get("encoder_channel")
    if channel not in {"encoder_1", "encoder_2"}:
        raise EvidenceError("encoder_channel must be 'encoder_1' or 'encoder_2'")
    encoder_index = 1 if channel == "encoder_1" else 2

    trials = plan.get("trials")
    if not isinstance(trials, list) or not trials:
        raise EvidenceError("plan must contain at least one trial")

    analyzed = [_analyze_trial(rows, encoder_index, trial) for trial in trials]
    positive = [trial for trial in analyzed if trial["direction"] == "positive"]
    negative = [trial for trial in analyzed if trial["direction"] == "negative"]

    positive_signs = {1 if trial["net_counts"] > 0 else -1 if trial["net_counts"] < 0 else 0 for trial in positive}
    negative_signs = {1 if trial["net_counts"] > 0 else -1 if trial["net_counts"] < 0 else 0 for trial in negative}

    direction_consistent = (
        bool(positive)
        and bool(negative)
        and 0 not in positive_signs
        and 0 not in negative_signs
        and len(positive_signs) == 1
        and len(negative_signs) == 1
        and positive_signs != negative_signs
    )

    magnitudes = {trial["absolute_counts"] for trial in analyzed if trial["absolute_counts"] > 0}
    revolution_magnitude_consistent = bool(analyzed) and len(magnitudes) == 1 and 0 not in {
        trial["absolute_counts"] for trial in analyzed
    }

    positive_counter_sign = None
    if direction_consistent:
        positive_counter_sign = next(iter(positive_signs))

    counts_per_revolution_candidate = None
    if direction_consistent and revolution_magnitude_consistent:
        counts_per_revolution_candidate = next(iter(magnitudes))

    status = (
        "consistent_bidirectional_one_revolution_evidence"
        if counts_per_revolution_candidate is not None
        else "inconsistent_or_incomplete_evidence"
    )

    return {
        "schema": 1,
        "assembly": plan.get("assembly", "reference-assembly"),
        "encoder_channel": channel,
        "mechanical_coordinate": plan.get("mechanical_coordinate"),
        "positive_direction_definition": plan.get("positive_direction_definition"),
        "source": {
            "kind": "RecordedObservation raw capture",
            "sha256": capture_sha256,
            "record_kind": "raw-observation-v1",
        },
        "analysis": {
            "status": status,
            "direction_consistent": direction_consistent,
            "revolution_magnitude_consistent": revolution_magnitude_consistent,
            "counter_sign_for_mechanical_positive": positive_counter_sign,
            "counts_per_mechanical_revolution_candidate": counts_per_revolution_candidate,
            "observed_manual_run_max_abs_step_counts": max(
                trial["observed_max_abs_step_counts"] for trial in analyzed
            ),
            "note": (
                "The observed manual-run step maximum is not the runtime anti-alias bound. "
                "encoder_max_abs_delta_counts_per_sample still requires an evidenced speed envelope."
            ),
        },
        "trials": analyzed,
    }


def decoded_records(stream):
    repo_root = Path(__file__).resolve().parents[2]
    recording_dir = repo_root / "tools" / "recording"
    if str(recording_dir) not in sys.path:
        sys.path.insert(0, str(recording_dir))
    try:
        from decode import records
    except ModuleNotFoundError as exc:
        raise EvidenceError("tools/recording/decode.py is not available") from exc
    yield from records(stream)


def load_plan(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise EvidenceError("commissioning plan must be a JSON object")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze marked one-revolution encoder trials without actuating the motors."
    )
    parser.add_argument("capture", type=Path, help="binary RecordedObservation capture")
    parser.add_argument("--plan", required=True, type=Path, help="commissioning trial plan JSON")
    parser.add_argument("--output", type=Path, help="write evidence JSON here; default is stdout")
    args = parser.parse_args()

    try:
        plan = load_plan(args.plan)
        with args.capture.open("rb") as stream:
            result = analyze_rows(decoded_records(stream), plan, sha256_file(args.capture))
    except (OSError, json.JSONDecodeError, EvidenceError, KeyError, TypeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())