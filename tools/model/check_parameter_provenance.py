#!/usr/bin/env python3
"""Validate the accepted parameter registry against its provenance classification."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
REGISTRY_PATH = ROOT / "parameters" / "reference-assembly.json"
EVIDENCE_PATH = ROOT / "parameters" / "reference-assembly-evidence.json"

ALLOWED_EVIDENCE = {"unknown", "measured", "identified", "datasheet", "derived"}


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def registry_leaves(node: object, prefix: str = "") -> dict[str, dict]:
    leaves: dict[str, dict] = {}
    if isinstance(node, dict) and {"value", "evidence"} <= node.keys():
        leaves[prefix] = node
        return leaves

    if isinstance(node, dict):
        for key, value in node.items():
            if key in {"schema", "assembly"} and not prefix:
                continue
            child = f"{prefix}.{key}" if prefix else key
            leaves.update(registry_leaves(value, child))
    return leaves


def main() -> None:
    registry = load_json(REGISTRY_PATH)
    evidence = load_json(EVIDENCE_PATH)

    if evidence.get("accepted_registry") != "parameters/reference-assembly.json":
        raise SystemExit("provenance file does not name the canonical accepted registry")

    leaves = registry_leaves(registry)
    if not leaves:
        raise SystemExit("no parameter leaves found in accepted registry")

    expected = {"known": set(), "derived": set(), "unknown": set()}

    for path, leaf in leaves.items():
        evidence_class = leaf["evidence"]
        value = leaf["value"]

        if evidence_class not in ALLOWED_EVIDENCE:
            raise SystemExit(f"{path}: invalid evidence class {evidence_class!r}")

        if value is None:
            if evidence_class != "unknown":
                raise SystemExit(
                    f"{path}: null value must remain evidence='unknown', got {evidence_class!r}"
                )
            expected["unknown"].add(path)
        else:
            if evidence_class == "unknown":
                raise SystemExit(f"{path}: numeric value cannot use evidence='unknown'")
            bucket = "derived" if evidence_class == "derived" else "known"
            expected[bucket].add(path)

    classification = evidence.get("classification", {})
    actual: dict[str, set[str]] = {}
    for bucket in ("known", "derived", "unknown"):
        values = classification.get(bucket)
        if not isinstance(values, list):
            raise SystemExit(f"classification.{bucket} must be a list")
        if len(values) != len(set(values)):
            raise SystemExit(f"classification.{bucket} contains duplicate parameter paths")
        actual[bucket] = set(values)

    all_actual = actual["known"] | actual["derived"] | actual["unknown"]
    if len(all_actual) != sum(len(values) for values in actual.values()):
        raise SystemExit("a parameter path appears in more than one provenance classification")

    unknown_paths = all_actual - set(leaves)
    if unknown_paths:
        raise SystemExit(
            "provenance classification contains unknown registry paths: "
            + ", ".join(sorted(unknown_paths))
        )

    for bucket in ("known", "derived", "unknown"):
        if actual[bucket] != expected[bucket]:
            missing = sorted(expected[bucket] - actual[bucket])
            extra = sorted(actual[bucket] - expected[bucket])
            raise SystemExit(
                f"classification.{bucket} does not match accepted registry; "
                f"missing={missing}, extra={extra}"
            )

    for index, claim in enumerate(evidence.get("candidate_claims", [])):
        targets = claim.get("targets")
        if not isinstance(targets, list) or not targets:
            raise SystemExit(f"candidate_claims[{index}].targets must be a non-empty list")
        invalid_targets = sorted(set(targets) - set(leaves))
        if invalid_targets:
            raise SystemExit(
                f"candidate_claims[{index}] targets unknown registry paths: {invalid_targets}"
            )

    print(
        "parameter provenance OK: "
        f"{len(expected['known'])} known, "
        f"{len(expected['derived'])} derived, "
        f"{len(expected['unknown'])} unknown"
    )


if __name__ == "__main__":
    main()
