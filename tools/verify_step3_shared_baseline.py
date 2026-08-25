#!/usr/bin/env python3
"""Verify that Step3 and Table 1 produced the same authenticated baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TABLE1 = ROOT / "CP_result/session02/late_table1/shared_train_only_baseline.json"


def load_verified(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    unhashed = dict(payload)
    expected = str(unhashed.pop("artifact_sha256", ""))
    actual = hashlib.sha256(json.dumps(
        unhashed, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    if not expected or actual != expected:
        raise AssertionError(f"invalid baseline SHA-256: {path}")
    if payload.get("heldout_information_used") is not False:
        raise AssertionError(f"held-out information leaked into baseline: {path}")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--step3_baseline", required=True)
    parser.add_argument("--table1_baseline", default=str(DEFAULT_TABLE1))
    args = parser.parse_args()
    step3 = load_verified(Path(args.step3_baseline))
    table1 = load_verified(Path(args.table1_baseline))
    if step3 != table1:
        raise AssertionError("Step3 and Table 1 baseline payloads differ")
    print(
        "OK: Step3 == Table 1 shared baseline; "
        f"sha256={table1['artifact_sha256']}")


if __name__ == "__main__":
    main()
