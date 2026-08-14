#!/usr/bin/env python3
"""Summarize a public pipeline run and reject incomplete artifacts."""

from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path


def read_json(path: Path) -> dict:
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_root", type=Path)
    args = parser.parse_args()
    root = args.run_root.expanduser().resolve()
    failures: list[str] = []

    status_path = root / "pipeline_status.json"
    if status_path.is_file():
        status = read_json(status_path)
        print(f"pipeline complete={status.get('complete')} profile={status.get('profile')} stage={status.get('stage_request')}")
        for stage in status.get("stages", []):
            print(f"  {stage.get('name')}: {stage.get('status')} rc={stage.get('return_code')} wall={stage.get('wall_s')}s")
        if not status.get("complete"):
            failures.append("pipeline_status.json says complete=false")
    else:
        print("pipeline_status.json: not present (direct runner invocation)")

    s3 = root / "sim_s3"
    if s3.exists():
        required = [s3 / "reset_bank.json", s3 / "da_result.json", s3 / "fp_gate/result.json"]
        missing = [str(path) for path in required if not path.is_file()]
        if missing:
            failures.extend("missing " + path for path in missing)
        else:
            reset = read_json(required[0])
            fp = read_json(required[2])
            reset_count = len(reset.get("entries", reset.get("episodes", reset.get("records", []))))
            fp_rows = fp.get("rows", fp.get("records", fp.get("per_condition", [])))
            print(f"SIM-S3 reset entries={reset_count or 'schema-specific'} FP rows={len(fp_rows) if isinstance(fp_rows, list) else 'schema-specific'}")

    s4 = root / "sim_s4"
    if s4.exists():
        labels = sorted(path.name for path in s4.iterdir() if path.is_dir() and (path.name.startswith("A_GT_frozen_") or path.name.startswith("B_deployment_proxy_")))
        for label in labels:
            cell = s4 / label
            status = cell / "status.txt"
            result = cell / "result.json"
            if not result.is_file() and (cell / "result.json.gz").is_file():
                result = cell / "result.json.gz"
            if not status.is_file() or not result.is_file():
                failures.append(f"{label}: missing status.txt or result.json")
                continue
            payload = read_json(result)
            rows = payload.get("episode_records", [])
            successes = sum(bool(row.get("success", row.get("is_success", False))) for row in rows)
            print(f"{label}: episodes={len(rows)} success_field_count={successes}; use the dated analysis contract for publication numbers")

    if failures:
        print("RESULT_AUDIT_FAIL")
        for failure in failures:
            print("  - " + failure)
        return 2
    print("RESULT_AUDIT_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
