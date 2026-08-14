#!/usr/bin/env python3
"""Create a private P5c evaluator copy with per-policy-step pose-age audit.

The canonical and WSL mirror Model_evaluation.py remain byte-identical.  This
script copies the mirror file into a task-private runtime directory and applies
four exact anchor substitutions.  Anchor counts are checked before writing so
an upstream edit cannot silently produce a partly patched evaluator.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one anchor, found {count}")
    return text.replace(old, new, 1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    source = args.source.read_text(encoding="utf-8")
    patched = replace_once(
        source,
        "        live_fp_records = []\n        live_fp_sequence = -1\n",
        "        live_fp_records = []\n"
        "        live_fp_sequence = -1\n"
        "        live_fp_step_age_ms = []\n"
        "        live_fp_last_capture_time_ns = None\n",
        "episode audit initialization",
    )
    patched = replace_once(
        patched,
        "            live_fp_sequence = int(live_payload[\"sequence\"])\n"
        "            live_fp_records.append(\n"
        "                {**live_payload, \"goal_age_ms\": live_age_ms}\n"
        "            )\n"
        "        fixed_episode_goal = getattr(env.unwrapped, \"fixed_episode_goal\", None)\n",
        "            live_fp_sequence = int(live_payload[\"sequence\"])\n"
        "            live_fp_last_capture_time_ns = int(live_payload[\"capture_time_ns\"])\n"
        "            live_fp_records.append(\n"
        "                {**live_payload, \"goal_age_ms\": live_age_ms}\n"
        "            )\n"
        "        fixed_episode_goal = getattr(env.unwrapped, \"fixed_episode_goal\", None)\n",
        "initial live pose timestamp",
    )
    patched = replace_once(
        patched,
        "                live_fp_sequence = int(live_payload[\"sequence\"])\n"
        "                live_fp_records.append(\n"
        "                    {**live_payload, \"goal_age_ms\": live_age_ms}\n"
        "                )\n"
        "            servo_diagnostic = None\n",
        "                live_fp_sequence = int(live_payload[\"sequence\"])\n"
        "                live_fp_last_capture_time_ns = int(live_payload[\"capture_time_ns\"])\n"
        "                live_fp_records.append(\n"
        "                    {**live_payload, \"goal_age_ms\": live_age_ms}\n"
        "                )\n"
        "            if live_fp_last_capture_time_ns is not None:\n"
        "                live_fp_step_age_ms.append(\n"
        "                    (time.time_ns() - live_fp_last_capture_time_ns) / 1.0e6\n"
        "                )\n"
        "            servo_diagnostic = None\n",
        "per-step age sample",
    )
    patched = replace_once(
        patched,
        "                \"live_fp_age_ms_p95\": float(np.percentile(ages, 95)) if ages else None,\n"
        "                \"live_fp_track_rotation_error_deg_p95\": (\n",
        "                \"live_fp_age_ms_p95\": float(np.percentile(ages, 95)) if ages else None,\n"
        "                \"live_fp_step_age_sample_count\": len(live_fp_step_age_ms),\n"
        "                \"live_fp_step_age_ms\": live_fp_step_age_ms,\n"
        "                \"live_fp_step_age_ms_p50\": (\n"
        "                    float(np.percentile(live_fp_step_age_ms, 50))\n"
        "                    if live_fp_step_age_ms else None\n"
        "                ),\n"
        "                \"live_fp_step_age_ms_p95\": (\n"
        "                    float(np.percentile(live_fp_step_age_ms, 95))\n"
        "                    if live_fp_step_age_ms else None\n"
        "                ),\n"
        "                \"live_fp_step_age_ms_max\": (\n"
        "                    float(np.max(live_fp_step_age_ms))\n"
        "                    if live_fp_step_age_ms else None\n"
        "                ),\n"
        "                \"live_fp_track_rotation_error_deg_p95\": (\n",
        "episode summary",
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(patched, encoding="utf-8")
    print(f"P5C_PRIVATE_EVAL_READY source_sha={sha256(args.source)} out_sha={sha256(args.out)} out={args.out}")


if __name__ == "__main__":
    main()
