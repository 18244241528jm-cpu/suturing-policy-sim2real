#!/usr/bin/env python3
"""One front door for code checks, SIM-S3 perception, and SIM-S4 Reach.

Run this inside Ubuntu/WSL2.  The wrapper does not hide the research runners;
it records exactly which one failed and keeps every stage in a separate result
directory.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_env(path: Path, base: dict[str, str]) -> dict[str, str]:
    env = dict(base)
    for number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].strip()
        if "=" not in line:
            raise SystemExit(f"D9-E03-CONFIG {path}:{number}: expected KEY=VALUE")
        key, value = line.split("=", 1)
        key, value = key.strip(), value.strip().strip("'\"")
        if not key.replace("_", "").isalnum():
            raise SystemExit(f"D9-E03-CONFIG {path}:{number}: invalid key {key!r}")
        value = os.path.expanduser(os.path.expandvars(value))
        env[key] = value
        os.environ[key] = value
    return env


def write_status(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def run(command: list[str], env: dict[str, str], cwd: Path, dry: bool) -> int:
    print("+ " + " ".join(command), flush=True)
    if dry:
        return 0
    return subprocess.run(command, cwd=cwd, env=env, check=False).returncode


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/pipeline.env")
    parser.add_argument("--stage", choices=("code", "s3", "s4", "full"), default="code")
    parser.add_argument("--profile", choices=("smoke", "formal"), default="smoke")
    parser.add_argument("--run-root", type=Path)
    parser.add_argument("--s3-root", type=Path, help="Existing SIM-S3 root when --stage s4")
    parser.add_argument("--depth", choices=("da", "gt"), default="da", help="Depth source for SIM-S3; gt is an AMBF privilege A/B")
    parser.add_argument("--goal", choices=("both", "gt", "fp"), default="both", help="Goal source(s) for SIM-S4")
    parser.add_argument("--controller", choices=("d2", "rl"), default="d2", help="D2 goal servo or learned policy")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    env = os.environ.copy()
    if args.config.is_file():
        env = load_env(args.config.resolve(), env)
    elif args.stage != "code":
        raise SystemExit(
            f"D9-E03-CONFIG missing {args.config}; copy configs/pipeline.env.example first"
        )
    profile_path = ROOT / f"configs/profiles/{args.profile}.env"
    env = load_env(profile_path, env)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    result_base = Path(env.get("SURGICAI_RESULT_ROOT", str(Path.home() / "surgicai_runs")))
    run_root = (args.run_root or result_base / f"pipeline_{args.profile}_{timestamp}").expanduser().resolve()
    status_path = run_root / "pipeline_status.json"

    if args.profile == "smoke":
        env["SIM_S3_EPISODES"] = "2"
        env["SIM_S4_EPISODES"] = "2"
    else:
        env["SIM_S3_EPISODES"] = "40"
        env["SIM_S4_EPISODES"] = "30"

    s3_root = run_root / "sim_s3"
    s4_root = run_root / "sim_s4"
    env["SIM_S3_PROJECT_MNT"] = str(ROOT)
    env["SIM_S4_PROJECT_MNT"] = str(ROOT)
    env["SIM_S3_REPO"] = str(ROOT / "src/SurgicAI")
    env["SIM_S4_REPO"] = str(ROOT / "src/SurgicAI")
    env["SIM_S3_PERCEPTION_ROOT"] = str(ROOT / "src/perception")
    env["SIM_S3_RESULT_ROOT"] = str(s3_root)
    env["SIM_S3_DEPTH_SOURCE"] = args.depth
    env["SIM_S4_CONTROLLER"] = "goal-servo" if args.controller == "d2" else "policy"
    env["SIM_S4_RESULT_ROOT"] = str(s4_root)
    env["SIM_S4_SIM_S3_ROOT"] = str(
        args.s3_root.expanduser().resolve()
        if args.s3_root
        else (s3_root if args.stage in {"s3", "full"} else ROOT / "data/reference/sim_s3_20260811")
    )
    env.setdefault("SIM_S4_MODEL", str(ROOT / "models/rl/m3_measured_r3_100k.zip"))

    stages: list[dict] = []
    state = {
        "schema": "SurgicAI.public_simulation_run.v1",
        "stage_request": args.stage,
        "profile": args.profile,
        "run_root": str(run_root),
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "complete": False,
        "stages": stages,
    }

    def execute(name: str, error_code: str, command: list[str]) -> bool:
        started = time.monotonic()
        record = {"name": name, "error_code": error_code, "command": command, "status": "running"}
        stages.append(record)
        if not args.dry_run:
            write_status(status_path, state)
        rc = run(command, env, ROOT, args.dry_run)
        record.update(return_code=rc, wall_s=round(time.monotonic() - started, 3))
        record["status"] = "passed" if rc == 0 else "failed"
        if not args.dry_run:
            write_status(status_path, state)
        if rc != 0:
            print(f"{error_code}: {name} failed with exit code {rc}", file=sys.stderr)
            print(f"Read {status_path} and the newest log under {run_root}", file=sys.stderr)
            return False
        return True

    if args.stage == "code":
        commands = [
            [sys.executable, "scripts/doctor.py", "--profile", "code"],
            [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-v"],
            [sys.executable, "src/handeye/synthetic_self_test.py", "--output-root", str(run_root / "handeye_selftest")],
        ]
        for index, command in enumerate(commands, 1):
            if not execute(f"code_check_{index}", "D9-E05-CODECHECK", command):
                return 5
    else:
        doctor_profile = ("full" if args.depth == "da" else "fp") if args.stage in {"s3", "full"} else "reach"
        doctor_json = run_root / "doctor.json"
        doctor = [sys.executable, "scripts/doctor.py", "--profile", doctor_profile, "--json", str(doctor_json), "--check-domain-clean"]
        if not execute("preflight", "D9-E10-PREFLIGHT", doctor):
            return 10
        if args.stage in {"s3", "full"}:
            if not execute("sim_s3_perception", "D9-E60-S3", ["bash", "src/runners/run_sim_s3_live_gate.sh"]):
                return 60
        if args.stage in {"s4", "full"}:
            mode = {"both": "all", "gt": "gt-only", "fp": "fp-only"}[args.goal]
            if not execute("sim_s4_reach", "D9-E90-S4", ["bash", "src/runners/run_sim_s4_deployment_proxy_reach.sh", mode]):
                return 90

    state["complete"] = True
    state["finished_utc"] = datetime.now(timezone.utc).isoformat()
    if not args.dry_run:
        write_status(status_path, state)
    print(f"PIPELINE_COMPLETE stage={args.stage} profile={args.profile} root={run_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
