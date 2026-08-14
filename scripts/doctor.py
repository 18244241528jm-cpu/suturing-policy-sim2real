#!/usr/bin/env python3
"""Fail-closed environment doctor for the public simulation pipeline.

The script never starts AMBF, ROS nodes, FoundationPose, or robot motion.  It
turns the common silent setup failures into stable error codes that can be
copied into an issue or lab message.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DA_SHA256 = "fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88"
M3_SHA256 = "0407987e296d78b8b63ccf49c16e35395b00cf8d4ebc4cfe857b57f3381f2a2f"
R6_SHA256 = "6286a88c21f04abfbc4b0747a87a67bc2c5dcba17f710692c6b5138f7776e525"
FP_COMMIT = "a1b694b83e633c2cb6115b9063d940a687759392"


@dataclass
class Check:
    code: str
    status: str
    item: str
    detail: str
    fix: str = ""


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def path_from_env(name: str, default: Path | None = None) -> Path | None:
    raw = os.environ.get(name)
    if raw:
        return Path(os.path.expandvars(raw)).expanduser()
    return default


def run_text(command: list[str], timeout: int = 15) -> tuple[int, str]:
    try:
        result = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
        return result.returncode, result.stdout.strip()
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 127, str(exc)


def load_env_file(path: Path) -> None:
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
        value = os.path.expanduser(os.path.expandvars(value))
        os.environ[key] = value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        choices=("code", "reach", "fp", "full"),
        default="code",
        help="code=no simulator; reach=frozen bank; fp=GT depth + FP; full=DA + FP + reach",
    )
    parser.add_argument("--json", type=Path, help="Write the complete audit as JSON")
    parser.add_argument("--config", type=Path, help="Load a pipeline.env file before checking")
    parser.add_argument(
        "--skip-large-hash",
        action="store_true",
        help="Skip the 3.8 GB DA hash during quick debugging; never use for a formal run",
    )
    parser.add_argument(
        "--check-domain-clean",
        action="store_true",
        help="Run ros2 topic list after the caller has sourced ROS; cached daemon data can still mislead",
    )
    args = parser.parse_args()
    if args.config:
        if not args.config.is_file():
            raise SystemExit(f"D9-E03-CONFIG missing {args.config}")
        load_env_file(args.config.resolve())
    checks: list[Check] = []

    def add(code: str, ok: bool, item: str, detail: str, fix: str = "") -> None:
        checks.append(Check(code, "PASS" if ok else "FAIL", item, detail, fix))

    required_code = {
        "Approach environment": ROOT / "src/SurgicAI/RL/Approach_env.py",
        "evaluation entrypoint": ROOT / "src/SurgicAI/RL/Model_evaluation.py",
        "S3 reset capture": ROOT / "src/perception/capture_p9a_reset_bank.py",
        "S3 camera capture": ROOT / "src/perception/capture_p9a_camera_daemon.py",
        "DA inference": ROOT / "src/perception/infer_p5c_gate_da.py",
        "FP physical gate": ROOT / "src/perception/run_fp_sim_s3_live_gate.py",
        "D2 controller": ROOT / "src/control/controllers.py",
        "S3 runner": ROOT / "src/runners/run_sim_s3_live_gate.sh",
        "S4 runner": ROOT / "src/runners/run_sim_s4_deployment_proxy_reach.sh",
    }
    for label, path in required_code.items():
        add("D9-E01-CODE", path.is_file(), label, str(path), "Re-clone the public repository.")

    for module in ("numpy", "scipy", "yaml"):
        add(
            "D9-E02-PYTHON",
            importlib.util.find_spec(module) is not None,
            f"Python module {module}",
            sys.executable,
            "Activate .venv and install requirements/analysis.txt.",
        )

    if args.profile in {"reach", "fp", "full"}:
        domain = os.environ.get("ROS_DOMAIN_ID", "")
        add(
            "D9-E10-DOMAIN",
            domain.isdigit() and 1 <= int(domain) <= 232,
            "isolated ROS_DOMAIN_ID",
            domain or "unset",
            "Set an unused Fast DDS domain in 1..232, for example ROS_DOMAIN_ID=220.",
        )
        for executable in ("bash", "timeout"):
            add(
                "D9-E11-COMMAND",
                shutil.which(executable) is not None,
                executable,
                shutil.which(executable) or "not found",
                f"Install {executable} in Ubuntu/WSL2.",
            )

        ros_setup = path_from_env("SIM_S4_ROS_SETUP", Path("/opt/ros/humble/setup.bash"))
        ambf_ros_setup = path_from_env(
            "SIM_S4_AMBF_ROS_SETUP", Path.home() / "ambf_ros_ws/install/setup.bash"
        )
        for code, label, path in (
            ("D9-E12-ROS", "ROS 2 setup", ros_setup),
            ("D9-E13-AMBFROS", "AMBF ROS bridge setup", ambf_ros_setup),
        ):
            add(code, bool(path and path.is_file()), label, str(path), "Build/source the required ROS workspace.")

        ambf_root = path_from_env("AMBF_ROOT")
        src_root = path_from_env("SRC_ROOT")
        launch = path_from_env("SIM_S4_STEREO_LAUNCH") or path_from_env("SIM_S3_STEREO_LAUNCH")
        ambf_bin = ambf_root / "core/build/bin/ambf_simulator" if ambf_root else None
        needle_mesh = (
            src_root / "ADF/Phantoms/3D_MED/high_res/Needle_stage_d_v0.OBJ" if src_root else None
        )
        for code, label, path, fix in (
            ("D9-E14-AMBF", "AMBF simulator", ambf_bin, "Build AMBF 3.0 and set AMBF_ROOT."),
            ("D9-E15-SRC", "SRC needle mesh", needle_mesh, "Clone SRC and set SRC_ROOT."),
            ("D9-E16-LAUNCH", "rendered AMBF launch", launch, "Run scripts/prepare_ambf_launch.py."),
        ):
            add(code, bool(path and path.is_file()), label, str(path), fix)

        model = path_from_env("SIM_S4_MODEL", ROOT / "models/rl/m3_measured_r3_100k.zip")
        model_ok = bool(model and model.is_file())
        add("D9-E17-RL", model_ok, "M3-100k checkpoint", str(model), "Restore models/rl from Git.")
        if model_ok:
            actual = digest(model)
            add(
                "D9-E18-RLSHA",
                actual == M3_SHA256,
                "M3-100k SHA256",
                actual,
                "Do not substitute a differently trained checkpoint.",
            )

        if args.profile == "reach":
            bank = path_from_env(
                "SIM_S4_SIM_S3_ROOT", ROOT / "data/reference/sim_s3_20260811"
            )
            bank_files = (
                bank / "reset_bank.json" if bank else Path("missing"),
                bank / "da_result.json" if bank else Path("missing"),
                bank / "fp_gate/result.json" if bank else Path("missing"),
            )
            add(
                "D9-E19-BANK",
                all(path.is_file() for path in bank_files),
                "frozen SIM-S3 reference bank",
                str(bank),
                "Restore data/reference/sim_s3_20260811 or run SIM-S3 first.",
            )

        for module in ("gymnasium", "stable_baselines3"):
            add(
                "D9-E20-RLPY",
                importlib.util.find_spec(module) is not None,
                f"Python module {module}",
                sys.executable,
                "Install requirements/simulation.txt after installing the matching PyTorch build.",
            )

    if args.profile in {"fp", "full"}:
        for executable in ("docker",):
            add(
                "D9-E30-CONTAINER",
                shutil.which(executable) is not None,
                executable,
                shutil.which(executable) or "not found",
                "Install Docker plus NVIDIA Container Toolkit or WSL GPU support.",
            )
        fp_root = path_from_env("SIM_S3_FP_ROOT") or path_from_env("FOUNDATIONPOSE_ROOT")
        add("D9-E34-FP", bool(fp_root and fp_root.is_dir()), "FoundationPose repository", str(fp_root), "Clone FoundationPose and set FOUNDATIONPOSE_ROOT.")
        if fp_root and (fp_root / ".git").exists():
            rc, actual = run_text(["git", "-C", str(fp_root), "rev-parse", "HEAD"])
            add("D9-E35-FPCOMMIT", rc == 0 and actual == FP_COMMIT, "FoundationPose commit", actual, f"Checkout {FP_COMMIT}.")
        fp_image = os.environ.get("SIM_S3_FP_IMAGE", "foundationpose:blackwell")
        rc, output = run_text(["docker", "image", "inspect", fp_image]) if shutil.which("docker") else (127, "docker missing")
        add("D9-E36-FPIMAGE", rc == 0, "FoundationPose Docker image", fp_image, "Build/pull the validated image and set SIM_S3_FP_IMAGE.")
        if args.profile == "full":
            da_root = path_from_env("SIM_S3_DA_REPO") or path_from_env("DA_ROOT")
            da_checkpoint = path_from_env("SIM_S3_DA_CHECKPOINT") or path_from_env("DA_CHECKPOINT")
            add("D9-E31-DA", bool(da_root and da_root.is_dir()), "Depth Anything repository", str(da_root), "Set DA_ROOT.")
            add("D9-E32-DACKPT", bool(da_checkpoint and da_checkpoint.is_file()), "DA checkpoint", str(da_checkpoint), "Obtain the external 3.8 GB checkpoint and set DA_CHECKPOINT.")
            if da_checkpoint and da_checkpoint.is_file() and not args.skip_large_hash:
                actual = digest(da_checkpoint)
                add("D9-E33-DASHA", actual == DA_SHA256, "DA checkpoint SHA256", actual, "Use the validated ViT-L FP32 checkpoint.")
            elif args.skip_large_hash:
                checks.append(Check("D9-W33-DASHA", "WARN", "DA checkpoint SHA256", "skipped by request", "Run again without --skip-large-hash before formal data collection."))

    if args.check_domain_clean and args.profile in {"reach", "fp", "full"}:
        if shutil.which("ros2"):
            rc, output = run_text(["ros2", "topic", "list"], timeout=10)
            live = [line for line in output.splitlines() if line not in {"/parameter_events", "/rosout", ""}]
            add("D9-E40-DOMAINLIVE", rc == 0 and not live, "live ROS-domain topic check", ", ".join(live) or "clean", "Choose another domain or stop only the process you started.")
        else:
            add("D9-E40-DOMAINLIVE", False, "live ROS-domain topic check", "ros2 not in PATH", "Source ROS before using --check-domain-clean.")

    failures = [check for check in checks if check.status == "FAIL"]
    report = {
        "schema": "SurgicAI.public_simulation_doctor.v1",
        "profile": args.profile,
        "passed": not failures,
        "failure_count": len(failures),
        "checks": [asdict(check) for check in checks],
    }
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    for check in checks:
        marker = {"PASS": "OK", "FAIL": "FAIL", "WARN": "WARN"}[check.status]
        print(f"[{marker}] {check.code} {check.item}: {check.detail}")
        if check.status != "PASS" and check.fix:
            print(f"       FIX: {check.fix}")
    print("DOCTOR_PASS" if not failures else f"DOCTOR_FAIL count={len(failures)}")
    return 0 if not failures else 2


if __name__ == "__main__":
    raise SystemExit(main())
