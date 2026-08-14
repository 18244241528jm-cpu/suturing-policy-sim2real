from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


class OnboardingToolsTest(unittest.TestCase):
    def test_code_doctor_passes(self) -> None:
        result = subprocess.run(
            [sys.executable, str(ROOT / "scripts/doctor.py"), "--profile", "code"],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("DOCTOR_PASS", result.stdout)

    def test_code_wrapper_dry_run_needs_no_host_config(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts/run_simulation.py"),
                "--stage",
                "code",
                "--dry-run",
                "--config",
                str(ROOT / "configs/does_not_exist.env"),
            ],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("PIPELINE_COMPLETE", result.stdout)

    def test_gt_depth_adapter_labels_privilege(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            capture = root / "capture"
            source = capture / "depth_gt_m"
            source.mkdir(parents=True)
            np.save(source / "frame_000000.npy", np.ones((3, 4), dtype=np.float32))
            output = root / "prediction"
            report = root / "result.json"
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "src/perception/use_gt_depth.py"),
                    "--capture-dir",
                    str(capture),
                    "--prediction-dir",
                    str(output),
                    "--out",
                    str(report),
                ],
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout)
            payload = json.loads(report.read_text(encoding="utf-8"))
            self.assertEqual(payload["source"], "AMBF_GT_PRIVILEGED")
            self.assertTrue((output / "frame_000000.npy").is_file())


if __name__ == "__main__":
    unittest.main()
