from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from argparse import Namespace
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def load_stage_module():
    path = ROOT / "scripts" / "run_real_perception_stage.py"
    spec = importlib.util.spec_from_file_location("real_perception_stage", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class RealPerceptionStageTest(unittest.TestCase):
    def setUp(self):
        self.stage = load_stage_module()
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        yy, xx = np.mgrid[:48, :64]
        self.image = np.dstack((xx * 3, yy * 4, 80 + xx)).clip(0, 255).astype(np.uint8)
        self.image_path = self.root / "rgb.png"
        cv2.imwrite(str(self.image_path), self.image)

    def tearDown(self):
        self.temp.cleanup()

    def test_inspect_mask_bundle_are_independent(self):
        inspect_out = self.root / "inspect"
        self.assertEqual(
            self.stage.command_inspect(Namespace(image=self.image_path, out=inspect_out)), 0
        )
        inspect = json.loads((inspect_out / "inspect.json").read_text())
        self.assertEqual((inspect["width"], inspect["height"]), (64, 48))

        annotation = self.root / "annotation.json"
        annotation.write_text(json.dumps({"polyline_xy": [[10, 20], [30, 22], [50, 20]]}))
        mask_out = self.root / "mask"
        self.assertEqual(
            self.stage.command_mask(
                Namespace(image=self.image_path, annotation=annotation, widths=[5, 9], out=mask_out)
            ), 0
        )

        depth_path = self.root / "depth.npy"
        depth = np.full((48, 64), 0.10, dtype=np.float32)
        depth[18:25, 8:53] = 0.099
        np.save(depth_path, depth, allow_pickle=False)
        bundle_out = self.root / "bundle"
        self.assertEqual(
            self.stage.command_bundle(
                Namespace(
                    image=self.image_path,
                    depth=depth_path,
                    mask=mask_out / "mask_w5.png",
                    camera_profile="jhu-left-rect-1300x1024",
                    k="100,0,32,0,100,24,0,0,1",
                    ring_px=3,
                    depth_scale=1.0,
                    out=bundle_out,
                )
            ), 0
        )
        bundle = json.loads((bundle_out / "bundle.json").read_text())
        self.assertTrue(bundle["complete"])
        self.assertGreater(bundle["mask_pixels"], 0)
        self.assertEqual(np.load(bundle_out / "depth_m.npy").shape, (48, 64))

    def test_bundle_rejects_shape_mismatch(self):
        depth_path = self.root / "bad_depth.npy"
        np.save(depth_path, np.ones((20, 20), dtype=np.float32), allow_pickle=False)
        mask_path = self.root / "mask.png"
        cv2.imwrite(str(mask_path), np.ones((48, 64), dtype=np.uint8) * 255)
        with self.assertRaisesRegex(ValueError, "D16-E404-SHAPE"):
            self.stage.command_bundle(
                Namespace(
                    image=self.image_path,
                    depth=depth_path,
                    mask=mask_path,
                    camera_profile="jhu-left-rect-1300x1024",
                    k=None,
                    ring_px=3,
                    depth_scale=1.0,
                    out=self.root / "should_not_exist",
                )
            )


if __name__ == "__main__":
    unittest.main()
