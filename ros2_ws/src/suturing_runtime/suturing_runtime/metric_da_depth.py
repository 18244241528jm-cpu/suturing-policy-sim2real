"""Run the validated P5a metric Depth Anything checkpoint on a latched RGB frame.

This node owns R2.  It deliberately publishes to the *external* depth boundary,
so perception_input_adapter still checks the source-frame contract independently.
"""

from __future__ import annotations

import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .message_contract import stamp_ns


EXPECTED_CHECKPOINT_SHA256 = "fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88"
MODEL_CONFIGS = {
    "vits": {"encoder": "vits", "features": 64, "out_channels": [48, 96, 192, 384]},
    "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768]},
    "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
    "vitg": {"encoder": "vitg", "features": 384, "out_channels": [1536, 1536, 1536, 1536]},
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class MetricDADepth(Node):
    def __init__(self) -> None:
        super().__init__("metric_da_depth")
        self.declare_parameter("enabled", False)
        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("depth_anything_repo", "")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("expected_checkpoint_sha256", EXPECTED_CHECKPOINT_SHA256)
        self.declare_parameter("input_topic", "/suturing/initialization/left/image")
        self.declare_parameter("output_topic", "/suturing/external/depth")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(Image, self.get_parameter("output_topic").value, latched)
        self.status = self.create_publisher(String, "/suturing/da/status", latched)
        self.model = self.torch = self.cv2 = None
        self.device = str(self.get_parameter("device").value)
        self.input_size = None
        self.checkpoint_sha = None
        self.last_stamp_ns = None
        if not bool(self.get_parameter("enabled").value):
            self._status("DISABLED", code="D13-E201-DA_DISABLED")
            self.get_logger().warning("D13_R2_DISABLED set metric_da_depth.enabled=true after configuring assets")
            return
        self._load_model()
        self.create_subscription(Image, self.get_parameter("input_topic").value,
                                 self._image, latched)
        self._status("READY", checkpoint_sha256=self.checkpoint_sha,
                     input_size=self.input_size, device=self.device)
        self.get_logger().info("D13_R2_READY exact P5a ViT-L/518/FP32 model loaded once")

    def _status(self, state: str, **fields) -> None:
        msg = String(); msg.data = json.dumps({"schema": "suturing.da_status.v1",
            "state": state, **fields}, sort_keys=True); self.status.publish(msg)

    def _load_model(self) -> None:
        checkpoint = Path(str(self.get_parameter("checkpoint_path").value)).expanduser()
        repository = Path(str(self.get_parameter("depth_anything_repo").value)).expanduser()
        if not checkpoint.is_file():
            raise FileNotFoundError(f"D13-E202-CHECKPOINT_NOT_FOUND {checkpoint}")
        if not (repository / "depth_anything_v2" / "dpt.py").is_file():
            raise FileNotFoundError(f"D13-E203-DA_REPO_INVALID {repository}")
        expected = str(self.get_parameter("expected_checkpoint_sha256").value).lower()
        actual = sha256(checkpoint)
        if actual != expected or actual != EXPECTED_CHECKPOINT_SHA256:
            raise RuntimeError(f"D13-E204-CHECKPOINT_SHA expected={expected} actual={actual}")
        started = time.perf_counter()
        import cv2
        import torch
        import torch.nn as nn
        import torch.nn.functional as functional

        sys.path.insert(0, str(repository.resolve()))
        from depth_anything_v2.dpt import DepthAnythingV2

        class MetricDepthModel(nn.Module):
            def __init__(self, backbone, max_depth):
                super().__init__(); self.backbone = backbone
                self.log_scale = nn.Parameter(torch.tensor(-2.0))
                self.shift_raw = nn.Parameter(torch.tensor(-2.0))
                self.bias_raw = nn.Parameter(torch.tensor(-10.0))
                self.max_depth = float(max_depth)

            def forward(self, image):
                relative = self.backbone(image)
                if relative.ndim == 3: relative = relative.unsqueeze(1)
                relative = functional.relu(relative)
                scale = torch.exp(torch.clamp(self.log_scale, -12.0, 12.0))
                shift = functional.softplus(self.shift_raw) + 1.0e-4
                bias = functional.softplus(self.bias_raw)
                depth = scale / (relative + shift) + bias
                return torch.clamp(depth, min=1.0e-6, max=self.max_depth)

        payload = torch.load(checkpoint, map_location="cpu", weights_only=False, mmap=True)
        if "args" not in payload or "model" not in payload:
            raise RuntimeError("D13-E205-CHECKPOINT_SCHEMA requires args and model")
        saved = payload["args"]
        contract = (str(saved.get("encoder")), int(saved.get("input_size", -1)),
                    str(saved.get("precision")))
        if contract != ("vitl", 518, "fp32"):
            raise RuntimeError(f"D13-E206-MODEL_CONTRACT got={contract}")
        max_depth = float(saved["max_depth"])
        try:
            backbone = DepthAnythingV2(**MODEL_CONFIGS["vitl"], max_depth=max_depth)
        except TypeError:
            backbone = DepthAnythingV2(**MODEL_CONFIGS["vitl"])
        model = MetricDepthModel(backbone, max_depth)
        incompatible = model.load_state_dict(payload["model"], strict=True)
        if incompatible.missing_keys or incompatible.unexpected_keys:
            raise RuntimeError(f"D13-E207-STATE_DICT {incompatible}")
        self.model = model.to(self.device).eval()
        self.torch, self.cv2 = torch, cv2
        self.input_size = 518; self.checkpoint_sha = actual
        self._status("MODEL_LOADED", load_seconds=time.perf_counter()-started,
                     checkpoint_sha256=actual, device=self.device)

    @staticmethod
    def _decode_color(msg: Image) -> np.ndarray:
        if msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"D13-E208-RGB_ENCODING {msg.encoding!r}")
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(int(msg.height), int(msg.step))
        image = row[:, :int(msg.width)*3].reshape(int(msg.height), int(msg.width), 3).copy()
        return image[..., ::-1].copy() if msg.encoding == "rgb8" else image

    def _preprocess(self, bgr: np.ndarray):
        rgb = self.cv2.cvtColor(bgr, self.cv2.COLOR_BGR2RGB)
        square = self.cv2.resize(rgb, (518, 518), interpolation=self.cv2.INTER_LINEAR)
        image = square.astype(np.float32) / 255.0
        image = (image - np.asarray([0.485,0.456,0.406], np.float32)) / \
                np.asarray([0.229,0.224,0.225], np.float32)
        return self.torch.from_numpy(image.transpose(2,0,1)).unsqueeze(0)

    def _image(self, msg: Image) -> None:
        source_stamp = stamp_ns(msg.header)
        if source_stamp == self.last_stamp_ns: return
        started = time.perf_counter()
        try:
            bgr = self._decode_color(msg)
            tensor = self._preprocess(bgr).to(self.device)
            if self.device.startswith("cuda"): self.torch.cuda.synchronize()
            inference_started = time.perf_counter()
            with self.torch.inference_mode():
                square = self.model(tensor)[0,0].float().cpu().numpy()
            if self.device.startswith("cuda"): self.torch.cuda.synchronize()
            inference_s = time.perf_counter() - inference_started
            depth = self.cv2.resize(square, (int(msg.width),int(msg.height)),
                                    interpolation=self.cv2.INTER_LINEAR).astype("<f4")
            if not np.isfinite(depth).all() or np.any(depth <= 0.0):
                raise ValueError("D13-E209-DEPTH_NONFINITE_OR_NONPOSITIVE")
            out = Image(); out.header = msg.header; out.height = msg.height; out.width = msg.width
            out.encoding = "32FC1"; out.is_bigendian = 0; out.step = int(msg.width)*4
            out.data = depth.tobytes(order="C")
            self.output.publish(out); self.last_stamp_ns = source_stamp
            self._status("PUBLISHED", source_stamp_ns=source_stamp,
                         inference_seconds=inference_s,
                         total_seconds=time.perf_counter()-started,
                         minimum_depth_m=float(depth.min()), maximum_depth_m=float(depth.max()),
                         checkpoint_sha256=self.checkpoint_sha)
        except Exception as exc:
            self._status("FAILED", code="D13-E210-INFERENCE", source_stamp_ns=source_stamp,
                         detail=str(exc)); self.get_logger().error(f"D13-E210-INFERENCE {exc}")


def main() -> None:
    rclpy.init(); node = MetricDADepth()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
