#!/usr/bin/env python3
"""Re-render the frozen A3 pose commands as synchronized stereo observations."""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from ambf_msgs.msg import CameraState, RigidBodyCmd, RigidBodyState
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2


HEIGHT, WIDTH = 480, 640
K = np.array(
    [[358.807027298745, 0, 320], [0, 358.807027298745, 240], [0, 0, 1]],
    dtype=np.float64,
)
NEEDLE_RGB = np.array([143, 143, 143], dtype=np.int16)


def stamp_ns(msg: Any) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def pose_matrix(msg: Any) -> np.ndarray:
    p, q = msg.pose.position, msg.pose.orientation
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [p.x, p.y, p.z]
    return out


def pose_command(matrix: np.ndarray) -> RigidBodyCmd:
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    cmd = RigidBodyCmd()
    cmd.cartesian_cmd_type = RigidBodyCmd.TYPE_POSITION
    cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z = map(
        float, matrix[:3, 3]
    )
    (
        cmd.pose.orientation.x,
        cmd.pose.orientation.y,
        cmd.pose.orientation.z,
        cmd.pose.orientation.w,
    ) = map(float, q)
    return cmd


def rgb_image(msg: Image) -> np.ndarray:
    channels = max(1, int(msg.step) // int(msg.width))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
        int(msg.height), int(msg.width), channels
    )[..., :3]
    if msg.encoding.lower() == "bgr8":
        raw = raw[..., ::-1]
    return np.ascontiguousarray(raw)


def depth_image(msg: PointCloud2) -> np.ndarray:
    x = point_cloud2.read_points_numpy(
        msg, field_names=["x"], skip_nans=False
    ).reshape(-1)
    if x.size != HEIGHT * WIDTH:
        raise ValueError(f"Expected {HEIGHT}x{WIDTH}, received {x.size} points")
    return np.flipud((-x).reshape(HEIGHT, WIDTH)).astype(np.float32)


def needle_mask(segmentation: np.ndarray) -> np.ndarray:
    delta = np.abs(segmentation.astype(np.int16) - NEEDLE_RGB[None, None, :])
    return (np.max(delta, axis=2) <= 1).astype(np.uint8) * 255


class StereoNode(Node):
    def __init__(self) -> None:
        super().__init__("stage_c2_stereo_capture")
        self.messages: dict[str, Any] = {}
        self.serials: dict[str, int] = {}
        topics = {
            "Lrgb": ("/ambf/env/cameras/cameraL/ImageData", Image),
            "Lseg": ("/ambf/env/cameras/cameraL2/ImageData", Image),
            "Ldepth": ("/ambf/env/cameras/cameraL/DepthData", PointCloud2),
            "Rrgb": ("/ambf/env/cameras/cameraR/ImageData", Image),
            "Rseg": ("/ambf/env/cameras/cameraR2/ImageData", Image),
            "Rdepth": ("/ambf/env/cameras/cameraR/DepthData", PointCloud2),
            "needle": ("/ambf/env/phantom/Needle/State", RigidBodyState),
            "phantom": ("/ambf/env/phantom/phantom/State", RigidBodyState),
            "camera_frame": (
                "/ambf/env/phantom/CameraFrame/State",
                RigidBodyState,
            ),
            "Lcamera": ("/ambf/env/cameras/cameraL/State", CameraState),
            "Rcamera": ("/ambf/env/cameras/cameraR/State", CameraState),
        }
        for name, (topic, message_type) in topics.items():
            self.create_subscription(
                message_type,
                topic,
                lambda message, name=name: self.receive(name, message),
                1,
            )
        self.needle_pub = self.create_publisher(
            RigidBodyCmd, "/ambf/env/phantom/Needle/Command", 20
        )
        self.psm2_pub = self.create_publisher(
            RigidBodyCmd, "/ambf/env/psm2/tool_id_420006/Command", 20
        )

    def receive(self, name: str, message: Any) -> None:
        self.messages[name] = message
        self.serials[name] = self.serials.get(name, 0) + 1

    def wait_ready(self, timeout_s: float) -> None:
        required = {
            "Lrgb", "Lseg", "Ldepth", "Rrgb", "Rseg", "Rdepth",
            "needle", "phantom", "camera_frame", "Lcamera", "Rcamera",
        }
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not required.issubset(self.messages):
            rclpy.spin_once(self, timeout_sec=0.05)
        missing = required.difference(self.messages)
        if missing:
            raise RuntimeError(f"Missing topics: {sorted(missing)}")

    def hold(self, needle: np.ndarray, psm2: np.ndarray, seconds: float) -> None:
        needle_cmd, psm2_cmd = pose_command(needle), pose_command(psm2)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.needle_pub.publish(needle_cmd)
            self.psm2_pub.publish(psm2_cmd)
            rclpy.spin_once(self, timeout_sec=0.01)

    def fresh_six(
        self,
        prior_serials: dict[str, int],
        prior_stamps: dict[str, int],
        timeout_s: float,
        max_sync_ms: float,
    ) -> dict[str, int]:
        names = ("Lrgb", "Lseg", "Ldepth", "Rrgb", "Rseg", "Rdepth")
        deadline = time.monotonic() + timeout_s
        latest: dict[str, int] = {}
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if not all(name in self.messages for name in names):
                continue
            latest = {name: stamp_ns(self.messages[name]) for name in names}
            serial_fresh = all(
                self.serials.get(name, 0) > prior_serials.get(name, -1)
                for name in names
            )
            stamp_fresh = all(
                latest[name] != prior_stamps.get(name, -1) for name in names
            )
            spread_ms = (max(latest.values()) - min(latest.values())) / 1e6
            if serial_fresh and stamp_fresh and spread_ms <= max_sync_ms:
                return latest
        raise RuntimeError(
            f"No synchronized six-stream bundle; latest={latest}, "
            f"max_sync_ms={max_sync_ms}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-a3", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--settle-s", type=float, default=0.20)
    parser.add_argument("--warmup-s", type=float, default=3.0)
    parser.add_argument("--ready-timeout-s", type=float, default=30.0)
    parser.add_argument("--fresh-timeout-s", type=float, default=8.0)
    parser.add_argument("--max-sync-error-ms", type=float, default=5.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.out.exists() and any(args.out.iterdir()):
        raise SystemExit(f"Output directory must be empty: {args.out}")
    for side in ("L", "R"):
        for folder in ("rgb", "depth_gt_m", "needle_mask", "segmentation_raw"):
            (args.out / side / folder).mkdir(parents=True, exist_ok=True)
    (args.out / "poses").mkdir(parents=True, exist_ok=True)

    source_ids = sorted(
        path.stem for path in (args.source_a3 / "poses").glob("*.npz")
    )
    if len(source_ids) != 100:
        raise ValueError(f"Expected 100 A3 commands, got {len(source_ids)}")

    rclpy.init()
    node = StereoNode()
    rows = []
    try:
        node.wait_ready(args.ready_timeout_s)
        first = np.load(args.source_a3 / "poses" / f"{source_ids[0]}.npz")
        node.hold(
            first["commanded_T_Wneedle"],
            first["commanded_T_Wpsm2"],
            args.warmup_s,
        )
        prior_serials = dict(node.serials)
        prior_stamps = {
            name: stamp_ns(node.messages[name])
            for name in ("Lrgb", "Lseg", "Ldepth", "Rrgb", "Rseg", "Rdepth")
        }
        for index, frame_id in enumerate(source_ids):
            source = np.load(args.source_a3 / "poses" / f"{frame_id}.npz")
            node.hold(
                source["commanded_T_Wneedle"],
                source["commanded_T_Wpsm2"],
                args.settle_s,
            )
            stamps = node.fresh_six(
                prior_serials,
                prior_stamps,
                args.fresh_timeout_s,
                args.max_sync_error_ms,
            )
            prior_serials, prior_stamps = dict(node.serials), dict(stamps)
            T_Wframe = pose_matrix(node.messages["camera_frame"])
            T_Wneedle = pose_matrix(node.messages["needle"])
            T_Wphantom = pose_matrix(node.messages["phantom"])
            side_poses = {}
            mask_counts = {}
            for side in ("L", "R"):
                rgb = rgb_image(node.messages[f"{side}rgb"])
                seg = rgb_image(node.messages[f"{side}seg"])
                depth = depth_image(node.messages[f"{side}depth"])
                mask = needle_mask(seg)
                T_Wcamera = T_Wframe @ pose_matrix(node.messages[f"{side}camera"])
                side_poses[f"{side}T_Wcamera"] = T_Wcamera
                mask_counts[f"{side}_mask_pixels"] = int((mask > 0).sum())
                cv2.imwrite(
                    str(args.out / side / "rgb" / f"{frame_id}.png"),
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                )
                np.save(args.out / side / "depth_gt_m" / f"{frame_id}.npy", depth)
                cv2.imwrite(
                    str(args.out / side / "needle_mask" / f"{frame_id}.png"),
                    mask,
                )
                cv2.imwrite(
                    str(args.out / side / "segmentation_raw" / f"{frame_id}.png"),
                    cv2.cvtColor(seg, cv2.COLOR_RGB2BGR),
                )
            np.savez(
                args.out / "poses" / f"{frame_id}.npz",
                T_Wneedle=T_Wneedle,
                T_Wphantom=T_Wphantom,
                commanded_T_Wneedle=source["commanded_T_Wneedle"],
                K=K,
                **side_poses,
            )
            spread_ms = (max(stamps.values()) - min(stamps.values())) / 1e6
            row = {
                "frame_id": frame_id,
                **mask_counts,
                "six_stream_sync_spread_ms": spread_ms,
                "needle_x_m": float(T_Wneedle[0, 3]),
                "needle_y_m": float(T_Wneedle[1, 3]),
                "needle_z_m": float(T_Wneedle[2, 3]),
            }
            rows.append(row)
            print(
                f"{frame_id}: Lmask={row['L_mask_pixels']} "
                f"Rmask={row['R_mask_pixels']} sync={spread_ms:.3f}ms",
                flush=True,
            )
        with (args.out / "metadata.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        summary = {
            "schema_version": 1,
            "complete": True,
            "frames": len(rows),
            "K": K.tolist(),
            "source_pose_commands": str(args.source_a3.resolve()),
            "max_six_stream_sync_spread_ms": max(
                row["six_stream_sync_spread_ms"] for row in rows
            ),
            "L_mask_pixels": {
                "min": min(row["L_mask_pixels"] for row in rows),
                "max": max(row["L_mask_pixels"] for row in rows),
            },
            "R_mask_pixels": {
                "min": min(row["R_mask_pixels"] for row in rows),
                "max": max(row["R_mask_pixels"] for row in rows),
            },
        }
        (args.out / "summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
        print(json.dumps(summary, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
