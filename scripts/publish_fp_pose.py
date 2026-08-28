#!/usr/bin/env python3
"""Publish a FoundationPose 4x4 as the PoseStamped that fp_goal_bridge subscribes to.

FoundationPose writes the estimate as a plain 4x4 text matrix -- run_demo.py leaves it in
debug/ob_in_cam/<frame>.txt -- while the runtime speaks PoseStamped. This is the adapter
between the two, so nothing downstream has to know how the estimator emits its result.

The matrix is T_camera_from_needle in **metres**, which is what the rest of the chain
expects; do not pre-scale it. The centimetre convention exists only inside the two
controllers, which convert at their own boundaries.

    python3 scripts/publish_fp_pose.py --file ~/FoundationPose/debug/ob_in_cam/000000.txt \
        --frame ecm_left_optical_frame

    python3 scripts/publish_fp_pose.py --file <path> --frame <id> --watch

    python3 scripts/publish_fp_pose.py --frame <id> --matrix \
        0.898 -0.295 0.326 0.0194  0.125 -0.540 -0.832 0.0127 \
        0.422  0.788 -0.448 0.0801  0 0 0 1

--watch re-reads the file whenever it changes, so a tracker that keeps overwriting one
file becomes a live pose stream. Without it the pose is published once and latched,
which suits a one-shot register.

The matrix is checked before publishing. A rotation block that is not orthonormal, or a
determinant of -1, means the estimate is not a rigid transform -- a mirrored frame or a
scale that crept in -- and publishing it would send the arm confidently to a wrong pose
that every downstream guard still accepts.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation


def parse_matrix(text: str) -> np.ndarray:
    values = [float(token) for token in text.replace(",", " ").split()]
    if len(values) != 16:
        raise ValueError(f"expected 16 numbers, found {len(values)}")
    return np.asarray(values, dtype=np.float64).reshape(4, 4)


def check_rigid(T: np.ndarray, tolerance: float = 1e-3) -> None:
    R = T[:3, :3]
    orthonormality = float(np.abs(R @ R.T - np.eye(3)).max())
    determinant = float(np.linalg.det(R))
    if orthonormality > tolerance:
        raise ValueError(
            f"rotation block is not orthonormal, |RR^T - I| = {orthonormality:.2e}. "
            "A scale factor here becomes a proportional position error downstream.")
    if abs(determinant - 1.0) > tolerance:
        raise ValueError(
            f"det(R) = {determinant:.6f}, expected +1. A value near -1 is a mirrored "
            "frame, usually a column-order or handedness mistake in the export.")
    if not np.allclose(T[3, :], [0, 0, 0, 1], atol=tolerance):
        raise ValueError(f"last row is {T[3, :].tolist()}, expected [0, 0, 0, 1]")


def describe(T: np.ndarray) -> str:
    translation_mm = np.round(T[:3, 3] * 1000, 2)
    rpy_deg = np.round(Rotation.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True), 2)
    return (f"t_mm={translation_mm.tolist()} range={np.linalg.norm(T[:3, 3])*1000:.1f}mm "
            f"rpy_deg={rpy_deg.tolist()}")


def to_message(T: np.ndarray, frame_id: str, stamp) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    msg.pose.orientation.x, msg.pose.orientation.y = float(q[0]), float(q[1])
    msg.pose.orientation.z, msg.pose.orientation.w = float(q[2]), float(q[3])
    return msg


class FPPosePublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("fp_pose_publisher")
        self.args = args
        self.last_signature = None
        # Latched, so a subscriber that starts late still receives a one-shot estimate.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PoseStamped, args.topic, latched)
        if args.watch:
            self.create_timer(args.poll_interval_s, self._poll)

    def publish_matrix(self, T: np.ndarray) -> None:
        check_rigid(T)
        self.publisher.publish(to_message(T, self.args.frame, self.get_clock().now().to_msg()))
        self.get_logger().info(f"published on {self.args.topic} frame={self.args.frame} "
                               f"{describe(T)}")

    def _poll(self) -> None:
        path = Path(self.args.file)
        try:
            signature = (path.stat().st_mtime_ns, path.stat().st_size)
        except OSError:
            return
        if signature == self.last_signature:
            return
        try:
            T = parse_matrix(path.read_text())
            self.publish_matrix(T)
        except ValueError as exc:
            # A tracker mid-write leaves a truncated file; say so and try again next tick
            # rather than dying and taking the stream with it.
            self.get_logger().warning(f"skipping {path}: {exc}")
            return
        self.last_signature = signature


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--file", help="text file holding the 4x4, whitespace separated")
    source.add_argument("--matrix", nargs=16, type=float, help="the 16 numbers, row major")
    parser.add_argument("--frame", required=True,
                        help="camera frame id; must equal fp_goal_bridge.required_needle_frame")
    parser.add_argument("--topic", default="/suturing/needle/pose_camera")
    parser.add_argument("--watch", action="store_true",
                        help="republish whenever the file changes")
    parser.add_argument("--poll-interval-s", type=float, default=0.2)
    parser.add_argument("--hold-s", type=float, default=4.0,
                        help="one-shot mode: keep republishing this long so subscribers "
                             "have time to discover the publisher")
    args = parser.parse_args()

    if args.matrix:
        T = np.asarray(args.matrix, dtype=np.float64).reshape(4, 4)
    else:
        T = parse_matrix(Path(args.file).read_text())

    try:
        check_rigid(T)
    except ValueError as exc:
        print(f"refusing to publish: {exc}", file=sys.stderr)
        raise SystemExit(2)
    print(f"matrix accepted: {describe(T)}")

    rclpy.init()
    node = FPPosePublisher(args)
    try:
        node.publish_matrix(T)
        if args.watch:
            node.last_signature = None
            rclpy.spin(node)
        else:
            # ROS 2 discovery takes on the order of a second, so a publisher that
            # published once and exited would routinely be gone before any subscriber
            # matched it -- the sample would simply never arrive. Repeat over a few
            # seconds; TRANSIENT_LOCAL then covers subscribers that join later still.
            deadline = time.monotonic() + args.hold_s
            next_publish = 0.0
            while time.monotonic() < deadline:
                if time.monotonic() >= next_publish:
                    node.publisher.publish(
                        to_message(T, args.frame, node.get_clock().now().to_msg()))
                    next_publish = time.monotonic() + 0.5
                rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
