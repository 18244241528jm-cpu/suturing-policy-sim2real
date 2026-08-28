#!/usr/bin/env python3
"""Sweep the arm by hand and see whether the executor's workspace box contains it.

guarded_pose_executor rejects an execute_once when either the current pose or the
segment target falls outside workspace_min_m/workspace_max_m, with D10-E58-WORKSPACE.
The box is axis-aligned and therefore convex, so two endpoints inside it are enough to
put the whole straight-line path inside -- checking the start pose and the touch-off
goal is sufficient to be allowed to run.

What two points cannot tell you is the margin. A goal 5 mm from a face passes today and
fails the moment the needle is moved. This samples the pose stream while an operator
walks the arm through the working volume, reports the observed bounding box against the
configured one, and names the tightest face.

Read-only: it subscribes and prints. It publishes nothing and cannot move the arm.

    python3 scripts/workspace_audit.py                    # audit an existing box
    python3 scripts/workspace_audit.py --teach            # derive a box from taught points

Teach mode inverts the question. Instead of checking a box someone guessed, it records
the poses you actually visit and emits the box that contains them plus a margin, ready
to paste into jhu_real.yaml. A box grown from the volume the arm is really used in beats
one carried over from the simulator, which is how the shipped z bounds ended up marked
as an assumption.
"""

from __future__ import annotations

import argparse

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

AXES = ("x", "y", "z")


class WorkspaceAudit(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("workspace_audit")
        self.args = args
        self.low = np.asarray(args.workspace_min, dtype=float)
        self.high = np.asarray(args.workspace_max, dtype=float)
        self.minimum = np.full(3, np.inf)
        self.maximum = np.full(3, -np.inf)
        self.samples = 0
        self.outside = 0
        self.frames: set[str] = set()
        self.create_subscription(PoseStamped, args.topic, self._pose, 50)
        self.create_timer(args.report_interval_s, self._report)
        self.get_logger().info(
            f"auditing {args.topic} against box {self.low.tolist()} .. {self.high.tolist()}; "
            "move the arm through the working volume, then Ctrl-C"
        )

    def _pose(self, msg: PoseStamped) -> None:
        self.frames.add(msg.header.frame_id)
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.minimum = np.minimum(self.minimum, p)
        self.maximum = np.maximum(self.maximum, p)
        self.samples += 1
        if np.any(p < self.low) or np.any(p > self.high):
            self.outside += 1

    def _report(self) -> None:
        if not self.samples:
            self.get_logger().warning(f"no messages yet on {self.args.topic}")
            return
        spans = ", ".join(
            f"{AXES[i]} [{self.minimum[i]:+.4f}, {self.maximum[i]:+.4f}]" for i in range(3)
        )
        self.get_logger().info(f"n={self.samples} outside={self.outside} observed {spans}")

    def summary(self) -> int:
        print()
        if not self.samples:
            print("No samples. Is the arm publishing, and is the topic name right?")
            return 1

        # A frame mismatch makes every number below meaningless, so say so first.
        expected = self.args.expect_frame
        print(f"frames seen: {sorted(self.frames)}")
        if expected and self.frames != {expected}:
            print(f"  WARNING expected only {expected!r}; the box is defined in that frame")

        print(f"samples: {self.samples}   outside the box: {self.outside}")
        print()
        print(f"{'axis':<5} {'observed min':>13} {'observed max':>13} "
              f"{'box min':>10} {'box max':>10} {'margin lo':>11} {'margin hi':>11}")
        worst = (np.inf, None, None)
        for i, axis in enumerate(AXES):
            margin_low = self.minimum[i] - self.low[i]
            margin_high = self.high[i] - self.maximum[i]
            print(f"{axis:<5} {self.minimum[i]:>13.4f} {self.maximum[i]:>13.4f} "
                  f"{self.low[i]:>10.4f} {self.high[i]:>10.4f} "
                  f"{margin_low*1000:>9.1f}mm {margin_high*1000:>9.1f}mm")
            for margin, face in ((margin_low, f"{axis} lower"), (margin_high, f"{axis} upper")):
                if margin < worst[0]:
                    worst = (margin, face, axis)
        print()
        print(f"tightest face: {worst[1]} with {worst[0]*1000:.1f} mm of margin")

        if self.outside:
            print("VERDICT: part of the swept volume is OUTSIDE the box; "
                  "execute_once would return D10-E58-WORKSPACE there.")
            print("Widen the box only for the axis and face named above, and only as far "
                  "as the arm actually needs -- the box is a guard, not a formality.")
            return 2
        if worst[0] < self.args.margin_warn_m:
            print(f"VERDICT: inside, but only {worst[0]*1000:.1f} mm from a face "
                  f"(warning under {self.args.margin_warn_m*1000:.0f} mm). "
                  "Moving the needle could push a future goal out.")
            return 3
        print("VERDICT: the swept volume is comfortably inside the box.")
        return 0


class WorkspaceTeach(Node):
    """Record one pose per keypress, then emit the enclosing box plus a margin."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("workspace_teach")
        self.args = args
        self.latest = None
        self.frames: set[str] = set()
        self.points: list[np.ndarray] = []
        self.create_subscription(PoseStamped, args.topic, self._pose, 50)

    def _pose(self, msg: PoseStamped) -> None:
        self.frames.add(msg.header.frame_id)
        self.latest = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def capture(self) -> bool:
        if self.latest is None:
            print("  no pose received yet; is the arm publishing?")
            return False
        self.points.append(self.latest.copy())
        p = self.latest
        print(f"  recorded #{len(self.points)}: "
              f"x={p[0]:+.4f} y={p[1]:+.4f} z={p[2]:+.4f}")
        return True

    def emit(self) -> int:
        if len(self.points) < 2:
            print()
            print("Need at least two points to define a box.")
            return 1
        pts = np.vstack(self.points)
        low = pts.min(axis=0) - self.args.margin_m
        high = pts.max(axis=0) + self.args.margin_m
        print()
        print(f"frames seen: {sorted(self.frames)}")
        print(f"{len(self.points)} points, margin {self.args.margin_m*1000:.0f} mm on every face")
        print()
        for i, axis in enumerate(AXES):
            print(f"  {axis}  visited [{pts[:,i].min():+.4f}, {pts[:,i].max():+.4f}]"
                  f"  ->  box [{low[i]:+.4f}, {high[i]:+.4f}]")
        print()
        print("Paste into jhu_real.yaml under guarded_pose_executor.ros__parameters:")
        print()
        print(f"    workspace_min_m: [{low[0]:.4f}, {low[1]:.4f}, {low[2]:.4f}]")
        print(f"    workspace_max_m: [{high[0]:.4f}, {high[1]:.4f}, {high[2]:.4f}]")
        print()
        print("The box is a guard, not a description: it should cover where the arm is")
        print("meant to go and stop short of where it is not. Widen it deliberately.")
        return 0


def run_teach(args: argparse.Namespace) -> int:
    rclpy.init()
    node = WorkspaceTeach(args)
    print(f"Teach mode on {args.topic}. Move the arm to a corner of the working volume,")
    print("press Enter to record it. Enter 'q' then Enter when done.")
    try:
        while True:
            # Service callbacks between prompts so the recorded pose is the live one.
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.02)
            answer = input(f"[{len(node.points)} recorded] Enter to record, q to finish: ")
            if answer.strip().lower() == "q":
                break
            node.capture()
    except (KeyboardInterrupt, EOFError):
        pass
    code = node.emit()
    node.destroy_node()
    rclpy.try_shutdown()
    return code


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teach", action="store_true",
                        help="record visited poses and emit a box instead of auditing one")
    parser.add_argument("--margin-m", type=float, default=0.020,
                        help="teach mode: padding added to every face of the taught box")
    parser.add_argument("--topic", default="/PSM1/measured_cp",
                        help="pose stream the executor ultimately compares against")
    parser.add_argument("--expect-frame", default="ECM",
                        help="frame the workspace box is defined in; empty to skip the check")
    parser.add_argument("--workspace-min", type=float, nargs=3,
                        default=[-0.20, -0.20, -0.05], metavar=("X", "Y", "Z"))
    parser.add_argument("--workspace-max", type=float, nargs=3,
                        default=[0.20, 0.20, 0.25], metavar=("X", "Y", "Z"))
    parser.add_argument("--margin-warn-m", type=float, default=0.010,
                        help="warn when the closest face is nearer than this")
    parser.add_argument("--report-interval-s", type=float, default=2.0)
    args = parser.parse_args()

    if args.teach:
        raise SystemExit(run_teach(args))

    rclpy.init()
    node = WorkspaceAudit(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        code = node.summary()
        node.destroy_node()
        rclpy.try_shutdown()
        raise SystemExit(code)


if __name__ == "__main__":
    main()
