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

    python3 scripts/workspace_audit.py
    python3 scripts/workspace_audit.py --topic /PSM1/measured_cp --expect-frame ECM
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
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
