"""Small ROS message helpers shared by the real initialization nodes."""

from __future__ import annotations

import copy

from sensor_msgs.msg import Image

from .contract import FrameContract


def stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def image_contract(msg: Image) -> FrameContract:
    return FrameContract(
        stamp_ns=stamp_ns(msg.header),
        frame_id=str(msg.header.frame_id),
        width=int(msg.width),
        height=int(msg.height),
        encoding=str(msg.encoding),
    )


def clone_message(msg):
    """ROS messages are mutable; a latch must keep an independent copy."""
    return copy.deepcopy(msg)
