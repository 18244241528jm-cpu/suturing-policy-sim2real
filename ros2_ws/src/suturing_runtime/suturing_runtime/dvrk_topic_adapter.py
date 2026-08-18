"""Republish JHU dVRK topics onto the stable /suturing contract."""

from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String


class DVRKTopicAdapter(Node):
    def __init__(self) -> None:
        super().__init__("dvrk_topic_adapter")
        defaults = {
            "raw_left_image": "/jhu_daVinci/left/image_rect",
            "raw_right_image": "/jhu_daVinci/right/image_rect",
            "raw_left_info": "/jhu_daVinci/left/camera_info",
            "raw_right_info": "/jhu_daVinci/right/camera_info",
            "raw_psm_pose": "/PSM1/measured_cp",
            "raw_psm_pose_type": "geometry_msgs/msg/PoseStamped",
            "raw_psm_twist": "/PSM1/measured_cv",
            "raw_psm_jaw": "/PSM1/jaw/measured_js",
            "left_image": "/suturing/camera/left/image",
            "right_image": "/suturing/camera/right/image",
            "left_info": "/suturing/camera/left/camera_info",
            "right_info": "/suturing/camera/right/camera_info",
            "psm_pose": "/suturing/psm1/measured_pose",
            "psm_twist": "/suturing/psm1/measured_twist",
            "psm_jaw": "/suturing/psm1/jaw/measured_js",
            "fallback_psm_frame": "PSM1_psm_base_link",
        }
        for name, value in defaults.items(): self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.pose_pub = self.create_publisher(PoseStamped, value("psm_pose"), 5)
        self.twist_pub = self.create_publisher(TwistStamped, value("psm_twist"), 5)
        self.jaw_pub = self.create_publisher(JointState, value("psm_jaw"), 5)
        self.left_pub = self.create_publisher(Image, value("left_image"), qos_profile_sensor_data)
        self.right_pub = self.create_publisher(Image, value("right_image"), qos_profile_sensor_data)
        self.left_info_pub = self.create_publisher(CameraInfo, value("left_info"), 5)
        self.right_info_pub = self.create_publisher(CameraInfo, value("right_info"), 5)
        self.status_pub = self.create_publisher(String, "/suturing/adapter/status", 5)
        self.counts = {"left": 0, "right": 0, "left_info": 0, "right_info": 0,
                       "psm_pose": 0, "psm_twist": 0, "psm_jaw": 0}
        self.started = time.monotonic()
        self.create_subscription(Image, value("raw_left_image"),
            lambda msg: self._relay(msg, self.left_pub, "left"), qos_profile_sensor_data)
        self.create_subscription(Image, value("raw_right_image"),
            lambda msg: self._relay(msg, self.right_pub, "right"), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, value("raw_left_info"),
            lambda msg: self._relay(msg, self.left_info_pub, "left_info"), 5)
        self.create_subscription(CameraInfo, value("raw_right_info"),
            lambda msg: self._relay(msg, self.right_info_pub, "right_info"), 5)
        pose_type = str(value("raw_psm_pose_type"))
        if pose_type == "geometry_msgs/msg/PoseStamped":
            self.create_subscription(PoseStamped, value("raw_psm_pose"), self._pose_stamped, 10)
        elif pose_type == "geometry_msgs/msg/TransformStamped":
            self.create_subscription(TransformStamped, value("raw_psm_pose"), self._transform_stamped, 10)
        else:
            raise ValueError(f"D10-E11-POSE-TYPE unsupported raw_psm_pose_type={pose_type}")
        self.create_subscription(TwistStamped, value("raw_psm_twist"),
            lambda msg: self._relay(msg, self.twist_pub, "psm_twist"), 10)
        self.create_subscription(JointState, value("raw_psm_jaw"),
            lambda msg: self._relay(msg, self.jaw_pub, "psm_jaw"), 10)
        self.create_timer(1.0, self._status)
        self.get_logger().info("D10_ADAPTER_READY read-only; no command publisher exists in this node")

    def _relay(self, msg, publisher, key: str) -> None:
        self.counts[key] += 1; publisher.publish(msg)

    def _publish_pose(self, out: PoseStamped) -> None:
        if not out.header.frame_id:
            out.header.frame_id = str(self.get_parameter("fallback_psm_frame").value)
        self.counts["psm_pose"] += 1; self.pose_pub.publish(out)

    def _pose_stamped(self, msg: PoseStamped) -> None:
        self._publish_pose(msg)

    def _transform_stamped(self, msg: TransformStamped) -> None:
        out = PoseStamped(); out.header = msg.header
        out.pose.position.x = msg.transform.translation.x
        out.pose.position.y = msg.transform.translation.y
        out.pose.position.z = msg.transform.translation.z
        out.pose.orientation = msg.transform.rotation
        self._publish_pose(out)

    def _status(self) -> None:
        msg = String(); msg.data = json.dumps({"node": self.get_name(),
            "uptime_s": time.monotonic()-self.started, "counts": self.counts,
            "command_capability": False}, sort_keys=True)
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init(); node = DVRKTopicAdapter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
