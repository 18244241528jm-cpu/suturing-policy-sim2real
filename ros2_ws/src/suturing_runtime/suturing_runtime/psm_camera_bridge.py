"""Express dVRK measured_cp in the camera frame using the calibrated hand-eye TF."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .contract import matrix_to_quaternion_xyzw, pose_matrix, transform_matrix


class PSMCameraBridge(Node):
    def __init__(self) -> None:
        super().__init__("psm_camera_bridge")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("input_topic", "/suturing/psm1/measured_pose")
        self.declare_parameter("output_topic", "/suturing/psm1/measured_pose_camera")
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(PoseStamped,self.get_parameter("output_topic").value,10)
        self.create_subscription(PoseStamped,self.get_parameter("input_topic").value,self._pose,10)
        self.get_logger().info("D12_R7_PSM_BRIDGE_READY requires calibrated camera<-PSM-base TF")

    def _pose(self,msg):
        camera=str(self.get_parameter("camera_frame").value)
        if not camera:
            self.get_logger().error("D12-E601-CAMERA_FRAME_UNCONFIGURED"); return
        if not msg.header.frame_id:
            self.get_logger().error("D12-E602-PSM_FRAME_EMPTY"); return
        try:
            tf=self.buffer.lookup_transform(camera,msg.header.frame_id,Time.from_msg(msg.header.stamp),
                                            timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self.get_logger().error(f"D12-E603-HANDEYE_TF {exc}"); return
        t,q=tf.transform.translation,tf.transform.rotation
        camera_from_base=transform_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
        p,o=msg.pose.position,msg.pose.orientation
        camera_from_ee=camera_from_base@pose_matrix([p.x,p.y,p.z],[o.x,o.y,o.z,o.w])
        out=PoseStamped(); out.header.stamp=msg.header.stamp; out.header.frame_id=camera
        out.pose.position.x,out.pose.position.y,out.pose.position.z=map(float,camera_from_ee[:3,3])
        quat=matrix_to_quaternion_xyzw(camera_from_ee[:3,:3])
        out.pose.orientation.x,out.pose.orientation.y,out.pose.orientation.z,out.pose.orientation.w=map(float,quat)
        self.pub.publish(out)


def main() -> None:
    rclpy.init(); node=PSMCameraBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
