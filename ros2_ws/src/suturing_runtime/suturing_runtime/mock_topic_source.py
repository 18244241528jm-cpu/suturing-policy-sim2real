"""Synthetic standard-topic publisher for installation smoke tests."""
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

class MockSource(Node):
    def __init__(self):
        super().__init__("suturing_mock_topic_source")
        self.left=self.create_publisher(Image,"/suturing/camera/left/image",5)
        self.right=self.create_publisher(Image,"/suturing/camera/right/image",5)
        self.info=self.create_publisher(CameraInfo,"/suturing/camera/left/camera_info",5)
        self.depth=self.create_publisher(Image,"/suturing/depth/metric",5)
        self.mask=self.create_publisher(Image,"/suturing/needle/mask",5)
        self.needle=self.create_publisher(PoseWithCovarianceStamped,"/suturing/needle/pose_gated",5)
        self.psm=self.create_publisher(PoseStamped,"/suturing/psm1/measured_pose",5)
        self.create_timer(0.1,self._publish)
    def _image(self,encoding,step,data):
        msg=Image(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id="ecm_left_optical_frame"
        msg.height=4; msg.width=4; msg.encoding=encoding; msg.is_bigendian=0; msg.step=step; msg.data=data
        return msg
    def _publish(self):
        rgb=self._image("rgb8",12,bytes(48)); self.left.publish(rgb); self.right.publish(rgb)
        self.depth.publish(self._image("32FC1",16,bytes(64)))
        self.mask.publish(self._image("mono8",4,bytes([255]*16)))
        info=CameraInfo(); info.header=rgb.header; info.height=4; info.width=4
        info.k=[100.,0.,2.,0.,100.,2.,0.,0.,1.]; self.info.publish(info)
        needle=PoseWithCovarianceStamped(); needle.header=rgb.header; needle.pose.pose.orientation.w=1.; needle.pose.pose.position.z=.1
        self.needle.publish(needle)
        psm=PoseStamped(); psm.header.stamp=rgb.header.stamp; psm.header.frame_id="PSM1_psm_base_link"
        psm.pose.orientation.w=1.; psm.pose.position.z=.1; self.psm.publish(psm)
def main():
    rclpy.init(); node=MockSource()
    try:rclpy.spin(node)
    finally:node.destroy_node(); rclpy.shutdown()

