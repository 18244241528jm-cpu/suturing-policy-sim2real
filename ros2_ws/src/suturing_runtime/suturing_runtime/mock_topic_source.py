"""Synthetic external providers for installation smoke tests only."""
import json
import struct
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

class MockSource(Node):
    def __init__(self):
        super().__init__("suturing_mock_topic_source")
        self.left=self.create_publisher(Image,"/suturing/camera/left/image",5)
        self.right=self.create_publisher(Image,"/suturing/camera/right/image",5)
        self.info=self.create_publisher(CameraInfo,"/suturing/camera/left/camera_info",5)
        self.right_info=self.create_publisher(CameraInfo,"/suturing/camera/right/camera_info",5)
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.depth=self.create_publisher(Image,"/suturing/external/depth",latched)
        self.mask=self.create_publisher(Image,"/suturing/external/needle_mask",latched)
        self.candidates=self.create_publisher(String,"/suturing/external/needle_candidates",latched)
        self.psm=self.create_publisher(PoseStamped,"/suturing/psm1/measured_pose",5)
        self.create_subscription(Image,"/suturing/initialization/left/image",self._snapshot,latched)
        self.create_subscription(String,"/suturing/fp_input/ready",self._fp_ready,latched)
        self.create_timer(0.1,self._publish)
    def _image(self,encoding,step,data):
        msg=Image(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id="ecm_left_optical_frame"
        msg.height=4; msg.width=4; msg.encoding=encoding; msg.is_bigendian=0; msg.step=step; msg.data=data
        return msg
    def _publish(self):
        rgb=self._image("rgb8",12,bytes(48)); self.left.publish(rgb); self.right.publish(rgb)
        info=CameraInfo(); info.header=rgb.header; info.height=4; info.width=4
        info.k=[100.,0.,2.,0.,100.,2.,0.,0.,1.]; self.info.publish(info); self.right_info.publish(info)
        psm=PoseStamped(); psm.header.stamp=rgb.header.stamp; psm.header.frame_id="PSM1_psm_base_link"
        psm.pose.orientation.w=1.; psm.pose.position.z=.1; self.psm.publish(psm)
    def _snapshot(self,rgb):
        depth=Image(); depth.header=rgb.header; depth.height=rgb.height; depth.width=rgb.width
        depth.encoding="32FC1"; depth.is_bigendian=0; depth.step=rgb.width*4
        depth.data=struct.pack("<"+"f"*(rgb.width*rgb.height),*([0.1]*(rgb.width*rgb.height)))
        mask=Image(); mask.header=rgb.header; mask.height=rgb.height; mask.width=rgb.width
        mask.encoding="mono8"; mask.is_bigendian=0; mask.step=rgb.width; mask.data=bytes([255]*(rgb.width*rgb.height))
        self.depth.publish(depth); self.mask.publish(mask)
    def _fp_ready(self,msg):
        ready=json.loads(msg.data)
        data={"schema":"suturing.fp_candidates.v1","init_id":"mock",
              "stamp_ns":ready["stamp_ns"],"frame_id":ready["frame_id"],
              "mesh_frame":"needle_mesh","poses":[{
                  "position_m":[0.0,0.0,0.1],"quaternion_xyzw":[0.0,0.0,0.0,1.0],"score":0.9}]}
        out=String(); out.data=json.dumps(data); self.candidates.publish(out)
def main():
    rclpy.init(); node=MockSource()
    try:rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
