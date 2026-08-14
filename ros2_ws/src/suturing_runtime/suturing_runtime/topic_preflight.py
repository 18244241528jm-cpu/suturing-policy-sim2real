"""Read-only graph/type/message preflight for known JHU dVRK topics."""

from __future__ import annotations
import json, time
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState

EXPECTED = {
    "/jhu_daVinci/left/image_rect": ("sensor_msgs/msg/Image", Image),
    "/jhu_daVinci/right/image_rect": ("sensor_msgs/msg/Image", Image),
    "/jhu_daVinci/left/camera_info": ("sensor_msgs/msg/CameraInfo", CameraInfo),
    "/jhu_daVinci/right/camera_info": ("sensor_msgs/msg/CameraInfo", CameraInfo),
    "/PSM1/measured_cp": (("geometry_msgs/msg/PoseStamped", "geometry_msgs/msg/TransformStamped"),
                           (PoseStamped, TransformStamped)),
    "/PSM1/measured_cv": ("geometry_msgs/msg/TwistStamped", TwistStamped),
    "/PSM1/jaw/measured_js": ("sensor_msgs/msg/JointState", JointState),
    "/PSM1/servo_cp": (("geometry_msgs/msg/PoseStamped", "geometry_msgs/msg/TransformStamped"), None),
}

class Preflight(Node):
    def __init__(self):
        super().__init__("suturing_topic_preflight"); self.declare_parameter("wait_s",5.0)
        self.counts={topic:0 for topic in EXPECTED}; self.frames={}
        for topic,(_,typ) in EXPECTED.items():
            if typ is not None:
                types = typ if isinstance(typ, tuple) else (typ,)
                for message_type in types:
                    qos=qos_profile_sensor_data if message_type is Image else 10
                    self.create_subscription(message_type,topic,lambda msg,t=topic:self._seen(t,msg),qos)
    def _seen(self,topic,msg):
        self.counts[topic]+=1; self.frames[topic]=getattr(getattr(msg,"header",None),"frame_id","")
    def result(self):
        graph={name:types for name,types in self.get_topic_names_and_types()}; rows=[]
        for topic,(expected,_) in EXPECTED.items():
            actual=graph.get(topic,[]); input_stream=topic!="/PSM1/servo_cp"
            accepted = expected if isinstance(expected, tuple) else (expected,)
            rows.append({"topic":topic,"accepted_types":list(accepted),"actual_types":actual,
                         "type_ok":any(item in actual for item in accepted),"messages_received":self.counts[topic],
                         "frame_id":self.frames.get(topic,""),"input_stream":input_stream})
        failures=[]
        for row in rows:
            if not row["type_ok"]: failures.append(f"type:{row['topic']}")
            if row["input_stream"] and row["messages_received"]==0: failures.append(f"no_message:{row['topic']}")
        return {"schema":"suturing_runtime.preflight.v1","read_only":True,
                "rows":rows,"failures":failures,"passed":not failures}

def main():
    rclpy.init(); node=Preflight(); deadline=time.monotonic()+float(node.get_parameter("wait_s").value)
    while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=0.1)
    result=node.result(); print(json.dumps(result,indent=2,sort_keys=True),flush=True)
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(0 if result["passed"] else 2)
