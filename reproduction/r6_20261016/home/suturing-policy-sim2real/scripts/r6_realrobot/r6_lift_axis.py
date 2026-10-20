#!/usr/bin/env python3
"""R6: lift PSM1 along its tool z axis (away from the tip direction) by D mm in <= 3 mm steps, keeping orientation."""
import sys, time, json, numpy as np, rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as Rot
D = float(sys.argv[1]); rclpy.init(); n = rclpy.create_node('r6_lift'); L = {}
n.create_subscription(PoseStamped, '/PSM1/setpoint_cp', lambda m: L.__setitem__('sp', m), 10)
n.create_subscription(PoseStamped, '/PSM1/measured_cp', lambda m: L.__setitem__('cp', m), 10)
n.create_subscription(Bool, '/PSM1/goal_reached', lambda m: L.__setitem__('r', m.data), 10)
pub = n.create_publisher(PoseStamped, '/PSM1/move_cp', 1); t0 = time.time()
while ('sp' not in L or 'cp' not in L) and time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.1)
m = L['sp']; p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]); q = [m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w]
z = Rot.from_quat(q).as_matrix()[:, 2]; k = int(np.ceil(D / 3.0))
for j in range(1, k + 1):
    tg = p - z * (D / 1000.0) * j / k; msg = PoseStamped(); msg.header.frame_id = m.header.frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, tg); msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
    L.pop('r', None); pub.publish(msg); t = time.time()
    while 'r' not in L and time.time() - t < 10: rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.4); print(j, 'reached', L.get('r'), flush=True)
