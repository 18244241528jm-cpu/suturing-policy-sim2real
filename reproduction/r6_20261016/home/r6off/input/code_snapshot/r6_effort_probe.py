#!/usr/bin/env python3
"""R6: read PSM1 joint effort for N seconds (baseline / noise)."""
import os, sys
if 'AMENT_PREFIX_PATH' not in os.environ:
    import shlex
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && exec python3 ' + ' '.join(shlex.quote(x) for x in [os.path.abspath(__file__)] + sys.argv[1:])])
import rclpy, time, numpy as np
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node('r6_eff'); E = []; P = []; S = []
n.create_subscription(JointState, '/PSM1/measured_js', lambda m: (E.append(list(m.effort)[:6]), P.append(list(m.position)[:6])), 10)
n.create_subscription(JointState, '/PSM1/setpoint_js', lambda m: S.append(list(m.position)[:6]), 10)
t = time.time()
while time.time() - t < float(sys.argv[1] if len(sys.argv) > 1 else 3): rclpy.spin_once(n, timeout_sec=0.05)
E = np.array(E); print('n', len(E), 'effort mean', E.mean(0).round(3).tolist(), 'std', E.std(0).round(4).tolist()); print('pos - setpoint', (np.array(P[-1]) - np.array(S[-1])).round(5).tolist(), flush=True)
os._exit(0)
