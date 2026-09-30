#!/usr/bin/env python3
"""R4 day-2 1-a (B-1): record SUJ / ECM readings while the user pushes a SUJ, then report per-SUJ max |dt| (addendum 1-a).
usage: r4d2_b1_suj_record.py --tag psm1_push --seconds 60     (self-sources nothing: run after sourcing ROS 2 + ros2_ws)
Writes records/logs/R4_20260928/day2/1a_B1/<tag>.jsonl and prints: 'updates' iff max |dt| >= 2.0 mm, 'does not update' iff < 0.2 mm."""
import argparse, json, os, time, numpy as np, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
ap = argparse.ArgumentParser(); ap.add_argument('--tag', required=True); ap.add_argument('--seconds', type=float, default=60); a = ap.parse_args()
OUT = 'records/logs/R4_20260928/day2/1a_B1'; os.makedirs(OUT, exist_ok=True)
CP = ['/SUJ/PSM1/measured_cp', '/SUJ/PSM1/local/measured_cp', '/SUJ/ECM/measured_cp', '/SUJ/ECM/local/measured_cp', '/PSM1/measured_cp', '/ECM/measured_cp']
JS = ['/SUJ/PSM1/measured_js', '/SUJ/ECM/measured_js', '/ECM/measured_js']
rclpy.init(); n = rclpy.create_node('r4d2_b1'); last = {}; qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
for t in CP: n.create_subscription(PoseStamped, t, lambda m, t=t: last.__setitem__(t, [m.pose.position.x * 1000, m.pose.position.y * 1000, m.pose.position.z * 1000, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w]), qos)
for t in JS: n.create_subscription(JointState, t, lambda m, t=t: last.__setitem__(t, list(m.position)), qos)
rows = []; t0 = time.time(); nxt = t0
with open(f'{OUT}/{a.tag}.jsonl', 'w') as f:
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(n, timeout_sec=0.01)
        if time.time() >= nxt and last:
            r = {'t': time.time(), **{k: v for k, v in last.items()}}; f.write(json.dumps(r) + '\n'); rows.append(r); nxt += 0.1
summ = {}
for t in CP:
    P = np.array([r[t][:3] for r in rows if t in r])
    if len(P): dt = np.linalg.norm(P - P[0], axis=1); summ[t] = {'n': len(P), 'max_dt_mm': float(dt.max()), 'end_dt_mm': float(dt[-1]),
                                                              'verdict': 'updates' if dt.max() >= 2.0 else ('does not update' if dt.max() < 0.2 else 'ambiguous')}
    else: summ[t] = {'n': 0, 'verdict': 'no messages'}
for t in JS:
    Q = np.array([r[t] for r in rows if t in r])
    if len(Q): summ[t] = {'n': len(Q), 'max_abs_dq': np.abs(Q - Q[0]).max(0).round(5).tolist()}
json.dump(summ, open(f'{OUT}/{a.tag}_summary.json', 'w'), indent=1); print(json.dumps(summ, indent=1))
