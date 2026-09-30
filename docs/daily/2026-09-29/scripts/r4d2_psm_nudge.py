#!/usr/bin/env python3
"""R4 day-2: guarded relative PSM1 joint nudge (deg for q1,q2,q4-q6; mm for q3), jaw kept closed.
Per call: |dq1|,|dq2| <= 3 deg, |dq3| <= 5 mm (+ = deeper, towards the pad), |wrist| <= 15 deg. Log: day2/psm_nudges.jsonl
  r4d2_psm_nudge.py --dq 3 2 0 0 0 0
"""
import os, sys
if 'AMENT_PREFIX_PATH' not in os.environ:
    import shlex
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && exec python3 '
                       + ' '.join(shlex.quote(x) for x in [os.path.abspath(__file__)] + sys.argv[1:])])
import argparse, json, math, time, rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
LOG = os.path.expanduser('~/suturing-policy-sim2real/records/logs/R4_20260928/day2/psm_nudges.jsonl')
ap = argparse.ArgumentParser(); ap.add_argument('--dq', type=float, nargs=6, required=True); ap.add_argument('--note', default=''); a = ap.parse_args()
d = a.dq
assert abs(d[0]) <= 3 and abs(d[1]) <= 3 and abs(d[2]) <= 5 and all(abs(x) <= 15 for x in d[3:]), f'step too large {d}'
rclpy.init(); n = rclpy.create_node('r4d2_psm_nudge'); st = {}
n.create_subscription(JointState, '/PSM1/setpoint_js', lambda m: st.__setitem__('sp', m), 10)
n.create_subscription(JointState, '/PSM1/measured_js', lambda m: st.__setitem__('js', m), 10)
n.create_subscription(Bool, '/PSM1/goal_reached', lambda m: st.__setitem__('reached', m.data), 10)
pub = n.create_publisher(JointState, '/PSM1/move_jp', 1); pj = n.create_publisher(JointState, '/PSM1/jaw/move_jp', 1)
t0 = time.time()
while ('sp' not in st or pub.get_subscription_count() == 0) and time.time() - t0 < 5: rclpy.spin_once(n, timeout_sec=0.05)
assert 'sp' in st, 'no PSM1 setpoint'
q = list(st['sp'].position)
g = [q[0] + math.radians(d[0]), q[1] + math.radians(d[1]), q[2] + d[2] / 1000] + [q[3 + i] + math.radians(d[3 + i]) for i in range(3)]
t0 = time.time()
while time.time() - t0 < 0.3: rclpy.spin_once(n, timeout_sec=0.05)
st.pop('reached', None); m = JointState(); m.position = [float(x) for x in g]; pub.publish(m)
j = JointState(); j.position = [0.0]; pj.publish(j)
t0 = time.time()
while 'reached' not in st and time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.05)
t0 = time.time()
while time.time() - t0 < 0.5: rclpy.spin_once(n, timeout_sec=0.05)
qm = list(st['js'].position)
open(LOG, 'a').write(json.dumps({'time': time.time(), 'from_sp': q, 'goal': g, 'final': qm, 'reached': st.get('reached'), 'dq': d, 'note': a.note}) + '\n')
print('reached', st.get('reached'), 'q (deg/mm):', [round(math.degrees(x), 2) for x in qm[:2]] + [round(qm[2] * 1000, 2)] + [round(math.degrees(x), 2) for x in qm[3:]])
