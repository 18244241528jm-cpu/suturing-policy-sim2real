#!/usr/bin/env python3
"""R4 day-2 diagnostic: pure Cartesian translations of PSM1 (orientation held) in the /PSM1/measured_cp frame, stereo capture at each state.
States: base, +dx, base, +dy, base, +dz, base (default d = 5 mm). Saves <out>/sNN_<name>/{right,left}_rect_mean.png + state.json.
usage: r4d2_cart_probe.py --out DIR [--d 5] [--zsign -1]"""
import os, sys
if 'AMENT_PREFIX_PATH' not in os.environ:
    import shlex
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && exec python3 '
                       + ' '.join(shlex.quote(x) for x in [os.path.abspath(__file__)] + sys.argv[1:])])
import argparse, json, time, threading, numpy as np, cv2, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import message_filters as mf
ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--d', type=float, default=5.0); ap.add_argument('--zsign', type=float, default=1.0)
a = ap.parse_args(); assert 0 < a.d <= 5.0; os.makedirs(a.out, exist_ok=True)
rclpy.init(); n = rclpy.create_node('r4d2_cart_probe'); br = CvBridge(); L = {}; grab = {'buf': None, 'n': 0}
subs = [mf.Subscriber(n, Image, f'/jhu_daVinci/{c}/image_rect', qos_profile=qos_profile_sensor_data) for c in ('left', 'right')]
sy = mf.ApproximateTimeSynchronizer(subs, 10, 0.02)
def cb(l, r):
    if grab['buf'] is not None and len(grab['buf']) < grab['n']: grab['buf'].append((br.imgmsg_to_cv2(l, 'bgr8').astype(np.float32), br.imgmsg_to_cv2(r, 'bgr8').astype(np.float32)))
sy.registerCallback(cb)
for k, t, ty in (('cp', '/PSM1/measured_cp', PoseStamped), ('sp', '/PSM1/setpoint_cp', PoseStamped), ('local', '/PSM1/local/measured_cp', PoseStamped), ('js', '/PSM1/measured_js', JointState), ('ecm', '/ECM/measured_js', JointState)):
    n.create_subscription(ty, t, lambda m, k=k: L.__setitem__(k, m), 10)
n.create_subscription(Bool, '/PSM1/goal_reached', lambda m: L.__setitem__('reached', m.data), 10)
pub = n.create_publisher(PoseStamped, '/PSM1/move_cp', 1)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
t0 = time.time()
while not all(k in L for k in ('cp', 'sp', 'local', 'js', 'ecm')) and time.time() - t0 < 10: time.sleep(0.05)
assert pub.get_subscription_count() > 0, 'no /PSM1/move_cp subscriber'
P = lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z]
Q = lambda m: [m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w]
sp0 = L['sp']; p0 = np.array(P(sp0)); q0 = Q(sp0)
def move(dp):
    m = PoseStamped(); m.header.frame_id = sp0.header.frame_id; g = p0 + np.array(dp) / 1000.0
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, g)
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q0
    L.pop('reached', None); pub.publish(m); t = time.time()
    while 'reached' not in L and time.time() - t < 10: time.sleep(0.02)
    time.sleep(1.5); return L.get('reached')
def record(name, k=10):
    grab['n'] = k; grab['buf'] = []; t = time.time()
    while len(grab['buf']) < k and time.time() - t < 15: time.sleep(0.02)
    b = grab['buf']; grab['buf'] = None; d = f'{a.out}/{name}'; os.makedirs(d, exist_ok=True)
    cv2.imwrite(f'{d}/left_rect_mean.png', np.mean([x[0] for x in b], 0).round().astype(np.uint8)); cv2.imwrite(f'{d}/right_rect_mean.png', np.mean([x[1] for x in b], 0).round().astype(np.uint8))
    st = {'n_frames': len(b), 'cp_p': P(L['cp']), 'cp_q': Q(L['cp']), 'cp_frame': L['cp'].header.frame_id, 'local_p': P(L['local']), 'local_q': Q(L['local']),
          'js': list(L['js'].position), 'ecm_js': list(L['ecm'].position), 't': time.time()}
    json.dump(st, open(f'{d}/state.json', 'w'), indent=1); return st
d = a.d; plan = [('s0_base', [0, 0, 0]), ('s1_x', [d, 0, 0]), ('s2_base', [0, 0, 0]), ('s3_y', [0, d, 0]), ('s4_base', [0, 0, 0]), ('s5_z', [0, 0, a.zsign * d]), ('s6_base', [0, 0, 0])]
for name, dp in plan:
    ok = move(dp); st = record(name)
    print(name, 'reached', ok, 'cp-p0 (mm)', ((np.array(st['cp_p']) - p0) * 1000).round(2), 'frames', st['n_frames'], flush=True)
rclpy.shutdown()
