#!/usr/bin/env python3
"""R4 day-2: capture N synced static stereo frames (rect + raw, both topics) into <out>/{left_rect,right_rect,left_raw,right_raw}/NNN.png
plus camera_info_{left,right}.json and stamps. topic 'right' = physical LEFT.   usage: r4d2_capture.py --out DIR [--n 10]"""
import os, sys
if 'AMENT_PREFIX_PATH' not in os.environ:
    import shlex
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && exec python3 '
                       + ' '.join(shlex.quote(x) for x in [os.path.abspath(__file__)] + sys.argv[1:])])
import argparse, json, time, cv2, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters as mf
ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--n', type=int, default=10); a = ap.parse_args()
rclpy.init(); n = rclpy.create_node('r4d2_capture'); br = CvBridge(); got = []; info = {}
keys = (('left', 'image_raw'), ('right', 'image_raw'), ('left', 'image_rect'), ('right', 'image_rect'))
subs = [mf.Subscriber(n, Image, f'/jhu_daVinci/{c}/{k}', qos_profile=qos_profile_sensor_data) for c, k in keys]
sy = mf.ApproximateTimeSynchronizer(subs, 10, 0.02)
sy.registerCallback(lambda *ms: got.append(([br.imgmsg_to_cv2(m, 'bgr8') for m in ms], [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in ms])) if len(got) < a.n else None)
for c in ('left', 'right'):
    n.create_subscription(CameraInfo, f'/jhu_daVinci/{c}/camera_info', lambda m, c=c: info.__setitem__(c, {'K': list(m.k), 'D': list(m.d), 'R': list(m.r), 'P': list(m.p), 'width': m.width, 'height': m.height}), 5)
t0 = time.time()
while (len(got) < a.n or len(info) < 2) and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.05)
for sub in ('left_raw', 'right_raw', 'left_rect', 'right_rect'): os.makedirs(f'{a.out}/{sub}', exist_ok=True)
for i, (ims, st) in enumerate(got):
    for nm, im in zip(('left_raw', 'right_raw', 'left_rect', 'right_rect'), ims): cv2.imwrite(f'{a.out}/{nm}/{i:03d}.png', im)
for c in info: json.dump(info[c], open(f'{a.out}/camera_info_{c}.json', 'w'), indent=1)
json.dump({'n': len(got), 'stamps': [st for _, st in got], 'time': time.time()}, open(f'{a.out}/capture.json', 'w'), indent=1)
print('captured', len(got), 'frames ->', a.out)
