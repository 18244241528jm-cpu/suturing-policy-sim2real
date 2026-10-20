#!/usr/bin/env python3
"""R6: save k synced rect stereo frames (mean) + robot state to <out>/. Read-only (no commands).
Files: left_rect.png / right_rect.png use the TOPIC names (/jhu_daVinci/{left,right}/image_rect); physical-left = topic right."""
import os, sys
if 'AMENT_PREFIX_PATH' not in os.environ:
    import shlex
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && exec python3 '
                       + ' '.join(shlex.quote(x) for x in [os.path.abspath(__file__)] + sys.argv[1:])])
import traceback; sys.excepthook = lambda t, v, tb: (traceback.print_exception(t, v, tb), sys.stdout.flush(), sys.stderr.flush(), os._exit(1))   # no std::terminate core dump (stale FastDDS shm)
import argparse, json, time, numpy as np, cv2, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import message_filters as mf
ap = argparse.ArgumentParser(); ap.add_argument('out'); ap.add_argument('-k', type=int, default=6); a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
rclpy.init(); n = rclpy.create_node('r6_snap'); br = CvBridge(); buf = []; L = {}
subs = [mf.Subscriber(n, Image, f'/jhu_daVinci/{c}/image_rect', qos_profile=qos_profile_sensor_data) for c in ('left', 'right')]
sy = mf.ApproximateTimeSynchronizer(subs, 10, 0.02)
sy.registerCallback(lambda l, r: buf.append((br.imgmsg_to_cv2(l, 'bgr8').astype(np.float32), br.imgmsg_to_cv2(r, 'bgr8').astype(np.float32), l.header.stamp.sec + 1e-9 * l.header.stamp.nanosec)) if len(buf) < a.k else None)
for k, t, ty in (('cp', '/PSM1/measured_cp', PoseStamped), ('sp', '/PSM1/setpoint_cp', PoseStamped), ('js', '/PSM1/measured_js', JointState),
                 ('jaw', '/PSM1/jaw/measured_js', JointState), ('ecm', '/ECM/measured_js', JointState),
                 ('local', '/PSM1/local/measured_cp', PoseStamped), ('ecm_cp', '/ECM/measured_cp', PoseStamped), ('suj_psm1', '/SUJ/PSM1/measured_js', JointState),
                 ('suj_ecm', '/SUJ/ECM/measured_js', JointState), ('suj_psm1_cp', '/SUJ/PSM1/measured_cp', PoseStamped)):
    n.create_subscription(ty, t, lambda m, k=k: L.__setitem__(k, m), 10)
t0 = time.time()
while (len(buf) < a.k or len(L) < 10) and time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.1)
assert len(buf) >= a.k, 'no frames'
cv2.imwrite(f'{a.out}/left_rect.png', np.mean([b[0] for b in buf], 0).round().astype(np.uint8))
cv2.imwrite(f'{a.out}/right_rect.png', np.mean([b[1] for b in buf], 0).round().astype(np.uint8))
pose = lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w]
st = {'t': time.time(), 'stamps': [b[2] for b in buf], 'n_frames': len(buf), 'cp': pose(L['cp']) if 'cp' in L else None, 'sp': pose(L['sp']) if 'sp' in L else None,
      'cp_frame': L['cp'].header.frame_id if 'cp' in L else None, 'js': list(L['js'].position) if 'js' in L else None,
      'jaw': list(L['jaw'].position) if 'jaw' in L else None, 'ecm_js': list(L['ecm'].position) if 'ecm' in L else None,
      'local_cp': pose(L['local']) if 'local' in L else None, 'ecm_cp': pose(L['ecm_cp']) if 'ecm_cp' in L else None,
      'suj_psm1_js': list(L['suj_psm1'].position) if 'suj_psm1' in L else None, 'suj_ecm_js': list(L['suj_ecm'].position) if 'suj_ecm' in L else None,
      'suj_psm1_cp': pose(L['suj_psm1_cp']) if 'suj_psm1_cp' in L else None}
json.dump(st, open(f'{a.out}/state.json', 'w'), indent=1); print(json.dumps({k: st[k] for k in ('n_frames', 'cp', 'js', 'ecm_js')}))
sys.stdout.flush(); os._exit(0)
