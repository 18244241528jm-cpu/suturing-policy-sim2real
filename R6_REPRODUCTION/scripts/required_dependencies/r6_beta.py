#!/usr/bin/env python3
"""R6 §1.2: largest reachable approach angle beta with ProGrasp (wrist pitch only), hovering >= 10 mm above the needle.

phases:
  goto   : translate (fixed reference orientation, sub-steps <= 3 mm) so the tracked jaw feature sits at calib/needle_region_pre3.json
           'hover_cam_25' (25 mm above the needle middle along the local pad normal); every sub-target must keep the predicted
           feature >= 12 mm above the FS surface under it; then track the feature (10 frames, NCC>=0.5, median) to confirm
  sweep  : wrist_pitch (joint index 4) only, 10 deg per command, both directions from the reference value, until beta stops increasing
           for 2 steps, the joint limit (+-80 deg, keep 5 deg margin), or 60 deg offset; returns to the reference wrist value at the end.
           beta = 90 - angle(tool z in cam, needle-plane normal); tool z rotated to cam by the polar rotation of the R6 affine.
  home   : back to calib base (fixed orientation), sub-steps <= 3 mm"""
import os, sys
if 'AMENT_PREFIX_PATH' not in os.environ:
    import shlex
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && exec python3 '
                       + ' '.join(shlex.quote(x) for x in [os.path.abspath(__file__)] + sys.argv[1:])])
import traceback; sys.excepthook = lambda t, v, tb: (traceback.print_exception(t, v, tb), sys.stdout.flush(), sys.stderr.flush(), os._exit(1))   # no std::terminate core dump (stale FastDDS shm)
import argparse, json, time, threading, numpy as np, cv2, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import message_filters as mf
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, os.path.expanduser('~/suturing-policy-sim2real/scripts/r5_realrobot')); import r5_grasp_v3 as G
ap = argparse.ArgumentParser(); ap.add_argument('--phase', required=True, choices=['goto', 'sweep', 'home']); a = ap.parse_args()
R6 = os.path.expanduser('~/suturing-policy-sim2real/records/logs/R6_20261016'); C = R6 + '/calib'; OUT = R6 + '/beta'; os.makedirs(OUT, exist_ok=True)
FX, CX, CY = G.FX, G.CX, G.CY; FB = G.FB
man = json.load(open(C + '/manifest.json')); cal = json.load(open(C + '/calibration_r6.json')); M = np.array(cal['M']); A3, b3 = M[:3], M[3]
R_cam = G.rotation_from_affine(M)
nr = json.load(open(C + '/needle_region_pre3.json')); n_loc = np.array(nr['local_n']); HOVER = np.array(nr['hover_cam_25'])
qref = man['ref_orientation_xyzw']; base = np.array(man['base_cp_m']) * 1000
cam2cp = lambda X: (np.asarray(X) - b3) @ np.linalg.inv(A3)
dsp = np.load(C + '/pre3/fs_disp.npy'); Zs = FB / np.clip(dsp, 1, None)
def surf_height(X):
    u, v = int(round(FX * X[0] / X[2] + CX)), int(round(FX * X[1] / X[2] + CY))
    if not (5 <= u < 1295 and 5 <= v < 1019): return None
    z = np.median(Zs[v - 3:v + 4, u - 3:u + 4]); S = np.array([(u - CX) * z / FX, (v - CY) * z / FX, z]); return float((X - S) @ n_loc)
LOG = open(OUT + '/beta_log.jsonl', 'a')
def rec(**k): k['t'] = time.time(); k['date'] = time.strftime('%Y-%m-%dT%H:%M:%S%z'); LOG.write(json.dumps(k, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)) + '\n'); LOG.flush()

rclpy.init(); node = rclpy.create_node('r6_beta'); br = CvBridge(); L = {}; grab = {'buf': None, 'n': 0}
subs = [mf.Subscriber(node, Image, f'/jhu_daVinci/{c}/image_rect', qos_profile=qos_profile_sensor_data) for c in ('left', 'right')]
sy = mf.ApproximateTimeSynchronizer(subs, 10, 0.02)
sy.registerCallback(lambda l, r: grab['buf'].append((br.imgmsg_to_cv2(l, 'bgr8').astype(np.float32), br.imgmsg_to_cv2(r, 'bgr8').astype(np.float32))) if grab['buf'] is not None and len(grab['buf']) < grab['n'] else None)
for k, t, ty in (('cp', '/PSM1/measured_cp', PoseStamped), ('sp', '/PSM1/setpoint_cp', PoseStamped), ('js', '/PSM1/measured_js', JointState), ('spj', '/PSM1/setpoint_js', JointState)):
    node.create_subscription(ty, t, lambda msg, k=k: L.__setitem__(k, msg), 10)
node.create_subscription(Bool, '/PSM1/goal_reached', lambda msg: L.__setitem__('reached', msg.data), 10)
pub = node.create_publisher(PoseStamped, '/PSM1/move_cp', 1); pubj = node.create_publisher(JointState, '/PSM1/move_jp', 1)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start(); t0 = time.time()
while not all(k in L for k in ('cp', 'sp', 'js')) and time.time() - t0 < 10: time.sleep(0.05)
assert all(k in L for k in ('cp', 'sp', 'js')), 'no PSM1 state'
Pm = lambda msg: np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) * 1000
Qm = lambda msg: [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
def wait_reached(tmo=10):
    t = time.time()
    while 'reached' not in L and time.time() - t < tmo: time.sleep(0.02)
    time.sleep(0.5); return bool(L.get('reached'))
def move_abs(cp_mm, check=True):
    cur = Pm(L['sp']); d = np.asarray(cp_mm) - cur; k = max(1, int(np.ceil(np.linalg.norm(d) / 3.0)))
    for j in range(1, k + 1):
        tgt = cur + d * j / k; Xp = np.r_[tgt, 1] @ M; h = surf_height(Xp)
        if check: assert h is None or h >= 12.0, f'sub-target too low: {h:.1f} mm'
        msg = PoseStamped(); msg.header.frame_id = L['sp'].header.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, tgt / 1000)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = qref
        L.pop('reached', None); pub.publish(msg)
        if not wait_reached(): rec(ev='not_reached', tgt=tgt); return False
    time.sleep(0.5); return True
def capture(k):
    grab['n'] = k; grab['buf'] = []; t = time.time()
    while len(grab['buf']) < k and time.time() - t < 10: time.sleep(0.02)
    b = grab['buf']; grab['buf'] = None; return b
def save(name, b):
    d = f'{OUT}/{name}'; os.makedirs(d, exist_ok=True)
    cv2.imwrite(d + '/right_rect.png', np.mean([x[1] for x in b], 0).round().astype(np.uint8)); cv2.imwrite(d + '/left_rect.png', np.mean([x[0] for x in b], 0).round().astype(np.uint8))
def beta_now():
    q = Qm(L['cp']); z = R_cam @ Rot.from_quat(q).as_matrix()[:, 2]; ang = float(np.degrees(np.arccos(min(1.0, abs(z @ n_loc)))))
    return {'tool_z_cam': z, 'angle_z_normal_deg': ang, 'beta_deg': 90.0 - ang, 'js': list(L['js'].position)[:6], 'cp_mm': Pm(L['cp']), 'q': q}

if a.phase == 'goto':
    tgt = cam2cp(HOVER); print('hover cam', HOVER.round(1), 'cp', tgt.round(2), 'dist', np.linalg.norm(tgt - Pm(L['sp'])).round(1), flush=True)
    ok = move_abs(tgt); b = capture(10); save('hover', b); r = beta_now(); rec(ev='hover', ok=ok, **r)
    print('reached', ok, 'beta at reference orientation %.1f (angle z-normal %.1f)' % (r['beta_deg'], r['angle_z_normal_deg']), flush=True)
elif a.phase == 'sweep':
    js0 = list(L['js'].position)[:6]; w0 = js0[4]; lim = np.radians(75.0); rows = []
    r = beta_now(); r['offset_deg'] = 0; rows.append(r); save('sweep_p+0', capture(6)); print('offset 0: beta %.1f' % r['beta_deg'], flush=True)
    for sgn in (+1, -1):
        best = r['beta_deg']; worse = 0; off = 0
        while True:
            off += 10
            w = w0 + sgn * np.radians(off)
            if abs(w) > lim or off > 60: print(f'dir {sgn:+d}: stop at offset {off - 10} (limit/60deg)'); break
            msg = JointState(); p = list(L['spj'].position)[:6] if 'spj' in L else js0[:]; p[4] = float(w); msg.position = p
            L.pop('reached', None); pubj.publish(msg); ok = wait_reached(); time.sleep(0.5)
            rr = beta_now(); rr['offset_deg'] = sgn * off; rr['reached'] = ok; rows.append(rr); save(f'sweep_p{sgn * off:+d}', capture(6))
            Xc = np.r_[rr['cp_mm'], 1] @ M; rr['cp_feature_height_mm'] = surf_height(Xc); rec(ev='sweep', **rr)
            print(f"offset {sgn * off:+d}: wrist_pitch {np.degrees(w):.1f} deg  beta {rr['beta_deg']:.1f}  reached {ok}  feature-model height {rr['cp_feature_height_mm']}", flush=True)
            if rr['beta_deg'] > best + 0.5: best = rr['beta_deg']; worse = 0
            else:
                worse += 1
                if worse >= 2: print(f'dir {sgn:+d}: beta stopped increasing'); break
        msg = JointState(); p = list(L['spj'].position)[:6]; p[4] = float(w0); msg.position = p
        for _ in range(int(np.ceil(abs(p[4] - L['js'].position[4]) / np.radians(10)))):   # return in <= 10 deg steps
            cur = L['js'].position[4]; step = np.clip(w0 - cur, -np.radians(10), np.radians(10)); p[4] = float(cur + step); msg.position = p
            L.pop('reached', None); pubj.publish(msg); wait_reached()
        print('back to reference wrist pitch', np.degrees(L['js'].position[4]).round(1), flush=True)
    bmax = max(rows, key=lambda x: x['beta_deg']); json.dump({'rows': rows, 'max': bmax, 'normal': n_loc.tolist(), 'normal_src': 'FS local pad plane around needle middle (pre3)'},
                                                             open(OUT + '/beta_sweep.json', 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
    print('MAX beta %.1f at offset %+d, js %s' % (bmax['beta_deg'], bmax['offset_deg'], np.round(bmax['js'], 4).tolist()))
elif a.phase == 'home':
    print('home', move_abs(base, check=False))
sys.stdout.flush(); os._exit(0)
