#!/usr/bin/env python3
"""R4 day-2 1-b: robot-driven stereo tip session for a LOCAL camera<->PSM1 registration (addendum 1-b).

Based on scripts/r2_realrobot/r2_tip_session.py. Differences:
  * 32 planned poses around the start, sampled once (fixed seed) and saved in the manifest before any move:
      dq1,dq2 so that the tip moves <= +-20 mm laterally (deg = atan(20 / tip-to-RCM distance ~ q3), capped at 8 deg),
      dq3 in [-15, +3] mm (+ = deeper = towards the pad), wrist roll +-60, pitch/yaw +-30 deg.
    Pose 0 = start. Test split = first ceil(25%) of a permutation drawn with numpy default_rng(20260930) (addendum).
  * --dry-run: same trajectory lifted by --lift mm (retracted along the tool), no image recording, for operator check.
  * start / end snapshot of ECM and SUJ readings (precondition check).
Start: operator puts the closed jaw tip ~25-30 mm above the pad, centred on the needle work area.
The only move towards the pad is dq3 <= +3 mm; wrist rotations and dq1/dq2 raise the tip slightly.
usage:  r4d2_tip_session.py --dry-run --lift 15      then      r4d2_tip_session.py --out records/logs/R4_20260928/day2/1b_tip
"""
import os, sys
if 'AMENT_PREFIX_PATH' not in os.environ:
    import shlex
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && exec python3 '
                       + ' '.join(shlex.quote(x) for x in [os.path.abspath(__file__)] + sys.argv[1:])])
import argparse, json, math, time, threading
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import message_filters as mf

NPOSE, PLAN_SEED, SPLIT_SEED = 32, 19300930, 20260930
LIM = dict(lat_mm=20.0, dq12_cap=8.0, dq3_min=-15.0, dq3_max=3.0, roll=60.0, pw=30.0)
SUJ_TOPICS = ('/SUJ/PSM1/measured_js', '/SUJ/ECM/measured_js')


def make_plan(q3_mm, lat_mm=None):
    dq12 = min(math.degrees(math.atan((lat_mm or LIM['lat_mm']) / max(q3_mm, 60.0))), LIM['dq12_cap'])
    r = np.random.default_rng(PLAN_SEED); plan = [(0.0,) * 6]
    for _ in range(NPOSE - 1):
        plan.append(tuple(float(round(x, 2)) for x in (r.uniform(-dq12, dq12), r.uniform(-dq12, dq12), r.uniform(LIM['dq3_min'], LIM['dq3_max']),
                                                        r.uniform(-LIM['roll'], LIM['roll']), r.uniform(-LIM['pw'], LIM['pw']), r.uniform(-LIM['pw'], LIM['pw']))))
    perm = np.random.default_rng(SPLIT_SEED).permutation(NPOSE); test = sorted(int(i) for i in perm[:math.ceil(0.25 * NPOSE)])
    return plan, dq12, test


class Sess(Node):
    def __init__(s, record):
        super().__init__('r4d2_tip_session'); s.br = CvBridge(); s.last = {}; s.grab = None
        if record:
            subs = [mf.Subscriber(s, Image, f'/jhu_daVinci/{c}/{k}', qos_profile=qos_profile_sensor_data)
                    for c, k in (('left', 'image_raw'), ('right', 'image_raw'), ('left', 'image_rect'), ('right', 'image_rect'))]
            s.sy = mf.ApproximateTimeSynchronizer(subs, 10, 0.02); s.sy.registerCallback(s._img)
        for k, t, ty in (('psm_cp', '/PSM1/measured_cp', PoseStamped), ('psm_local', '/PSM1/local/measured_cp', PoseStamped),
                         ('psm_js', '/PSM1/measured_js', JointState), ('ecm_js', '/ECM/measured_js', JointState),
                         ('ecm_cp', '/ECM/measured_cp', PoseStamped), ('jaw', '/PSM1/jaw/measured_js', JointState),
                         ('suj_psm1', SUJ_TOPICS[0], JointState), ('suj_ecm', SUJ_TOPICS[1], JointState)):
            s.create_subscription(ty, t, lambda m, k=k: s.last.__setitem__(k, m), 10)
        s.reached = None; s.create_subscription(Bool, '/PSM1/goal_reached', lambda m: setattr(s, 'reached', m.data), 10)
        s.pub = s.create_publisher(JointState, '/PSM1/move_jp', 1); s.pubjaw = s.create_publisher(JointState, '/PSM1/jaw/move_jp', 1)

    def _img(s, *ms):
        if s.grab is not None and len(s.grab) < s.nwant:
            s.grab.append(([s.br.imgmsg_to_cv2(m, 'bgr8') for m in ms], [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in ms], s.snapshot()))

    def snapshot(s):
        def P(m): p, q = m.pose.position, m.pose.orientation; return {'frame': m.header.frame_id, 'p': [p.x, p.y, p.z], 'q_xyzw': [q.x, q.y, q.z, q.w]}
        L = s.last
        return {'psm_cp': P(L['psm_cp']), 'psm_local': P(L['psm_local']), 'psm_js': list(L['psm_js'].position),
                'ecm_js': list(L['ecm_js'].position), 'ecm_cp': P(L['ecm_cp']), 'jaw': list(L['jaw'].position) if 'jaw' in L else None}

    def fixed_state(s):
        return {'ecm_js': list(s.last['ecm_js'].position), 'suj_psm1_js': list(s.last['suj_psm1'].position), 'suj_ecm_js': list(s.last['suj_ecm'].position), 't': time.time()}

    def record(s, n):
        s.nwant = n; s.grab = []; t0 = time.time()
        while len(s.grab) < n and time.time() - t0 < 10: time.sleep(0.02)
        g = s.grab; s.grab = None; return g

    def move(s, q, timeout=20):
        s.reached = None; m = JointState(); m.position = [float(x) for x in q]; s.pub.publish(m)
        j = JointState(); j.position = [0.0]; s.pubjaw.publish(j)
        t0 = time.time()
        while s.reached is None and time.time() - t0 < timeout: time.sleep(0.02)
        return s.reached


def drift(a, b):
    return {k: float(np.abs(np.array(a[k]) - np.array(b[k])).max()) for k in ('ecm_js', 'suj_psm1_js', 'suj_ecm_js')}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--out'); ap.add_argument('--n', type=int, default=10)
    ap.add_argument('--settle', type=float, default=1.5); ap.add_argument('--dry-run', action='store_true'); ap.add_argument('--lift', type=float, default=15.0)
    ap.add_argument('--only', type=int, nargs='*')
    ap.add_argument('--lat-mm', type=float, default=LIM['lat_mm'], help='lateral tip range; run2 uses 15 (run1 20 left the view)')
    a = ap.parse_args(); assert a.dry_run or a.out, '--out required unless --dry-run'
    rclpy.init(); s = Sess(record=not a.dry_run); threading.Thread(target=rclpy.spin, args=(s,), daemon=True).start(); t0 = time.time()
    need = ('psm_js', 'ecm_js', 'psm_cp', 'psm_local', 'ecm_cp', 'suj_psm1', 'suj_ecm')
    while not all(k in s.last for k in need) and time.time() - t0 < 15: time.sleep(0.1)
    assert all(k in s.last for k in need), f'missing topics: {[k for k in need if k not in s.last]}'
    assert s.pub.get_subscription_count() > 0, 'no subscriber on /PSM1/move_jp'
    q0 = np.array(s.last['psm_js'].position); fs0 = s.fixed_state()
    plan, dq12, test = make_plan(q0[2] * 1000, a.lat_mm)
    print(f'start q (deg/mm): {np.round(np.r_[np.degrees(q0[:2]), q0[2] * 1000, np.degrees(q0[3:])], 2)}; lateral dq1/dq2 limit {dq12:.2f} deg; test poses {test}', flush=True)
    lift = a.lift if a.dry_run else 0.0
    man = {'start_q': q0.tolist(), 'fixed_state_start': fs0, 'plan': plan, 'limits': LIM, 'dq12_deg': dq12, 'lat_mm': a.lat_mm, 'plan_seed': PLAN_SEED,
           'split_seed': SPLIT_SEED, 'test_poses': test, 'dry_run': a.dry_run, 'lift_mm': lift, 'poses': []}
    if a.out:
        os.makedirs(a.out, exist_ok=True); json.dump(man, open(os.path.join(a.out, 'manifest.json'), 'w'), indent=1)
    for i, dq in enumerate(plan):
        if a.only is not None and i not in a.only: continue
        q = q0.copy(); q[0] += math.radians(dq[0]); q[1] += math.radians(dq[1]); q[2] += (dq[2] - lift) / 1000
        q[3] += math.radians(dq[3]); q[4] += math.radians(dq[4]); q[5] += math.radians(dq[5])
        ok = s.move(q); time.sleep(a.settle)
        qm = np.array(s.last['psm_js'].position); err = float(np.abs(qm - q).max())
        rec = {'i': i, 'dq': dq, 'q_cmd': q.tolist(), 'q_meas': qm.tolist(), 'q_err_max': err, 'goal_reached': ok, 'fixed_drift': drift(fs0, s.fixed_state())}
        if not a.dry_run:
            g = s.record(a.n); d = os.path.join(a.out, f'pose_{i:02d}'); os.makedirs(d, exist_ok=True)
            for k, (ims, st, snap) in enumerate(g):
                for nm, im in zip(('left_raw', 'right_raw', 'left_rect', 'right_rect'), ims):
                    os.makedirs(os.path.join(d, nm), exist_ok=True); cv2.imwrite(os.path.join(d, nm, f'{k:03d}.png'), im)
            rec.update(n_frames=len(g), frames=[{'stamps': st, **snap} for _, st, snap in g])
            json.dump(rec, open(os.path.join(d, 'pose.json'), 'w'), indent=1)
        man['poses'].append({k: rec[k] for k in ('i', 'dq', 'q_err_max', 'goal_reached', 'fixed_drift') if k in rec} | ({'n_frames': rec['n_frames']} if 'n_frames' in rec else {}))
        if a.out: json.dump(man, open(os.path.join(a.out, 'manifest.json'), 'w'), indent=1)
        print(f"pose {i:02d}{' (test)' if i in test else ''} dq={dq} q_err_max={err:.4f} reached={ok} drift={rec['fixed_drift']}", flush=True)
    s.move(q0); time.sleep(a.settle); man['fixed_state_end'] = s.fixed_state(); man['fixed_drift_end'] = drift(fs0, man['fixed_state_end'])
    if a.out: json.dump(man, open(os.path.join(a.out, 'manifest.json'), 'w'), indent=1)
    print('returned to start; fixed drift', man['fixed_drift_end']); rclpy.shutdown()


if __name__ == '__main__':
    main()
