#!/usr/bin/env python3
"""R4 day-2: fixed-wrist translation-grid calibration  camera(rect physical-left) <- PSM1 measured_cp frame (ECM).
Wrist orientation is held at the start orientation; PSM1 visits a 3x3x3 grid of offsets from the start setpoint (x,y in {-8,0,8} mm,
z in {-4,-10,-16} mm; in the ECM frame -z is towards the camera, i.e. away from the pad; no offset goes towards the pad).
At each point: 10 synced rect frames (mean saved) + measured_cp/local/js. The start pose is revisited every 9 points.
Registration is valid only for this wrist orientation (the tracked jaw feature absorbs the TCP).
usage: r4d2_grid_calib.py --out DIR  (capture)      r4d2_grid_calib.py --out DIR --solve --u U --v V [--h 30]  (fit)"""
import os, sys
if 'AMENT_PREFIX_PATH' not in os.environ:
    import shlex
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && exec python3 '
                       + ' '.join(shlex.quote(x) for x in [os.path.abspath(__file__)] + sys.argv[1:])])
import argparse, json, math, time, threading, numpy as np, cv2
ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--solve', action='store_true')
ap.add_argument('--u', type=int); ap.add_argument('--v', type=int); ap.add_argument('--h', type=int, default=30); a = ap.parse_args()
FX, CX, CY = 1860.724932, 690.538345, 576.803638; FB = FX * 5.4476353950418295
GRID = [(x, y, z) for z in (-4, -10, -16) for y in (-8, 0, 8) for x in (-8, 0, 8)]
SPLIT_SEED = 20260929

def capture():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool
    from cv_bridge import CvBridge
    import message_filters as mf
    os.makedirs(a.out, exist_ok=True)
    rclpy.init(); n = rclpy.create_node('r4d2_grid_calib'); br = CvBridge(); L = {}; grab = {'buf': None, 'n': 0}
    subs = [mf.Subscriber(n, Image, f'/jhu_daVinci/{c}/image_rect', qos_profile=qos_profile_sensor_data) for c in ('left', 'right')]
    sy = mf.ApproximateTimeSynchronizer(subs, 10, 0.02)
    def cb(l, r):
        if grab['buf'] is not None and len(grab['buf']) < grab['n']: grab['buf'].append((br.imgmsg_to_cv2(l, 'bgr8').astype(np.float32), br.imgmsg_to_cv2(r, 'bgr8').astype(np.float32)))
    sy.registerCallback(cb)
    for k, t, ty in (('cp', '/PSM1/measured_cp', PoseStamped), ('sp', '/PSM1/setpoint_cp', PoseStamped), ('local', '/PSM1/local/measured_cp', PoseStamped),
                     ('js', '/PSM1/measured_js', JointState), ('ecm', '/ECM/measured_js', JointState)):
        n.create_subscription(ty, t, lambda m, k=k: L.__setitem__(k, m), 10)
    n.create_subscription(Bool, '/PSM1/goal_reached', lambda m: L.__setitem__('reached', m.data), 10)
    pub = n.create_publisher(PoseStamped, '/PSM1/move_cp', 1)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start(); t0 = time.time()
    while not all(k in L for k in ('cp', 'sp', 'local', 'js', 'ecm')) and time.time() - t0 < 10: time.sleep(0.05)
    assert pub.get_subscription_count() > 0
    P = lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z]; Q = lambda m: [m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w]
    sp0 = L['sp']; p0 = np.array(P(sp0)); q0 = Q(sp0); ecm0 = np.array(L['ecm'].position)
    def move(dp):
        m = PoseStamped(); m.header.frame_id = sp0.header.frame_id; g = p0 + np.array(dp) / 1000.0
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, g)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q0
        L.pop('reached', None); pub.publish(m); t = time.time()
        while 'reached' not in L and time.time() - t < 10: time.sleep(0.02)
        time.sleep(1.2); return L.get('reached')
    def record(name):
        grab['n'] = 10; grab['buf'] = []; t = time.time()
        while len(grab['buf']) < 10 and time.time() - t < 15: time.sleep(0.02)
        b = grab['buf']; grab['buf'] = None; d = f'{a.out}/{name}'; os.makedirs(d, exist_ok=True)
        cv2.imwrite(f'{d}/left_rect_mean.png', np.mean([x[0] for x in b], 0).round().astype(np.uint8)); cv2.imwrite(f'{d}/right_rect_mean.png', np.mean([x[1] for x in b], 0).round().astype(np.uint8))
        st = {'n_frames': len(b), 'cp_p': P(L['cp']), 'cp_q': Q(L['cp']), 'cp_frame': L['cp'].header.frame_id, 'local_p': P(L['local']), 'local_q': Q(L['local']),
              'js': list(L['js'].position), 'ecm_js': list(L['ecm'].position), 'ecm_drift': float(np.abs(np.array(L['ecm'].position) - ecm0).max()), 't': time.time()}
        json.dump(st, open(f'{d}/state.json', 'w'), indent=1); return st
    man = {'p0_setpoint': p0.tolist(), 'q0': q0, 'frame': sp0.header.frame_id, 'grid_mm': GRID, 'points': []}
    seq = []
    for i, g in enumerate(GRID):
        if i % 9 == 0: seq.append((f'base_{i // 9}', (0, 0, 0)))
        seq.append((f'g{i:02d}', g))
    seq.append(('base_end', (0, 0, 0)))
    # go up first before any lateral move: first visit (0,0,-4)
    move((0, 0, -4))
    for name, g in seq:
        if name.startswith('base'): move((0, 0, -4)); ok = move((0, 0, 0))
        else: ok = move(g)
        st = record(name); man['points'].append({'name': name, 'cmd_mm': list(g), 'reached': ok, 'ecm_drift': st['ecm_drift']})
        print(name, g, 'reached', ok, 'cp-p0', ((np.array(st['cp_p']) - p0) * 1000).round(2), 'ecm drift %.1e' % st['ecm_drift'], flush=True)
        json.dump(man, open(f'{a.out}/manifest.json', 'w'), indent=1)
    move((0, 0, -4)); move((0, 0, 0)); rclpy.shutdown()

def solve():
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation as Rot
    man = json.load(open(f'{a.out}/manifest.json')); names = [p['name'] for p in man['points']]
    R0 = cv2.imread(f'{a.out}/base_0/right_rect_mean.png', 0).astype(np.float32); h = a.h
    T = R0[a.v - h:a.v + h + 1, a.u - h:a.u + h + 1]
    def sub1(r, j):
        if 0 < j < len(r) - 1:
            p, q, s = r[j - 1], r[j], r[j + 1]; den = p - 2 * q + s; return j + (0.5 * (p - s) / den if den < 0 else 0)
        return float(j)
    pts = {}
    for nm in names:
        R = cv2.imread(f'{a.out}/{nm}/right_rect_mean.png', 0).astype(np.float32); Lm = cv2.imread(f'{a.out}/{nm}/left_rect_mean.png', 0).astype(np.float32)
        W = 260; x0, y0 = max(a.u - h - W, 0), max(a.v - h - W, 0); win = R[y0:min(a.v + h + 1 + W, 1024), x0:min(a.u + h + 1 + W, 1300)]
        r = cv2.matchTemplate(win, T, cv2.TM_CCOEFF_NORMED); _, mx, _, (lx, ly) = cv2.minMaxLoc(r)
        u = x0 + sub1(r[ly], lx) + h; v = y0 + sub1(r[:, lx], ly) + h
        yi, xi = int(round(v)), int(round(u)); T2 = R[yi - h:yi + h + 1, xi - h:xi + h + 1]
        lo = max(xi - h - 260, 0); strip = Lm[yi - h:yi + h + 1, lo:xi + h + 1 - 30]
        r2 = cv2.matchTemplate(strip, T2, cv2.TM_CCOEFF_NORMED)[0]; j = int(np.argmax(r2)); xl = lo + sub1(r2, j) + h
        d = (xi - xl) + (u - xi); Z = FB / d
        st = json.load(open(f'{a.out}/{nm}/state.json'))
        pts[nm] = {'uv': [u, v], 'ncc': float(mx), 'ncc_disp': float(r2[j]), 'disp': float(d), 'X_cam_mm': [(u - CX) * Z / FX, (v - CY) * Z / FX, Z], 'cp_mm': (np.array(st['cp_p']) * 1000).tolist()}
    grid = [nm for nm in names if nm.startswith('g')]; bases = [nm for nm in names if nm.startswith('base')]
    perm = np.random.default_rng(SPLIT_SEED).permutation(len(grid)); test = sorted(grid[i] for i in perm[:math.ceil(0.25 * len(grid))]); train = [g for g in grid if g not in test] + bases
    Xc = lambda ns: np.array([pts[k]['X_cam_mm'] for k in ns]); Xk = lambda ns: np.array([pts[k]['cp_mm'] for k in ns])
    def umeyama(A, B, scale):
        ma, mb = A.mean(0), B.mean(0); U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma) / len(A)); D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(U @ Vt))
        R = U @ D @ Vt; s = (S * np.diag(D)).sum() / ((A - ma) ** 2).sum() * len(A) if scale else 1.0; return s, R, mb - s * R @ ma
    out = {'template_uv_h': [a.u, a.v, h], 'train': train, 'test': test, 'points': pts, 'models': {}}
    for name, sc in (('SE3', False), ('Sim3', True)):
        s, R, t = umeyama(Xk(train), Xc(train), sc)
        e_tr = np.linalg.norm((s * (R @ Xk(train).T)).T + t - Xc(train), axis=1); e_te = (s * (R @ Xk(test).T)).T + t - Xc(test)
        # affine (12 dof) as a check of anisotropic scale
        A = np.c_[Xk(train), np.ones(len(train))]; M = np.linalg.lstsq(A, Xc(train), rcond=None)[0]; e_aff = np.c_[Xk(test), np.ones(len(test))] @ M - Xc(test)
        out['models'][name] = {'scale': s, 'R': R.tolist(), 't_mm': t.tolist(), 'train_rms_mm': float(np.sqrt((e_tr ** 2).mean())), 'train_max_mm': float(e_tr.max()),
                               'test_rms_mm': float(np.sqrt((np.linalg.norm(e_te, axis=1) ** 2).mean())), 'test_max_mm': float(np.linalg.norm(e_te, axis=1).max()),
                               'test_err_vec_mm': dict(zip(test, e_te.round(3).tolist())), 'rot_deg_from_diag(-1,-1,1)': float(np.degrees(Rot.from_matrix(R @ np.diag([-1, -1, 1])).magnitude()))}
    out['models']['affine_check'] = {'test_rms_mm': float(np.sqrt((np.linalg.norm(e_aff, axis=1) ** 2).mean())), 'singular_values': np.linalg.svd(M[:3]).S.tolist()}
    bX = Xc(bases); out['base_repeatability_mm'] = float(np.linalg.norm(bX - bX.mean(0), axis=1).max())
    json.dump(out, open(f'{a.out}/calibration.json', 'w'), indent=1)
    for k, m in out['models'].items(): print(k, {kk: (round(v, 3) if isinstance(v, float) else v) for kk, v in m.items() if kk in ('scale', 'train_rms_mm', 'train_max_mm', 'test_rms_mm', 'test_max_mm', 'singular_values', 'rot_deg_from_diag(-1,-1,1)')})
    print('base repeatability max %.2f mm; min ncc %.2f / %.2f; test %s' % (out['base_repeatability_mm'], min(p['ncc'] for p in pts.values()), min(p['ncc_disp'] for p in pts.values()), test))

solve() if a.solve else capture()
