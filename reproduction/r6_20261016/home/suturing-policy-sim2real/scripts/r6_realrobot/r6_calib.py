#!/usr/bin/env python3
"""R6 on-site quick calibration (user-approved deviation D1), fixed wrist = today's reference orientation.

phases:
  manifest : write calib/manifest.json BEFORE any motion: reference orientation, tracked feature template (from calib/pre3),
             pad plane (FS on pre3), 3 probe moves (cp frame, +-2 mm, return), 12 grid points in PAD frame
             (a, b in [-8, 8] mm in-plane, h in [0, +8] mm along the pad normal, away from pad), seed 20261016; 3 random test points.
  probe    : 3 probe moves of 2 mm along cp x / y / z (each returns to base); track feature -> rough Jacobian J (cam mm per cp mm)
  grid     : convert pad-frame grid to cp with J^-1, visit each point (sub-steps <= 3 mm), track feature, return to base at the end
  fit      : affine cam <- cp from train points; report test errors (gate: every test error <= 3 mm)
Safety: per command translation <= 3 mm, fixed orientation, predicted feature height above the pad plane >= 8 mm, abort on low NCC."""
import os, sys
if 'AMENT_PREFIX_PATH' not in os.environ:
    import shlex
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && exec python3 '
                       + ' '.join(shlex.quote(x) for x in [os.path.abspath(__file__)] + sys.argv[1:])])
import traceback; sys.excepthook = lambda t, v, tb: (traceback.print_exception(t, v, tb), sys.stdout.flush(), sys.stderr.flush(), os._exit(1))   # no std::terminate core dump (stale FastDDS shm)
import argparse, json, time, numpy as np, cv2
ap = argparse.ArgumentParser(); ap.add_argument('--phase', required=True, choices=['manifest', 'probe', 'grid', 'fit', 'remeasure', 'check', 'check2']); ap.add_argument('--feat', type=int, nargs=2); ap.add_argument('--half', type=int); ap.add_argument('--point', type=int, default=11); a = ap.parse_args()
ROOT = os.path.expanduser('~/suturing-policy-sim2real/records/logs/R6_20261016/calib'); MAN = ROOT + '/manifest.json'
FX, CX, CY = 1860.724932, 690.538345, 576.803638; FB = FX * 5.4476353950418295
FEAT_UV = (382, 352); HALF = 35; MIN_H = 8.0; STEP = 3.0

def cam_points(disp):
    Z = FB / np.clip(disp, 1, None); H, W = disp.shape; v, u = np.mgrid[0:H, 0:W]; return np.dstack([(u - CX) * Z / FX, (v - CY) * Z / FX, Z])

if a.phase == 'manifest':
    assert not os.path.exists(MAN), 'manifest already written (read-only)'
    st = json.load(open(ROOT + '/pre3/state.json')); P = cam_points(np.load(ROOT + '/pre3/fs_disp.npy'))
    Q = P[400:500, 350:600].reshape(-1, 3)[::5]; rng = np.random.default_rng(0); best = None
    for _ in range(300):
        s = Q[rng.choice(len(Q), 3, replace=False)]; n = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(n) < 1e-9: continue
        n /= np.linalg.norm(n); k = (np.abs((Q - s[0]) @ n) < 0.5).sum()
        if best is None or k > best[0]: best = (k, n, s[0])
    I = Q[np.abs((Q - best[2]) @ best[1]) < 0.5]; c = I.mean(0); n = np.linalg.svd(I - c)[2][2]; n = n if n[2] < 0 else -n
    e1 = np.cross(n, [0, 0, 1.0]); e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)
    rg = np.random.default_rng(20261016); pts = np.c_[rg.uniform(-8, 8, (12, 2)), rg.uniform(0, 8, 12)]; test = sorted(rg.choice(12, 3, replace=False).tolist())
    feat = np.median(P[FEAT_UV[1] - 3:FEAT_UV[1] + 4, FEAT_UV[0] - 3:FEAT_UV[0] + 4].reshape(-1, 3), 0)
    m = {'created': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'deviation': 'D1 on-site quick calibration (user approved ~11:07); R4 calibration_v3 invalid after re-setup',
         'ref_orientation_xyzw': st['cp'][3:], 'base_cp_m': st['cp'][:3], 'base_js': st['js'], 'ecm_js': st['ecm_js'],
         'feature': {'uv_phys_left': FEAT_UV, 'half': HALF, 'template_src': 'calib/pre3/right_rect.png', 'cam_mm_fs': feat.tolist(),
                     'note': 'feature = jaw centre near the tip; fixed orientation so any rigid jaw point works for the affine'},
         'pad_plane': {'normal_toward_cam': n.tolist(), 'centre_cam_mm': c.tolist(), 'src': 'FS on calib/pre3, box rows 400-500 cols 350-600',
                       'feature_height_mm': float((feat - c) @ n)},
         'pad_frame': {'e1': e1.tolist(), 'e2': e2.tolist(), 'up': n.tolist()},
         'probe_cp_mm': [[2, 0, 0], [0, 2, 0], [0, 0, 2]],
         'grid_pad_mm': pts.round(2).tolist(), 'test_idx': test, 'seed': 20261016,
         'rules': {'max_step_mm': STEP, 'min_height_above_pad_mm': MIN_H, 'gate_test_err_mm': 3.0, 'time_cap_min': 30}}
    json.dump(m, open(MAN, 'w'), indent=1); os.chmod(MAN, 0o444); print(json.dumps({k: m[k] for k in ('pad_plane', 'grid_pad_mm', 'test_idx')}, indent=0)); sys.exit()

m = json.load(open(MAN)); n_up = np.array(m['pad_plane']['normal_toward_cam']); c_pad = np.array(m['pad_plane']['centre_cam_mm'])
LOG = open(ROOT + '/calib_log.jsonl', 'a')
def rec(**k): k['t'] = time.time(); k['date'] = time.strftime('%Y-%m-%dT%H:%M:%S%z'); LOG.write(json.dumps(k, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)) + '\n'); LOG.flush()

if a.phase == 'fit':
    R = [json.loads(l) for l in open(ROOT + '/calib_log.jsonl')]; G = [r for r in R if r.get('ev') == 'grid_point' and r.get('ok')]
    tr = [g for g in G if g['i'] not in m['test_idx']]; te = [g for g in G if g['i'] in m['test_idx']]
    X = np.array([np.r_[g['cp_mm'], 1] for g in tr]); Y = np.array([g['X'] for g in tr]); M = np.linalg.lstsq(X, Y, rcond=None)[0]
    res = lambda gs: [float(np.linalg.norm(np.r_[g['cp_mm'], 1] @ M - g['X'])) for g in gs]
    sv = np.linalg.svd(M[:3])[1]; out = {'n_train': len(tr), 'n_test': len(te), 'M': M.tolist(), 'sv': sv.tolist(), 'train_rms': float(np.sqrt(np.mean(np.square(res(tr))))),
                                          'test_err_mm': dict(zip([g['i'] for g in te], res(te))), 'gate_pass': bool(len(te) == 3 and max(res(te)) <= 3.0)}
    json.dump(out, open(ROOT + '/calibration_r6.json', 'w'), indent=1); print(json.dumps({k: v for k, v in out.items() if k != 'M'}, indent=1)); sys.exit()

import rclpy, threading
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import message_filters as mf
rclpy.init(); node = rclpy.create_node('r6_calib'); br = CvBridge(); L = {}; grab = {'buf': None, 'n': 0}
subs = [mf.Subscriber(node, Image, f'/jhu_daVinci/{c}/image_rect', qos_profile=qos_profile_sensor_data) for c in ('left', 'right')]
sy = mf.ApproximateTimeSynchronizer(subs, 10, 0.02)
sy.registerCallback(lambda l, r: grab['buf'].append((br.imgmsg_to_cv2(l, 'bgr8').astype(np.float32), br.imgmsg_to_cv2(r, 'bgr8').astype(np.float32))) if grab['buf'] is not None and len(grab['buf']) < grab['n'] else None)
for k, t, ty in (('cp', '/PSM1/measured_cp', PoseStamped), ('sp', '/PSM1/setpoint_cp', PoseStamped)):
    node.create_subscription(ty, t, lambda msg, k=k: L.__setitem__(k, msg), 10)
node.create_subscription(Bool, '/PSM1/goal_reached', lambda msg: L.__setitem__('reached', msg.data), 10)
pub = node.create_publisher(PoseStamped, '/PSM1/move_cp', 1)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start(); t0 = time.time()
while not all(k in L for k in ('cp', 'sp')) and time.time() - t0 < 10: time.sleep(0.05)
assert all(k in L for k in ('cp', 'sp')), 'no PSM1 state'
Pm = lambda msg: np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) * 1000
qref = m['ref_orientation_xyzw']; base = np.array(m['base_cp_m']) * 1000

def move_abs(cp_mm):
    """sub-stepped absolute move, each command <= STEP mm, fixed reference orientation"""
    cur = Pm(L['sp']); d = np.asarray(cp_mm) - cur; k = max(1, int(np.ceil(np.linalg.norm(d) / STEP)))
    for j in range(1, k + 1):
        tgt = cur + d * j / k; msg = PoseStamped(); msg.header.frame_id = L['sp'].header.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, tgt / 1000)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = qref
        L.pop('reached', None); pub.publish(msg); t = time.time()
        while 'reached' not in L and time.time() - t < 10: time.sleep(0.02)
        time.sleep(0.4)
        if not L.get('reached'): rec(ev='move_not_reached', tgt=tgt); return False
    time.sleep(0.6); return True

def capture(k=6):
    grab['n'] = k; grab['buf'] = []; t = time.time()
    while len(grab['buf']) < k and time.time() - t < 10: time.sleep(0.02)
    b = grab['buf']; grab['buf'] = None; return np.mean([x[0] for x in b], 0), np.mean([x[1] for x in b], 0)

B0 = cv2.cvtColor(cv2.imread(ROOT + '/pre3/right_rect.png'), cv2.COLOR_BGR2GRAY).astype(np.float32)
B0L = cv2.cvtColor(cv2.imread(ROOT + '/pre3/left_rect.png'), cv2.COLOR_BGR2GRAY).astype(np.float32)
U0, V0 = a.feat or m['feature']['uv_phys_left']; HALF = a.half or HALF; T0 = B0[V0 - HALF:V0 + HALF + 1, U0 - HALF:U0 + HALF + 1]
D0 = float(np.load(ROOT + '/pre3/fs_disp.npy')[V0, U0]); UL0 = int(round(U0 - D0))   # feature in the topic-left image located once with FS
T0L = B0L[V0 - HALF:V0 + HALF + 1, UL0 - HALF:UL0 + HALF + 1]
def sub1(r, j):
    if 0 < j < len(r) - 1:
        p, q, s = r[j - 1], r[j], r[j + 1]; den = p - 2 * q + s; return j + (0.5 * (p - s) / den if den < 0 else 0)
    return float(j)
def match(img, T, cu, cv_, rad=120):
    best = None
    for s in (0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.15):
        Ts = cv2.resize(T, None, fx=s, fy=s); th, tw = Ts.shape
        x0, y0 = int(max(cu - rad - tw / 2, 0)), int(max(cv_ - rad - th / 2, 0)); x1, y1 = int(min(cu + rad + tw / 2, 1300)), int(min(cv_ + rad + th / 2, 1024))
        r = cv2.matchTemplate(img[y0:y1, x0:x1], Ts, cv2.TM_CCOEFF_NORMED); _, mx, _, (lx, ly) = cv2.minMaxLoc(r)
        if best is None or mx > best[0]: best = (mx, x0 + sub1(r[ly], lx) + tw / 2 - 0.5, y0 + sub1(r[:, lx], ly) + th / 2 - 0.5)
    return best
def track(name, pred_uv, pred_disp=None):
    """per-view temporal template tracking (same viewpoint -> high NCC); disparity = u(phys-left) - u(phys-right)"""
    Lm, Rm = capture(); d = f'{ROOT}/pts/{name}'; os.makedirs(d, exist_ok=True)
    cv2.imwrite(d + '/right_rect.png', Rm.round().astype(np.uint8)); cv2.imwrite(d + '/left_rect.png', Lm.round().astype(np.uint8))
    R = cv2.cvtColor(Rm.astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(np.float32); Lg = cv2.cvtColor(Lm.astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(np.float32)
    mx, u, v = match(R, T0, *pred_uv); pd = pred_disp if pred_disp is not None else D0
    m2, uL, vL = match(Lg, T0L, u - pd, v, rad=60); dsp = u - uL; Z = FB / dsp
    X = np.array([(u - CX) * Z / FX, (v - CY) * Z / FX, Z]); h = float((X - c_pad) @ n_up)
    rec(ev='track', name=name, uv=[u, v], uvL=[uL, vL], dv=v - vL, pred_uv=pred_uv, ncc=mx, ncc_d=m2, disp=dsp, X=X, h_pad=h, cp=Pm(L['cp']), sp=Pm(L['sp']))
    return X, mx, m2, h
proj = lambda X: (FX * X[0] / X[2] + CX, FX * X[1] / X[2] + CY)

if a.phase == 'probe':
    assert np.linalg.norm(Pm(L['sp']) - base) < 1.0, f'not at base: {Pm(L["sp"]) - base}'
    X0, c1, c2, h0 = track('probe_base', m['feature']['uv_phys_left']); print('base', X0.round(2), f'ncc {c1:.2f}/{c2:.2f} h {h0:.1f}', flush=True)
    assert c1 > 0.6 and c2 > 0.6, 'low NCC at base'
    J = np.zeros((3, 3))
    for i, dv in enumerate(m['probe_cp_mm']):
        assert move_abs(base + dv), 'probe move failed'; X, c1, c2, h = track(f'probe_{i}', proj(X0)); J[:, i] = (X - X0) / 2.0
        print(f'probe {i} {dv}: dX {(X - X0).round(2)} ncc {c1:.2f}/{c2:.2f} h {h:.1f}', flush=True); assert c1 > 0.5 and c2 > 0.5, 'low NCC'
        assert move_abs(base), 'return failed'
    rec(ev='probe_J', J=J, X0=X0, sv=np.linalg.svd(J)[1]); json.dump({'J': J.tolist(), 'X0': X0.tolist()}, open(ROOT + '/probe_J.json', 'w'), indent=1)
    print('J', J.round(3), 'sv', np.linalg.svd(J)[1].round(3))
elif a.phase == 'grid':
    pj = json.load(open(ROOT + '/probe_J.json')); J = np.array(pj['J']); X0 = np.array(pj['X0']); Ji = np.linalg.inv(J)
    E = np.c_[m['pad_frame']['e1'], m['pad_frame']['e2'], m['pad_frame']['up']]
    for i, p in enumerate(m['grid_pad_mm']):
        dcam = E @ np.array(p); Xp = X0 + dcam; hp = float((Xp - c_pad) @ n_up)
        if hp < MIN_H: rec(ev='grid_skip', i=i, reason=f'pred height {hp:.1f} < {MIN_H}'); print('skip', i, hp); continue
        tgt = base + Ji @ dcam; ok = move_abs(tgt)
        X, c1, c2, h = track(f'g{i:02d}', proj(Xp)); good = bool(ok and c1 > 0.5 and c2 > 0.5)
        rec(ev='grid_point', i=i, pad=p, cp_mm=Pm(L['cp']), sp_mm=Pm(L['sp']), X=X, pred_X=Xp, ncc=[c1, c2], h_pad=h, ok=good, test=i in m['test_idx'])
        print(f'g{i:02d} pad {np.round(p, 1)} X {X.round(2)} pred_err {np.linalg.norm(X - Xp):.2f} ncc {c1:.2f}/{c2:.2f} h {h:.1f} ok {good}', flush=True)
        if h < MIN_H - 2: print('too low, abort'); break
    move_abs(base); X, c1, c2, h = track('grid_end_base', proj(X0)); print('back at base', X.round(2), 'drift', np.linalg.norm(X - X0).round(2))
elif a.phase == 'remeasure':
    # user decision 11:2x: re-measure one test point with a jaw-only template; the feature changes, so re-shoot the base and carry the constant offset
    cal = json.load(open(ROOT + '/calibration_r6.json')); M = np.array(cal['M']); G = {r['i']: r for r in map(json.loads, open(ROOT + '/calib_log.jsonl')) if r.get('ev') == 'grid_point'}
    assert np.linalg.norm(Pm(L['sp']) - base) < 1.5, 'not at base'
    out = {}
    for tag in ('base_before', 'point', 'base_after'):
        if tag == 'point': assert move_abs(np.array(G[a.point]['sp_mm'])), 'move failed'
        if tag == 'base_after': assert move_abs(base), 'return failed'
        Xs = []
        for k in range(3):
            X, c1, c2, h = track(f'remeasure_{tag}_{k}', (U0, V0) if tag != 'point' else proj(np.r_[G[a.point]['cp_mm'], 1] @ M + (out['offset'] if 'offset' in out else 0)))
            Xs.append((X, c1, c2, h, Pm(L['cp'])))
        good = [x for x in Xs if x[1] >= 0.5 and x[2] >= 0.5]; out[tag] = {'n_good': len(good), 'ncc': [[x[1], x[2]] for x in Xs], 'X': [x[0] for x in Xs]}
        if tag == 'base_before':
            assert good, 'base tracking failed with the new template'
            out['offset'] = np.median([x[0] - np.r_[x[4], 1] @ M for x in good], 0)
        else:
            out[tag]['err_mm'] = [float(np.linalg.norm(np.r_[x[4], 1] @ M + out['offset'] - x[0])) for x in good]
        print(tag, {k: v for k, v in out[tag].items() if k != 'X'}, flush=True)
    rec(ev='remeasure', point_i=a.point, feat=[U0, V0], half=HALF, result=out)
if a.phase == 'check2':
    # after a restart: lift 15 mm along tool z, joint-interpolate to base_js, fresh jaw template at base (left by NCC in a +-6 px band around the
    # affine-predicted disparity), then g04, g08, base; errors relative to affine + base offset
    from sensor_msgs.msg import JointState as JS
    from scipy.spatial.transform import Rotation as Rot
    pubj = node.create_publisher(JS, '/PSM1/move_jp', 1); JL = {}
    node.create_subscription(JS, '/PSM1/measured_js', lambda m_: JL.__setitem__('j', list(m_.position)[:6]), 10); time.sleep(1.0)
    q = [L['sp'].pose.orientation.x, L['sp'].pose.orientation.y, L['sp'].pose.orientation.z, L['sp'].pose.orientation.w]; z = Rot.from_quat(q).as_matrix()[:, 2]
    p0 = Pm(L['sp'])
    for k in range(1, 6):
        tg = p0 - z * 15.0 * k / 5; msg = PoseStamped(); msg.header.frame_id = L['sp'].header.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, tg / 1000); msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        L.pop('reached', None); pub.publish(msg); t = time.time()
        while 'reached' not in L and time.time() - t < 10: time.sleep(0.02)
        time.sleep(0.3)
    j0 = np.array(JL['j']); jb = np.array(m['base_js'])
    for k in range(1, 6):
        msg = JS(); msg.position = (j0 + (jb - j0) * k / 5).tolist(); L.pop('reached', None); pubj.publish(msg); t = time.time()
        while 'reached' not in L and time.time() - t < 10: time.sleep(0.02)
        time.sleep(0.5)
    assert move_abs(base), 'base move failed'
    cal = json.load(open(ROOT + '/calibration_r6.json')); M2 = np.array(cal['M']); G2 = {r['i']: r for r in map(json.loads, open(ROOT + '/calib_log.jsonl')) if r.get('ev') == 'grid_point'}
    Lm, Rm = capture(10); Rg = cv2.cvtColor(Rm.astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(np.float32); Lg = cv2.cvtColor(Lm.astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(np.float32)
    od = f'{ROOT}/check2'; os.makedirs(od, exist_ok=True); cv2.imwrite(od + '/base_right.png', Rm.round().astype(np.uint8)); cv2.imwrite(od + '/base_left.png', Lm.round().astype(np.uint8))
    fu, fv = a.feat or (372, 335); hf = a.half or 16; TRn = Rg[fv - hf:fv + hf + 1, fu - hf:fu + hf + 1]
    Xb = np.r_[Pm(L['cp']), 1] @ M2; dpred = FB / Xb[2]; best = None
    for dd in np.arange(dpred - 6, dpred + 6.01, 0.5):
        x = int(round(fu - dd)); r = cv2.matchTemplate(Lg[fv - hf - 2:fv + hf + 3, x - hf - 2:x + hf + 3], TRn, cv2.TM_CCOEFF_NORMED); _, mx, _, (lx, ly) = cv2.minMaxLoc(r)
        if best is None or mx > best[0]: best = (mx, x - 2 + lx, fv - 2 + ly)
    mxs, xl, yl = best; TLn = Lg[yl - hf:yl + hf + 1, xl - hf:xl + hf + 1]
    pass
    T0, T0L, U0, V0, D0 = TRn, TLn, fu, fv, float(fu - xl)
    print('fresh template stereo NCC %.2f disp %.1f (pred %.1f)' % (mxs, D0, dpred), flush=True)
    out = {'template_stereo_ncc': float(mxs)}; Xs = {}
    for name, tgt in (('base', base), ('g04', np.array(G2[4]['sp_mm'])), ('g08', np.array(G2[8]['sp_mm'])), ('base_end', base)):
        assert move_abs(tgt), 'move failed'; pred = np.r_[Pm(L['cp']), 1] @ M2
        X, c1, c2, h = track(f'check2_{name}', (U0, V0) if name.startswith('base') else proj(pred + out.get('offset', 0)), pred_disp=None)
        if name == 'base': out['offset'] = X - pred
        e = float(np.linalg.norm(X - pred - out['offset'])); out[name] = {'err_mm': e, 'ncc': [c1, c2]}; print(name, 'err %.2f mm ncc %.2f/%.2f' % (e, c1, c2), flush=True)
    out['offset'] = out['offset'].tolist(); rec(ev='check2_after_restart', result=out); json.dump(out, open(ROOT + '/check2_after_restart2.json', 'w'), indent=1)
if a.phase == 'check':
    # after a dvrk_system restart: joint-interpolate to base_js (5 sub-steps), then base + test points g04/g08/g11 with the original template
    from sensor_msgs.msg import JointState as JS
    pubj = node.create_publisher(JS, '/PSM1/move_jp', 1); JL = {}
    node.create_subscription(JS, '/PSM1/measured_js', lambda m_: JL.__setitem__('j', list(m_.position)[:6]), 10); time.sleep(1.0)
    j0 = np.array(JL['j']); jb = np.array(m['base_js'])
    for k in range(1, 6):
        msg = JS(); msg.position = (j0 + (jb - j0) * k / 5).tolist(); L.pop('reached', None); pubj.publish(msg); t = time.time()
        while 'reached' not in L and time.time() - t < 10: time.sleep(0.02)
        time.sleep(0.5)
    cal = json.load(open(ROOT + '/calibration_r6.json')); M2 = np.array(cal['M']); G2 = {r['i']: r for r in map(json.loads, open(ROOT + '/calib_log.jsonl')) if r.get('ev') == 'grid_point'}
    assert move_abs(base), 'base move failed'; out = {}
    for name, tgt in (('base', base), ('g04', np.array(G2[4]['sp_mm'])), ('g08', np.array(G2[8]['sp_mm'])), ('g11', np.array(G2[11]['sp_mm'])), ('base_end', base)):
        assert move_abs(tgt), 'move failed'; pred = np.r_[Pm(L['cp']), 1] @ M2; X, c1, c2, h = track(f'check_{name}', proj(pred))
        out[name] = {'err_mm': float(np.linalg.norm(X - pred)), 'err_vec': (X - pred).tolist(), 'ncc': [c1, c2]}; print(name, 'err %.2f mm ncc %.2f/%.2f' % (out[name]['err_mm'], c1, c2), flush=True)
    rec(ev='check_after_restart', result=out); json.dump(out, open(ROOT + '/check_after_restart1.json', 'w'), indent=1)
sys.stdout.flush(); os._exit(0)
