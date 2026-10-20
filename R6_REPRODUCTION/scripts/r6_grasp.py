#!/usr/bin/env python3
"""R6 grasp executor (grasp v3 per R6 §3), conditions A / B / C. New file; r5_grasp_v3.py is imported, not modified.

Flow per attempt (one phase per call, the operator checks between calls; trial state in <trial>/trial_state.json):
  approach  : needs a fresh <trial>/perception.json (ok, created after <trial>/start.json, < 20 min old). Fixed reference orientation;
              translate so the calibrated jaw feature sits 25 mm above p* along the needle normal (sub-steps <= 3 mm,
              predicted feature >= 12 mm above the FS surface of the trial snapshot)
  wrist     : A: nothing. B: instrument roll (joint 3) so that psi3 (kinematic jaw axis projected into the needle plane vs the
              tangent) is minimal: one 10 deg roll probe gives the shaft axis, then r5_grasp_v3.plan_roll; executed in <= 10 deg steps.
              C: B, then wrist pitch (joint 4) in 10 deg steps in the direction that increases beta until beta stops increasing
              (cap = measured max beta + 2 deg, joint limit +-75 deg). Records psi3 and beta.
  open      : open the jaw (0.8 rad) at 25 mm, capture 10 frames for the template (operator clicks the jaw feature and grasp point)
  template  : <trial>/template_clicks.json {"feat_R":[u,v], "grasp_R":[u,v], "grasp_L":[u,v]} -> per-view templates (feature in the
              topic-left image found by NCC in a +-6 px disparity band around the clicked grasp disparity), delta = grasp - feature (cam)
  hover     : move the grasp point to H = p* + 10 mm * n (affine, sub-steps <= 3 mm), then <= 3 stereo corrections:
              10 frames, drop NCC < 0.5, median, IQR; < 5 valid frames -> tracking failure, no motion;
              stop if |e| < 0.5 mm, or (IQR > 0.5 mm and two consecutive corrections < IQR); stop reason recorded
  descend   : straight down along -n by 10 mm to p* (sub-steps <= 3 mm, open loop), close-up capture before closing
  close_lift: close (-0.25 rad), capture, lift 10 mm along +n, hold 2 s, capture 10 frames, record jaw angle
  release   : (after any outcome) open the jaw, lift to 25 mm above p*, restore reference wrist joints in <= 10 deg steps, back to calib base
Safety: <= 3 mm per command, <= 10 deg per wrist command, wrist moves only at 25 mm above p*, no motion without a fresh valid perception."""
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
ap = argparse.ArgumentParser(); ap.add_argument('--trial', required=True); ap.add_argument('--mm', type=float, default=1.0)
ap.add_argument('--phase', required=True, choices=['approach', 'wrist', 'open', 'template', 'hover', 'descend', 'close_lift', 'release', 'rot90', 'putdown', 'lower', 'touch'])
ap.add_argument('--cond', choices=['A', 'B', 'C', 'S']); a = ap.parse_args()   # amendment 3: C == S flow; A = same flow without wrist change
R6 = os.path.expanduser('~/suturing-policy-sim2real/records/logs/R6_20261016'); C = R6 + '/calib'; T = a.trial
FX, CX, CY, FB = G.FX, G.CX, G.CY, G.FB
man = json.load(open(C + '/manifest.json')); cal = json.load(open(C + '/calibration_r6.json')); M = np.array(cal['M']); A3, b3 = M[:3], M[3]
R_cam = G.rotation_from_affine(M); qref = man['ref_orientation_xyzw']; base = np.array(man['base_cp_m']) * 1000
BETA_MAX = json.load(open(R6 + '/beta/beta_sweep.json'))['max']['beta_deg']
cam2cp_d = lambda d: np.asarray(d) @ np.linalg.inv(A3)
SF = T + '/trial_state.json'; S = json.load(open(SF)) if os.path.exists(SF) else {}
def save_state(): json.dump(S, open(SF, 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
LOG = open(T + '/trial_log.jsonl', 'a')
def rec(**k): k['t'] = time.time(); k['date'] = time.strftime('%Y-%m-%dT%H:%M:%S%z'); LOG.write(json.dumps(k, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)) + '\n'); LOG.flush()

per = json.load(open(T + '/perception.json')); st0 = json.load(open(T + '/start.json'))
assert per['ok'], 'perception not ok -> no motion'
assert per['created_t'] > st0['t'] and time.time() - per['created_t'] < 1200, 'perception not fresh for this trial -> no motion'
p_star = np.array(per['p_star_mm']); t_n = np.array(per['tangent']); n = np.array(per['normal_toward_cam'])
H25, H10 = p_star + 25 * n, p_star + 10 * n
snap = per['frames'][-1]['snap']; Zs = FB / np.clip(np.load(snap + '/fs_disp.npy'), 1, None)
PLANE_N, PLANE_C = n, p_star - 0.5 * n      # amendment 2: tangent plane of the pad surface under p* (perception)
def surf_h(X): return float((np.asarray(X) - PLANE_C) @ PLANE_N)
proj = lambda X: np.array([FX * X[0] / X[2] + CX, FX * X[1] / X[2] + CY])

rclpy.init(); node = rclpy.create_node('r6_grasp'); br = CvBridge(); L = {}; grab = {'buf': None, 'n': 0}
subs = [mf.Subscriber(node, Image, f'/jhu_daVinci/{c}/image_rect', qos_profile=qos_profile_sensor_data) for c in ('left', 'right')]
sy = mf.ApproximateTimeSynchronizer(subs, 10, 0.02)
sy.registerCallback(lambda l, r: grab['buf'].append((br.imgmsg_to_cv2(l, 'bgr8'), br.imgmsg_to_cv2(r, 'bgr8'))) if grab['buf'] is not None and len(grab['buf']) < grab['n'] else None)
for k, tp, ty in (('cp', '/PSM1/measured_cp', PoseStamped), ('sp', '/PSM1/setpoint_cp', PoseStamped), ('js', '/PSM1/measured_js', JointState),
                  ('spj', '/PSM1/setpoint_js', JointState), ('jaw', '/PSM1/jaw/measured_js', JointState), ('ecm', '/ECM/measured_js', JointState)):
    node.create_subscription(ty, tp, lambda msg, k=k: L.__setitem__(k, msg), 10)
node.create_subscription(Bool, '/PSM1/goal_reached', lambda msg: L.__setitem__('reached', msg.data), 10)
pub = node.create_publisher(PoseStamped, '/PSM1/move_cp', 1); pubj = node.create_publisher(JointState, '/PSM1/move_jp', 1); pjaw = node.create_publisher(JointState, '/PSM1/jaw/move_jp', 1)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start(); t0 = time.time()
while not all(k in L for k in ('cp', 'sp', 'js', 'spj', 'ecm')) and time.time() - t0 < 20: time.sleep(0.05)
assert all(k in L for k in ('cp', 'sp', 'js', 'spj', 'ecm')), 'no robot state'
assert np.abs(np.array(L['ecm'].position[:4]) - np.array(st0['ecm_js'])).max() < 2e-3, 'ECM moved since trial start'
Pm = lambda msg: np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) * 1000
Qm = lambda msg: [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
def wait_reached(tmo=10):
    t = time.time()
    while 'reached' not in L and time.time() - t < tmo: time.sleep(0.02)
    time.sleep(0.15); return bool(L.get('reached'))
def move_rel_cam(dcam, min_h=None, Xref=None):
    """translate by dcam (camera mm) keeping the CURRENT orientation; sub-steps <= 3 mm; optional height check of Xref + partial dcam"""
    q = Qm(L['sp']); cur = Pm(L['sp']); dcp = cam2cp_d(dcam); k = max(1, int(np.ceil(np.linalg.norm(dcam) / 3.0)))
    for j in range(1, k + 1):
        if min_h is not None and Xref is not None:
            h = surf_h(Xref + dcam * j / k); assert h is None or h >= min_h, f'predicted height {h:.1f} < {min_h}'
        tgt = cur + dcp * j / k; msg = PoseStamped(); msg.header.frame_id = L['sp'].header.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, tgt / 1000)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        ok = False
        for attempt in range(2):
            L.pop('reached', None); pub.publish(msg); ok = wait_reached()
            if not ok and np.linalg.norm(Pm(L['cp']) - tgt) < 0.3: ok = True      # arrived but no goal_reached message
            if ok: break
            rec(ev='not_reached_retry', tgt=tgt, attempt=attempt, cp=Pm(L['cp']))
        if not ok: rec(ev='not_reached', tgt=tgt); return False
    return True
pubs = node.create_publisher(PoseStamped, '/PSM1/servo_cp', 1)
def move_fast_cam(dcam, v=4.5):
    """user 12:5x: one continuous move (servo_cp at 50 Hz, <= v mm/s) when the tip stays > 20 mm above the pad over the whole segment
    (plane height is linear along a segment -> both end points checked); otherwise falls back to <= 3 mm steps"""
    dcam = np.asarray(dcam, float); h0 = tip_height(); h1 = surf_h(tip_cam() + dcam)
    if not (h0 > 20 and h1 > 20): rec(ev='fast_fallback', h0=h0, h1=h1); return move_rel_cam(dcam)
    q = Qm(L['sp']); cur = Pm(L['sp']); dcp = cam2cp_d(dcam); dist = max(np.linalg.norm(dcam), 1e-6); nstep = max(1, int(np.ceil(dist / v * 50)))
    for j in range(1, nstep + 1):
        tgt = cur + dcp * j / nstep; msg = PoseStamped(); msg.header.frame_id = L['sp'].header.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, tgt / 1000)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        pubs.publish(msg); time.sleep(0.02)
    t = time.time()
    while np.linalg.norm(Pm(L['cp']) - (cur + dcp)) > 0.3 and time.time() - t < 3: time.sleep(0.02)
    rec(ev='fast_move', dcam=dcam, h0=h0, h1=h1, v=v, err_end=float(np.linalg.norm(Pm(L['cp']) - (cur + dcp)))); return True
def move_joint(idx, target, max_step=np.radians(10)):
    while True:
        cur = L['js'].position[idx]; d = target - cur
        if abs(d) < np.radians(0.3): return True
        p = list(L['spj'].position)[:6]; p[idx] = float(cur + np.clip(d, -max_step, max_step)); msg = JointState(); msg.position = p
        L.pop('reached', None); pubj.publish(msg)
        if not wait_reached(): rec(ev='joint_not_reached', idx=idx, target=target); return False
        time.sleep(0.3)
def jaw(q):
    msg = JointState(); msg.position = [float(q)]; pjaw.publish(msg); time.sleep(1.8)
def frames(k=10):
    grab['n'] = k; grab['buf'] = []; t = time.time()
    while len(grab['buf']) < k and time.time() - t < 10: time.sleep(0.02)
    b = grab['buf']; grab['buf'] = None
    assert len(b) > 0, 'no camera frames'
    return b
def save_frames(name, b, all_frames=False):
    d = f'{T}/{name}'; os.makedirs(d, exist_ok=True)
    cv2.imwrite(d + '/right_rect.png', np.mean([x[1] for x in b], 0).round().astype(np.uint8)); cv2.imwrite(d + '/left_rect.png', np.mean([x[0] for x in b], 0).round().astype(np.uint8))
    if all_frames:
        for i, x in enumerate(b): cv2.imwrite(f'{d}/R{i:02d}.png', x[1]); cv2.imwrite(f'{d}/L{i:02d}.png', x[0])
OPEN_AX = 1   # jaw opening axis in the tool frame: y (D12 corrected after debug6; x was wrong by 90 deg)
TCP = np.array([-2.70, 1.08, 23.81])   # R2-E ProGrasp 400093, tool frame mm (user 11:4x)
Rref = Rot.from_quat(qref).as_matrix()
Rcp = lambda: Rot.from_quat(Qm(L['cp'])).as_matrix()
def tip_cam(p_cp=None, R=None):
    p_cp = Pm(L['cp']) if p_cp is None else p_cp; R = Rcp() if R is None else R
    return np.r_[p_cp, 1] @ M + ((R - Rref) @ TCP) @ A3          # affine maps cp to the tip-near feature at the reference orientation
def tip_height(): return surf_h(tip_cam())
def move_cp_abs(p_mm, q):
    cur = Pm(L['sp']); d = p_mm - cur; k = max(1, int(np.ceil(np.linalg.norm(d) / 3.0)))
    for j in range(1, k + 1):
        tgt = cur + d * j / k; msg = PoseStamped(); msg.header.frame_id = L['sp'].header.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, tgt / 1000)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        L.pop('reached', None); pub.publish(msg)
        if not wait_reached(): rec(ev='not_reached', tgt=tgt); return False
    return True
def rot_tipfixed(idx, target, p_tip, max_step=None):
    max_step = max_step or (np.radians(6) if idx == 3 else np.radians(4))
    """joint idx -> target in <= 6 deg steps; after each step translate so that the tip (TCP) stays at p_tip (ECM mm); comp <= 3 mm"""
    comps = []
    while True:
        cur = L['js'].position[idx]; d = target - cur
        if abs(d) < np.radians(0.3): return comps
        p = list(L['spj'].position)[:6]; p[idx] = float(cur + np.clip(d, -max_step, max_step)); msg = JointState(); msg.position = p
        ok = False
        for _att in range(2):
            L.pop('reached', None); pubj.publish(msg); ok = wait_reached()
            if not ok and abs(L['js'].position[idx] - p[idx]) < np.radians(0.5): ok = True
            if ok: break
            rec(ev='joint_retry', idx=idx, target=p[idx], now=L['js'].position[idx])
        assert ok, 'joint step not reached'; time.sleep(0.1)
        Rn = Rot.from_quat(Qm(L['sp'])).as_matrix(); pt = p_tip - Rn @ TCP; dc = pt - Pm(L['sp'])
        assert np.linalg.norm(dc) <= 3.2, f'compensation {np.linalg.norm(dc):.2f} mm > 3 mm'
        assert move_cp_abs(pt, Qm(L['sp'])), 'compensation move failed'; comps.append(float(np.linalg.norm(dc)))
def needle_shift(cur_gray, ref_snap=None):
    """2D shift of the needle vs the placement's first frame: NCC of 25x25 patches at the clicked polyline points (search +-30 px);
    patches with NCC >= 0.7 count (occluded ones drop out); median shift -> mm at the needle depth; ok if >= 3 patches and <= 1.0 mm"""
    ref_snap = ref_snap or per.get('placement_ref_snap') or per['frames'][0]['snap']
    ref = cv2.imread(ref_snap + '/right_rect.png', 0).astype(np.float32); cur = cur_gray.astype(np.float32)
    pts = np.array(per['method']['clicks']['polyline_px'], float); rows = []
    for (u, v) in pts:
        u, v = int(round(u)), int(round(v)); h = 12; Tp = ref[v - h:v + h + 1, u - h:u + h + 1]
        if Tp.shape != (25, 25): continue
        W = cur[v - h - 30:v + h + 31, u - h - 30:u + h + 31]
        if W.shape != (85, 85): continue
        r = cv2.matchTemplate(W, Tp, cv2.TM_CCOEFF_NORMED); _, mx, _, (lx, ly) = cv2.minMaxLoc(r); rows.append({'uv': [u, v], 'ncc': float(mx), 'dx': lx - 30, 'dy': ly - 30})
    good = [r for r in rows if r['ncc'] >= 0.7]
    if len(good) < 3: return {'ok': False, 'n_valid': len(good), 'shift_mm': None, 'per_point': rows, 'reason': 'unverifiable'}
    d = np.median([[r['dx'], r['dy']] for r in good], 0); mm = float(np.linalg.norm(d) * p_star[2] / FX)
    return {'ok': mm <= 1.0, 'n_valid': len(good), 'shift_px': d.tolist(), 'shift_mm': mm, 'per_point': rows, 'ref': ref_snap}
def angles():
    z = R_cam @ Rot.from_quat(Qm(L['cp'])).as_matrix()[:, 2]; psi3, tilt = G.psi_inplane(z, t_n, n)
    Rm = R_cam @ Rot.from_quat(Qm(L['cp'])).as_matrix(); ang = lambda v: float(np.degrees(np.arccos(min(1.0, abs(v @ t_n)))))
    return {'psi3_deg': psi3, 'angle_z_normal_deg': tilt, 'beta_deg': 90 - tilt, 'x_tangent_deg': ang(Rm[:, 0]), 'y_tangent_deg': ang(Rm[:, 1]),
            'tool_z_cam': z, 'tool_x_cam': Rm[:, 0], 'js': list(L['js'].position)[:6], 'cp_mm': Pm(L['cp']), 'q': Qm(L['cp']), 'tip_height_mm': tip_height()}
def state_snapshot(): return {'cp': Pm(L['cp']), 'q': Qm(L['cp']), 'js': list(L['js'].position)[:6], 'jaw': list(L['jaw'].position) if 'jaw' in L else None, 'ecm': list(L['ecm'].position)[:4]}

# ---------- per-view tracking with the trial template ----------
def sub1(r, j):
    if 0 < j < len(r) - 1:
        p, q, s = r[j - 1], r[j], r[j + 1]; den = p - 2 * q + s; return j + (0.5 * (p - s) / den if den < 0 else 0)
    return float(j)
def match(img, Tp, cu, cv_, rad=60):
    th, tw = Tp.shape; x0, y0 = int(max(cu - rad - tw / 2, 0)), int(max(cv_ - rad - th / 2, 0)); x1, y1 = int(min(cu + rad + tw / 2, 1300)), int(min(cv_ + rad + th / 2, 1024))
    r = cv2.matchTemplate(img[y0:y1, x0:x1], Tp, cv2.TM_CCOEFF_NORMED); _, mx, _, (lx, ly) = cv2.minMaxLoc(r)
    return mx, x0 + sub1(r[ly], lx) + tw / 2 - 0.5, y0 + sub1(r[:, lx], ly) + th / 2 - 0.5
def track10(name, pred_feat_uv):
    tp = S['template']; TR = cv2.imread(T + '/tpl_R.png', 0).astype(np.float32); TL = cv2.imread(T + '/tpl_L.png', 0).astype(np.float32)
    b = frames(5); Xs = []; rows = []
    def _proc(bb):
      for (Li, Ri) in bb:
        Rg = cv2.cvtColor(Ri, cv2.COLOR_BGR2GRAY).astype(np.float32); Lg = cv2.cvtColor(Li, cv2.COLOR_BGR2GRAY).astype(np.float32)
        m1, u, v = match(Rg, TR, *pred_feat_uv); m2, uL, vL = match(Lg, TL, u - tp['disp0'], v, rad=25)
        X = np.array([(u - CX), (v - CY), FX]) * (FB / (u - uL)) / FX; rows.append({'ncc': [m1, m2], 'uv': [u, v], 'uvL': [uL, vL], 'X': X})
        if m1 >= 0.5 and m2 >= 0.5: Xs.append(X)
    _proc(b); X5 = np.array(Xs) if Xs else None
    iqr5 = float(np.linalg.norm(np.percentile(X5, 75, 0) - np.percentile(X5, 25, 0))) if X5 is not None and len(X5) == 5 else None
    if iqr5 is None or iqr5 >= 0.3:            # user 12:5x: 5 frames suffice only when all valid and IQR < 0.3 mm
        b2 = frames(5); _proc(b2); b = b + b2
    save_frames(name, b, all_frames=True)
    out = {'n_valid': len(Xs), 'n_frames': len(b), 'rows': rows}
    if len(Xs) >= 5:
        Xs = np.array(Xs); med = np.median(Xs, 0); iqr = np.percentile(Xs, 75, 0) - np.percentile(Xs, 25, 0)
        out.update({'X_feat_med': med, 'iqr_xyz': iqr, 'iqr_norm': float(np.linalg.norm(iqr)), 'X_grasp': med + np.array(tp['delta_cam'])})
    rec(ev='track10', name=name, **{k: v for k, v in out.items() if k != 'rows'}, rows=rows); return out

if a.phase == 'approach':
    assert a.cond, '--cond required'; S.update({'cond': a.cond, 'phases': [], 'ref_js': list(L['js'].position)[:6]})
    S['flatness_deg'] = max(f['surface']['normal_variation_deg_tail_mid_tip'] for f in per['frames']); print('surface normal variation along the needle %.1f deg' % S['flatness_deg'])
    assert np.linalg.norm(Pm(L['sp']) - base) < 2.0, 'start from calib base'
    assert max(abs(np.array(Qm(L['sp'])) - np.array(qref))) < 0.01 or max(abs(np.array(Qm(L['sp'])) + np.array(qref))) < 0.01, 'not at reference orientation'
    Xf = np.r_[Pm(L['sp']), 1] @ M; d = H25 - Xf
    hs = surf_h(Xf + d); assert hs >= 12.0, f'H25 too low {hs:.1f}'
    ok = move_fast_cam(d)
    S['approach'] = {'ok': ok, 'H25': H25, 'feature_model_after': np.r_[Pm(L['cp']), 1] @ M, **state_snapshot()}; S['phases'].append('approach')
    b = frames(6); save_frames('approach', b); S['angles_ref'] = angles(); rec(ev='approach', **S['approach'], angles=S['angles_ref'])
    print('approach ok', ok, 'psi3 %.1f beta %.1f (reference orientation)' % (S['angles_ref']['psi3_deg'], S['angles_ref']['beta_deg']))
elif a.phase == 'wrist':
    cond = S['cond']; info = {'cond': cond, 'before': angles()}; p_tip = Pm(L['cp']) + Rcp() @ TCP; info['tip_ecm_fixed'] = p_tip
    h0 = tip_height(); info['tip_height_before'] = h0; print('tip height before wrist %.1f mm' % h0, flush=True)
    assert h0 >= 10.0, 'tip < 10 mm above the pad: lift first'
    def plan_and_roll(tag):
        R1 = Rcp(); q3 = L['js'].position[3]; c1 = rot_tipfixed(3, q3 + np.radians(6), p_tip); R2 = Rcp()
        rv = Rot.from_matrix(R2 @ R1.T).as_rotvec(); ax = rv / np.linalg.norm(rv)          # shaft axis (ECM frame), +6 deg of joint 3
        x2 = R2[:, OPEN_AX]; best = None
        for dlt in np.radians(np.arange(-120, 120.1, 1.0)):
            xe = G.rodrigues(ax, dlt) @ x2; ang = float(np.degrees(np.arccos(min(1, abs((R_cam @ xe) @ t_n)))))
            key = (ang >= 85, -abs(dlt)) if ang >= 85 else (False, ang)
            if best is None or key > best[0]: best = (key, dlt, ang)
        _, dlt, ang = best; info[tag] = {'probe_rotvec_deg': np.degrees(rv), 'offset_deg': float(np.degrees(dlt)), 'pred_x_tangent_deg': ang, 'probe_comp_mm': c1}
        c2 = rot_tipfixed(3, L['js'].position[3] + dlt, p_tip); info[tag]['comp_mm'] = c2; info[tag]['after'] = angles()
        print(f"{tag}: roll {np.degrees(dlt):+.0f} deg, predicted open(y)-t {ang:.1f}, measured y-t {info[tag]['after']['y_tangent_deg']:.1f} x-t {info[tag]['after']['x_tangent_deg']:.1f}, beta {info[tag]['after']['beta_deg']:.1f}", flush=True)
    if cond in ('S', 'C'):   # user 11:58: jaw in line with the shaft, then roll so that the opening x is perpendicular to the tangent
        rot_tipfixed(4, 0.0, p_tip); rot_tipfixed(5, 0.0, p_tip); info['straight'] = angles(); print('straightened: beta %.1f x-t %.1f' % (info['straight']['beta_deg'], info['straight']['x_tangent_deg']), flush=True)
    if cond in ('B', 'C', 'S'): plan_and_roll('roll1')
    if cond == 'C_OLD':     # original pitch-to-max-beta C, dropped in amendment 3
        w0 = L['js'].position[4]; b0 = angles()['beta_deg']; sweep = [{'w': w0, 'beta': b0}]
        rot_tipfixed(4, w0 + np.radians(4), p_tip); bp = angles()['beta_deg']; sweep.append({'w': L['js'].position[4], 'beta': bp})
        sgn = +1 if bp > b0 + 0.3 else -1
        if sgn < 0: rot_tipfixed(4, w0, p_tip); cur_b = b0
        else: cur_b = bp
        while True:
            w = L['js'].position[4] + sgn * np.radians(4)
            if abs(w) > np.radians(75) or cur_b >= BETA_MAX + 2: break
            rot_tipfixed(4, w, p_tip); nb = angles()['beta_deg']; sweep.append({'w': L['js'].position[4], 'beta': nb})
            if nb <= cur_b + 0.3: rot_tipfixed(4, w - sgn * np.radians(4), p_tip); break
            cur_b = nb
        info['pitch_sweep'] = sweep; print('pitch: beta %.1f' % angles()['beta_deg'], flush=True)
        if angles()['y_tangent_deg'] < 80: plan_and_roll('roll2')
    info['final'] = angles(); info['tip_height_after'] = tip_height(); info['tip_model_drift_mm'] = float(np.linalg.norm((Pm(L['cp']) + Rcp() @ TCP) - p_tip))
    S['wrist'] = info; S['phases'].append('wrist'); b = frames(6); save_frames('after_wrist', b); rec(ev='wrist', **info)
    f = info['final']; print('wrist done: psi3 %.1f  x-t %.1f  y-t %.1f  beta %.1f  tip height %.1f' % (f['psi3_deg'], f['x_tangent_deg'], f['y_tangent_deg'], f['beta_deg'], info['tip_height_after']))
elif a.phase == 'open':
    jaw(0.6 if S.get('cond') in ('S', 'C', 'A') else 0.8); b = frames(10); save_frames('open25', b, all_frames=True); S['phases'].append('open'); S['open'] = state_snapshot(); rec(ev='open', **S['open'])
    print('jaw open; click feature + grasp point on', T + '/open25/right_rect.png (and grasp point on left_rect.png)')
elif a.phase == 'template':
    ck = json.load(open(T + '/template_clicks.json')); Rimg = cv2.imread(T + '/open25/right_rect.png', 0).astype(np.float32); Limg = cv2.imread(T + '/open25/left_rect.png', 0).astype(np.float32)
    hf = ck.get('half', 16); fu, fv = ck['feat_R']; TR = Rimg[fv - hf:fv + hf + 1, fu - hf:fu + hf + 1]
    dg = ck['grasp_R'][0] - ck['grasp_L'][0]; best = None
    for dd in np.arange(dg - 6, dg + 6.01, 1):
        x = int(round(fu - dd)); r = cv2.matchTemplate(Limg[fv - hf - 3:fv + hf + 4, x - hf - 3:x + hf + 4], TR, cv2.TM_CCOEFF_NORMED); _, mx, _, (lx, ly) = cv2.minMaxLoc(r)
        if best is None or mx > best[0]: best = (mx, x - 3 + lx, fv - 3 + ly)
    mx, xl, yl = best; disp0 = fu - xl; TL = Limg[yl - hf:yl + hf + 1, xl - hf:xl + hf + 1]
    cv2.imwrite(T + '/tpl_R.png', TR.astype(np.uint8)); cv2.imwrite(T + '/tpl_L.png', TL.astype(np.uint8))
    Xf = np.array([fu - CX, fv - CY, FX]) * (FB / disp0) / FX; gu, gv = ck['grasp_R']; Xg = np.array([gu - CX, gv - CY, FX]) * (FB / dg) / FX
    S['template'] = {'clicks': ck, 'disp0': float(disp0), 'stereo_ncc': float(mx), 'X_feat': Xf, 'X_grasp_click': Xg, 'delta_cam': Xg - Xf,
                     'method': 'manual click (feature patch + grasp point both views) + per-view templates'}
    S['template']['tip_model_cam'] = tip_cam(); S['template']['grasp_minus_tipmodel'] = Xg - tip_cam()
    S['phases'].append('template'); rec(ev='template', **S['template'])
    print('template: stereo NCC %.2f disp %.1f (clicked grasp disp %.1f); delta %s; grasp at %s, H25 %s' % (mx, disp0, dg, (Xg - Xf).round(2), Xg.round(1), H25.round(1)))
elif a.phase == 'hover':
    if S.get('cond') in ('S', 'C', 'A'):
        zc = R_cam @ Rcp()[:, 2]; S['axis_cam'] = zc; H10 = p_star - 10.0 * zc        # above p* along the shaft/jaw axis
    tr = track10('hover_init', S['template']['clicks']['feat_R']); assert tr['n_valid'] >= 5, 'tracking failure, no motion'
    Xg = tr['X_grasp']; d = H10 - Xg; print('to hover10: %s (|d| %.1f), grasp height now %.1f' % (d.round(2), np.linalg.norm(d), (surf_h(Xg) or -1)), flush=True)
    assert np.linalg.norm(d) < 25, 'hover move too large'
    assert move_rel_cam(d, min_h=7.0, Xref=Xg)
    corr = []; applied = []; reason = None; pred = proj(tr['X_feat_med'] + d)
    for it in range(4):                       # measurement it; at most 3 corrections applied
        tr = track10(f'hover_c{it}', pred)
        if tr['n_valid'] < 5: reason = 'tracking_failure'; break
        e = H10 - tr['X_grasp']; en = float(np.linalg.norm(e)); iqr = tr['iqr_norm']
        corr.append({'it': it, 'err': e, 'err_norm': en, 'iqr_norm': iqr, 'n_valid': tr['n_valid']}); print(f'  check {it}: |e| {en:.2f} mm  IQR {iqr:.2f}  valid {tr["n_valid"]}/10', flush=True)
        if en < 0.5: reason = 'err<0.5'; break
        if iqr > 0.5 and len(applied) >= 2 and applied[-1] < iqr and applied[-2] < iqr: reason = 'two consecutive corrections < IQR'; break
        if len(applied) == 3: reason = 'max 3 corrections'; break
        step = e if en <= 3 else 3 * e / en; assert move_rel_cam(step); applied.append(float(np.linalg.norm(step))); pred = proj(tr['X_feat_med'] + step)
    n_corr = len(applied); last = corr[-1] if corr else None
    if last:
        e = np.asarray(last['err']); e_n = float(e @ n); last['lateral_mm'] = float(np.linalg.norm(e - e_n * n)); last['height_mm'] = -e_n
    S['hover'] = {'corrections': corr, 'applied_mm': applied, 'n_corrections': n_corr, 'stop_reason': reason, 'final': last, 'angles': angles(), **state_snapshot()}
    S['phases'].append('hover'); rec(ev='hover', **S['hover'])
    print('hover: stop', reason, 'corrections', S['hover']['n_corrections'], 'final lateral %.2f height %.2f' % (last['lateral_mm'], last['height_mm']) if last else '')
elif a.phase == 'descend' and S.get('cond') in ('S', 'C', 'A'):
    assert 'descend' not in S.get('phases', []) and 'touch' not in S.get('phases', []), 'descend already done in this trial (refuse to repeat)'
    S.setdefault('phases', []).append('descend_started'); save_state()
    assert S['hover']['stop_reason'] != 'tracking_failure', 'tracking failure at hover'
    zc = np.array(S['axis_cam']); steps = [1.0] * 8          # D16: 8 mm here, then phase 'touch' (0.5 mm steps until joint-effort contact)
    for k, st in enumerate(steps):
        assert move_rel_cam(st * zc); time.sleep(0.25)
    b = frames(10); save_frames('pre_close', b, all_frames=True)
    sh = needle_shift(cv2.cvtColor(np.mean([x[1] for x in b], 0).round().astype(np.uint8), cv2.COLOR_BGR2GRAY))
    S['needle_check_preclose'] = sh; rec(ev='needle_check_preclose', **sh); print('needle check before closing:', {k: v for k, v in sh.items() if k != 'per_point'}, flush=True)
    S['descend'] = {'ok': True, 'steps_mm': steps, **state_snapshot(), 'angles': angles()}; S['phases'].append('descend'); rec(ev='descend', **S['descend'])
    Rt = Rcp(); p_cp = Pm(L['cp']); tip_e = p_cp + Rt @ TCP; Ainv = np.linalg.inv(A3)
    fr = per['frames'][-1]['fit']; c_cam = np.array(fr['centre_mm']); cam2ecm = lambda X: (np.asarray(X) - b3) @ Ainv + Rref @ TCP   # affine maps cp(+ref TCP) -> cam
    pose = {'t': time.time(), 'date': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'frame_note': 'ECM = measured_cp frame (PSM1 measured_cp header ECM); cam = rectified physical-left camera (mm); cam<->ECM via the R6 affine (calib/calibration_r6.json; rotation = polar part); tip = measured_cp + R * TCP (R2-E)',
            'tip': {'ecm_mm': tip_e, 'R_ecm': Rt, 'cam_mm': tip_cam(), 'R_cam': R_cam @ Rt, 'cp_ecm_mm': p_cp, 'q_cp_xyzw': Qm(L['cp']), 'TCP_tool_mm': TCP, 'jaw_rad': list(L['jaw'].position) if 'jaw' in L else None},
            'needle_cam': {'centre_mm': c_cam, 'normal_toward_cam': n, 'grasp_point_mm': p_star, 'tangent': t_n, 'R_mm': fr['R_mm'], 'arc_deg': fr['arc_deg']},
            'needle_ecm': {'centre_mm': cam2ecm(c_cam), 'normal': R_cam.T @ n, 'grasp_point_mm': cam2ecm(p_star), 'tangent': R_cam.T @ t_n},
            'angles': angles(), 'hover_final_err': S['hover']['final'], 'js': list(L['js'].position)[:6]}
    json.dump(pose, open(T + '/preclose_pose.json', 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
    print('descended %.1f mm along the jaw axis -> close-up + preclose_pose.json saved; tell the user before closing' % sum(steps))
elif a.phase == 'descend':
    assert S['hover']['stop_reason'] != 'tracking_failure', 'tracking failure at hover'
    Xg = S['hover']['final']; d = -10.0 * n; ok = move_rel_cam(d); b = frames(10); save_frames('pre_close', b, all_frames=True)
    S['descend'] = {'ok': ok, **state_snapshot(), 'angles': angles()}; S['phases'].append('descend'); rec(ev='descend', **S['descend'])
    print('descended 10 mm along -n, reached', ok, '-> close-up saved; tell the user before closing')
elif a.phase == 'close_lift' and S.get('cond') in ('S', 'C', 'A'):
    nc = S.get('needle_check_preclose', {})
    assert nc.get('ok'), f"needle moved / unverifiable before closing -> abort (no close): {nc.get('shift_mm')} mm, {nc.get('n_valid')} patches"
    S['close_time'] = time.strftime('%Y-%m-%dT%H:%M:%S%z'); j0 = L['jaw'].position[0]
    for q in np.linspace(j0, -0.25, 11)[1:]:
        msg = JointState(); msg.position = [float(q)]; pjaw.publish(msg); time.sleep(0.3)
    time.sleep(1.0); b = frames(6); save_frames('closed', b); jc = list(L['jaw'].position)
    zc = np.array(S['axis_cam']); ok = True
    for k in range(10): ok = move_rel_cam(-1.0 * zc) and ok; time.sleep(0.15)
    time.sleep(2.0); b = frames(10); save_frames('lifted_2s', b, all_frames=True)
    S['close_lift'] = {'jaw_after_close': jc, 'lift_ok': ok, 'jaw_after_lift': list(L['jaw'].position), 'slow_close_s': 3.0, **state_snapshot()}
    S['phases'].append('close_lift'); rec(ev='close_lift', **S['close_lift']); print('closed slowly at', S['close_time'], 'jaw', jc, 'lifted 10 mm along the axis + 2 s, ok', ok)
elif a.phase == 'close_lift':
    S['close_time'] = time.strftime('%Y-%m-%dT%H:%M:%S%z'); jaw(-0.25); b = frames(6); save_frames('closed', b); jc = list(L['jaw'].position) if 'jaw' in L else None
    ok = move_rel_cam(10.0 * n); time.sleep(2.0); b = frames(10); save_frames('lifted_2s', b, all_frames=True)
    S['close_lift'] = {'jaw_after_close': jc, 'lift_ok': ok, 'jaw_after_lift': list(L['jaw'].position) if 'jaw' in L else None, **state_snapshot()}
    S['phases'].append('close_lift'); rec(ev='close_lift', **S['close_lift']); print('closed at', S['close_time'], 'jaw', jc, 'lifted 10 mm + 2 s, ok', ok)
elif a.phase == 'rot90':   # user 12:1x: jaw was ON the needle -> lift 3 mm, roll 90 deg about the shaft (tip fixed), descend 3 + 1 mm to touch the pad
    zc = np.array(S['axis_cam']); before = angles()
    for k in range(3): assert move_rel_cam(-1.0 * zc); time.sleep(0.4)
    q3 = L['js'].position[3]; tgt = q3 + np.radians(90) if q3 + np.radians(90) < 4.3 else q3 - np.radians(90)
    p_tip = Pm(L['cp']) + Rcp() @ TCP; comps = rot_tipfixed(3, tgt, p_tip); after = angles()
    print('rolled %+.1f deg: x-t %.1f y-t %.1f psi3 %.1f beta %.1f' % (np.degrees(L['js'].position[3] - q3), after['x_tangent_deg'], after['y_tangent_deg'], after['psi3_deg'], after['beta_deg']), flush=True)
    for st in [1.0, 1.0, 1.0, 0.5, 0.5]: assert move_rel_cam(st * zc); time.sleep(0.6)
    b = frames(10); save_frames('pre_close_rot90', b, all_frames=True)
    S['rot90'] = {'before': before, 'after': after, 'roll_deg': float(np.degrees(L['js'].position[3] - q3)), 'comp_mm': comps, 'extra_descent_mm': 1.0, **state_snapshot()}
    S['phases'].append('rot90'); rec(ev='rot90', **S['rot90']); print('rotated and descended to 1.0 mm below the previous stop (touching the pad expected)')
elif a.phase == 'lower':   # user: tips did not touch the phantom -> extra descent along the jaw axis, 0.5 mm steps
    zc = np.array(S['axis_cam']); k = int(round(a.mm / 0.5))
    for i in range(k): assert move_rel_cam(0.5 * zc); time.sleep(0.25)
    b = frames(10); save_frames(f'pre_close_lower{len([x for x in S["phases"] if x == "lower"])}', b, all_frames=True)
    sh = needle_shift(cv2.cvtColor(np.mean([x[1] for x in b], 0).round().astype(np.uint8), cv2.COLOR_BGR2GRAY)); S['needle_check_preclose'] = sh
    S.setdefault('extra_descent_mm', 0.0); S['extra_descent_mm'] += k * 0.5; S['phases'].append('lower'); rec(ev='lower', mm=k * 0.5, total_extra=S['extra_descent_mm'], needle=sh, **state_snapshot())
    Rt = Rcp(); pc = json.load(open(T + '/preclose_pose.json')); pc['tip'].update({'ecm_mm': (Pm(L['cp']) + Rt @ TCP).tolist(), 'cam_mm': tip_cam().tolist(), 'cp_ecm_mm': Pm(L['cp']).tolist()}); pc['extra_descent_mm'] = S['extra_descent_mm']
    json.dump(pc, open(T + '/preclose_pose.json', 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
    print('lowered %.1f mm more (total extra %.1f); needle check %s' % (k * 0.5, S['extra_descent_mm'], {k_: v for k_, v in sh.items() if k_ != 'per_point'}))
elif a.phase == 'touch':
    assert 'descend' in S.get('phases', []), 'descend not completed -> no touch'
    assert S['phases'].count('touch') == 0, 'touch already done (refuse to repeat)'   # user 13:1x: detect contact with the phantom from joint effort / tracking error while descending 0.5 mm per step
    zc = np.array(S['axis_cam'])
    def eff(dt=0.3):
        E, Pe = [], []; t0_ = time.time()
        while time.time() - t0_ < dt: E.append(list(L['js'].effort)[:6]); Pe.append(np.array(L['js'].position[:6]) - np.array(L['spj'].position[:6])); time.sleep(0.01)
        return np.mean(E, 0), np.mean(Pe, 0)
    e0, _ = eff(1.0); rows = []; contact = None
    for k in range(int(round(a.mm / 0.5))):
        assert move_rel_cam(0.5 * zc); time.sleep(0.4); e, pe = eff(0.3); de = e - e0
        hit = {'ins_effort': abs(de[2]) > 0.15, 'j01_effort': max(abs(de[0]), abs(de[1])) > 0.05, 'ins_track_mm': abs(pe[2]) * 1000 > 0.3}
        rows.append({'step': k + 1, 'd_effort': de, 'track_err': pe, 'hit': hit}); print(f'  step {k + 1} (+{0.5 * (k + 1):.1f} mm): d_ins {de[2]:+.3f} N  d_j0 {de[0]:+.3f} d_j1 {de[1]:+.3f}  ins_err {pe[2] * 1000:+.2f} mm  {[x for x, v in hit.items() if v]}', flush=True)
        if any(hit.values()): contact = k + 1; break
    b = frames(10); save_frames('pre_close_touch', b, all_frames=True)
    sh = needle_shift(cv2.cvtColor(np.mean([x[1] for x in b], 0).round().astype(np.uint8), cv2.COLOR_BGR2GRAY)); S['needle_check_preclose'] = sh
    S.setdefault('extra_descent_mm', 0.0); S['extra_descent_mm'] += 0.5 * len(rows); S['touch'] = {'baseline': e0, 'rows': rows, 'contact_step': contact}
    S['phases'].append('touch'); rec(ev='touch', **S['touch'], needle=sh, **state_snapshot())
    Rt = Rcp(); pc = json.load(open(T + '/preclose_pose.json')); pc['tip'].update({'ecm_mm': (Pm(L['cp']) + Rt @ TCP).tolist(), 'cam_mm': tip_cam().tolist(), 'cp_ecm_mm': Pm(L['cp']).tolist()}); pc['extra_descent_mm'] = S['extra_descent_mm']; pc['contact'] = contact
    json.dump(pc, open(T + '/preclose_pose.json', 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
    print('contact detected at step %s; total extra %.1f mm; needle check %s' % (contact, S['extra_descent_mm'], {k_: v for k_, v in sh.items() if k_ != 'per_point'}))
elif a.phase == 'putdown':   # after a successful lift: lower 10 mm along the axis (1 mm steps), open slowly -> needle back on the pad
    zc = np.array(S['axis_cam'])
    for k in range(10): assert move_rel_cam(1.0 * zc); time.sleep(0.3)
    j0 = L['jaw'].position[0]
    for q in np.linspace(j0, 0.6, 8)[1:]:
        msg = JointState(); msg.position = [float(q)]; pjaw.publish(msg); time.sleep(0.3)
    time.sleep(0.8); b = frames(6); save_frames('putdown', b); S['putdown'] = state_snapshot(); S['phases'].append('putdown'); rec(ev='putdown', **S['putdown']); print('needle put down, jaw open')
elif a.phase == 'release':
    Xf_now = np.r_[Pm(L['sp']), 1] @ M
    jaw(0.8); up = 15.0 * n; move_rel_cam(up)
    ref = S['ref_js']; pt = Pm(L['cp']) + Rcp() @ TCP
    rot_tipfixed(4, ref[4], pt); rot_tipfixed(5, ref[5], pt); rot_tipfixed(3, ref[3], pt); jaw(0.0)
    # back to base with the reference orientation (sub-steps)
    cur = Pm(L['sp']); d = base - cur; k = max(1, int(np.ceil(np.linalg.norm(d) / 3.0)))
    for j in range(1, k + 1):
        tgt = cur + d * j / k; msg = PoseStamped(); msg.header.frame_id = L['sp'].header.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, tgt / 1000)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = qref
        L.pop('reached', None); pub.publish(msg); wait_reached()
    S['release'] = state_snapshot(); S['phases'].append('release'); rec(ev='release', **S['release']); b = frames(6); save_frames('after_release', b)
    print('released, at base', np.linalg.norm(Pm(L['sp']) - base).round(2), 'mm from base')
save_state(); sys.stdout.flush(); os._exit(0)
