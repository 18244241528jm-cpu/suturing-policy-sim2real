#!/usr/bin/env python3
"""R4 day-2 grasp attempt with the fixed-wrist affine calibration (grid_calib/calibration_v3.json, camera <- measured_cp[ECM]).
phases (argument --phase):  correct  : camera-measure the tracked jaw feature, move so it sits at the hover target (<=3 iterations)
                            descend  : open jaw, descend along the needle normal in 2 steps to 0.5 mm above the needle centre
                            grab     : close jaw, lift 10 mm, wait 2 s, capture
Targets come from day2/reach_target.json (arc fit on fsfp_s02) + a 3 mm shift of the feature along the jaw axis (needle deeper in the jaw).
Aborts on low tracking NCC (<0.6) or a correction step > 6 mm."""
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
ap = argparse.ArgumentParser(); ap.add_argument('--phase', required=True, choices=['targets', 'correct', 'descend', 'grab', 'measure', 'descend_closed', 'finish']); ap.add_argument('--fracs', type=float, nargs='+', default=[0.5, 1.0])
ap.add_argument('--target', default=None, help='grasp_target.json from r4d2_grasp_plan.py (default: day2/reach_target.json)'); ap.add_argument('--tag', default=''); a = ap.parse_args()
ROOT = os.path.expanduser('~/suturing-policy-sim2real/records/logs/R4_20260928/day2'); OUT = ROOT + '/grasp' + ('_' + sys.argv[sys.argv.index('--tag') + 1] if '--tag' in sys.argv else ''); os.makedirs(OUT, exist_ok=True)
FX, CX, CY = 1860.724932, 690.538345, 576.803638; FB = FX * 5.4476353950418295
cal = json.load(open(ROOT + '/grid_calib/calibration_v3.json')); M = np.array(cal['models']['affine']['M']); A3, b3 = M[:3], M[3]
tgt = json.load(open(a.target or ROOT + '/reach_target.json')); p_star = np.array(tgt['grasp_point_cam_mm']); n = np.array(tgt['arc']['normal_toward_cam'])
jaw_axis = np.array([0.55, 0.83, 0.0]); jaw_axis /= np.linalg.norm(jaw_axis)
shift = 3.0 * jaw_axis                      # feature sits 3 mm further along the jaw axis -> needle ~4 mm inside the tip
HOVER = p_star + shift + 10.0 * n; FINAL = p_star + shift + 0.5 * n
cam2cp = lambda X: (np.asarray(X) - b3) @ np.linalg.inv(A3)
dcam2dcp = lambda d: np.asarray(d) @ np.linalg.inv(A3)
if a.phase == 'targets':
    print('hover cam', HOVER.round(2), 'cp(m)', (cam2cp(HOVER) / 1000).round(6).tolist()); print('final cam', FINAL.round(2), 'cp(m)', (cam2cp(FINAL) / 1000).round(6).tolist())
    json.dump({'hover_cam': HOVER.tolist(), 'final_cam': FINAL.tolist(), 'hover_cp_m': (cam2cp(HOVER) / 1000).tolist(), 'final_cp_m': (cam2cp(FINAL) / 1000).tolist()}, open(OUT + '/targets.json', 'w'), indent=1); sys.exit()

rclpy.init(); node = rclpy.create_node('r4d2_grasp'); br = CvBridge(); L = {}; grab = {'buf': None, 'n': 0}
subs = [mf.Subscriber(node, Image, f'/jhu_daVinci/{c}/image_rect', qos_profile=qos_profile_sensor_data) for c in ('left', 'right')]
sy = mf.ApproximateTimeSynchronizer(subs, 10, 0.02)
sy.registerCallback(lambda l, r: grab['buf'].append((br.imgmsg_to_cv2(l, 'bgr8').astype(np.float32), br.imgmsg_to_cv2(r, 'bgr8').astype(np.float32))) if grab['buf'] is not None and len(grab['buf']) < grab['n'] else None)
for k, t, ty in (('cp', '/PSM1/measured_cp', PoseStamped), ('sp', '/PSM1/setpoint_cp', PoseStamped), ('jaw', '/PSM1/jaw/measured_js', JointState)):
    node.create_subscription(ty, t, lambda m, k=k: L.__setitem__(k, m), 10)
node.create_subscription(Bool, '/PSM1/goal_reached', lambda m: L.__setitem__('reached', m.data), 10)
pub = node.create_publisher(PoseStamped, '/PSM1/move_cp', 1); pj = node.create_publisher(JointState, '/PSM1/jaw/move_jp', 1)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start(); t0 = time.time()
while not all(k in L for k in ('cp', 'sp')) and time.time() - t0 < 10: time.sleep(0.05)
P = lambda m: np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]) * 1000
q0 = json.load(open(ROOT + '/grid_calib/manifest.json'))['q0']
log = open(OUT + '/grasp_log.jsonl', 'a')
def rec(**k): k['t'] = time.time(); log.write(json.dumps(k, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)) + '\n'); log.flush()
def move_to(cp_mm):
    m = PoseStamped(); m.header.frame_id = L['sp'].header.frame_id
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, np.asarray(cp_mm) / 1000)
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q0
    L.pop('reached', None); pub.publish(m); t = time.time()
    while 'reached' not in L and time.time() - t < 15: time.sleep(0.02)
    time.sleep(1.0); return L.get('reached')
def jaw(q):
    m = JointState(); m.position = [float(q)]; pj.publish(m); time.sleep(1.5)
def capture(name, k=6):
    grab['n'] = k; grab['buf'] = []; t = time.time()
    while len(grab['buf']) < k and time.time() - t < 10: time.sleep(0.02)
    b = grab['buf']; grab['buf'] = None; Lm = np.mean([x[0] for x in b], 0); Rm = np.mean([x[1] for x in b], 0)
    cv2.imwrite(f'{OUT}/{name}_right.png', Rm.round().astype(np.uint8)); cv2.imwrite(f'{OUT}/{name}_left.png', Lm.round().astype(np.uint8)); return Lm, Rm
B0 = cv2.imread(ROOT + '/grid_calib/base_0/right_rect_mean.png', 0).astype(np.float32); U0, V0, h = 600, 450, 35; T0 = B0[V0 - h:V0 + h + 1, U0 - h:U0 + h + 1]
def sub1(r, j):
    if 0 < j < len(r) - 1:
        p, q, s = r[j - 1], r[j], r[j + 1]; den = p - 2 * q + s; return j + (0.5 * (p - s) / den if den < 0 else 0)
    return float(j)
def track(name):
    """feature position in camera mm; search around the affine prediction from the current measured_cp"""
    Lm, Rm = capture(name); R = cv2.cvtColor(Rm.astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(np.float32); Lg = cv2.cvtColor(Lm.astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(np.float32)
    Xp = P(L['cp']) @ A3 + b3; cu, cv_ = FX * Xp[0] / Xp[2] + CX, FX * Xp[1] / Xp[2] + CY; best = None
    for s in (0.95, 1.0, 1.05, 1.1, 1.15, 1.2):
        Ts = cv2.resize(T0, None, fx=s, fy=s); th, tw = Ts.shape; rad = 90
        x0, y0 = int(max(cu - rad - tw / 2, 0)), int(max(cv_ - rad - th / 2, 0)); x1, y1 = int(min(cu + rad + tw / 2, 1300)), int(min(cv_ + rad + th / 2, 1024))
        r = cv2.matchTemplate(R[y0:y1, x0:x1], Ts, cv2.TM_CCOEFF_NORMED); _, mx, _, (lx, ly) = cv2.minMaxLoc(r)
        if best is None or mx > best[0]: best = (mx, x0 + sub1(r[ly], lx) + tw / 2 - 0.5, y0 + sub1(r[:, lx], ly) + th / 2 - 0.5, Ts)
    mx, u, v, Ts = best; th, tw = Ts.shape; yi, xi = int(round(v - th / 2 + 0.5)), int(round(u - tw / 2 + 0.5)); T2 = R[yi:yi + th, xi:xi + tw]
    Rc = Rm[yi:yi + th, xi:xi + tw]; mk = ((Rc[:, :, 2] - Rc[:, :, 1]) < 45).astype(np.float32)
    lo = max(xi - 260, 0); r2 = np.nan_to_num(cv2.matchTemplate(Lg[yi:yi + th, lo:xi + tw - 30], T2, cv2.TM_CCOEFF_NORMED, mask=mk)[0], nan=-1, posinf=-1, neginf=-1); j = int(np.argmax(r2)); d = xi - (lo + sub1(r2, j)); Z = FB / d
    X = np.array([(u - CX) * Z / FX, (v - CY) * Z / FX, Z]); rec(ev='track', name=name, uv=[u, v], pred_uv=[cu, cv_], ncc=mx, ncc_d=float(r2[j]), X=X, cp=P(L['cp']))
    return X, mx, float(r2[j])

if a.phase == 'measure':
    X, c1, c2 = track('measure'); print('feature cam', X.round(2), 'ncc %.2f/%.2f' % (c1, c2), '| to hover', (HOVER - X).round(2), '| to final', (FINAL - X).round(2))
elif a.phase == 'correct':
    jaw(0.0)
    for it in range(4):
        X, c1, c2 = track(f'correct_{it}'); e = HOVER - X; print(f'iter {it}: feature {X.round(2)} ncc {c1:.2f}/{c2:.2f} err {e.round(2)} |e| {np.linalg.norm(e):.2f} mm', flush=True)
        assert c1 > 0.3 and c2 > 0.5 and X[2] < 101, 'tracking confidence too low or depth on pad, abort'
        if np.linalg.norm(e) < 0.7: break
        dcp = dcam2dcp(e); assert np.linalg.norm(dcp) < 6.0, f'correction too large {dcp}'
        ok = move_to(P(L['sp']) + dcp); rec(ev='correct_move', it=it, dcp=dcp, reached=ok)
elif a.phase == 'descend':
    st = OUT + '/descend_base.json'
    if a.fracs[0] <= 0.5:
        X, c1, c2 = track('pre_descend'); print('pre-descend feature', X.round(2), 'ncc %.2f/%.2f, |X-hover| %.2f' % (c1, c2, np.linalg.norm(X - HOVER)), flush=True)
        assert np.linalg.norm(X - HOVER) < 1.5, 'not at hover, run --phase correct first'
        json.dump({'base_sp_mm': P(L['sp']).tolist()}, open(st, 'w'))
    jaw(0.8); base = np.array(json.load(open(st))['base_sp_mm']); total = dcam2dcp(FINAL - HOVER)
    for k, frac in enumerate(a.fracs):
        ok = move_to(base + frac * total); capture(f'descend_{k}'); rec(ev='descend', k=k, frac=frac, reached=ok, cp=P(L['cp']))
        print(f'descend step {k} ({frac * 100:.0f}%) reached {ok}', flush=True)
elif a.phase == 'grab':
    jaw(-0.15); capture('closed'); cp = P(L['sp']); lift = dcam2dcp(10.0 * n); ok = move_to(cp + lift); time.sleep(2.0); capture('lifted', 10)
    rec(ev='grab', lift_dcp=lift, reached=ok, jaw=list(L['jaw'].position) if 'jaw' in L else None); print('closed, lifted 10 mm, reached', ok, 'jaw', list(L['jaw'].position) if 'jaw' in L else None)
elif a.phase == 'descend_closed':
    jaw(0.0); PRE = FINAL + 2.5 * n
    for it in range(8):
        X, c1, c2 = track(f'dclosed_{it}'); e = PRE - X; print(f'closed-descent iter {it}: feature {X.round(2)} ncc {c1:.2f}/{c2:.2f} err {e.round(2)} |e| {np.linalg.norm(e):.2f}', flush=True)
        assert c1 > 0.3 and c2 > 0.5 and X[2] < 104, 'tracking confidence too low or depth on pad, abort'
        if np.linalg.norm(e) < 0.5: break
        step = e if np.linalg.norm(e) < 3.0 else 3.0 * e / np.linalg.norm(e)
        ok = move_to(P(L['sp']) + dcam2dcp(step)); rec(ev='dclosed_move', it=it, step=step, reached=ok)
elif a.phase == 'finish':
    jaw(0.8); capture('open_low'); ok = move_to(P(L['sp']) + dcam2dcp(-2.5 * n)); capture('at_final'); jaw(-0.15); capture('closed')
    ok2 = move_to(P(L['sp']) + dcam2dcp(10.0 * n)); time.sleep(2.0); capture('lifted', 10)
    rec(ev='finish', reached=[ok, ok2], jaw=list(L['jaw'].position) if 'jaw' in L else None); print('finish: descended 2.5 mm, closed, lifted 10 mm', ok, ok2, flush=True)
rclpy.shutdown()
