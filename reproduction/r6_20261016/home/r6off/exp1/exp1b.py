#!/usr/bin/env python3
"""EXP1 post-hoc diagnostics (pre_registration_addendum_1.json): a) hover geometry, b) staged start inside support, c) regression on the checkpoint's own pair.
Run (WSL): /usr/bin/python3 ~/r6off/exp1/exp1b.py"""
import os
_src = open(os.path.expanduser('~/r6off/exp1/exp1.py')).read().split('# ---------------- sensitivity grid')[0]
exec(compile(_src, 'exp1_head', 'exec'))     # same helpers, calibration and attempt data as exp1.py (no grid run)
from surgicai_rl_deploy.contract import R6_START_OFFSET_TOOL_MEAN, R6_REPRESENTATIVE_START_VEC7, R6_REPRESENTATIVE_GOAL_VEC7

OUT_B = {}
# ---------- a) hover geometry ----------
GA = {}
for t in TR:
    a = A[t]
    if a['hover_cp'] is None: continue
    R = Rot.from_quat(a['hover_q']).as_matrix()
    p = ecm_from_cam(a['p_cam']) / 1000.0
    cp_grasp = p - R @ (TCP / 1000.0)
    u = R.T @ (a['hover_cp'] - cp_grasp) * 1000.0
    tip_h = a['hover_cp'] + R @ (TCP / 1000.0)
    v_ecm = (tip_h - p) * 1000.0
    pj = json.load(open(f'{IN}/trials/{t}/perception.json')); n_cam = np.array(pj['normal_toward_cam'])
    v_cam = v_ecm @ M[:3]                                                   # linear part only (a difference vector)
    along_n = float(v_cam @ n_cam); lat = float(np.linalg.norm(v_cam - along_n * n_cam))
    GA[t] = {'u_tool_mm': u.tolist(), 'u_lateral_xy_mm': float(np.linalg.norm(u[:2])), 'u_z_mm': float(u[2]),
             'kin_tip_height_above_grasp_along_pad_normal_mm': along_n, 'kin_tip_lateral_to_grasp_mm': lat, 'kin_tip_dist_to_grasp_mm': float(np.linalg.norm(v_ecm))}
    print('a)', t, 'u_tool %s lateral_xy %.1f u_z %.1f | tip above p* along normal %.1f lateral %.1f' % (np.round(u, 1), GA[t]['u_lateral_xy_mm'], u[2], along_n, lat))
OUT_B['a_hover_geometry'] = GA
lat = [v['u_lateral_xy_mm'] for v in GA.values()]; uz = [v['u_z_mm'] for v in GA.values()]
print('a) median lateral %.1f mm [%.1f, %.1f]; median u_z %.1f mm [%.1f, %.1f]' % (np.median(lat), min(lat), max(lat), np.median(uz), min(uz), max(uz)))
OUT_B['a_summary'] = {'lateral_median': float(np.median(lat)), 'lateral_min': float(min(lat)), 'lateral_max': float(max(lat)), 'uz_median': float(np.median(uz)), 'uz_min': float(min(uz)), 'uz_max': float(max(uz)), 'n': len(lat)}

# ---------- b) staged start inside support ----------
Rel = Rot.from_rotvec(R6_START_TO_GOAL_ROTVEC_MEAN).as_matrix()
OFF = np.asarray(R6_START_OFFSET_TOOL_MEAN) / 100.0
GB = {}
for orient in ('GO_day', 'GO_rel'):
    GB[orient] = {}
    for t in TR:
        a = A[t]
        if orient == 'GO_day' and a['hover_cp'] is None: continue
        goal, start0 = goal_pose(a, orient, 0.007)
        Rs = goal.R @ Rel.T
        start = Pose(goal.p - Rs @ OFF, Rs, 0.0)
        r = run_policy(start, goal, 0.76)
        p, R = endpoint_robot(r['Xb'], r['hist'][r['first'] if r['first'] is not None else 200])
        off = (start.R.T @ (goal.p - start.p)) * 100.0; rot = float(np.degrees(rotation_error_rad(start, goal)))
        GB[orient][t] = {'first_success_step': r['first'], 'd_goal_mm': float(np.linalg.norm(p - goal.p) * 1000.0), 'rot_goal_deg': float(np.degrees(Rot.from_matrix(R.T @ goal.R).magnitude())),
                         'in_support': len(CT.in_support(off, rot)) == 0, 'workspace_violation': bool(r['ws'])}
        if a['hover_cp'] is not None: GB[orient][t]['d_hover_mm'] = float(np.linalg.norm(p - a['hover_cp']) * 1000.0)
    ok = [v for v in GB[orient].values() if v['first_success_step'] is not None]
    print('b)', orient, 'n', len(GB[orient]), 'in_support', sum(v['in_support'] for v in GB[orient].values()), 'first-success', len(ok),
          'median d_goal at first success %.1f mm' % (np.median([v['d_goal_mm'] for v in ok]) if ok else float('nan')),
          'median steps %s' % (np.median([v['first_success_step'] for v in ok]) if ok else None))
OUT_B['b_staged'] = GB

# ---------- c) regression on the checkpoint's own pair ----------
def roll(sv, gv, steps=200):
    state = np.array(sv, float).copy(); g = np.array(gv, float).copy(); first = None
    for k in range(1, steps + 1):
        a = pol.act(build_observation(state, g)); state = state + a.astype(float) * STEP; state[6] = float(np.clip(state[6], 0, 1))
        if first is None and np.linalg.norm(state[:3] - g[:3]) * 100.0 < CT.success_trans_cm and np.linalg.norm(state[3:6] - g[3:6]) <= CT.success_rot_rad: first = k
    return first, float(np.linalg.norm(state[:3] - g[:3]) * 1000.0)
f0, d0 = roll(R6_REPRESENTATIVE_START_VEC7, R6_REPRESENTATIVE_GOAL_VEC7)
rng = np.random.default_rng(20261016); n_ok = 0; steps = []
for i in range(20):
    sv = np.array(R6_REPRESENTATIVE_START_VEC7, float).copy(); sv[:3] += rng.normal(0, 0.005, 3); sv[3:6] += rng.normal(0, np.radians(5), 3)
    f, d = roll(sv, R6_REPRESENTATIVE_GOAL_VEC7); n_ok += f is not None; steps.append(f)
print('c) representative pair: first success step', f0, 'final d_goal %.1f mm' % d0, '| 20 noisy draws success', n_ok, '/ 20', 'median steps', np.median([s for s in steps if s]) if n_ok else None)
OUT_B['c_regression'] = {'representative_first_success': f0, 'noisy_draws_success': int(n_ok), 'n_draws': 20, 'steps': steps}
json.dump(OUT_B, open(OUT + '/exp1b_results.json', 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
print('wrote exp1b_results.json')
