#!/usr/bin/env python3
"""EXP1 policy offline rollout. Pre-registered in pre_registration.json. CPU only.
Run (WSL): /usr/bin/python3 ~/r6off/exp1/exp1.py"""
import os, sys, json, csv, glob, time, warnings
warnings.filterwarnings('ignore')
import numpy as np
from scipy.spatial.transform import Rotation as Rot, Slerp
ROOT = os.path.expanduser('~/r6off'); IN = ROOT + '/input'; OUT = ROOT + '/exp1'
sys.path.insert(0, OUT)
from surgicai_rl_deploy.contract import (CONTRACTS, R6_TRAINED_GOAL_VEC7, R6_START_TO_GOAL_ROTVEC_MEAN, TRAIN_WORKSPACE_LOW_M, TRAIN_WORKSPACE_HIGH_M)
from surgicai_rl_deploy.frames import Pose, FrameBridge, bound_roll, vec7_bound, rotation_error_rad
from surgicai_rl_deploy.obs import build_observation
from surgicai_rl_deploy.policy import ApproachPolicy

CK = os.path.expanduser('~/ref/daily-notes-ro/models/rl/r6_unified_single_goal_yaw15_seed1_final.zip')
pol = ApproachPolicy.load(CK, device='cpu', verify=True)
CT = CONTRACTS['approach_r6']; STEP = np.asarray(CT.step_size, float)
print('policy:', pol.describe(), '|', CT.describe(), flush=True)

# ---------------- calibration ----------------
cal = json.load(open(IN + '/calibration_summary.json'))
M = np.array(cal['affine_M_cp_ecm_mm_to_cam_mm (X_cam = [p_cp,1] @ M)']); TCP = np.array(cal['TCP_tool_mm'])
C_OFF = np.array([3.15, -12.88, -15.70])      # derived from preclose_pose.json (see pre-registration); refined below from the files
cs = []
for f in sorted(glob.glob(IN + '/trials/*/preclose_pose.json')):
    pc = json.load(open(f))
    for cam, ecm in ((pc['tip']['cam_mm'], pc['tip']['ecm_mm']), (pc['needle_cam']['grasp_point_mm'], pc['needle_ecm']['grasp_point_mm'])):
        cs.append(np.array(cam) - np.r_[ecm, 1] @ M)
cs = np.array(cs); C_OFF = cs.mean(0); C_SPREAD = float(np.abs(cs - C_OFF).max())
print('c offset', C_OFF.round(3), 'max dev over %d pairs %.4f mm' % (len(cs), C_SPREAD))
Minv = np.linalg.inv(M[:3])
U, S, Vt = np.linalg.svd(M[:3].T); RP = U @ Vt
P0 = np.mean([p['cp_mm'] for p in cal['train_points']], 0)
aff = lambda p: np.r_[p, 1] @ M


def ecm_from_cam(x_cam, kind='affine'):
    x = np.asarray(x_cam, float) - C_OFF
    if kind == 'affine':
        return (x - M[3]) @ Minv
    return P0 + RP.T @ (x - aff(P0))                      # SE(3): rigid, matches the affine at the training centroid


# ---------------- attempts ----------------
summ = {r['trial']: r for r in csv.DictReader(open(IN + '/trials_summary.csv'))}
TR = sorted(os.path.basename(p) for p in glob.glob(IN + '/trials/*') if os.path.isdir(p))
PLACE = [t for t in TR if t != 'T3b_C']
A = {}
for t in TR:
    st = json.load(open(f'{IN}/trials/{t}/snap0/state.json')); pj = json.load(open(f'{IN}/trials/{t}/perception.json'))
    ts = json.load(open(f'{IN}/trials/{t}/trial_state.json')) if os.path.exists(f'{IN}/trials/{t}/trial_state.json') else {}
    hov = (ts.get('hover') or {})
    A[t] = {'cp7': st['cp'], 'jaw_day': st['jaw'][0], 'p_cam': np.array(pj['p_star_mm']), 'p_spread': pj['consistency']['p_spread_mm'],
            'hover_cp': (np.array(hov['cp']) / 1000.0) if 'cp' in hov else None, 'hover_q': (np.array(hov['q']) if 'q' in hov else None),
            'place': 'T3_C' if t == 'T3b_C' else t}
print('hover reference available for', sum(a['hover_cp'] is not None for a in A.values()), 'of', len(A), flush=True)

BASE_PLACE_IDX = {t: i for i, t in enumerate(PLACE)}


def goal_pose(a, orient, standoff_m, kind='affine', dp_cam=None, drot=None):
    start = Pose.from_pos_quat(a['cp7'][:3], a['cp7'][3:7], 0.0)
    if orient == 'GO_day':
        if a['hover_q'] is None: return None, start
        R = Rot.from_quat(a['hover_q']).as_matrix()
    elif orient == 'GO_rel':
        R = start.R @ Rot.from_rotvec(R6_START_TO_GOAL_ROTVEC_MEAN).as_matrix()
    elif orient == 'GO_hold':
        R = start.R.copy()
    else: raise ValueError(orient)
    if drot is not None: R = drot @ R
    pc = a['p_cam'] + (0 if dp_cam is None else dp_cam)
    p = ecm_from_cam(pc, kind) / 1000.0                    # m
    cp_grasp = p - R @ (TCP / 1000.0)
    cp_goal = cp_grasp - R[:, 2] * standoff_m
    return Pose(cp_goal, R, 0.0), start


def run_policy(start, goal, jaw0, steps=200):
    traj_note = {}
    Xb = FrameBridge.rebase(goal, Pose.from_vec7(R6_TRAINED_GOAL_VEC7).with_jaw(0.0))
    s = vec7_bound(Xb.to_policy(start.with_jaw(jaw0)).with_jaw(jaw0)); s[6] = jaw0
    g = vec7_bound(Xb.to_policy(goal))
    g[6] = 0.0
    state = s.copy(); first = None; maxstep = 0.0; ws = False; hist = [state.copy()]
    for k in range(1, steps + 1):
        a = pol.act(build_observation(state, g))
        new = state + a.astype(np.float64) * STEP; new[6] = float(np.clip(new[6], 0.0, 1.0))
        maxstep = max(maxstep, float(np.linalg.norm(new[:3] - state[:3]) * 1000.0)); state = new; hist.append(state.copy())
        if (state[:3] < TRAIN_WORKSPACE_LOW_M).any() or (state[:3] > TRAIN_WORKSPACE_HIGH_M).any(): ws = True
        if first is None and np.linalg.norm(state[:3] - g[:3]) * 100.0 < CT.success_trans_cm and np.linalg.norm(state[3:6] - g[3:6]) <= CT.success_rot_rad:
            first = k
    hist = np.array(hist)
    return dict(Xb=Xb, g=g, s=s, first=first, hist=hist, maxstep=maxstep, ws=ws)


def endpoint_robot(Xb, vec):
    P = Xb.to_robot(Pose.from_vec7(vec)); return P.p, P.R


def one(a, orient, standoff_mm, jaw0, kind='affine', dp_cam=None, drot=None):
    goal, start = goal_pose(a, orient, standoff_mm / 1000.0, kind, dp_cam, drot)
    if goal is None: return None
    r = run_policy(start, goal, jaw0)
    off = (start.R.T @ (goal.p - start.p)) * 100.0; rot = float(np.degrees(rotation_error_rad(start, goal)))
    why = CT.in_support(off, rot)
    out = {'first_success_step': r['first'], 'max_step_mm': r['maxstep'], 'workspace_violation': bool(r['ws']), 'in_support': (len(why) == 0), 'support_reasons': why,
           'offset_tool_cm': off.tolist(), 'start_goal_rot_deg': rot}
    for tag, idx in (('first', r['first'] if r['first'] is not None else 200), ('end200', 200)):
        p, R = endpoint_robot(r['Xb'], r['hist'][idx])
        out[tag] = {'step': idx, 'd_goal_mm': float(np.linalg.norm(p - goal.p) * 1000.0), 'rot_goal_deg': float(np.degrees(Rot.from_matrix(R.T @ goal.R).magnitude())), 'cp_m': p.tolist()}
        if a['hover_cp'] is not None:
            qh = Rot.from_quat(a['hover_q']).as_matrix()
            out[tag]['d_hover_mm'] = float(np.linalg.norm(p - a['hover_cp']) * 1000.0); out[tag]['angle_hover_deg'] = float(np.degrees(Rot.from_matrix(R.T @ qh).magnitude()))
    if a['hover_cp'] is not None:
        out['goalcp_vs_hover_mm'] = float(np.linalg.norm(goal.p - a['hover_cp']) * 1000.0)
        out['goal_rot_vs_hover_deg'] = float(np.degrees(Rot.from_matrix(goal.R.T @ Rot.from_quat(a['hover_q']).as_matrix()).magnitude()))
    dist = np.linalg.norm(goal.p - start.p) * 1000.0
    out['baseline_straight'] = {'dist_mm': float(dist), 'rot_deg': rot, 'steps': int(max(np.ceil(dist / 1.0), np.ceil(rot / 3.0))), 'within_200': bool(max(np.ceil(dist / 1.0), np.ceil(rot / 3.0)) <= 200)}
    return out


# ---------------- sensitivity grid ----------------
GRID = {}
for orient in ('GO_day', 'GO_rel', 'GO_hold'):
    for so in (7.0, 10.0):
        for jaw0 in (0.76, 0.0):
            key = f'{orient}|standoff{so:g}|jaw{jaw0:g}'; GRID[key] = {}
            for t in TR:
                o = one(A[t], orient, so, jaw0)
                if o is not None: GRID[key][t] = o
            print('grid', key, 'n', len(GRID[key]), flush=True)


def tiers(res):
    rows = [v for v in res.values() if 'd_hover_mm' in v['first']]
    n = len(rows)
    return {'n_with_hover_ref': n, 'le3mm': sum(v['first']['d_hover_mm'] <= 3.0 for v in rows), 'le5mm': sum(v['first']['d_hover_mm'] <= 5.0 for v in rows),
            'median_d_hover_mm': float(np.median([v['first']['d_hover_mm'] for v in rows])) if n else None,
            'median_goalcp_vs_hover_mm': float(np.median([v['goalcp_vs_hover_mm'] for v in rows])) if n else None,
            'n_first_success': sum(v['first_success_step'] is not None for v in res.values()), 'n': len(res),
            'n_in_support': sum(v['in_support'] for v in res.values()), 'n_workspace_violation': sum(v['workspace_violation'] for v in res.values()),
            'max_step_mm': float(max(v['max_step_mm'] for v in res.values()))}
SUMM = {k: tiers(v) for k, v in GRID.items()}
for k, v in SUMM.items(): print('SUMMARY', k, v)

# ---------------- robustness ----------------
ROB = {}
for label, (orient, so, jaw0) in {'primary': ('GO_day', 7.0, 0.76), 'GO_rel': ('GO_rel', 7.0, 0.76)}.items():
    ROB[label] = {}
    for t in PLACE:
        a = A[t]
        if orient == 'GO_day' and a['hover_cp'] is None: continue
        pi = BASE_PLACE_IDX[t]; draws = []
        for k in range(5):
            rng = np.random.default_rng(20261016 * 100 + 10 * pi + k)
            dstart = rng.uniform(-0.010, 0.010, 3)                           # m, start position noise
            sig = max(a['p_spread'], 0.1); dp = rng.normal(0, sig, 3)           # mm, grasp point noise (cam frame)
            ang = np.radians(abs(rng.normal(0, 5.0))); ax = rng.normal(size=3); ax /= np.linalg.norm(ax); drot = Rot.from_rotvec(ang * ax).as_matrix()
            a2 = dict(a); cp7 = list(a['cp7']); cp7[0] += dstart[0]; cp7[1] += dstart[1]; cp7[2] += dstart[2]; a2['cp7'] = cp7
            r_aff = one(a2, orient, so, jaw0, 'affine', dp, drot); r_se3 = one(a2, orient, so, jaw0, 'se3', dp, drot)
            d = {'affine': r_aff, 'se3': r_se3}
            if r_aff and r_se3:
                pa, ps = np.array(r_aff['first']['cp_m']), np.array(r_se3['first']['cp_m'])
                d['affine_vs_se3_endpoint_mm'] = float(np.linalg.norm(pa - ps) * 1000.0)
            draws.append(d)
        ROB[label][t] = draws
    ok = [d['affine'] for v in ROB[label].values() for d in v if d['affine']]
    hv = [d for d in ok if 'd_hover_mm' in d['first']]
    sepdiff = [d['affine_vs_se3_endpoint_mm'] for v in ROB[label].values() for d in v if 'affine_vs_se3_endpoint_mm' in d]
    print('ROB', label, 'draws', len(ok), 'first_success', sum(d['first_success_step'] is not None for d in ok), 'le3', sum(d['first']['d_hover_mm'] <= 3 for d in hv), 'le5', sum(d['first']['d_hover_mm'] <= 5 for d in hv), 'of', len(hv),
          'median affine-vs-se3 endpoint diff %.2f mm' % np.median(sepdiff), flush=True)

json.dump({'policy': pol.describe(), 'contract': CT.describe(), 'c_offset_mm': C_OFF.tolist(), 'c_max_dev_mm': C_SPREAD, 'grid': GRID, 'summary': SUMM, 'robustness': ROB},
          open(OUT + '/exp1_results.json', 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
print('wrote exp1_results.json')
