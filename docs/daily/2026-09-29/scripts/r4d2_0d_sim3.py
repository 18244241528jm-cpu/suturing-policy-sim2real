#!/usr/bin/env python3
"""R4 day-2 0-d: SE(3) vs Sim(3) (+ TCP) on the R2 tip session, random 25% holdout x 20 (addendum 0-d).
Reads R2 data read-only; same tip pixels / disparity choices as r2_tip_solve2.py (recomputed, not imported: that script writes into R2)."""
import json, os, numpy as np, cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
FX, CX, CY, FB = 1860.724932, 690.538345, 576.803638, 10136.551
base = 'records/logs/R2_20260922/E_tip_session'; out = 'records/logs/R4_20260928/day2/0d_sim3'
v2 = json.load(open(base + '/tip_solution_v2.json')); sol = json.load(open(base + '/tip_solution.json'))
amb = {int(k): v['chosen'][0] for k, v in v2['ambiguous_disparity_resolved_by_model'].items()}
P = {}
for r in sol['rows']:
    if not r['usable']: continue
    i = r['pose']; Rm = cv2.imread(f'{base}/pose_{i:02d}/right_rect_mean.png', 0).astype(np.float32); Lm = cv2.imread(f'{base}/pose_{i:02d}/left_rect_mean.png', 0).astype(np.float32)
    x, y = map(int, r['tip_R']); h = 20; T = Rm[y - h:y + h + 1, x - h:x + h + 1]
    ds = np.arange(60, 200); c = np.nan_to_num(np.array([np.corrcoef(T.ravel(), Lm[y - h:y + h + 1, x - d - h:x - d + h + 1].ravel())[0, 1] if x - d - h >= 0 else np.nan for d in ds]), nan=-9)
    pk = sorted([k for k in range(1, len(c) - 1) if c[k] >= c[k - 1] and c[k] >= c[k + 1] and c[k] > 0.3], key=lambda k: -c[k])[0]
    den = c[pk - 1] - 2 * c[pk] + c[pk + 1]; d0 = ds[pk] + (0.5 * (c[pk - 1] - c[pk + 1]) / den if den < 0 else 0)
    d = amb.get(i, d0); Z = FB / d
    fr = json.load(open(f'{base}/pose_{i:02d}/pose.json'))['frames']
    P[i] = dict(X=np.array([(x - CX) * Z / FX, (y - CY) * Z / FX, Z]), tl=np.mean([f['psm_local']['p'] for f in fr], 0) * 1000, Rl=Rot.from_quat(fr[0]['psm_local']['q_xyzw']).as_matrix())
ids = sorted(P)

def fit(tr, sim):
    Xc = np.array([P[i]['X'] for i in tr]); A = np.array([P[i]['tl'] for i in tr])
    ma, mb = A.mean(0), Xc.mean(0); U, _, Vt = np.linalg.svd((Xc - mb).T @ (A - ma)); Dg = np.eye(3); Dg[2, 2] = np.sign(np.linalg.det(U @ Vt)); R0 = U @ Dg @ Vt
    def res(x):
        R = Rot.from_rotvec(x[:3]).as_matrix(); s = np.exp(x[9]) if sim else 1.0
        return np.concatenate([s * R @ (P[i]['Rl'] @ x[6:9] + P[i]['tl']) + x[3:6] - P[i]['X'] for i in tr])
    x0 = np.r_[Rot.from_matrix(R0).as_rotvec(), mb - R0 @ ma, 0, 0, 20, 0.0][:10 if sim else 9]
    r = least_squares(res, x0, method='lm'); return r.x, np.linalg.norm(r.fun.reshape(-1, 3), axis=1)

def pred(x, i, sim):
    R = Rot.from_rotvec(x[:3]).as_matrix(); s = np.exp(x[9]) if sim else 1.0
    return s * R @ (P[i]['Rl'] @ x[6:9] + P[i]['tl']) + x[3:6]

rms = lambda e: float(np.sqrt(np.mean(np.square(e))))
res = {'n_poses': len(ids), 'pose_ids': ids, 'full': {}}
for name, sim in (('SE3', False), ('Sim3', True)):
    x, e = fit(ids, sim)
    res['full'][name] = {'rms_mm': rms(e), 'max_mm': float(e.max()), 'tcp_mm': x[6:9].round(3).tolist(), 'scale': float(np.exp(x[9])) if sim else 1.0, 'per_pose_mm': dict(zip(map(str, ids), e.round(2).tolist()))}
rng = np.random.default_rng(20260929); reps = []
for k in range(20):
    perm = rng.permutation(ids); te, tr = sorted(perm[:4].tolist()), sorted(perm[4:].tolist()); row = {'test': te}
    for name, sim in (('SE3', False), ('Sim3', True)):
        x, e = fit(tr, sim); et = [float(np.linalg.norm(pred(x, i, sim) - P[i]['X'])) for i in te]
        row[name] = {'train_rms': rms(e), 'holdout_rms': rms(et), 'holdout_max': max(et), 'scale': float(np.exp(x[9])) if sim else 1.0}
    reps.append(row)
hs, hm = [r['SE3']['holdout_rms'] for r in reps], [r['Sim3']['holdout_rms'] for r in reps]
ratio = float(np.median(hm) / np.median(hs))
res['holdout_20reps'] = reps
res['summary'] = {'SE3_holdout_rms_median_p25_p75': np.percentile(hs, [50, 25, 75]).round(3).tolist(), 'Sim3_holdout_rms_median_p25_p75': np.percentile(hm, [50, 25, 75]).round(3).tolist(),
                  'Sim3_scale_median_min_max': [float(np.median([r['Sim3']['scale'] for r in reps])), float(min(r['Sim3']['scale'] for r in reps)), float(max(r['Sim3']['scale'] for r in reps))],
                  'reps_Sim3_better': int(sum(a < b for a, b in zip(hm, hs))), 'ratio_median_Sim3_over_SE3': ratio,
                  'decision_scale_main_source (ratio<=0.70)': ratio <= 0.70}
os.makedirs(out, exist_ok=True); json.dump(res, open(out + '/result.json', 'w'), indent=1)
print(json.dumps({'full': {k: {kk: v[kk] for kk in ('rms_mm', 'max_mm', 'tcp_mm', 'scale')} for k, v in res['full'].items()}, 'summary': res['summary']}, indent=1))
