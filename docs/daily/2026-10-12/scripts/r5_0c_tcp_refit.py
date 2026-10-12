#!/usr/bin/env python3
"""R5 0-c: refit the R4 day-2 fixed-wrist grid (grid_calib/tracked_v3.json, read-only) with the TCP offset as an unknown.
Model: X_cam = f(p_cp + R_cp_i @ p_tcp), f in {SE3, Sim3, affine}; p_tcp in the measured_cp tool frame (mm).
Hold-out as R4: 25% of the 27 grid points left out (bases always train), 20 random splits, seed fixed.
Also reports how identifiable p_tcp is: the measured wrist orientation spread and the singular values of d(residual)/d(p_tcp).
usage: python3 r5_0c_tcp_refit.py --out records/logs/R5_<date>/0c_tcp"""
import argparse, json, math, os, numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--seed', type=int, default=20261012); a = ap.parse_args()
G = os.path.expanduser('~/suturing-policy-sim2real/records/logs/R4_20260928/day2/grid_calib')
trk = json.load(open(f'{G}/tracked_v3.json')); names = list(trk)
Rq = {n: Rot.from_quat(json.load(open(f'{G}/{n}/state.json'))['cp_q']).as_matrix() for n in names}
Xc = {n: np.array(trk[n]['X_cam_mm']) for n in names}; Pk = {n: np.array(trk[n]['cp_mm']) for n in names}
grid = [n for n in names if n.startswith('g')]; bases = [n for n in names if n.startswith('base')]

def fit(model, tcp, ns):
    """returns params vector and a predict(ns) closure"""
    X = np.array([Xc[n] for n in ns])
    def pts(ns_, t):
        return np.array([Pk[n] + Rq[n] @ t for n in ns_])
    if model == 'affine':
        def solve_aff(t):
            A = np.c_[pts(ns, t), np.ones(len(ns))]; return np.linalg.lstsq(A, X, rcond=None)[0]
        if tcp:
            res = lambda t: (np.c_[pts(ns, t), np.ones(len(ns))] @ solve_aff(t) - X).ravel()
            t = least_squares(res, np.zeros(3), x_scale=10.0).x
        else:
            t = np.zeros(3)
        M = solve_aff(t); return t, (lambda ns_: np.c_[pts(ns_, t), np.ones(len(ns_))] @ M)
    def unpack(x):
        R = Rot.from_rotvec(x[:3]).as_matrix(); tr = x[3:6]; s = np.exp(x[6]) if model == 'Sim3' else 1.0
        t = x[-3:] if tcp else np.zeros(3); return R, tr, s, t
    def pred(x, ns_):
        R, tr, s, t = unpack(x); return s * pts(ns_, t) @ R.T + tr
    # init from Umeyama without TCP
    A = pts(ns, np.zeros(3)); ma, mb = A.mean(0), X.mean(0); U, S, Vt = np.linalg.svd((X - mb).T @ (A - ma)); D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R0 = U @ D @ Vt; x0 = np.r_[Rot.from_matrix(R0).as_rotvec(), mb - R0 @ ma]
    if model == 'Sim3': x0 = np.r_[x0, 0.0]
    if tcp: x0 = np.r_[x0, np.zeros(3)]
    x = least_squares(lambda x: (pred(x, ns) - X).ravel(), x0).x
    return unpack(x)[3], (lambda ns_: pred(x, ns_))

rms = lambda e: float(np.sqrt((np.linalg.norm(e, axis=1) ** 2).mean()))
rng = np.random.default_rng(a.seed); k = math.ceil(0.25 * len(grid)); splits = [sorted(rng.permutation(grid)[:k].tolist()) for _ in range(20)]
out = {'source': f'{G}/tracked_v3.json (R4 day2, read-only)', 'seed': a.seed, 'n_grid': len(grid), 'n_bases': len(bases), 'holdout_per_split': k, 'models': {}}
for model in ('SE3', 'Sim3', 'affine'):
    for tcp in (False, True):
        key = model + ('+TCP' if tcp else ''); te = []; tcps = []
        for sp in splits:
            tr = [n for n in names if n not in sp]; t, P = fit(model, tcp, tr)
            te.append(rms(P(sp) - np.array([Xc[n] for n in sp]))); tcps.append(t.tolist())
        tf, Pf = fit(model, tcp, names)
        out['models'][key] = {'holdout_rms_mm': {'median': float(np.median(te)), 'p25': float(np.percentile(te, 25)), 'p75': float(np.percentile(te, 75)), 'min': float(min(te)), 'max': float(max(te)), 'all': te},
                              'full_fit_rms_mm': rms(Pf(names) - np.array([Xc[n] for n in names])), 'tcp_full_fit_mm': tf.tolist(),
                              'tcp_over_splits_mm': {'median': np.median(tcps, 0).tolist(), 'min': np.min(tcps, 0).tolist(), 'max': np.max(tcps, 0).tolist()} if tcp else None}
# identifiability
Rm = np.array([Rq[n] for n in names]); dev = [float(np.degrees(Rot.from_matrix(Rq[names[0]].T @ Rq[n]).magnitude())) for n in names]
J = np.concatenate([Rq[n] - Rm.mean(0) for n in names])          # d(point)/d(p_tcp) after removing the common (translation-absorbed) part
sv = np.linalg.svd(J, compute_uv=False)
p24 = np.array([-2.70, 1.08, 23.81])
out['identifiability'] = {'wrist_orientation_dev_from_first_deg': {'max': max(dev), 'median': float(np.median(dev))},
                          'sv_of_centered_dpoint_dtcp': sv.tolist(),
                          'max_point_shift_from_R2_tcp_beyond_common_translation_mm': float(max(np.linalg.norm((Rq[n] - Rm.mean(0)) @ p24) for n in names)),
                          'note': 'with a fixed wrist orientation a TCP offset is a constant translation and is absorbed by the transform; it cannot change displacement ratios'}
os.makedirs(a.out, exist_ok=True); json.dump(out, open(f'{a.out}/result.json', 'w'), indent=1)
for kk, m in out['models'].items():
    h = m['holdout_rms_mm']; print(f"{kk:12s} holdout RMS median {h['median']:.3f} [{h['p25']:.3f}, {h['p75']:.3f}]  full {m['full_fit_rms_mm']:.3f}  tcp {np.round(m['tcp_full_fit_mm'], 1)}")
print('identifiability', {k2: v for k2, v in out['identifiability'].items() if k2 != 'note'})
