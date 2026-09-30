#!/usr/bin/env python3
"""R4 day-2 0-b: where does the 1-2% stereo-vs-PnP gap come from? (addendum 0-b; day-1 F-2 data, read-only)
Per placement: rel = median(Z_method - Z_gt)/Z_gt (FS1, SG100), board centroid (u_c, v_c) from frame-000 PnP board mask.
Models: M_Z rel=a+bZ; M_full adds tiltB, u_c, v_c (partial F-test); M_disp Z_meas = fB/(fB/Z + delta).
Run with ~/venv_ambf/bin/python (cv2 aruco)."""
import glob, json, os, sys, numpy as np, cv2
from scipy import stats, optimize
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import r4_board_pnp as B
F2 = 'records/logs/R4_20260928/F2_board'; OUT = 'records/logs/R4_20260928/day2/0b_calib'
FB = 1860.724932 * 5.4476353950418295
rows = []
for d in sorted(glob.glob(F2 + '/p??')):
    r = json.load(open(d + '/f2_result.json')); K = B.K_rect(d)
    g0 = cv2.imread(f'{d}/right_rect/000.png', 0); H, W = g0.shape; p = B.pnp(g0, K); m = B.board_mask(p['img'], H, W)
    vv, uu = np.nonzero(m); Z = r['gt']['board_Z_median_mm']
    row = {'pl': os.path.basename(d), 'Z_gt': Z, 'tilt': r['gt']['tilt_deg'], 'tiltB': int(r['gt']['tilt_deg'] > 10),
           'u_c': float((uu.mean() - K[0, 2]) / 100), 'v_c': float((vv.mean() - K[1, 2]) / 100), 'board_px': int(m.sum())}
    for mth in ('FS1', 'SG100'):
        mm = r['methods'][mth]; row[mth] = mm.get('median_rel_pct') / 100 if mm.get('valid_frac', 0) >= 0.3 and 'median_rel_pct' in mm else None
    rows.append(row)

def ols(X, y):
    beta, *_ = np.linalg.lstsq(X, y, rcond=None); res = y - X @ beta; dof = len(y) - X.shape[1]; s2 = res @ res / dof
    se = np.sqrt(np.diag(s2 * np.linalg.inv(X.T @ X))); t = stats.t.ppf(0.975, dof)
    return beta, se, np.c_[beta - t * se, beta + t * se], float(res @ res), dof

out = {'rows': rows, 'fB_px_mm': FB, 'models': {}}
for mth in ('FS1', 'SG100'):
    R = [r for r in rows if r[mth] is not None]; y = np.array([r[mth] for r in R]); Z = np.array([r['Z_gt'] for r in R])
    XZ = np.c_[np.ones(len(R)), Z]; bz, sez, ciz, rssz, dofz = ols(XZ, y)
    Xf = np.c_[XZ, [r['tiltB'] for r in R], [r['u_c'] for r in R], [r['v_c'] for r in R]]; bf, sef, cif, rssf, doff = ols(Xf, y)
    Fs = ((rssz - rssf) / 3) / (rssf / doff); pF = float(1 - stats.f.cdf(Fs, 3, doff))
    Zm = Z * (1 + y)
    fd = optimize.least_squares(lambda x: FB / (FB / Z + x[0]) * x[1] - Zm, [0.0, 1.0]); Jd = fd.jac; s2 = fd.fun @ fd.fun / (len(Z) - 2)
    sed = np.sqrt(np.diag(s2 * np.linalg.inv(Jd.T @ Jd))); tq = stats.t.ppf(0.975, len(Z) - 2)
    fd0 = optimize.least_squares(lambda x: FB / (FB / Z + x[0]) - Zm, [0.0]); s20 = fd0.fun @ fd0.fun / (len(Z) - 1); se0 = np.sqrt(s20 / (fd0.jac.T @ fd0.jac)[0, 0])
    cls = {'Z-invariant (B or f scale)': bool(ciz[1, 0] <= 0 <= ciz[1, 1] and not (ciz[0, 0] <= 0 <= ciz[0, 1])),
           'Z-linear (disparity zero point)': bool(not (ciz[1, 0] <= 0 <= ciz[1, 1])),
           'tilt/image position': pF < 0.05}
    out['models'][mth] = {'n': len(R),
        'M_Z': {'a': bz[0], 'b_per_mm': bz[1], 'a_ci': ciz[0].tolist(), 'b_ci': ciz[1].tolist(), 'rel_at_Z': {z: float(bz[0] + bz[1] * z) for z in (55, 85, 100, 120, 150)}, 'rss': rssz},
        'M_full': {'coef[a,b,tiltB,u_c,v_c]': bf.tolist(), 'ci': cif.tolist(), 'rss': rssf, 'partial_F': float(Fs), 'p': pF},
        'M_disp_delta_only': {'delta_px': float(fd0.x[0]), 'delta_ci': [float(fd0.x[0] - stats.t.ppf(0.975, len(Z) - 1) * se0), float(fd0.x[0] + stats.t.ppf(0.975, len(Z) - 1) * se0)], 'rms_mm': float(np.sqrt(np.mean(fd0.fun ** 2)))},
        'M_disp_delta_plus_scale': {'delta_px': float(fd.x[0]), 'scale': float(fd.x[1]), 'ci': [[float(fd.x[k] - tq * sed[k]), float(fd.x[k] + tq * sed[k])] for k in range(2)], 'rms_mm': float(np.sqrt(np.mean(fd.fun ** 2)))},
        'classification': cls}
os.makedirs(OUT, exist_ok=True); json.dump(out, open(OUT + '/result.json', 'w'), indent=1, default=float)
for r in rows: print({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})
print(json.dumps(out['models'], indent=1, default=float))
