#!/usr/bin/env python3
"""R4 day-2 grid calibration solver v2: metal-masked, multi-scale template tracking of a jaw feature + two-pass (wide, then around a
prediction from a robust first fit). Same split (seed 20260929, 25% of grid points) as r4d2_grid_calib.py.
usage: r4d2_grid_solve2.py --out DIR --u 560 --v 420 --h 40"""
import argparse, json, math, numpy as np, cv2
from scipy.spatial.transform import Rotation as Rot
ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--u', type=int, required=True); ap.add_argument('--v', type=int, required=True)
ap.add_argument('--h', type=int, default=40); a = ap.parse_args()
FX, CX, CY = 1860.724932, 690.538345, 576.803638; FB = FX * 5.4476353950418295
man = json.load(open(f'{a.out}/manifest.json')); names = [p['name'] for p in man['points']]
B0 = cv2.imread(f'{a.out}/base_0/right_rect_mean.png'); h = a.h
tpl = B0[a.v - h:a.v + h + 1, a.u - h:a.u + h + 1]; ti = tpl.astype(int)
mask0 = ((ti[:, :, 2] - ti[:, :, 1]) < 45).astype(np.uint8)
mask0 = cv2.morphologyEx(mask0, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
g = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).astype(np.float32)

def sub1(r, j):
    if 0 < j < len(r) - 1:
        p, q, s = r[j - 1], r[j], r[j + 1]; den = p - 2 * q + s; return j + (0.5 * (p - s) / den if den < 0 else 0)
    return float(j)

def track(nm, center=None, rad=None):
    R = cv2.imread(f'{a.out}/{nm}/right_rect_mean.png'); Lm = g(cv2.imread(f'{a.out}/{nm}/left_rect_mean.png')); Rg = g(R)
    best = None
    for s in (0.9, 0.95, 1.0, 1.05, 1.1, 1.15, 1.2, 1.25, 1.3):
        T = cv2.resize(g(tpl), None, fx=s, fy=s, interpolation=cv2.INTER_LINEAR); M = cv2.resize(mask0, (T.shape[1], T.shape[0]), interpolation=cv2.INTER_NEAREST).astype(np.float32)
        th, tw = T.shape
        if center is None: x0, y0, x1, y1 = 0, 0, 1300, 1024
        else: x0, y0 = int(max(center[0] - rad - tw / 2, 0)), int(max(center[1] - rad - th / 2, 0)); x1, y1 = int(min(center[0] + rad + tw / 2, 1300)), int(min(center[1] + rad + th / 2, 1024))
        if x1 - x0 <= tw or y1 - y0 <= th: continue
        r = cv2.matchTemplate(Rg[y0:y1, x0:x1], T, cv2.TM_CCOEFF_NORMED, mask=M); r = np.nan_to_num(r, nan=-1, posinf=-1, neginf=-1)
        _, mx, _, (lx, ly) = cv2.minMaxLoc(r)
        if best is None or mx > best[0]: best = (mx, s, x0 + sub1(r[ly], lx) + tw / 2 - 0.5, y0 + sub1(r[:, lx], ly) + th / 2 - 0.5, T, M)
    mx, s, u, v, T, M = best; th, tw = T.shape
    yi, xi = int(round(v - th / 2 + 0.5)), int(round(u - tw / 2 + 0.5)); T2 = Rg[yi:yi + th, xi:xi + tw]
    lo = max(xi - 260, 0); strip = Lm[yi:yi + th, lo:xi + tw - 30]
    r2 = cv2.matchTemplate(strip, T2, cv2.TM_CCOEFF_NORMED, mask=M)[0]; r2 = np.nan_to_num(r2, nan=-1); j = int(np.argmax(r2))
    d = xi - (lo + sub1(r2, j)); Z = FB / d
    st = json.load(open(f'{a.out}/{nm}/state.json'))
    return {'uv': [u, v], 'scale': s, 'ncc': float(mx), 'ncc_disp': float(r2[j]), 'disp': float(d), 'X_cam_mm': [(u - CX) * Z / FX, (v - CY) * Z / FX, Z], 'cp_mm': (np.array(st['cp_p']) * 1000).tolist()}

def umeyama(A, Bm, scale):
    ma, mb = A.mean(0), Bm.mean(0); U, S, Vt = np.linalg.svd((Bm - mb).T @ (A - ma) / len(A)); D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt; s = (S * np.diag(D)).sum() / ((A - ma) ** 2).sum() * len(A) if scale else 1.0; return s, R, mb - s * R @ ma

pts = {nm: track(nm) for nm in names}
# robust first fit on well-tracked points, then re-track everything near the prediction
good = [k for k, p in pts.items() if p['ncc'] > 0.85]
s, R, t = umeyama(np.array([pts[k]['cp_mm'] for k in good]), np.array([pts[k]['X_cam_mm'] for k in good]), False)
for _ in range(3):
    e = {k: np.linalg.norm(R @ np.array(pts[k]['cp_mm']) + t - np.array(pts[k]['X_cam_mm'])) for k in good}
    med = np.median(list(e.values())); good = [k for k in good if e[k] <= max(3 * med, 1.5)]
    s, R, t = umeyama(np.array([pts[k]['cp_mm'] for k in good]), np.array([pts[k]['X_cam_mm'] for k in good]), False)
pass1 = {k: dict(p) for k, p in pts.items()}
for nm in names:
    Xp = R @ np.array(pts[nm]['cp_mm']) + t; uvp = (FX * Xp[0] / Xp[2] + CX, FX * Xp[1] / Xp[2] + CY)
    pts[nm] = track(nm, center=uvp, rad=40); pts[nm]['pred_uv_pass1'] = [float(uvp[0]), float(uvp[1])]
grid = [nm for nm in names if nm.startswith('g')]; bases = [nm for nm in names if nm.startswith('base')]
perm = np.random.default_rng(20260929).permutation(len(grid)); test = sorted(grid[i] for i in perm[:math.ceil(0.25 * len(grid))]); train = [x for x in grid if x not in test] + bases
Xc = lambda ns: np.array([pts[k]['X_cam_mm'] for k in ns]); Xk = lambda ns: np.array([pts[k]['cp_mm'] for k in ns])
out = {'template': [a.u, a.v, h], 'train': train, 'test': test, 'pass1_inliers': good, 'points': pts, 'models': {}}
for name, sc in (('SE3', False), ('Sim3', True)):
    s, R, t = umeyama(Xk(train), Xc(train), sc)
    e_tr = np.linalg.norm((s * (R @ Xk(train).T)).T + t - Xc(train), axis=1); e_te = (s * (R @ Xk(test).T)).T + t - Xc(test)
    out['models'][name] = {'scale': float(s), 'R': R.tolist(), 't_mm': t.tolist(), 'train_rms_mm': float(np.sqrt((e_tr ** 2).mean())), 'train_max_mm': float(e_tr.max()),
                           'test_rms_mm': float(np.sqrt((np.linalg.norm(e_te, axis=1) ** 2).mean())), 'test_max_mm': float(np.linalg.norm(e_te, axis=1).max()),
                           'test_err_vec_mm': dict(zip(test, e_te.round(3).tolist())), 'train_err_mm': dict(zip(train, e_tr.round(3).tolist()))}
A = np.c_[Xk(train), np.ones(len(train))]; M = np.linalg.lstsq(A, Xc(train), rcond=None)[0]; e_aff = np.c_[Xk(test), np.ones(len(test))] @ M - Xc(test)
out['models']['affine_check'] = {'test_rms_mm': float(np.sqrt((np.linalg.norm(e_aff, axis=1) ** 2).mean())), 'singular_values': np.linalg.svd(M[:3]).S.tolist()}
bX = Xc(bases); out['base_repeatability_mm'] = float(np.linalg.norm(bX - bX.mean(0), axis=1).max())
json.dump(out, open(f'{a.out}/calibration_v2.json', 'w'), indent=1)
for k, m in out['models'].items(): print(k, {kk: (round(v, 3) if isinstance(v, float) else v) for kk, v in m.items() if kk in ('scale', 'train_rms_mm', 'train_max_mm', 'test_rms_mm', 'test_max_mm', 'singular_values')})
print('pass1 inliers %d/%d; base repeat %.2f mm; min ncc %.2f / %.2f' % (len(good), len(names), out['base_repeatability_mm'], min(p['ncc'] for p in pts.values()), min(p['ncc_disp'] for p in pts.values())))
for k in names: p = pts[k]; print(k, np.round(p['uv'], 1), 's %.2f ncc %.2f/%.2f Z %.1f' % (p['scale'], p['ncc'], p['ncc_disp'], p['X_cam_mm'][2]))
