#!/usr/bin/env python3
"""R6 needle perception for one trial (no robot I/O).

input : 1-3 snapshot dirs (each: right_rect.png = physical left, fs_disp.npy from r6_fs_remote.sh, fs_host.json), plus
        clicks.json {"polyline_px": [[u,v],...] along the needle from tail (swage/thread end) to tip, "method": "..."}
mask  : pixels within BAND px of the clicked polyline that are darker than the local pad (gray < annulus median - DARK)
fit   : r2_arc.fit on FS points (+0.5 mm along the ray: wire surface -> centreline), grasp point / tangent at s = 0.5
        (s measured from the tail end, i.e. the clicked first point)
checks: R in [6, 13] mm, arc in [100, 240] deg, rms <= 1.6 mm (amendment 1), >= 60 points; with >= 2 frames: p* spread <= 1.0 mm,
        tangent spread <= 10 deg, normal spread <= 10 deg
output: <trial>/perception.json (fresh, 'created' + snapshot stamps) and overlay png per frame"""
import argparse, json, os, sys, time, numpy as np, cv2
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE + '/../r2_realrobot'); sys.path.insert(0, HERE + '/../r5_realrobot')
import r2_arc, r5_grasp_v3 as G
ap = argparse.ArgumentParser(); ap.add_argument('--trial', required=True); ap.add_argument('--snaps', nargs='+', required=True)
ap.add_argument('--clicks', required=True); ap.add_argument('--s', type=float, default=0.5); a = ap.parse_args()
BAND, DARK, OFF = 5, 18, 0.5
ck = json.load(open(a.clicks)); poly = np.array(ck['polyline_px'], float)

def band_mask(shape):
    m = np.zeros(shape, np.uint8); cv2.polylines(m, [poly.round().astype(np.int32)], False, 1, thickness=2 * BAND + 1); return m.astype(bool)
def annulus(shape):
    m = np.zeros(shape, np.uint8); cv2.polylines(m, [poly.round().astype(np.int32)], False, 1, thickness=2 * (BAND + 10) + 1); return m.astype(bool) & ~band_mask(shape)

res = []
for sd in a.snaps:
    img = cv2.imread(sd + '/right_rect.png'); g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(float)
    bm, an = band_mask(g.shape), annulus(g.shape)
    gm = cv2.blur(np.where(an, g, 0), (41, 41)) / np.maximum(cv2.blur(an.astype(float), (41, 41)), 1e-6)   # local pad level from the annulus
    mask = bm & (g < gm - DARK)
    d = np.load(sd + '/fs_disp.npy'); Z = G.FB / np.clip(d, 1, None)
    # amendment 2 (D13): FS flattens the thin wire -> take depth from the pad on both sides of the needle:
    # quadratic surface Z = f(x/z, y/z) fitted (robust) to annulus pad pixels (pink, 10-25 px from the polyline), rays of the
    # needle-mask pixels intersected with it, then +0.5 mm along the local surface normal (wire centreline above the surface)
    img_pink = (img[..., 2].astype(int) - img[..., 1] > 30)
    sup = annulus(g.shape) & img_pink & np.isfinite(Z)
    ys, xs = np.nonzero(sup); a_ = (xs - G.CX) / G.FX; b_ = (ys - G.CY) / G.FX; zz = Z[ys, xs]
    Aq = lambda a_, b_: np.c_[np.ones_like(a_), a_, b_, a_ * a_, a_ * b_, b_ * b_]
    keep = np.ones(len(zz), bool)
    for _ in range(3):
        coef = np.linalg.lstsq(Aq(a_[keep], b_[keep]), zz[keep], rcond=None)[0]; r_ = zz - Aq(a_, b_) @ coef; sdv = r_[keep].std(); keep = np.abs(r_) < 2.5 * sdv + 1e-6
    ym, xm = np.nonzero(mask); am, bm = (xm - G.CX) / G.FX, (ym - G.CY) / G.FX; zm = Aq(am, bm) @ coef
    Psurf = np.c_[am * zm, bm * zm, zm]
    cs = Psurf.mean(0); ns = np.linalg.svd(Psurf - cs)[2][2]; ns = ns if ns[2] < 0 else -ns          # local surface normal (toward cam)
    P = Psurf + OFF * ns; fit = r2_arc.fit(P)
    def snormal(a0, b0, e=1e-3):
        S = lambda a1, b1: np.array([a1, b1, 1.0]) * float(Aq(np.array([a1]), np.array([b1])) @ coef)
        v = np.cross(S(a0 + e, b0) - S(a0 - e, b0), S(a0, b0 + e) - S(a0, b0 - e)); v /= np.linalg.norm(v); return v if v[2] < 0 else -v
    ends = [poly[0], poly[len(poly) // 2], poly[-1]]; nrm = [snormal((q[0] - G.CX) / G.FX, (q[1] - G.CY) / G.FX) for q in ends]
    nvar = max(np.degrees(np.arccos(min(1, abs(nrm[i] @ nrm[j])))) for i in range(3) for j in range(i + 1, 3))
    surf_info = {'normal_variation_deg_tail_mid_tip': float(nvar), 'n_support': int(keep.sum()), 'resid_std_mm': float(sdv), 'surface_normal_toward_cam': ns.tolist(),
                 'fs_wire_minus_surface_mm_median': float(np.median(Z[ym, xm] - zm))}
    # orient the arc so that s=0 is the tail (first clicked point)
    ea, eb = np.array(fit['end_a_mm']), np.array(fit['end_b_mm']); pa, pb = r2_arc.project(ea)[0], r2_arc.project(eb)[0]
    tail_is_a = np.linalg.norm(pa - poly[0]) <= np.linalg.norm(pb - poly[0])
    c, n, u, v, R, a0, span = G.arc_frame(fit)
    s_eff = a.s if tail_is_a else 1.0 - a.s
    p, t, nn = G.arc_point_tangent(fit, s_eff)
    tip_px = poly[-1]; tip3 = eb if tail_is_a else ea
    chk = {'flat_ok': surf_info['normal_variation_deg_tail_mid_tip'] <= 15.0, 'R_ok': 6 <= fit['R_mm'] <= 13, 'arc_ok': 100 <= fit['arc_deg'] <= 240, 'rms_ok': fit['rms_mm'] <= 1.6, 'npts_ok': fit['n_points'] >= 60}
    ov = img.copy(); ov[mask] = (0, 255, 0)
    angs = np.radians(a0 + np.linspace(0, 1, 60) * span); arc3 = c + R * (np.cos(angs)[:, None] * u + np.sin(angs)[:, None] * v)
    cv2.polylines(ov, [r2_arc.project(arc3).round().astype(np.int32)], False, (255, 255, 0), 1)
    pp = r2_arc.project(p)[0]; pt = r2_arc.project(p + 3 * t)[0]; cv2.circle(ov, tuple(pp.round().astype(int)), 5, (0, 0, 255), 2); cv2.line(ov, tuple(pp.round().astype(int)), tuple(pt.round().astype(int)), (0, 0, 255), 2)
    cv2.imwrite(f'{a.trial}/overlay_{os.path.basename(sd.rstrip("/"))}.png', ov)
    fh = json.load(open(sd + '/fs_host.json')) if os.path.exists(sd + '/fs_host.json') else None
    st = json.load(open(sd + '/state.json'))
    res.append({'snap': sd, 'snap_time': st['t'], 'stamps': st['stamps'], 'fs_host': fh, 'fit': {k: v for k, v in fit.items()}, 'tail_is_end_a': bool(tail_is_a),
                'p_star_mm': p.tolist(), 'tangent': t.tolist(), 'normal_toward_cam': nn.tolist(), 'tip_mm': tip3.tolist(), 'checks': chk, 'n_mask_px': int(mask.sum()), 'surface': surf_info})
    print(os.path.basename(sd), 'R %.2f arc %.0f rms %.2f n %d' % (fit['R_mm'], fit['arc_deg'], fit['rms_mm'], fit['n_points']), 'p*', p.round(2), 'checks', all(chk.values()), flush=True)

ok = all(all(r['checks'].values()) for r in res); cons = None
if len(res) >= 2:
    Ps = np.array([r['p_star_mm'] for r in res]); Ts = np.array([r['tangent'] for r in res]); Ns = np.array([r['normal_toward_cam'] for r in res])
    ang = lambda A: max(np.degrees(np.arccos(min(1, abs(A[i] @ A[j])))) for i in range(len(A)) for j in range(i + 1, len(A)))
    cons = {'p_spread_mm': float(max(np.linalg.norm(Ps[i] - Ps[j]) for i in range(len(Ps)) for j in range(i + 1, len(Ps)))), 't_spread_deg': float(ang(Ts)), 'n_spread_deg': float(ang(Ns))}
    cons['ok'] = cons['p_spread_mm'] <= 1.0 and cons['t_spread_deg'] <= 10 and cons['n_spread_deg'] <= 10; ok = ok and cons['ok']
    print('consistency', cons)
# use the last frame's fit (median of p* over frames for the target)
pm = np.median([r['p_star_mm'] for r in res], 0); tm = np.array(res[-1]['tangent']); nm = np.array(res[-1]['normal_toward_cam'])
out = {'created': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'created_t': time.time(), 'method': {'needle_mask': f"clicked polyline band {BAND}px + darker-than-annulus {DARK}", 'clicks': ck,
       'depth': 'FoundationStereo 23-51-11 (laptop, on demand); needle points = rays x quadratic pad surface from the annulus (amendment 2, D13)', 'fit': 'r2_arc.fit, +0.5 mm ray offset', 's': a.s, 's_from': 'tail (first click)'},
       'frames': res, 'consistency': cons, 'p_star_mm': pm.tolist(), 'tangent': tm.tolist(), 'normal_toward_cam': nm.tolist(), 'ok': bool(ok)}
json.dump(out, open(a.trial + '/perception.json', 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else (bool(o) if isinstance(o, np.bool_) else str(o)))
print('PERCEPTION OK' if ok else 'PERCEPTION FAILED', pm.round(2))
import shutil   # amendment 3: archive everything needed for offline FP in xfer/<trial_id>/
X = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(a.trial))), 'xfer', os.path.basename(a.trial.rstrip('/'))); os.makedirs(X, exist_ok=True)
for sd in a.snaps:
    d = X + '/' + os.path.basename(sd.rstrip('/')); os.makedirs(d, exist_ok=True)
    for f in ('left_rect.png', 'right_rect.png', 'state.json', 'fs_disp.npy', 'fs_meta.json', 'fs_host.json'):
        if os.path.exists(sd + '/' + f): shutil.copy(sd + '/' + f, d + '/' + f)
for f in ('perception.json', 'clicks.json', 'start.json'):
    if os.path.exists(a.trial + '/' + f): shutil.copy(a.trial + '/' + f, X + '/' + f)
json.dump({'trial': a.trial, 'archived': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'note': 'left/right = TOPIC names; physical left = topic right; K: fx 1860.724932 cx 690.538345 cy 576.803638, baseline 5.4476 mm'}, open(X + '/meta.json', 'w'), indent=1)
