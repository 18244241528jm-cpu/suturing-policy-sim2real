#!/usr/bin/env python3
"""Automatic needle segmentation on a physical-left rectified image (no ROI, no human input).

1. colour candidates: pixels clearly less red (R-G) or darker (max RGB) than the pad (pad stats estimated from the image itself)
2. thin-structure filter: remove wide blobs (tool, background, shadows) found by a morphological opening with a disk
3. RANSAC circle on the thin candidates (radius limited to a needle range in px, from --z-mm range and 6..12 mm)
4. arc extent: contiguous angular run of inliers; purple (suture thread, B-G high) runs are trimmed; span capped at --max-span
5. mask = colour candidates within the circle band and the angular range
Returns the mask plus diagnostics (radius, span, inlier fraction, band residual, confidence flag).
usage: r4d2_auto_seg.py IMAGE [--out MASK.png] [--overlay OV.png]
"""
import argparse, json, math, os, sys, numpy as np, cv2
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'scripts/r2_realrobot'))
FX = 1860.724932

def pad_stats(im):
    red = im[:, :, 2] - im[:, :, 1]; V = im.max(2)
    pink = red > 40
    return float(np.median(red[pink])), float(np.median(V[pink]))

def candidates(im, dred=22, dV=55, ksz=41):
    """local-contrast candidates: compared with the local median (removes vignetting / illumination gradients).
    A pixel is needle-like if it is clearly less red than its surroundings, or much darker AND somewhat less red
    (pad grooves/shadows get darker but stay red)."""
    red = np.clip(im[:, :, 2] - im[:, :, 1] + 128, 0, 255).astype(np.uint8); V = im.max(2).astype(np.uint8)
    sm = lambda x: cv2.resize(cv2.medianBlur(cv2.resize(x, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA), ksz // 4 * 2 + 1), (x.shape[1], x.shape[0]), interpolation=cv2.INTER_LINEAR).astype(int)
    dr = sm(red) - red.astype(int); dv = sm(V) - V.astype(int)
    c = ((dr > dred) | ((dv > dV) & (dr > 8))).astype(np.uint8)
    return cv2.morphologyEx(c, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))

def thin_only(c, wide_r=18):
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * wide_r + 1, 2 * wide_r + 1))
    wide = cv2.morphologyEx(c, cv2.MORPH_OPEN, k)
    wide = cv2.dilate(wide, np.ones((7, 7), np.uint8))
    t = c & (1 - wide)
    n, lab, st, _ = cv2.connectedComponentsWithStats(t)
    keep = np.zeros_like(t)
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a >= 40 and max(w, h) >= 25: keep[lab == i] = 1   # drop specks
    return keep, wide

def circle3(p):
    (x1, y1), (x2, y2), (x3, y3) = p
    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(d) < 1e-6: return None
    ux = ((x1 ** 2 + y1 ** 2) * (y2 - y3) + (x2 ** 2 + y2 ** 2) * (y3 - y1) + (x3 ** 2 + y3 ** 2) * (y1 - y2)) / d
    uy = ((x1 ** 2 + y1 ** 2) * (x3 - x2) + (x2 ** 2 + y2 ** 2) * (x1 - x3) + (x3 ** 2 + y3 ** 2) * (x2 - x1)) / d
    return ux, uy, math.hypot(x1 - ux, y1 - uy)

def arc_run(ang, gap_deg=12):
    """largest contiguous angular run (deg) of the given angles; returns (start, span)"""
    a = np.sort(np.mod(ang, 360)); g = np.diff(np.r_[a, a[0] + 360])
    # split at gaps larger than gap_deg, take the run with most coverage
    cuts = np.where(g > gap_deg)[0]
    if len(cuts) == 0: return 0.0, 360.0
    best = (0, 0, 0)
    for k in range(len(cuts)):
        s_idx = (cuts[k] + 1) % len(a); e_idx = cuts[(k + 1) % len(cuts)]
        s, e = a[s_idx], a[e_idx]; span = (e - s) % 360
        if span > best[2]: best = (s, e, span)
    return float(best[0]), float(best[2])

def segment(im, z_range=(65, 115), r_mm=(6.0, 12.0), max_span=170.0, band=None, seed=0, needle_arc_deg=None, depth_mm=None):
    im = im.astype(int); c = candidates(im); t, wide = thin_only(c)
    ys, xs = np.nonzero(t); diag = {'n_thin_px': int(len(xs))}
    if len(xs) < 80: return None, dict(diag, ok=False, reason='too few thin candidates')
    rmin, rmax = r_mm[0] * FX / z_range[1], r_mm[1] * FX / z_range[0]
    dt = cv2.distanceTransform(t, cv2.DIST_L2, 3); halfw = float(np.clip(np.percentile(dt[t > 0], 90), 2, 15)); tol = band or halfw + 2.0
    P = np.c_[xs, ys].astype(float); rng = np.random.default_rng(seed); idx = rng.choice(len(P), size=min(len(P), 4000), replace=False); S = P[idx]
    best = None
    for _ in range(3000):
        c3 = circle3(S[rng.choice(len(S), 3, replace=False)])
        if c3 is None or not (rmin <= c3[2] <= rmax): continue
        r = np.abs(np.hypot(S[:, 0] - c3[0], S[:, 1] - c3[1]) - c3[2]); inl = r < tol
        if inl.sum() < 30: continue
        ang = np.degrees(np.arctan2(S[inl, 1] - c3[1], S[inl, 0] - c3[0])); _, span = arc_run(ang)
        score = inl.sum() * min(span, max_span) / max_span
        if best is None or score > best[0]: best = (score, c3)
    if best is None: return None, dict(diag, ok=False, reason='no circle found')
    cx, cy, R = best[1]
    for _ in range(3):  # refine on all thin pixels in the band (algebraic fit)
        d = np.abs(np.hypot(P[:, 0] - cx, P[:, 1] - cy) - R); Q = P[d < tol]
        A = np.c_[2 * Q[:, 0], 2 * Q[:, 1], np.ones(len(Q))]; s = np.linalg.lstsq(A, (Q ** 2).sum(1), rcond=None)[0]
        cx, cy = s[0], s[1]; R = math.sqrt(s[2] + cx ** 2 + cy ** 2)
    d = np.abs(np.hypot(P[:, 0] - cx, P[:, 1] - cy) - R); Q = P[d < tol]
    ang = np.degrees(np.arctan2(Q[:, 1] - cy, Q[:, 0] - cx)); a0, span = arc_run(ang)
    # thread trimming: per 5-degree bin colour along the arc, drop purple bins at the run ends
    hsv = cv2.cvtColor(im.astype(np.uint8), cv2.COLOR_BGR2HSV); Hh, Ss = hsv[:, :, 0].astype(int), hsv[:, :, 1].astype(int); bins = []
    for k in range(int(math.ceil(span / 5))):
        lo = a0 + 5 * k; sel = ((ang - lo) % 360) < 5
        pts = Q[sel].astype(int)
        if len(pts) >= 3:
            h_, s_ = Hh[pts[:, 1], pts[:, 0]], Ss[pts[:, 1], pts[:, 0]]
            bins.append(float(np.mean((h_ >= 115) & (h_ <= 165) & (s_ >= 45))))   # fraction of purple, saturated pixels
        else: bins.append(np.nan)
    bins = np.array(bins); purple = np.nan_to_num(bins, nan=0) > 0.5
    s_k, e_k = 0, len(bins)
    while s_k < e_k and purple[s_k]: s_k += 1
    while e_k > s_k and purple[e_k - 1]: e_k -= 1
    a0t, spant = a0 + 5 * s_k, 5 * (e_k - s_k); trimmed = 5 * (len(bins) - (e_k - s_k))
    if spant > max_span:  # keep the less purple end-segment of length max_span
        nb = int(max_span // 5); win = [np.nansum(bins[s_k + i: s_k + i + nb]) for i in range(e_k - s_k - nb + 1)]
        i0 = int(np.argmin(win)); a0t, spant = a0 + 5 * (s_k + i0), 5 * nb
    # which end carries the suture? thin candidates just outside the circle band beyond each end
    ends = {}
    for side, ae in (('start', a0t), ('end', a0t + spant)):
        ex, ey = cx + R * math.cos(math.radians(ae)), cy + R * math.sin(math.radians(ae))
        near = np.hypot(P[:, 0] - ex, P[:, 1] - ey) < 60; outside = np.abs(np.hypot(P[:, 0] - cx, P[:, 1] - cy) - R) > tol + 2
        ends[side] = int((near & outside).sum())
    thread_side = max(ends, key=ends.get) if max(ends.values()) > 30 else None
    capped = 0.0
    if needle_arc_deg and thread_side is not None and spant > needle_arc_deg + 10:
        capped = spant - (needle_arc_deg + 10)
        if thread_side == 'start': a0t += capped
        spant -= capped   # thread at 'end' (or unknown): trim the end
    H, W = t.shape; yy, xx = np.mgrid[0:H, 0:W]
    rr = np.hypot(xx - cx, yy - cy); aa = (np.degrees(np.arctan2(yy - cy, xx - cx)) - a0t) % 360
    mask = (np.abs(rr - R) < tol + 1) & (aa <= spant) & (c > 0) & (wide == 0)
    mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(mask); keep = np.zeros_like(mask)
    for i in range(1, n):
        if st[i, 4] >= 30: keep[lab == i] = 1
    inl_frac = float((d < tol).sum() / max(len(P), 1)); band_rms = float(np.sqrt(np.mean(d[d < tol] ** 2)))
    angk = (np.degrees(np.arctan2(Q[:, 1] - cy, Q[:, 0] - cx)) - a0t) % 360; angk = angk[angk <= spant]
    coverage = float(len(np.unique((angk // 2).astype(int))) / max(spant / 2, 1))
    diag.update(ok=True, centre_px=[cx, cy], R_px=R, halfwidth_px=halfw, arc_start_deg=a0t, arc_span_deg=float(spant), thread_trimmed_deg=float(trimmed),
                inlier_frac_of_thin=inl_frac, band_rms_px=band_rms, mask_px=int(keep.sum()), arc_coverage=coverage, thread_side=thread_side, end_outside_px=ends, capped_deg=float(capped),
                confident=bool(spant >= 60 and coverage >= 0.7 and band_rms < 0.6 * tol and keep.sum() > 300))
    if depth_mm is not None and keep.sum() > 50:
        import r2_arc as A
        try:
            fa = A.fit(A.backproject(depth_mm, cv2.erode(keep, np.ones((3, 3), np.uint8)) > 0)[0])
            diag.update(arc3d_R_mm=fa['R_mm'], arc3d_rms_mm=fa['rms_mm'], arc3d_span_deg=fa['arc_deg'])
            diag['confident'] = bool(diag['confident'] and 7.5 <= fa['R_mm'] <= 10.5 and fa['rms_mm'] < 0.6)
        except Exception as e: diag.update(arc3d_error=str(e), confident=False)
    return keep.astype(bool), diag

def overlay(im, mask, diag):
    ov = im.copy(); cnt, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE); cv2.drawContours(ov, cnt, -1, (0, 255, 0), 1)
    if diag.get('ok'):
        cx, cy = diag['centre_px']; cv2.circle(ov, (int(cx), int(cy)), int(diag['R_px']), (255, 255, 0), 1)
    return ov

if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('image'); ap.add_argument('--out'); ap.add_argument('--overlay'); a = ap.parse_args()
    im = cv2.imread(a.image); m, dg = segment(im); print(json.dumps(dg, indent=1))
    if m is not None and a.out: cv2.imwrite(a.out, m.astype(np.uint8) * 255)
    if m is not None and a.overlay: cv2.imwrite(a.overlay, overlay(im, m, dg))
