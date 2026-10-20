#!/usr/bin/env python3
"""Mesh-free 3D circular-arc fit of a needle from stereo depth (R2-C / B3 / R2-D), pre-registered method.

points  : needle-mask pixels with valid depth, back-projected with the topic-right rect P
offset  : optional +off_mm along each viewing ray (surface -> wire centreline)
fit     : plane SVD -> in-plane Kasa circle -> joint geometric LSQ (centre 3, normal 2, R 1);
          one robust pass (drop residual > median + 3*MAD, refit)
arc     : angular extent of points on the circle = 360 - largest angular gap
"""
import json
import numpy as np, cv2
from scipy.optimize import least_squares

FX = 1860.724932; CX = 690.538345; CY = 576.803638  # topic-right rect P (identical R1/R2)


def backproject(Z, mask, off_mm=0.0):
    ys, xs = np.where(mask & np.isfinite(Z) & (Z > 0))
    z = Z[ys, xs].astype(np.float64)
    P = np.c_[(xs - CX) * z / FX, (ys - CY) * z / FX, z]
    if off_mm:
        P = P + off_mm * P / np.linalg.norm(P, axis=1, keepdims=True)
    return P, np.c_[xs, ys]


def _basis(n):
    a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(n, a); u /= np.linalg.norm(u); v = np.cross(n, u)
    return u, v


def _n_from(th, ph):
    return np.array([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)])


def circle_resid(p, P):
    c, n, R = p[:3], _n_from(p[3], p[4]), p[5]
    v = P - c; h = v @ n; rad = np.linalg.norm(v - h[:, None] * n, axis=1)
    return np.sqrt(h ** 2 + (rad - R) ** 2)


def fit_once(P):
    c0 = P.mean(0); _, _, vt = np.linalg.svd(P - c0, full_matrices=False); n = vt[2]
    if n[2] > 0: n = -n  # normal toward camera (for initialisation only)
    u, v = _basis(n); x = (P - c0) @ u; y = (P - c0) @ v
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]; s = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)[0]
    cc = c0 + s[0] * u + s[1] * v; R0 = np.sqrt(s[2] + s[0] ** 2 + s[1] ** 2)
    th, ph = np.arccos(np.clip(n[2], -1, 1)), np.arctan2(n[1], n[0])
    r = least_squares(circle_resid, np.r_[cc, th, ph, R0], args=(P,), method='lm')
    return r.x


def fit(P, robust=True):
    p = fit_once(P); keep = np.ones(len(P), bool)
    if robust:
        r = circle_resid(p, P); med = np.median(r); mad = np.median(np.abs(r - med))
        keep = r <= med + 3 * mad
        p = fit_once(P[keep])
    return describe(p, P[keep], keep)


def describe(p, P, keep):
    c, n, R = p[:3], _n_from(p[3], p[4]), float(p[5])
    if n[2] > 0: n = -n  # report normal pointing toward the camera (sign fixed later by tip->tail)
    u, v = _basis(n); ang = np.sort(np.degrees(np.arctan2((P - c) @ v, (P - c) @ u)) % 360)
    gaps = np.diff(np.r_[ang, ang[0] + 360]); g = int(np.argmax(gaps))
    start, end = ang[(g + 1) % len(ang)], ang[g]  # arc runs from start (ccw about n) to end
    arc = 360 - gaps[g]
    inner_gaps = np.delete(gaps, g)
    endp = lambda a: c + R * (np.cos(np.radians(a)) * u + np.sin(np.radians(a)) * v)
    res = circle_resid(p, P)
    return {'centre_mm': c.tolist(), 'normal_toward_cam': n.tolist(), 'R_mm': R, 'arc_deg': float(arc),
            'max_internal_gap_deg': float(inner_gaps.max()) if len(inner_gaps) else 0.0,
            'end_a_mm': endp(start).tolist(), 'end_b_mm': endp(end).tolist(),  # a -> b is ccw about normal_toward_cam
            'rms_mm': float(np.sqrt((res ** 2).mean())), 'n_points': int(len(P)), 'n_dropped_robust': int((~keep).sum()),
            '_u': u.tolist(), '_v': v.tolist(), '_a_start_deg': float(start), '_a_end_deg': float(end)}


def project(X):
    X = np.atleast_2d(X); return np.c_[FX * X[:, 0] / X[:, 2] + CX, FX * X[:, 1] / X[:, 2] + CY]


def pose_from(d, tip_px):
    """6D pose: origin centre, x -> tip end, z = normal with tip->tail positive, y = z x x. tip_px: annotated tip (u,v)."""
    c = np.array(d['centre_mm']); n = np.array(d['normal_toward_cam'])
    a, b = np.array(d['end_a_mm']), np.array(d['end_b_mm'])
    pa, pb = project(a)[0], project(b)[0]
    tip_is_a = np.linalg.norm(pa - tip_px) <= np.linalg.norm(pb - tip_px)
    tip, tail = (a, b) if tip_is_a else (b, a)
    z = n if tip_is_a else -n  # a->b is ccw about n
    x = (tip - c) / np.linalg.norm(tip - c); y = np.cross(z, x)
    T = np.eye(4); T[:3, :3] = np.c_[x, y, z]; T[:3, 3] = c
    # apex = arc midpoint
    u, v = np.array(d['_u']), np.array(d['_v']); am = d['_a_start_deg'] + d['arc_deg'] / 2
    apex = c + d['R_mm'] * (np.cos(np.radians(am)) * u + np.sin(np.radians(am)) * v)
    return T, tip, tail, apex, bool(tip_is_a)


def rot_angle_deg(R1, R2):
    return float(np.degrees(np.arccos(np.clip((np.trace(R1.T @ R2) - 1) / 2, -1, 1))))


def bootstrap(P, tip_px, B=500, seed=0):
    rng = np.random.default_rng(seed); out = []
    for _ in range(B):
        k = rng.choice(len(P), len(P)); d = fit(P[k]); T, tip, tail, apex, _ = pose_from(d, tip_px)
        out.append((T, tip, d['R_mm'], d['arc_deg']))
    return out


def summarise_poses(Ts, tips, Tref=None):
    Ts = np.array(Ts); tips = np.array(tips)
    Tref = Ts[0] if Tref is None else Tref
    cen = Ts[:, :3, 3]; ang = np.array([rot_angle_deg(Tref[:3, :3], T[:3, :3]) for T in Ts])
    nang = np.array([np.degrees(np.arccos(np.clip(abs(T[:3, 2] @ Tref[:3, 2]), -1, 1))) for T in Ts])
    cd = np.linalg.norm(cen - cen.mean(0), axis=1); td = np.linalg.norm(tips - tips.mean(0), axis=1)
    return {'n': len(Ts), 'centre_std_xyz_mm': cen.std(0).tolist(), 'centre_dev_from_mean_p95_mm': float(np.percentile(cd, 95)),
            'centre_maxmin_mm': float(np.linalg.norm(cen.max(0) - cen.min(0))),
            'rot_vs_ref_deg_p95': float(np.percentile(ang, 95)), 'rot_vs_ref_deg_max': float(ang.max()),
            'normal_vs_ref_deg_p95': float(np.percentile(nang, 95)),
            'tip_dev_from_mean_p95_mm': float(np.percentile(td, 95)), 'tip_maxmin_mm': float(np.linalg.norm(tips.max(0) - tips.min(0)))}
