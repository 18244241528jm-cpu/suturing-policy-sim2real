#!/usr/bin/env python3
"""R5 grasp v3 geometry library (no ROS; import from executors / offline replay).
Frames: 'cam' = rectified physical-left camera (topic /jhu_daVinci/right/image_rect), mm. Topic-left camera centre is at (+B, 0, 0).
- grasp point / tangent from a stereo arc fit (r2_arc.fit dict) at arc fraction s (default 0.5)
- grasp point / tangent from an FP pose + needle mesh (mesh arc fitted once in the mesh frame)
- jaw axis from kinematics (tool z of measured_cp, rotated to cam by the calibration rotation) and from the images
  (Hough lines on the jaw edges in both rectified images -> two back-projected planes -> 3D direction)
- psi = angle between the needle tangent and the jaw axis projected into the needle plane, folded to [0, 90] deg
- roll planning: sample instrument roll (joint 3) offsets; jaw z rotated about the shaft axis; pick psi-minimising roll
  among the two 180-deg-equivalent solutions, preferring the one farther from the joint limits."""
import numpy as np, cv2

FX, CX, CY = 1860.724932, 690.538345, 576.803638
B_MM = 5.4476353950418295; FB = FX * B_MM
K = np.array([[FX, 0, CX], [0, FX, CY], [0, 0, 1.0]]); Kinv = np.linalg.inv(K)
proj = lambda X: np.array([FX * X[0] / X[2] + CX, FX * X[1] / X[2] + CY])
unit = lambda v: np.asarray(v, float) / np.linalg.norm(v)

# ---------------- needle arc ----------------
def arc_frame(r):
    c = np.array(r['centre_mm']); n = unit(r['normal_toward_cam'])
    if '_u' in r:
        u, v = unit(r['_u']), unit(r['_v']); a0 = r['_a_start_deg']; span = (r['_a_end_deg'] - a0) % 360
    else:   # rebuild from the two end points (stored fits drop the private keys)
        ea, eb = np.array(r['end_a_mm']), np.array(r['end_b_mm'])
        u = unit((ea - c) - ((ea - c) @ n) * n); v = np.cross(n, u)
        bb = eb - c; a_b = np.degrees(np.arctan2(bb @ v, bb @ u)) % 360
        a0, span = 0.0, a_b if abs(a_b - r['arc_deg']) < abs((360 - a_b) - r['arc_deg']) else -(360 - a_b)
    return c, n, u, v, float(r['R_mm']), float(a0), float(span)

def arc_point_tangent(r, s=0.5):
    c, n, u, v, R, a0, span = arc_frame(r); ang = np.radians(a0 + s * span)
    p = c + R * (np.cos(ang) * u + np.sin(ang) * v); t = unit(-np.sin(ang) * u + np.cos(ang) * v)
    return p, t, n

def nearest_on_arc(r, p):
    """arc fraction s of the point on the fitted circle nearest to p (for replays of R4 grasp points)"""
    c, n, u, v, R, a0, span = arc_frame(r); d = p - c; ang = np.degrees(np.arctan2(d @ v, d @ u))
    return float(((ang - a0) % 360) / span) if span > 0 else float(((a0 - ang) % 360) / -span)

# ---------------- needle mesh (for FP) ----------------
def load_obj_vertices(path, scale_to_mm=1000.0):
    return np.array([list(map(float, l.split()[1:4])) for l in open(path) if l.startswith('v ')]) * scale_to_mm

def mesh_arc(V):
    """centreline arc of a needle mesh in its own frame: plane SVD + algebraic circle; returns dict like r2_arc.fit"""
    m = V.mean(0); _, _, Vt = np.linalg.svd(V - m); n = Vt[2]; u0, v0 = Vt[0], Vt[1]
    q = np.c_[(V - m) @ u0, (V - m) @ v0]; Am = np.c_[2 * q, np.ones(len(q))]; sol = np.linalg.lstsq(Am, (q ** 2).sum(1), rcond=None)[0]
    c2 = sol[:2]; R = float(np.sqrt(sol[2] + c2 @ c2)); c = m + c2[0] * u0 + c2[1] * v0
    ang = np.degrees(np.arctan2(q[:, 1] - c2[1], q[:, 0] - c2[0])) % 360; a = np.sort(ang); gaps = np.diff(np.r_[a, a[0] + 360]); k = int(np.argmax(gaps))
    a_start = a[(k + 1) % len(a)]; span = 360 - gaps[k]
    rr = np.linalg.norm(np.c_[q[:, 0] - c2[0], q[:, 1] - c2[1]], axis=1); wire_d = float(2 * np.median(np.abs(rr - R)) * 2)
    return {'centre_mm': c.tolist(), 'normal_toward_cam': n.tolist(), 'R_mm': R, 'arc_deg': float(span), '_u': u0.tolist(), '_v': v0.tolist(),
            '_a_start_deg': float(a_start), '_a_end_deg': float(a_start + span), 'wire_diam_est_mm': wire_d}

def fp_point_tangent(T_ob_in_cam_m, marc, s=0.5):
    """FP pose (4x4, metres) + mesh arc (mm, mesh frame) -> grasp point and tangent in cam mm"""
    T = np.asarray(T_ob_in_cam_m, float); R, t = T[:3, :3], T[:3, 3] * 1000.0
    p, tg, n = arc_point_tangent(marc, s); return R @ p + t, unit(R @ tg), unit(R @ n)

def consistency(p_arc, t_arc, p_fp, t_fp, max_mm=2.0, max_deg=15.0):
    d = float(np.linalg.norm(p_arc - p_fp)); a = float(np.degrees(np.arccos(min(1.0, abs(t_arc @ t_fp)))))
    return {'dp_mm': d, 'dtangent_deg': a, 'ok': d <= max_mm and a <= max_deg}

# ---------------- psi ----------------
def psi_inplane(jaw_cam, t, n):
    j = np.asarray(jaw_cam, float); jp = j - (j @ n) * n
    return float(np.degrees(np.arccos(min(1.0, abs(unit(jp) @ t))))), float(np.degrees(np.arccos(min(1.0, abs(unit(j) @ n)))))

def psi_2d(jaw_img_dir, p, t):
    ti = unit(proj(p + t) - proj(p - t)); return float(np.degrees(np.arccos(min(1.0, abs(ti @ unit(jaw_img_dir))))))

def jaw_axis_kin(q_xyzw, R_cam_from_ecm):
    from scipy.spatial.transform import Rotation as Rot
    return unit(R_cam_from_ecm @ Rot.from_quat(q_xyzw).as_matrix()[:, 2])

def rotation_from_affine(M4x3):
    """calibration_v3 'affine' M (X_cam = [p_cp,1] @ M) -> nearest rotation (polar) of its linear part"""
    A = np.asarray(M4x3)[:3].T; U, _, Vt = np.linalg.svd(A); D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(U @ Vt)); return U @ D @ Vt

def dominant_line(gray, uv, half=170, canny=(40, 110), min_len=60, band_deg=12.0):
    """Hough segments in a window around uv; returns (angle_deg in [0,180), weighted centre uv, n_segments, total_len) of the dominant
    orientation cluster (peak +- band_deg); None if no segments."""
    u, v = int(round(uv[0])), int(round(uv[1])); H, W = gray.shape
    x0, y0 = max(u - half, 0), max(v - half, 0); x1, y1 = min(u + half, W), min(v + half, H)
    e = cv2.Canny(cv2.GaussianBlur(gray[y0:y1, x0:x1], (5, 5), 0), *canny)
    L = cv2.HoughLinesP(e, 1, np.pi / 360, 40, minLineLength=min_len, maxLineGap=6)
    if L is None: return None
    L = L[:, 0].astype(float); ln = np.hypot(L[:, 2] - L[:, 0], L[:, 3] - L[:, 1]); th = np.degrees(np.arctan2(L[:, 3] - L[:, 1], L[:, 2] - L[:, 0])) % 180
    best = None
    for c in np.arange(0, 180, 1.0):
        d = np.abs((th - c + 90) % 180 - 90); w = ln[d < band_deg].sum()
        if best is None or w > best[0]: best = (w, c)
    d = np.abs((th - best[1] + 90) % 180 - 90); sel = d < band_deg
    z = (ln[sel] * np.exp(2j * np.radians(th[sel]))).sum(); ang = float(np.degrees(np.angle(z)) / 2 % 180)
    mid = np.c_[(L[sel, 0] + L[sel, 2]) / 2 + x0, (L[sel, 1] + L[sel, 3]) / 2 + y0]; cen = (mid * ln[sel, None]).sum(0) / ln[sel].sum()
    return ang, cen, int(sel.sum()), float(ln[sel].sum())

def jaw_axis_stereo(gray_ref, gray_other, uv_ref, disp_guess_px, **kw):
    """3D jaw-axis direction (cam frame) from the dominant jaw-edge line in both rectified images.
    gray_ref = physical-left (topic right) rect image; gray_other = topic-left rect image (point shifted by -disparity in u)."""
    a = dominant_line(gray_ref, uv_ref, **kw); b = dominant_line(gray_other, (uv_ref[0] - disp_guess_px, uv_ref[1]), **kw)
    if a is None or b is None: return None
    def plane_normal(ang, cen):
        d = np.array([np.cos(np.radians(ang)), np.sin(np.radians(ang))]); r1 = Kinv @ np.r_[cen, 1]; r2 = Kinv @ np.r_[cen + 50 * d, 1]; return unit(np.cross(r1, r2))
    nL, nR = plane_normal(a[0], a[1]), plane_normal(b[0], b[1]); d3 = np.cross(nL, nR)
    cond = float(np.linalg.norm(d3))     # sin of the angle between the two planes; small -> ill-conditioned (line near-parallel to epipolar)
    return {'dir_cam': unit(d3).tolist(), 'plane_sin': cond, 'ref_line': [a[0], a[1].tolist(), a[2], a[3]], 'other_line': [b[0], b[1].tolist(), b[2], b[3]]}

# ---------------- roll planning ----------------
def rodrigues(axis, ang_rad):
    k = unit(axis); Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]); return np.eye(3) + np.sin(ang_rad) * Kx + (1 - np.cos(ang_rad)) * Kx @ Kx

def plan_roll(jaw_cam, shaft_cam, t, n, q3_now, q3_lim, step_deg=0.5):
    """jaw_cam: current jaw z axis (cam); shaft_cam: instrument shaft axis (cam) about which joint 3 rotates; q3_now, q3_lim (rad).
    Returns best roll offset (rad) to reach psi->0 with the two equivalent solutions and their limit margins; reachable flag."""
    offs = np.radians(np.arange(-180, 180, step_deg)); res = []
    for o in offs:
        j = rodrigues(shaft_cam, o) @ jaw_cam; psi, tilt = psi_inplane(j, t, n); res.append((psi, o, tilt))
    res = np.array(res); sols = []
    # local minima of psi
    for i in range(len(res)):
        if res[i, 0] <= res[i - 1, 0] and res[i, 0] <= res[(i + 1) % len(res), 0]:
            q = q3_now + res[i, 1]; margin = min(q - q3_lim[0], q3_lim[1] - q)
            sols.append({'offset_rad': float(res[i, 1]), 'psi_deg': float(res[i, 0]), 'jaw_tilt_from_normal_deg': float(res[i, 2]), 'q3_rad': float(q), 'limit_margin_rad': float(margin), 'within_limits': bool(margin > 0)})
    sols = sorted(sols, key=lambda s: s['psi_deg'])[:4]
    ok = [s for s in sols if s['within_limits'] and s['psi_deg'] < 5.0]
    best = max(ok, key=lambda s: s['limit_margin_rad']) if ok else None
    return {'best': best, 'candidates': sols, 'reachable': best is not None}
