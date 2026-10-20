#!/usr/bin/env python3
"""EXP0: orientation check, disparity-scale check, perception replay, aggregate. Pre-registered in pre_registration.json.
Run (WSL): ~/.venvs/foundationstereo-5080/bin/python ~/r6off/exp0/exp0.py"""
import json, os, sys, glob, csv
import numpy as np, cv2
ROOT = os.path.expanduser('~/r6off'); IN = ROOT + '/input'; OUT = ROOT + '/exp0'
sys.path.insert(0, IN + '/code_snapshot')
import r2_arc, r5_grasp_v3 as G

TR = sorted(os.path.basename(p) for p in glob.glob(IN + '/trials/*') if os.path.isdir(p))
PLACE = [t for t in TR if t != 'T3b_C']          # T3b reuses T3's perception -> not an independent placement
summ = {r['trial']: r for r in csv.DictReader(open(IN + '/trials_summary.csv'))}


def load(t, snap):
    d = f'{IN}/trials/{t}/{snap}'
    R = cv2.imread(d + '/right_rect.png'); L = cv2.imread(d + '/left_rect.png')   # R = topic right = physical left
    disp = np.load(d + '/fs_disp.npz')['disp'].astype(np.float32)
    return R, L, disp


# ---------------- A. orientation ----------------
def ncc_patch(a, b):
    a = a - a.mean(); b = b - b.mean(); den = np.sqrt((a * a).sum() * (b * b).sum())
    return float((a * b).sum() / den) if den > 1e-6 else np.nan


def orient_one(ref, oth, disp, seed=0, n=400, h=10):
    rg = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY).astype(np.float32); og = cv2.cvtColor(oth, cv2.COLOR_BGR2GRAY).astype(np.float32)
    gx = cv2.Sobel(rg, cv2.CV_32F, 1, 0); gy = cv2.Sobel(rg, cv2.CV_32F, 0, 1); g = np.hypot(gx, gy)
    ys, xs = np.mgrid[100:900, 150:1150]; ok = (disp[ys, xs] >= 40) & (disp[ys, xs] <= 130) & (g[ys, xs] > np.median(g))
    cand = np.c_[xs[ok], ys[ok]]; rng = np.random.default_rng(seed); sel = cand[rng.choice(len(cand), n, replace=False)]
    s = (2 * h + 1, 2 * h + 1); minus, plus = [], []
    for x, y in sel:
        d = float(disp[y, x]); P = cv2.getRectSubPix(rg, s, (float(x), float(y)))
        minus.append(ncc_patch(P, cv2.getRectSubPix(og, s, (float(x) - d, float(y)))))
        plus.append(ncc_patch(P, cv2.getRectSubPix(og, s, (float(x) + d, float(y)))))
    minus, plus = np.array(minus), np.array(plus)
    return float(np.nanmedian(minus)), float(np.nanmedian(plus)), float(np.nanmedian(minus - plus))


A = {'frames': {}}
for t in PLACE:
    R, L, disp = load(t, 'snap0')
    h1 = orient_one(R, L, disp)      # H1: ref = topic right, other = topic left
    h2 = orient_one(L, R, disp)      # H2: ref = topic left, other = topic right
    A['frames'][t] = {'H1': {'ncc_minus': h1[0], 'ncc_plus': h1[1], 'median_diff': h1[2]}, 'H2': {'ncc_minus': h2[0], 'ncc_plus': h2[1], 'median_diff': h2[2]}}
    print('A', t, 'H1 x-d %.3f x+d %.3f | H2 x-d %.3f x+d %.3f' % (h1[0], h1[1], h2[0], h2[1]), flush=True)


def passes(hk):
    n1 = sum(1 for v in A['frames'].values() if v[hk]['ncc_minus'] >= 0.8)
    n2 = sum(1 for v in A['frames'].values() if v[hk]['median_diff'] >= 0.3)
    return n1 >= 12 and n2 >= 12, n1, n2
p1, p2 = passes('H1'), passes('H2')
A['H1_pass'], A['H1_counts'] = bool(p1[0]), [p1[1], p1[2]]; A['H2_pass'], A['H2_counts'] = bool(p2[0]), [p2[1], p2[2]]
A['strict_verdict_prereg'] = 'H1' if p1[0] and not p2[0] else ('H2' if p2[0] and not p1[0] else 'INCONCLUSIVE')
A['diff_only'] = {'H1_frames_diff_ge_0.3': p1[2], 'H2_frames_diff_ge_0.3': p2[2], 'H1_min_diff': min(v['H1']['median_diff'] for v in A['frames'].values()), 'H2_max_diff': max(v['H2']['median_diff'] for v in A['frames'].values())}
A['verdict'] = 'H1 (disparity referenced to topic-RIGHT image = physical left)' if p1[0] and not p2[0] else ('H2' if p2[0] and not p1[0] else 'INCONCLUSIVE')

# baseline from the camera yaml (no yaml lib: parse the P list)
def p_list(fn):
    L = open(fn).read().split('\np:\n')[1].split('binning_x')[0]
    return [float(x.strip()[2:]) for x in L.strip().split('\n') if x.strip().startswith('- ')]
Pr = p_list(IN + '/camera/camera_info_topic_right.yaml'); Pl = p_list(IN + '/camera/camera_info_topic_left.yaml')
A['camera'] = {'P_right_03': Pr[3], 'P_left_03': Pl[3], 'fx': Pr[0], 'B_mm_from_yaml': Pr[3] / Pr[0] * 1000.0, 'B_mm_in_code': G.B_MM,
               'note': 'Tx>0 on the topic-right camera => topic-right camera sits to the LEFT of topic-left camera (physical left = topic right)'}
print('A verdict:', A['verdict'], A['H1_counts'], A['H2_counts'], 'B_yaml %.4f code %.4f' % (A['camera']['B_mm_from_yaml'], G.B_MM), flush=True)


# ---------------- B/C. perception replay (logic of r6_perceive.py, unchanged; only I/O differs) ----------------
BAND, DARK, OFF = 5, 18, 0.5
Aq = lambda a_, b_: np.c_[np.ones_like(a_), a_, b_, a_ * a_, a_ * b_, b_ * b_]


def perceive_frame(img, d, poly, s=0.5, old=False):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(float)
    def band_mask(shape):
        m = np.zeros(shape, np.uint8); cv2.polylines(m, [poly.round().astype(np.int32)], False, 1, thickness=2 * BAND + 1); return m.astype(bool)
    def annulus(shape):
        m = np.zeros(shape, np.uint8); cv2.polylines(m, [poly.round().astype(np.int32)], False, 1, thickness=2 * (BAND + 10) + 1); return m.astype(bool) & ~band_mask(shape)
    bm, an = band_mask(g.shape), annulus(g.shape)
    gm = cv2.blur(np.where(an, g, 0), (41, 41)) / np.maximum(cv2.blur(an.astype(float), (41, 41)), 1e-6)
    mask = bm & (g < gm - DARK)
    Z = G.FB / np.clip(d, 1, None)
    img_pink = (img[..., 2].astype(int) - img[..., 1] > 30)
    sup = annulus(g.shape) & img_pink & np.isfinite(Z)
    ys, xs = np.nonzero(sup); a_ = (xs - G.CX) / G.FX; b_ = (ys - G.CY) / G.FX; zz = Z[ys, xs]
    keep = np.ones(len(zz), bool)
    for _ in range(3):
        coef = np.linalg.lstsq(Aq(a_[keep], b_[keep]), zz[keep], rcond=None)[0]; r_ = zz - Aq(a_, b_) @ coef; sdv = r_[keep].std(); keep = np.abs(r_) < 2.5 * sdv + 1e-6
    ym, xm = np.nonzero(mask); am, bm_ = (xm - G.CX) / G.FX, (ym - G.CY) / G.FX; zm = Aq(am, bm_) @ coef
    Psurf = np.c_[am * zm, bm_ * zm, zm]
    cs = Psurf.mean(0); ns = np.linalg.svd(Psurf - cs)[2][2]; ns = ns if ns[2] < 0 else -ns
    P = Psurf + OFF * ns
    if old:   # reconstruction of the pre-amendment-2 method: FS depth of the needle pixels used directly
        P, _ = r2_arc.backproject(Z, mask, OFF)
    fit = r2_arc.fit(P)
    def snormal(a0, b0, e=1e-3):
        S = lambda a1, b1: np.array([a1, b1, 1.0]) * float(Aq(np.array([a1]), np.array([b1])) @ coef)
        v = np.cross(S(a0 + e, b0) - S(a0 - e, b0), S(a0, b0 + e) - S(a0, b0 - e)); v /= np.linalg.norm(v); return v if v[2] < 0 else -v
    ends = [poly[0], poly[len(poly) // 2], poly[-1]]; nrm = [snormal((q[0] - G.CX) / G.FX, (q[1] - G.CY) / G.FX) for q in ends]
    nvar = max(np.degrees(np.arccos(min(1, abs(nrm[i] @ nrm[j])))) for i in range(3) for j in range(i + 1, 3))
    surf = {'normal_variation_deg_tail_mid_tip': float(nvar), 'n_support': int(keep.sum()), 'resid_std_mm': float(sdv), 'surface_normal_toward_cam': ns.tolist(),
            'fs_wire_minus_surface_mm_median': float(np.median(Z[ym, xm] - zm))}
    ea, eb = np.array(fit['end_a_mm']), np.array(fit['end_b_mm']); pa, pb = r2_arc.project(ea)[0], r2_arc.project(eb)[0]
    tail_is_a = np.linalg.norm(pa - poly[0]) <= np.linalg.norm(pb - poly[0])
    s_eff = s if tail_is_a else 1.0 - s
    p, t, nn = G.arc_point_tangent(fit, s_eff)
    return {'fit': fit, 'p_star_mm': p, 'tangent': t, 'normal_toward_cam': nn, 'surface': surf, 'n_mask_px': int(mask.sum()), 'Zpad': (coef, sup)}


ang = lambda a, b: float(np.degrees(np.arccos(min(1.0, abs(float(np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b)))))))
C = {}
for t in PLACE:
    st = json.load(open(f'{IN}/trials/{t}/perception.json')); poly = np.array(json.load(open(f'{IN}/trials/{t}/clicks.json'))['polyline_px'], float)
    fr = []; s_day = st['method']['s']; old = 'surface' not in st['frames'][0]
    for k, snap in enumerate(['snap0', 'snap1']):
        R, L, disp = load(t, snap); fr.append(perceive_frame(R, disp, poly, s=s_day, old=old))
    pm = np.median([f['p_star_mm'] for f in fr], 0)
    pd = np.linalg.norm(pm - np.array(st['p_star_mm']))
    per = []
    for k, f in enumerate(fr):
        sf = st['frames'][k]; so = sf.get('surface') or {'n_support': 1, 'resid_std_mm': np.nan, 'surface_normal_toward_cam': [0, 0, -1], 'fs_wire_minus_surface_mm_median': np.nan}
        per.append({'dp_mm': float(np.linalg.norm(f['p_star_mm'] - np.array(sf['p_star_mm']))), 'dR_mm': float(f['fit']['R_mm'] - sf['fit']['R_mm']),
                    'darc_deg': float(f['fit']['arc_deg'] - sf['fit']['arc_deg']), 'dtan_deg': ang(f['tangent'], sf['tangent']), 'dnormal_deg': ang(f['normal_toward_cam'], sf['normal_toward_cam']),
                    'n_support': [f['surface']['n_support'], so['n_support']], 'resid_std': [f['surface']['resid_std_mm'], so['resid_std_mm']],
                    'surf_normal_deg': ang(f['surface']['surface_normal_toward_cam'], so['surface_normal_toward_cam']),
                    'wire_minus_surf': [f['surface']['fs_wire_minus_surface_mm_median'], so['fs_wire_minus_surface_mm_median']], 'n_mask_px': [f['n_mask_px'], sf['n_mask_px']],
                    'n_points': [f['fit']['n_points'], sf['fit']['n_points']]})
    C[t] = {'old_method_reconstruction': old, 's': s_day, 'dp_star_mm': float(pd), 'pass_0p1': bool(pd <= 0.1), 'per_frame': per, 'replay_p_star': pm.tolist(), 'day_p_star': st['p_star_mm'],
            'replay_R': [f['fit']['R_mm'] for f in fr], 'replay_arc': [f['fit']['arc_deg'] for f in fr], 'day_R': [f['fit']['R_mm'] for f in st['frames']], 'day_arc': [f['fit']['arc_deg'] for f in st['frames']]}
    print('C', t, 'dp* %.3f mm' % pd, 'dp per frame', [round(x['dp_mm'], 3) for x in per], 'dR', [round(x['dR_mm'], 3) for x in per], 'darc', [round(x['darc_deg'], 2) for x in per], flush=True)
rep = {t: v for t, v in C.items() if not v['old_method_reconstruction']}; old_ = {t: v for t, v in C.items() if v['old_method_reconstruction']}
print('C replay pass (<=0.1 mm), day code available: %d/%d ; old-method reconstructions: %d/%d' % (sum(v['pass_0p1'] for v in rep.values()), len(rep), sum(v['pass_0p1'] for v in old_.values()), len(old_)))

# B scale check summary from the replay (tolerances from the pre-registration)
Bsum = {}
for t, v in rep.items():
    ok = True; rows = []
    for x in v['per_frame']:
        ns, nd = x['n_support']; r1, r2 = x['resid_std']; w1, w2 = x['wire_minus_surf']
        r = {'n_support_rel': abs(ns - nd) / nd, 'resid_std_abs': abs(r1 - r2), 'normal_deg': x['surf_normal_deg'], 'wire_minus_surf_abs': abs(w1 - w2)}
        r['pass'] = r['n_support_rel'] <= 0.03 and r['resid_std_abs'] <= 0.05 and r['normal_deg'] <= 1.0 and r['wire_minus_surf_abs'] <= 0.15
        rows.append(r)
    Bsum[t] = {'frames': rows, 'pass': all(r['pass'] for r in rows)}
print('B scale/surface check pass: %d/%d placements' % (sum(b['pass'] for b in Bsum.values()), len(Bsum)))

# ---------------- B2. exploratory kinematic tip vs FS depth ----------------
from scipy.spatial.transform import Rotation as Rot
cal = json.load(open(IN + '/calibration_summary.json')); M = np.array(cal['affine_M_cp_ecm_mm_to_cam_mm (X_cam = [p_cp,1] @ M)']); TCP = np.array(cal['TCP_tool_mm'])
Alin = M[:3].T; U, S, Vt = np.linalg.svd(Alin); Rpol = U @ Vt


def tip_cam(cp7):
    p = np.array(cp7[:3]) * 1000.0; Re = Rot.from_quat(cp7[3:7]).as_matrix()
    return np.r_[p, 1] @ M + (Rpol @ Re) @ TCP, Rpol @ Re


B2 = {'formula_check_vs_preclose_pose': {}, 'snap0_tip_vs_fs': {}}
for t in PLACE:
    f = f'{IN}/trials/{t}/preclose_pose.json'
    if os.path.exists(f):
        pc = json.load(open(f)); # preclose_pose stores cp_ecm_mm and R_cam: check my cam tip formula against the file's tip.cam_mm
        cp_mm = np.array(pc['tip']['cp_ecm_mm']); q = pc['tip']['q_cp_xyzw']
        tc, _ = tip_cam(list(cp_mm / 1000.0) + list(q)); B2['formula_check_vs_preclose_pose'][t] = {'mine': tc.tolist(), 'file': pc['tip']['cam_mm'], 'diff_mm': float(np.linalg.norm(tc - np.array(pc['tip']['cam_mm'])))}
    st = json.load(open(f'{IN}/trials/{t}/snap0/state.json')); tc, _ = tip_cam(st['cp']); uv = r2_arc.project(tc)[0]
    R, L, disp = load(t, 'snap0'); u, v = int(round(uv[0])), int(round(uv[1])); inside = 5 <= u < 1295 and 5 <= v < 1019
    rec = {'tip_cam_mm': tc.tolist(), 'tip_px': uv.tolist(), 'inside_image': bool(inside)}
    if inside:
        dd = disp[v - 2:v + 3, u - 2:u + 3]; rec['fs_disp_at_tip'] = float(np.median(dd)); rec['Z_fs'] = float(G.FB / np.median(dd)); rec['Z_kin'] = float(tc[2]); rec['Z_fs_minus_kin_mm'] = rec['Z_fs'] - rec['Z_kin']
    B2['snap0_tip_vs_fs'][t] = rec
print('B2 formula check (diff mm):', {k: round(v['diff_mm'], 2) for k, v in B2['formula_check_vs_preclose_pose'].items()})
print('B2 tips inside image at snap0:', [k for k, v in B2['snap0_tip_vs_fs'].items() if v['inside_image']])

# ---------------- D. aggregate ----------------
Rs = np.array([np.mean(v['day_R']) for v in C.values()]); arcs = np.array([np.mean(v['day_arc']) for v in C.values()])
Rr = np.array([np.mean(v['replay_R']) for v in C.values()]); ar = np.array([np.mean(v['replay_arc']) for v in C.values()])
D = {'n_placements': len(C), 'R_day': {'median': float(np.median(Rs)), 'min': float(Rs.min()), 'max': float(Rs.max()), 'values': dict(zip(C, Rs.tolist()))},
     'arc_day': {'median': float(np.median(arcs)), 'min': float(arcs.min()), 'max': float(arcs.max()), 'values': dict(zip(C, arcs.tolist()))},
     'R_replay_median': float(np.median(Rr)), 'arc_replay_median': float(np.median(ar)),
     'p_star_spread_snap0_snap1_mm': {t: json.load(open(f'{IN}/trials/{t}/perception.json'))['consistency']['p_spread_mm'] for t in PLACE},
     'tangent_spread_deg': {t: json.load(open(f'{IN}/trials/{t}/perception.json'))['consistency']['t_spread_deg'] for t in PLACE}}
sp = np.array(list(D['p_star_spread_snap0_snap1_mm'].values())); D['p_star_spread_stats'] = {'median': float(np.median(sp)), 'p90': float(np.percentile(sp, 90)), 'max': float(sp.max()), 'n': len(sp)}
print('D: R median %.2f [%.2f, %.2f]  arc median %.1f [%.1f, %.1f]  p* spread median %.2f max %.2f mm' % (D['R_day']['median'], D['R_day']['min'], D['R_day']['max'], D['arc_day']['median'], D['arc_day']['min'], D['arc_day']['max'], D['p_star_spread_stats']['median'], D['p_star_spread_stats']['max']))

# ---------------- inventory ----------------
inv = []
for t in TR:
    td = f'{IN}/trials/{t}'; pj = json.load(open(td + '/perception.json'))
    inv.append({'trial': t, 'condition': summ[t]['condition'], 'counted': summ[t]['counted'], 'outcome': summ[t]['outcome'], 'placement': ('= T3_C (reused perception)' if t == 'T3b_C' else t),
                'snap0': os.path.isdir(td + '/snap0'), 'snap1': os.path.isdir(td + '/snap1'), 'hover_dirs': sorted(os.path.basename(p) for p in glob.glob(td + '/hover_*')),
                'pre_close': os.path.isdir(td + '/pre_close'), 'clicks': os.path.exists(td + '/clicks.json'), 'template_clicks': os.path.exists(td + '/template_clicks.json'),
                'preclose_pose': os.path.exists(td + '/preclose_pose.json'), 'perception_ok': pj.get('ok')})
B2['snap0_tip_vs_fs'] = {'status': 'NOT USED: my kinematic tip formula does not reproduce preclose_pose.json tip.cam_mm (differences 24-25 mm, see formula_check_vs_preclose_pose); the snap0 numbers computed with it are discarded'}
json.dump({'A_orientation': A, 'B_scale': Bsum, 'B2_kinematic': B2, 'C_replay': C, 'D_aggregate': D, 'inventory': inv}, open(OUT + '/exp0_results.json', 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
print('wrote exp0_results.json')
