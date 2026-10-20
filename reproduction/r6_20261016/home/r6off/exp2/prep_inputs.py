#!/usr/bin/env python3
"""EXP2 CPU preparation: masks, D-raw, D-fix, K, human reference, M0 copy, M2 mesh. Pre-registered in pre_registration.json.
Run (WSL): ~/.venvs/foundationstereo-5080/bin/python ~/r6off/exp2/prep_inputs.py
Writes to WSL home ~/r6off_exp2/ (not OneDrive: avoids the OneDrive append-loss problem), copied to records afterwards."""
import os, sys, json, glob, shutil, hashlib, numpy as np, cv2
ROOT = os.path.expanduser('~/r6off'); IN = ROOT + '/input'; W = os.path.expanduser('~/r6off_exp2')
sys.path.insert(0, IN + '/code_snapshot'); import r2_arc, r5_grasp_v3 as G
BAND, DARK, OFF = 5, 18, 0.5
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
TR = sorted(os.path.basename(p) for p in glob.glob(IN + '/trials/*') if os.path.isdir(p)); PLACE = [t for t in TR if t != 'T3b_C']
Aq = lambda a_, b_: np.c_[np.ones_like(a_), a_, b_, a_ * a_, a_ * b_, b_ * b_]
os.makedirs(W + '/inputs', exist_ok=True); os.makedirs(W + '/meshes', exist_ok=True)

# ---------------- meshes ----------------
M0_SRC = os.path.expanduser('~/surgical_robotics_challenge/ADF/Phantoms/3D_MED/high_res/Needle.OBJ')
assert sha(M0_SRC) == '2db5c6060c806498e3a18bde5fe4d7d7d53900dd1e167bcb61bc1c7873a8d7ca', 'unexpected M0'
shutil.copy(M0_SRC, W + '/meshes/M0_Needle.OBJ')
V0 = np.array([list(map(float, l.split()[1:4])) for l in open(M0_SRC) if l.startswith('v ')]) * 1000.0     # mm
m0 = G.mesh_arc(V0)
res0 = json.load(open(ROOT + '/exp0/exp0_results.json'))
R2 = float(res0['D_aggregate']['R_day']['median']); ARC2 = float(res0['D_aggregate']['arc_day']['median']); D_WIRE = 0.98
c0 = np.array(m0['centre_mm']); u = np.array(m0['_u']); v = np.array(m0['_v']); a0 = m0['_a_start_deg']; a1 = m0['_a_end_deg']
mid = 0.5 * (a0 + a1); A_S, A_E = mid - ARC2 / 2, mid + ARC2 / 2
nseg = int(np.ceil(ARC2 / 0.5)); nring = 16; rho = D_WIRE / 2
verts = []; faces = []
nz = np.cross(u, v); nz = nz / np.linalg.norm(nz)
for i in range(nseg + 1):
    a = np.radians(A_S + (A_E - A_S) * i / nseg); r = np.cos(a) * u + np.sin(a) * v; P = c0 + R2 * r
    for j in range(nring):
        ph = 2 * np.pi * j / nring; verts.append(P + rho * (np.cos(ph) * r + np.sin(ph) * nz))
nv = len(verts)
for i in range(nseg):
    for j in range(nring):
        a, b, c, d = i * nring + j, i * nring + (j + 1) % nring, (i + 1) * nring + j, (i + 1) * nring + (j + 1) % nring
        faces += [(a, b, d), (a, d, c)]
for end, idx0 in ((0, 0), (1, nseg * nring)):                              # flat caps (fan)
    ac = np.radians(A_S if end == 0 else A_E); Pc = c0 + R2 * (np.cos(ac) * u + np.sin(ac) * v); verts.append(Pc); ci = len(verts) - 1
    for j in range(nring):
        a, b = idx0 + j, idx0 + (j + 1) % nring; faces += [(ci, b, a)] if end == 0 else [(ci, a, b)]
M2 = W + '/meshes/M2_arc.obj'
if os.path.exists(M2): print('M2 exists, not overwritten')
else:
    with open(M2, 'w') as f:
        f.write('# M2 swept-circle needle, units m\n')
        for p in verts: f.write('v %.7f %.7f %.7f\n' % tuple(np.array(p) / 1000.0))
        for t in faces: f.write('f %d %d %d\n' % (t[0] + 1, t[1] + 1, t[2] + 1))
m2 = G.mesh_arc(np.array(verts))
par = {'R_mm': R2, 'arc_deg': ARC2, 'wire_d_mm': D_WIRE, 'wire_status': 'PROVISIONAL (no caliper value)', 'centre_mm': c0.tolist(), 'angle_start_deg': A_S, 'angle_end_deg': A_E,
       'basis_u': u.tolist(), 'basis_v': v.tolist(), 'm0_arc_fit': {k: (v_.tolist() if hasattr(v_, 'tolist') else v_) for k, v_ in m0.items()}, 'm2_arc_refit': {k: (v_.tolist() if hasattr(v_, 'tolist') else v_) for k, v_ in m2.items()},
       'sha256_M0': sha(W + '/meshes/M0_Needle.OBJ'), 'sha256_M2': sha(M2), 'n_vertices_M2': len(verts), 'n_faces_M2': len(faces),
       'tail_is_arc_start': True, 'tail_anchor_mm_M0': [-9.530351, 0.395811, 0.112054], 'tip_anchor_mm_M0': [6.658333, 6.864333, 0.112]}
json.dump(par, open(W + '/meshes/mesh_params.json', 'w'), indent=1)
print('M0 arc: R %.3f arc %.2f | M2 refit: R %.3f arc %.2f (target R %.3f arc %.2f) | M2 sha %s' % (m0['R_mm'], m0['arc_deg'], m2['R_mm'], m2['arc_deg'], R2, ARC2, par['sha256_M2'][:12]))


# ---------------- per-frame inputs ----------------
def frame_inputs(t, snap, k):
    d = f'{IN}/trials/{t}/{snap}'; img = cv2.imread(d + '/right_rect.png'); gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(float)
    disp = np.load(d + '/fs_disp.npz')['disp'].astype(np.float32); Z = G.FB / np.clip(disp, 1, None)
    poly = np.array(json.load(open(f'{IN}/trials/{t}/clicks.json'))['polyline_px'], float)
    bm = np.zeros(gray.shape, np.uint8); cv2.polylines(bm, [poly.round().astype(np.int32)], False, 1, thickness=2 * BAND + 1); bm = bm.astype(bool)
    an = np.zeros(gray.shape, np.uint8); cv2.polylines(an, [poly.round().astype(np.int32)], False, 1, thickness=2 * (BAND + 10) + 1); an = an.astype(bool) & ~bm
    gm = cv2.blur(np.where(an, gray, 0), (41, 41)) / np.maximum(cv2.blur(an.astype(float), (41, 41)), 1e-6); mask = bm & (gray < gm - DARK)
    sup = an & (img[..., 2].astype(int) - img[..., 1] > 30) & np.isfinite(Z)
    ys, xs = np.nonzero(sup); a_ = (xs - G.CX) / G.FX; b_ = (ys - G.CY) / G.FX; zz = Z[ys, xs]; keep = np.ones(len(zz), bool)
    for _ in range(3):
        coef = np.linalg.lstsq(Aq(a_[keep], b_[keep]), zz[keep], rcond=None)[0]; r_ = zz - Aq(a_, b_) @ coef; sd = r_[keep].std(); keep = np.abs(r_) < 2.5 * sd + 1e-6
    ym, xm = np.nonzero(mask); am, bm_ = (xm - G.CX) / G.FX, (ym - G.CY) / G.FX; zm = Aq(am, bm_) @ coef
    Psurf = np.c_[am * zm, bm_ * zm, zm]; cs = Psurf.mean(0); ns = np.linalg.svd(Psurf - cs)[2][2]; ns = ns if ns[2] < 0 else -ns
    Zfix = Z.copy(); Zfix[ym, xm] = (Psurf + OFF * ns)[:, 2]
    out = f'{W}/inputs/{t}_{snap}'; os.makedirs(out, exist_ok=True)
    cv2.imwrite(out + '/rgb.png', img); cv2.imwrite(out + '/mask.png', mask.astype(np.uint8) * 255)
    np.save(out + '/depth_raw.npy', (Z / 1000.0).astype(np.float32)); np.save(out + '/depth_fix.npy', (Zfix / 1000.0).astype(np.float32))
    pj = json.load(open(f'{IN}/trials/{t}/perception.json')); fr = pj['frames'][k]; fit = fr['fit']
    tail_is_a = bool(fr['tail_is_end_a']); ea, eb = np.array(fit['end_a_mm']), np.array(fit['end_b_mm']); tail_end, tip_end = (ea, eb) if tail_is_a else (eb, ea)
    ref = {'trial': t, 'snap': snap, 's': pj['method']['s'], 'fit': fit, 'tail_is_end_a': tail_is_a, 'p_star_mm': fr['p_star_mm'], 'tangent': fr['tangent'], 'normal_toward_cam': fr['normal_toward_cam'],
           'tail_end_mm': tail_end.tolist(), 'tip_end_mm': tip_end.tolist(), 'chord_tail_to_tip_mm': (tip_end - tail_end).tolist(), 'n_mask_px': int(mask.sum()),
           'n_mask_px_with_depth_raw': int((mask & (Z > 0)).sum()), 'dfix_minus_draw_mm_median': float(np.median((Zfix[ym, xm] - Z[ym, xm]))),
           'K': [[G.FX, 0, G.CX], [0, G.FX, G.CY], [0, 0, 1]], 'wh': [img.shape[1], img.shape[0]]}
    json.dump(ref, open(out + '/ref.json', 'w'), indent=1)
    return ref


bank = []
for t in PLACE:
    for k, snap in enumerate(['snap0', 'snap1']):
        ref = frame_inputs(t, snap, k)
        bank.append({'sample_id': f'{t}_{snap}', 'dir': f'{W}/inputs/{t}_{snap}', 's': ref['s'], 'n_mask_px': ref['n_mask_px']})
        print('%-22s mask px %5d  D-fix minus D-raw (needle px, median) %+.2f mm' % (f'{t}_{snap}', ref['n_mask_px'], ref['dfix_minus_draw_mm_median']), flush=True)
with open(W + '/bank.jsonl', 'w') as f:
    for b in bank: f.write(json.dumps(b) + '\n')
print('bank frames', len(bank), 'min mask px', min(b['n_mask_px'] for b in bank))
