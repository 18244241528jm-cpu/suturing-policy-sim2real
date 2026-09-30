#!/usr/bin/env python3
"""Offline validation of r4d2_auto_seg.py on stored images with reference masks.
Metrics per scene: IoU, precision/recall vs the reference mask, and (where depth exists) the 3D arc-fit centre difference
between auto and reference masks (r2_arc.fit, masks eroded 1 px). Writes records/logs/R4_20260928/day2/auto_seg/{eval.json,overlays}."""
import json, os, sys, time, numpy as np, cv2
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(os.path.dirname(HERE)); sys.path[:0] = [HERE, os.path.join(ROOT, 'scripts/r2_realrobot')]
import r4d2_auto_seg as S, r2_arc as A
L = os.path.join(ROOT, 'records/logs'); OUT = os.path.join(L, 'R4_20260928/day2/auto_seg'); os.makedirs(OUT + '/overlays', exist_ok=True)
cases = []
for s in sorted(os.listdir(f'{L}/R1_20260921/C_static')):
    d = f'{L}/R1_20260921/C_static/{s}'; cases.append((f'R1/{s}', f'{d}/right_rect/000.png', f'{d}/masks/needle.png', f'{d}/depth_gt/depth_fused_mm.npy', 'R1 human mask'))
for s in sorted(x for x in os.listdir(f'{L}/R2_20260922/D_needles') if x.startswith('n')):
    d = f'{L}/R2_20260922/D_needles/{s}'; cases.append((f'R2/{s}', f'{d}/right_rect/000.png', f'{d}/masks/needle.png', None, 'R2 human mask'))
D2 = f'{L}/R4_20260928/day2'
cases += [('day2/fsfp_s01 (thick needle)', f'{D2}/fsfp_s01/right_rect/000.png', f'{D2}/fsfp_s01/masks/needle.png', f'{D2}/fsfp_s01/fs_out/Z_FS1_mm.npy', 'Claude ROI mask'),
          ('day2/fsfp_s02 (R1 needle+thread)', f'{D2}/fsfp_s02/right_rect/000.png', f'{D2}/fsfp_s02/masks/needle_B_mesharc.png', f'{D2}/fsfp_s02/fs_out/Z_FS1_mm.npy', 'Claude ROI mask B'),
          ('day2/grasp2 (R1 needle+thread)', f'{D2}/grasp2_scene/right_rect/000.png', f'{D2}/grasp2_scene/needle_mask_auto.png', f'{D2}/grasp2_scene/fs_out/Z_FS1_mm.npy', 'Claude ROI mask'),
          ('day2/grasp_g2 hover (tool over needle)', f'{D2}/grasp_g2/correct_1_right.png', None, None, 'no reference')]
rows = []
for name, img, ref, dep, reftype in cases:
    if not os.path.exists(img): print('skip', name); continue
    im = cv2.imread(img); Zd = np.load(dep).astype(float) if dep and os.path.exists(dep) else None
    t0 = time.time(); m, dg = S.segment(im, needle_arc_deg=(136 if ('R1' in name or 's02' in name or 'grasp' in name) else None), depth_mm=Zd); dt = time.time() - t0
    row = {'case': name, 'ref': reftype, 'runtime_s': round(dt, 2), **{k: v for k, v in dg.items()}}
    if m is not None and ref and os.path.exists(ref):
        g = cv2.imread(ref, 0) > 0; inter = (m & g).sum(); uni = (m | g).sum()
        row.update(IoU=float(inter / uni), precision=float(inter / max(m.sum(), 1)), recall=float(inter / max(g.sum(), 1)))
        # tolerant overlap: within 3 px of each other (masks drawn by different people differ at the edges)
        gd = cv2.dilate(g.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0; md = cv2.dilate(m.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
        row.update(precision_3px=float((m & gd).sum() / max(m.sum(), 1)), recall_3px=float((g & md).sum() / max(g.sum(), 1)))
        if dep and os.path.exists(dep):
            Z = np.load(dep).astype(float); er = lambda k: cv2.erode(k.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
            try:
                fa = A.fit(A.backproject(Z, er(m))[0]); fg = A.fit(A.backproject(Z, er(g))[0])
                row.update(arc_R_auto=fa['R_mm'], arc_R_ref=fg['R_mm'], centre_diff_mm=float(np.linalg.norm(np.array(fa['centre_mm']) - np.array(fg['centre_mm']))),
                           normal_diff_deg=float(np.degrees(np.arccos(min(1, abs(np.dot(fa['normal_toward_cam'], fg['normal_toward_cam'])))))))
            except Exception as e: row['arc_error'] = str(e)
    if m is not None:
        ov = S.overlay(im, m, dg)
        if ref and os.path.exists(ref):
            g = cv2.imread(ref, 0) > 0; cnt, _ = cv2.findContours(g.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE); cv2.drawContours(ov, cnt, -1, (255, 0, 255), 1)
        cv2.imwrite(f"{OUT}/overlays/{name.replace('/', '_').replace(' ', '_')}.png", ov)
    rows.append(row)
json.dump(rows, open(f'{OUT}/eval.json', 'w'), indent=1, default=float)
f = lambda v, p=2: ('%.' + str(p) + 'f') % v if isinstance(v, (int, float)) else '-'
print(f"{'case':42s} {'ok':>3s} {'conf':>5s} {'IoU':>5s} {'P3px':>5s} {'R3px':>5s} {'span':>5s} {'thr':>4s} {'dC mm':>6s} {'R a/r':>11s} {'t s':>5s}")
for r in rows:
    print(f"{r['case'][:42]:42s} {str(r.get('ok'))[0]:>3s} {str(r.get('confident'))[0]:>5s} {f(r.get('IoU'))} {f(r.get('precision_3px'))} {f(r.get('recall_3px'))} {f(r.get('arc_span_deg'),0):>5s} {f(r.get('thread_trimmed_deg'),0):>4s} {f(r.get('centre_diff_mm')):>6s} {f(r.get('arc_R_auto'),1)+'/'+f(r.get('arc_R_ref'),1):>11s} {f(r.get('runtime_s'),1):>5s} R3d {f(r.get('arc3d_R_mm'),1)} thr_side {r.get('thread_side')} cap {f(r.get('capped_deg'),0)} {r.get('reason','')}")
