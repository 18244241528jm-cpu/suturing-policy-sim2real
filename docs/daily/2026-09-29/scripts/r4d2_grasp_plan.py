#!/usr/bin/env python3
"""R4 day-2 grasp planning for a new needle placement (run with ~/venv_ambf/bin/python):
capture dir (r4d2_capture.py) -> FoundationStereo on the 3090 -> needle mask inside --roi (low redness / dark, largest component)
-> stereo arc fit (r2_arc) -> grasp point = arc point in the middle 30-70% of the visible span whose image tangent is most parallel to the
jaw axis (0.55,0.83) -> writes <scene>/grasp_target.json with grasp point, normal, hover/final targets and hover target in measured_cp.
usage: r4d2_grasp_plan.py --scene DIR --roi X0 Y0 X1 Y1 [--skip-fs]"""
import argparse, json, os, sys, numpy as np, cv2
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [HERE, os.path.join(ROOT, 'scripts/r1_realrobot'), os.path.join(ROOT, 'scripts/r2_realrobot')]
import r2_arc as A
ap = argparse.ArgumentParser(); ap.add_argument('--scene', required=True); ap.add_argument('--roi', type=int, nargs=4, required=True); ap.add_argument('--skip-fs', action='store_true'); a = ap.parse_args()
FX, CX, CY = 1860.724932, 690.538345, 576.803638; FB = FX * 5.4476353950418295; d = a.scene
proj = lambda X: np.array([FX * X[0] / X[2] + CX, FX * X[1] / X[2] + CY])
if not a.skip_fs:
    import r4_f2_eval as E
    disp = E.run_fs(d, ['000'])[0]; Z = FB / np.where(disp > 0, disp, np.nan); np.save(f'{d}/fs_out/Z_FS1_mm.npy', Z.astype(np.float32))
Z = np.load(f'{d}/fs_out/Z_FS1_mm.npy')
im = cv2.imread(f'{d}/right_rect/000.png').astype(int); red = im[:, :, 2] - im[:, :, 1]; V = im.max(2)
x0, y0, x1, y1 = a.roi; roi = np.zeros(red.shape, bool); roi[y0:y1, x0:x1] = True
pad = ~roi; pad_red, pad_V = np.median(red[roi]), np.median(V[roi])
m = ((red < pad_red - 25) | (V < pad_V - 60)) & roi; m = m.astype(np.uint8)
m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8)); m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
nl, lab, st, _ = cv2.connectedComponentsWithStats(m); k = 1 + int(np.argmax(st[1:, 4])); mask = lab == k
cv2.imwrite(f'{d}/needle_mask_auto.png', mask.astype(np.uint8) * 255)
me = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
P, _ = A.backproject(Z, me); r = A.fit(P)
c = np.array(r['centre_mm']); n = np.array(r['normal_toward_cam']); u = np.array(r['_u']); v = np.array(r['_v']); R = r['R_mm']
a0, span = r['_a_start_deg'], (r['_a_end_deg'] - r['_a_start_deg']) % 360
pt = lambda ang: c + R * (np.cos(np.radians(ang)) * u + np.sin(np.radians(ang)) * v)
jaw_axis = np.array([0.55, 0.83]); jaw_axis /= np.linalg.norm(jaw_axis); cands = []
for f in np.linspace(0.30, 0.70, 41):
    ang = a0 + f * span; p = pt(ang); t2 = proj(pt(ang + 1)) - proj(pt(ang - 1)); t2 /= np.linalg.norm(t2)
    cands.append((float(np.degrees(np.arccos(min(1, abs(t2 @ jaw_axis))))), float(f), p))
best = min(cands, key=lambda x: x[0]); p = best[2]
out = {'scene': d, 'roi': a.roi, 'mask_px': int(mask.sum()), 'arc': {k2: v2 for k2, v2 in r.items() if not k2.startswith('_')},
       'grasp_point_cam_mm': p.tolist(), 'grasp_uv': proj(p).tolist(), 'grasp_frac_of_span': best[1], 'needle_vs_jaw_axis_deg': best[0]}
json.dump(out, open(f'{d}/grasp_target.json', 'w'), indent=1)
ov = im.astype(np.uint8).copy(); cnt, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE); cv2.drawContours(ov, cnt, -1, (0, 255, 0), 1)
circ = np.array([proj(pt(t)) for t in np.linspace(0, 360, 200)]).astype(np.int32); cv2.polylines(ov, [circ], True, (255, 255, 0), 1)
cv2.drawMarker(ov, tuple(proj(p).astype(int)), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 22, 2); cv2.rectangle(ov, (x0, y0), (x1, y1), (255, 0, 255), 1)
cv2.imwrite(f'{d}/grasp_plan_overlay.png', ov)
print('mask px', out['mask_px'], '| arc R %.2f span %.0f rms %.2f' % (R, span, r['rms_mm']), '| grasp uv', proj(p).round(0), 'Z %.1f' % p[2], '| needle vs jaw axis %.1f deg at %.2f of span' % (best[0], best[1]))
