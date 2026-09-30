#!/usr/bin/env python3
"""R4 day-2 package figures (static PNG, 200 dpi). Palette: dataviz reference categorical slots 1-4 (validated instance).
usage: r4d2_package_figs.py OUTDIR"""
import json, os, sys, numpy as np, cv2
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
L = os.path.expanduser('~/suturing-policy-sim2real/records/logs'); D2 = f'{L}/R4_20260928/day2'
C = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']; SURF = '#fcfcfb'; INK = '#0b0b0b'; INK2 = '#52514e'; GRID = '#e4e3df'
plt.rcParams.update({'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF, 'axes.edgecolor': GRID, 'axes.labelcolor': INK,
                     'xtick.color': INK2, 'ytick.color': INK2, 'text.color': INK, 'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'axes.axisbelow': True})
FX, CX, CY = 1860.724932, 690.538345, 576.803638
rgb = lambda p: cv2.cvtColor(cv2.imread(p), cv2.COLOR_BGR2RGB)

# ---------- F1: D7 bias ----------
met = json.load(open(f'{D2}/0a_D7/metrics_P5a-keepaspect_P5a-square_EXP8-main_EXP8-ctrl.json')); held = json.load(open(f'{D2}/0a_D7/held_eval.json'))
tags = ['P5a-keepaspect', 'P5a-square', 'EXP8-ctrl', 'EXP8-main']; lab = ['P5a\n(keep aspect, R2)', 'P5a\n(square 518)', 'EXP8-ctrl\n(old K)', 'EXP8-main\n(real K)']
fig, ax = plt.subplots(1, 2, figsize=(11, 4.6), sharey=True, constrained_layout=True)
for a, title in zip(ax, ('Flat scenes, needle ~104 mm from camera\n(24 frames, 4 scenes; dots = per-frame)', 'Held scenes, needle 70-82 mm\n(4 scenes; mask eroded 2 px)')):
    a.axhspan(-10, 10, color=C[2], alpha=0.10, lw=0); a.axhspan(-25, -12, color=C[3], alpha=0.12, lw=0); a.axhline(0, color=INK2, lw=0.8)
    a.set_title(title, fontsize=10)
for i, t in enumerate(tags):
    v = [r['target']['median_bias_mm'] for r in met[t]['frames'] if r['target']]
    ax[0].scatter(np.full(len(v), i) + np.random.default_rng(i).uniform(-0.15, 0.15, len(v)), v, s=14, color=C[0] if 'EXP8-main' not in t else C[1], alpha=0.55, lw=0)
    m = np.median(v); ax[0].hlines(m, i - 0.3, i + 0.3, color=INK, lw=2); ax[0].text(i + 0.33, m, f'{m:.1f}', va='center', fontsize=9, color=INK)
    if t in held:
        h = [r['needle_median_bias_mm'] for r in held[t]]; ax[1].scatter(np.full(4, i) + np.linspace(-0.12, 0.12, 4), h, s=28, color=C[0] if 'EXP8-main' not in t else C[1], lw=0)
        mh = np.median(h); ax[1].hlines(mh, i - 0.3, i + 0.3, color=INK, lw=2); ax[1].text(i + 0.33, mh, f'{mh:.1f}', va='center', fontsize=9)
for a in ax: a.set_xticks(range(4)); a.set_xticklabels(lab, fontsize=9); a.set_xlim(-0.6, 3.8)
ax[0].set_ylabel('needle-region median depth bias vs stereo GT (mm)')
ax[0].text(-0.55, 9, 'H_K: |bias| <= 10 mm', ha='left', va='top', fontsize=8.5, color=INK2); ax[0].text(-0.55, -12.5, 'H_dist: -25 ... -12 mm', ha='left', va='top', fontsize=8.5, color=INK2)
fig.suptitle('D7: retraining DA with the real intrinsics removes ~10 of 36 mm; the residual bias grows with distance', fontsize=11)
fig.savefig(f'{OUT}/F1_D7_depth_bias.png', dpi=200); plt.close(fig)

# ---------- F2: FS+FP on matching vs mismatched needle ----------
sys.path.insert(0, os.path.expanduser('~/suturing-policy-sim2real/scripts/r2_realrobot')); import r2_arc as A
K = np.array([[FX, 0, CX], [0, FX, CY], [0, 0, 1]]); proj = lambda P: (K @ P.T).T[:, :2] / P[:, 2:3]
def fp_panel(a, scene, stem, maskf, title, box):
    d = f'{D2}/{scene}'; im = rgb(f'{d}/right_rect/000.png').copy()
    V = np.array([l.split()[1:4] for l in open(f'{d}/fp_in/mesh/Needle.obj') if l.startswith('v ')], float)
    F = np.array([[int(t.split('/')[0]) - 1 for t in l.split()[1:4]] for l in open(f'{d}/fp_in/mesh/Needle.obj') if l.startswith('f ')])
    T = np.loadtxt(f'{d}/fp_out/ob_in_cam/{stem}.txt'); uv = proj((T[:3, :3] @ V.T).T * 1000 + T[:3, 3] * 1000)
    sil = np.zeros(im.shape[:2], np.uint8)
    for f in F: cv2.fillConvexPoly(sil, np.round(uv[f]).astype(np.int32), 1)
    Z = np.load(f'{d}/fs_out/Z_FS1_mm.npy'); m = cv2.imread(f'{d}/masks/{maskf}', 0) > 0
    r = A.fit(A.backproject(Z, cv2.erode(m.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0)[0]); c = np.array(r['centre_mm']); u, v = np.array(r['_u']), np.array(r['_v'])
    circ = proj(np.array([c + r['R_mm'] * (np.cos(t) * u + np.sin(t) * v) for t in np.linspace(0, 2 * np.pi, 200)]))
    cm = T[:3, :3] @ np.array([-0.0537, 0.3367, 0.1119]) + T[:3, 3] * 1000; err = np.linalg.norm(cm - c)
    x0, y0, x1, y1 = box; a.imshow(im[y0:y1, x0:x1], extent=(x0, x1, y1, y0))
    a.contour(np.arange(im.shape[1]), np.arange(im.shape[0]), sil, [0.5], colors=[C[1]], linewidths=1.8)
    a.plot(circ[:, 0], circ[:, 1], color=C[0], lw=1.2, ls='--'); a.plot(*proj(c[None])[0], marker='+', ms=14, mew=2.2, color=C[0]); a.plot(*proj(cm[None])[0], marker='x', ms=11, mew=2.2, color=C[1])
    a.set_xlim(x0, x1); a.set_ylim(y1, y0); a.set_xticks([]); a.set_yticks([]); a.grid(False)
    a.set_title(f'{title}\ncentre error {err:.1f} mm', fontsize=10)
fig, ax = plt.subplots(1, 2, figsize=(11, 5.2), constrained_layout=True)
fp_panel(ax[0], 'fsfp_s02', 's02_FS1_B_mesharc', 'needle_B_mesharc.png', 'R1 needle (matches mesh): FS depth -> FP', (560, 280, 900, 660))
fp_panel(ax[1], 'fsfp_s01', 's01_FS1', 'needle.png', 'Thicker, smaller needle (does not match mesh)', (560, 250, 1010, 700))
from matplotlib.lines import Line2D
fig.legend(handles=[Line2D([], [], color=C[1], lw=2, label='FP pose (mesh silhouette, x = arc centre)'), Line2D([], [], color=C[0], lw=1.5, ls='--', label='stereo arc fit on FS depth (+ = centre), used as GT')],
           loc='lower center', ncol=2, frameon=False, bbox_to_anchor=(0.5, -0.06))
fig.suptitle('FoundationStereo + FoundationPose on the real robot: works when the mesh matches the needle, fails when it does not', fontsize=11)
fig.savefig(f'{OUT}/F2_FS_FP_matching_vs_mismatched.png', dpi=200, bbox_inches='tight'); plt.close(fig)

# ---------- F3: positioning error chain ----------
cal = json.load(open(f'{D2}/grid_calib/calibration_v3.json'))
items = [('calibration model, held-out RMS', 'rigid SE(3)', cal['models']['SE3']['test_rms'], C[0]), ('', 'similarity Sim(3)', cal['models']['Sim3']['test_rms'], C[0]),
         ('', 'affine (12 dof)', cal['models']['affine']['test_rms'], C[0]),
         ('grasp 1 (target inside calibrated grid)', 'open-loop reach (residual RL)', 1.62, C[1]), ('', 'after 1 camera correction', 0.20, C[2]),
         ('grasp 2 (target 2.6 mm outside grid)', 'open-loop reach (residual RL)', 4.47, C[1]), ('', 'after 1 camera correction', 0.15, C[2]), ('', 'closed-jaw visual descent', 0.44, C[2])]
fig, ax = plt.subplots(figsize=(9.5, 5.6), constrained_layout=True)
y, cur = [], 0.0
for k, it in enumerate(items):
    if it[0] and k: cur += 0.9
    y.append(-cur); cur += 1.0
y = np.array(y)
for yy, (grp, name, val, col) in zip(y, items):
    ax.barh(yy, val, height=0.62, color=col); ax.text(val + 0.06, yy, f'{val:.2f} mm', va='center', fontsize=9)
    if grp: ax.text(-0.02, yy + 0.62, grp, ha='right', va='bottom', fontsize=9, color=INK, fontweight='bold', transform=ax.get_yaxis_transform())
ax.set_yticks(y); ax.set_yticklabels([i[1] for i in items], fontsize=9); ax.set_xlabel('camera-measured 3D error of the tracked jaw feature (mm)')
ax.set_xlim(0, 5.3); ax.grid(axis='y', visible=False)
fig.suptitle('Positioning accuracy: open-loop reach is 1.6-4.5 mm off;\none camera correction brings the jaw to 0.15-0.20 mm, the closed-jaw descent holds 0.44 mm', fontsize=11)
fig.savefig(f'{OUT}/F3_positioning_error_chain.png', dpi=200); plt.close(fig)

# ---------- F4: grasp 2 sequence ----------
G = f'{D2}/grasp_g2'; frames = [('correct_1', '1  hover 10 mm above (err 0.15 mm)'), ('dclosed_3', '2  closed-jaw visual descent\n    to 2.5 mm above (err 0.44 mm)'), ('open_low', '3  jaw opens at 2.5 mm: needle unmoved'),
                                ('at_final', '4  open-loop last 2.5 mm: needle unmoved'), ('closed', '5  jaw closes: needle pushed'), ('lifted', '6  lift 10 mm: needle still on pad')]
fig, ax = plt.subplots(2, 3, figsize=(12, 8.2), constrained_layout=True)
for a, (f, t) in zip(ax.ravel(), frames):
    p = f'{G}/{f}_right.png'
    if os.path.exists(p): a.imshow(rgb(p)[230:720, 560:1050])
    a.set_title(t, fontsize=10, loc='left'); a.set_xticks([]); a.set_yticks([]); a.grid(False)
fig.suptitle('Grasp attempt 2 (camera view): positioning was sub-millimetre and the needle stayed put while the jaw opened and descended;\nclosing the jaw pushed the needle out instead of clamping it (needle at 57 deg to the jaw axis)', fontsize=11)
fig.savefig(f'{OUT}/F4_grasp2_sequence.png', dpi=200); plt.close(fig)

# ---------- F5: automatic segmentation ----------
ev = {r['case']: r for r in json.load(open(f'{D2}/auto_seg/eval.json'))}
sel = [('R2/n1_black_printed_on_pad', 'R2_n1_black_printed_on_pad.png', 'black printed needle'), ('R2/n3_metal_purple_large_on_pad', 'R2_n3_metal_purple_large_on_pad.png', 'metal needle + purple suture'),
       ('day2/fsfp_s01 (thick needle)', 'day2_fsfp_s01_(thick_needle).png', 'thick needle, tool in view'), ('day2/fsfp_s02 (R1 needle+thread)', 'day2_fsfp_s02_(R1_needle+thread).png', 'R1 needle + suture (grasp scene)'),
       ('R1/held_needle_scene02', 'R1_held_needle_scene02.png', 'needle held in the jaw'), ('R1/flat_needle_scene03', 'R1_flat_needle_scene03.png', 'R1 needle flat on pad')]
fig, ax = plt.subplots(2, 3, figsize=(12, 8.4), constrained_layout=True)
for a, (case, f, t) in zip(ax.ravel(), sel):
    r = ev[case]; im = rgb(f'{D2}/auto_seg/overlays/{f}'); cx, cy = r['centre_px']; R = r['R_px']; pad = 70
    x0, y0 = int(max(cx - R - pad, 0)), int(max(cy - R - pad, 0)); x1, y1 = int(min(cx + R + pad, 1300)), int(min(cy + R + pad, 1024))
    a.imshow(im[y0:y1, x0:x1]); a.set_xticks([]); a.set_yticks([]); a.grid(False)
    s = f"P {r['precision_3px']:.2f}  R {r['recall_3px']:.2f}" + (f"  arc-centre diff {r['centre_diff_mm']:.2f} mm" if r.get('centre_diff_mm') is not None else '')
    a.set_title(f'{t}\n{s}', fontsize=9.5, loc='left')
fig.legend(handles=[Line2D([], [], color='#00ff00', lw=2, label='automatic mask (no ROI, no human input)'), Line2D([], [], color='#ff00ff', lw=2, label='reference mask'),
                    Line2D([], [], color='#00ffff', lw=1.5, label='fitted circle')], loc='lower center', ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.04))
fig.suptitle('Automatic needle segmentation: correct needle found in 15/15 reference scenes (Wilson 95% CI 0.80-1.00); P/R within 3 px', fontsize=11)
fig.savefig(f'{OUT}/F5_auto_segmentation.png', dpi=200, bbox_inches='tight'); plt.close(fig)
print('done')
