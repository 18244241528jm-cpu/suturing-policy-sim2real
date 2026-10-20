#!/usr/bin/env python3
"""EXP2 figures. Run (WSL): ~/.venvs/foundationstereo-5080/bin/python ~/r6off/exp2/make_figs.py"""
import os, sys, json, numpy as np, cv2
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
W = os.path.expanduser('~/r6off_exp2'); OUT = os.path.expanduser('~/r6off/figs'); sys.argv = ['x']
exec(open(os.path.expanduser('~/r6off/exp2/analyze_fp.py')).read().split("if __name__")[0])
mms = mesh_models(W); refs = load_refs(W); R = json.load(open(W + '/exp2_results.json'))
FX, CX, CY = 1860.724932, 690.538345, 576.803638
proj = lambda P: np.c_[FX * P[:, 0] / P[:, 2] + CX, FX * P[:, 1] / P[:, 2] + CY]
def arc_pts(arc, a0, a1, n=60):
    c, u, v = np.array(arc['centre_mm']), np.array(arc['_u']), np.array(arc['_v']); a = np.radians(np.linspace(a0, a1, n))
    return c + arc['R_mm'] * (np.cos(a)[:, None] * u + np.sin(a)[:, None] * v)
places = sorted({s.rsplit('_snap', 1)[0] for s in refs})
for cfg in R:
    name, dk = cfg.split('+D-'); mm = mms[name]
    fig, axes = plt.subplots(3, 5, figsize=(17, 10.6)); axes = axes.ravel()
    for ax, p in zip(axes, places):
        sid = p + '_snap0'; ref = refs[sid]; z = np.load(f'{W}/{name}/candidates/{sid}__{dk}.npz'); T = z['poses_camera'][0]
        img = cv2.cvtColor(cv2.imread(f'{W}/inputs/{sid}/rgb.png'), cv2.COLOR_BGR2RGB)
        f = ref['fit']; hum = arc_pts(f, f['_a_start_deg'], f['_a_start_deg'] + f['arc_deg'])
        arc = mm['arc']; P = arc_pts(arc, arc['_a_start_deg'], arc['_a_end_deg']); Pc = (T[:3, :3] @ P.T).T + T[:3, 3] * 1000.0
        pf = proj(Pc); ph = proj(hum); cen = proj(np.array([ref['p_star_mm']]))[0]; h = 150
        x0, y0 = int(cen[0] - h), int(cen[1] - h); crop = img[max(y0, 0):y0 + 2 * h, max(x0, 0):x0 + 2 * h]
        ox, oy = max(x0, 0), max(y0, 0)
        ax.imshow(crop); ax.plot(ph[:, 0] - ox, ph[:, 1] - oy, '-', c='yellow', lw=1.2); ax.plot(pf[:, 0] - ox, pf[:, 1] - oy, '-', c='cyan', lw=1.2)
        tail_h = proj(np.array([ref['tail_end_mm']]))[0]; tail_f = pf[0]
        ax.plot(*(tail_h - [ox, oy]), 'o', mfc='none', mec='yellow', ms=9); ax.plot(*(tail_f - [ox, oy]), 'o', mfc='none', mec='cyan', ms=5)
        t = R[cfg]['frames'][sid]['top1']; ax.set_title('%s  g=%.1f mm  t=%.0f deg %s' % (p, t['grasp_err_mm'], t['tangent_deg'], 'FLIP' if t['flip'] else ('ok' if t['pass'] else 'fail')), fontsize=9,
                                                        color=('green' if t['pass'] else 'red')); ax.axis('off')
    for ax in axes[len(places):]: ax.axis('off')
    fig.suptitle(f'{cfg}: FP top-1 (cyan) vs human reference (yellow); circles mark the TAIL end', fontsize=12)
    fig.text(0.01, 0.005, f'R6 offline 2026-10-16 | snap0 of 13 placements | {name}, depth {dk} | 252 hyp, iter 5, seed 2718, 640x480 crop | 300 px window around grasp point | pass = grasp<=2 mm, tangent<=15 deg, no flip', fontsize=7)
    fig.tight_layout(rect=[0, 0.02, 1, 0.97]); fig.savefig(f'{OUT}/exp2_overlay_{name}_{dk}.png', dpi=110); plt.close(fig)
fig, ax = plt.subplots(figsize=(7.5, 6.2)); col = {'M0+D-raw': 'tab:blue', 'M0+D-fix': 'tab:cyan', 'M2+D-raw': 'tab:red', 'M2+D-fix': 'tab:orange'}; mk = {'M0': 'o', 'M2': 's'}
for cfg, v in R.items():
    g = [r['top1']['grasp_err_mm'] for r in v['frames'].values()]; tg = [r['top1']['tangent_deg'] for r in v['frames'].values()]; fl = [r['top1']['flip'] for r in v['frames'].values()]
    ax.scatter(g, tg, c=col[cfg], marker=mk[cfg[:2]], s=34, alpha=0.75, label=cfg, edgecolors=['k' if f else 'none' for f in fl], linewidths=1.2)
ax.axvline(2, c='gray', ls='--', lw=0.8); ax.axhline(15, c='gray', ls='--', lw=0.8); ax.set_xlabel('grasp point error vs human reference (mm)'); ax.set_ylabel('tangent angle error (deg)'); ax.legend(fontsize=8, title='black edge = flipped top-1')
ax.set_title('FP top-1 error on 26 real frames'); fig.text(0.01, 0.005, 'R6 offline 2026-10-16 | snap0+snap1 of 13 placements | dashed = pass thresholds (2 mm, 15 deg)', fontsize=7)
fig.tight_layout(rect=[0, 0.02, 1, 1]); fig.savefig(f'{OUT}/exp2_scatter.png', dpi=130)
print('figs done')
