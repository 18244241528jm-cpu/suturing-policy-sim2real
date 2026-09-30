#!/usr/bin/env python3
"""R4 day-2 Part M: meeting figures (PNG, 200 dpi) -> records/logs/R4_20260928/figs_meeting/.
usage: r4d2_figs.py M1 M2 M3 [M4 M5 M6]   (run with ~/venv_ambf/bin/python; M1 needs r1_stereo_depth)"""
import json, os, sys, numpy as np, cv2
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [HERE, os.path.join(ROOT, 'scripts/r1_realrobot')]
R4 = os.path.join(ROOT, 'records/logs/R4_20260928'); OUT = R4 + '/figs_meeting'; os.makedirs(OUT, exist_ok=True)
FX, CX, CY = 1860.724932, 690.538345, 576.803638; FB = FX * 5.4476353950418295
K = np.array([[FX, 0, CX], [0, FX, CY], [0, 0, 1]])


def M1():
    import r1_stereo_depth as S, r4_board_pnp as B
    d = R4 + '/F2_board/p06'; g0 = cv2.imread(f'{d}/right_rect/000.png', 0); rgb = cv2.imread(f'{d}/right_rect/000.png')[:, :, ::-1]
    r = B.pnp(g0, K); m = B.board_mask(r['img'], *g0.shape)
    sg = S.make_sgbm(32, 128, 5); sgr = S.make_sgbm(-(32 + 128), 128, 5)
    Z1 = FB / S.disparity_lr_checked(sg, sgr, g0, cv2.imread(f'{d}/left_rect/000.png', 0)); Z1[(Z1 < 50) | (Z1 > 350)] = np.nan
    F1 = np.load(f'{d}/fs_out/Z_FS1_mm.npy'); f2 = json.load(open(f'{d}/f2_result.json'))
    lo, hi = np.nanpercentile(F1[m], [1, 99]); lo, hi = lo - 3, hi + 3
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.6), constrained_layout=True)
    ax[0].imshow(rgb); ax[0].set_title('physical-left rectified, frame 000 (board Z ≈ 54.6 mm)')
    cmap = plt.get_cmap('turbo').copy(); cmap.set_bad('0.55')
    ax[1].imshow(F1, cmap=cmap, vmin=lo, vmax=hi); ax[1].set_title(f"FoundationStereo single frame\nboard valid {f2['methods']['FS1']['valid_frac']:.2f}, median err {f2['methods']['FS1']['median_signed_mm']:+.2f} mm")
    Zs = Z1.copy(); Zs[~np.isfinite(Zs)] = np.nan
    im = ax[2].imshow(Zs, cmap=cmap, vmin=lo, vmax=hi); ax[2].set_title(f"SGBM single frame (R1 params, grey = no depth)\nboard valid {f2['methods']['SG1']['valid_frac']:.2f}")
    inval = m & ~(np.isfinite(Z1) & (np.abs(Z1 - np.nanmedian(F1[m])) < 0.02 * 55))
    for a in ax: a.contour(m, [0.5], colors='w', linewidths=1)
    ax[2].contourf(inval, [0.5, 1.5], colors='none', hatches=['////'])
    ax[2].contour(inval, [0.5], colors='m', linewidths=0.8)
    for a in ax: a.set_xlabel('u (px)'); a.set_ylabel('v (px)')
    fig.colorbar(im, ax=ax[1:], label='depth Z (mm)', shrink=0.85)
    fig.suptitle('M1  55 mm: same frame, same colour scale. White = ChArUco board region; magenta hatched = SGBM missing or >2% off')
    fig.savefig(OUT + '/M1_55mm_FS_vs_SGBM.png', dpi=200); plt.close(fig)


def M2(scene='flat01'):
    F = R4 + '/F3_fp'; V = np.array([l.split()[1:4] for l in open(f'{F}/data/mesh/Needle.obj') if l.startswith('v ')], float)
    Fc = np.array([[int(t.split('/')[0]) - 1 for t in l.split()[1:4]] for l in open(f'{F}/data/mesh/Needle.obj') if l.startswith('f ')])
    res = {(r['scene'], r['cond']): r for r in json.load(open(f'{F}/f3_result.json'))['rows']}
    proj = lambda P: (K @ P.T).T[:, :2] / P[:, 2:3]
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.6), constrained_layout=True)
    for a, (c, name) in zip(ax, (('D0', 'depth = DA P5a (sim-trained)'), ('D5', 'depth = FoundationStereo single frame'))):
        rgb = cv2.imread(f'{F}/data/rgb/{scene}_{c}.png')[:, :, ::-1]; T = np.loadtxt(f'{F}/ob_in_cam/{scene}_{c}.txt')
        P = (T[:3, :3] @ V.T).T + T[:3, 3]; uv = proj(P); r = res[(scene, c)]
        g = proj(np.array([r['gt_mm']])); p = proj(np.array([r['pred_mm']]))
        sil = np.zeros(rgb.shape[:2], np.uint8)
        for f in Fc: cv2.fillConvexPoly(sil, np.round(uv[f] * 8).astype(np.int32), 1, lineType=cv2.LINE_8, shift=3)
        a.imshow(rgb); a.contourf(sil, [0.5, 1.5], colors='lime', alpha=0.35); a.contour(sil, [0.5], colors='lime', linewidths=1.2)
        a.plot([], [], c='lime', lw=3, label='FP pose: mesh silhouette')
        a.plot(*g[0], 'c+', ms=16, mew=2.5, label='GT arc centre (stereo arc fit)'); a.plot(*p[0], 'rx', ms=12, mew=2.5, label='FP arc centre')
        cu, cv_ = g[0]; a.set_xlim(cu - 220, cu + 220); a.set_ylim(cv_ + 170, cv_ - 170)
        a.set_title(f"{name}\n|err| {r['err_mm']:.1f} mm, along ray {r['along_ray_mm']:+.1f} mm, reproj {r['reproj_px']:.0f} px")
        a.set_xlabel('u (px)'); a.set_ylabel('v (px)'); a.legend(loc='lower left', fontsize=8)
    fig.suptitle(f'M2  R1 {scene}: same image, mask, mesh and K; only the depth input to FoundationPose differs')
    fig.savefig(OUT + '/M2_FP_overlay_DA_vs_FS.png', dpi=200); plt.close(fig)


def M3():
    d = json.load(open(R4 + '/day2/0b_calib/result.json')); rows = d['rows']
    fig, ax = plt.subplots(figsize=(8.5, 5.2), constrained_layout=True)
    for mth, col in (('FS1', 'C0'), ('SG100', 'C1')):
        for tb, mk, tl in ((0, 'o', 'tilt A ≈4°'), (1, '^', 'tilt B ≈15°')):
            R = [r for r in rows if r[mth] is not None and r['tiltB'] == tb]
            ax.scatter([r['Z_gt'] for r in R], [100 * r[mth] for r in R], c=col, marker=mk, s=45, label=f'{mth}, {tl}', edgecolors='k', linewidths=0.4)
        mz = d['models'][mth]['M_Z']; z = np.linspace(50, 155, 50); ax.plot(z, 100 * (mz['a'] + mz['b_per_mm'] * z), c=col, lw=1.2, label=f"{mth} fit rel = a + bZ, b = {100 * mz['b_per_mm']:.3f} %/mm")
    dl = d['models']['FS1']['M_disp_delta_only']['delta_px']; z = np.linspace(50, 155, 50)
    ax.plot(z, 100 * ((FB / (FB / z + dl)) / z - 1), 'k:', lw=1, label=f'pure disparity offset, δ = {dl:+.2f} px (FS1)')
    ds = d['models']['FS1']['M_disp_delta_plus_scale']; ax.plot(z, 100 * ((ds['scale'] * FB / (FB / z + ds['delta_px'])) / z - 1), 'k--', lw=1, label=f"offset + scale, δ = {ds['delta_px']:+.2f} px, s = {ds['scale']:.3f} (FS1)")
    ax.axhline(0, c='0.6', lw=0.8); ax.set_xlabel('board depth Z from ChArUco PnP (mm)'); ax.set_ylabel('median depth error / Z (%)')
    ax.set_title('M3  Stereo depth vs ChArUco PnP: relative error becomes more negative with Z;\ntilt B (triangles) sits above tilt A; FS ≈ SGBM'); ax.legend(fontsize=7.5, loc='lower left')
    fig.savefig(OUT + '/M3_F2_relerr_vs_Z.png', dpi=200); plt.close(fig)


if __name__ == '__main__':
    for k in sys.argv[1:]: globals()[k](); print('wrote', k)
