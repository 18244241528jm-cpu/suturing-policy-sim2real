#!/usr/bin/env python3
"""EXP3 FS vs SGBM (pre_registration.json). Run (WSL): ~/.venvs/foundationstereo-5080/bin/python ~/r6off/exp3/exp3.py"""
import os, sys, json, glob, numpy as np, cv2, time
ROOT = os.path.expanduser('~/r6off'); IN = ROOT + '/input'; OUT = ROOT + '/exp3'
sys.path.insert(0, IN + '/code_snapshot'); import r5_grasp_v3 as G
BAND, DARK = 5, 18
TR = sorted(os.path.basename(p) for p in glob.glob(IN + '/trials/*') if os.path.isdir(p)); PLACE = [t for t in TR if t != 'T3b_C']
sg = cv2.StereoSGBM_create(minDisparity=30, numDisparities=128, blockSize=7, P1=8 * 3 * 7 * 7, P2=32 * 3 * 7 * 7, disp12MaxDiff=1, uniquenessRatio=8,
                           speckleWindowSize=100, speckleRange=2, preFilterCap=31, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
Aq = lambda a_, b_: np.c_[np.ones_like(a_), a_, b_, a_ * a_, a_ * b_, b_ * b_]


def surface_fit(Z, sup):
    ys, xs = np.nonzero(sup); a_ = (xs - G.CX) / G.FX; b_ = (ys - G.CY) / G.FX; zz = Z[ys, xs]; keep = np.ones(len(zz), bool)
    for _ in range(3):
        coef = np.linalg.lstsq(Aq(a_[keep], b_[keep]), zz[keep], rcond=None)[0]; r_ = zz - Aq(a_, b_) @ coef; sd = r_[keep].std(); keep = np.abs(r_) < 2.5 * sd + 1e-6
    return coef, float(sd), int(keep.sum()), int(len(zz))


rows = {}
for t in PLACE:
    d = f'{IN}/trials/{t}/snap0'; Rimg = cv2.imread(d + '/right_rect.png'); Limg = cv2.imread(d + '/left_rect.png')
    gR = cv2.cvtColor(Rimg, cv2.COLOR_BGR2GRAY); gL = cv2.cvtColor(Limg, cv2.COLOR_BGR2GRAY)
    t0 = time.time(); ds = sg.compute(gR, gL).astype(np.float32) / 16.0; tsg = time.time() - t0     # ref = physical left, other = physical right -> d > 0
    valid_s = ds >= 30.0; Zs = np.where(valid_s, G.FB / np.maximum(ds, 1e-3), np.nan)
    dfs = np.load(d + '/fs_disp.npz')['disp'].astype(np.float32); Zf = G.FB / np.clip(dfs, 1, None)
    poly = np.array(json.load(open(f'{IN}/trials/{t}/clicks.json'))['polyline_px'], float); g = gR.astype(float)
    bm = np.zeros(g.shape, np.uint8); cv2.polylines(bm, [poly.round().astype(np.int32)], False, 1, thickness=2 * BAND + 1); bm = bm.astype(bool)
    an = np.zeros(g.shape, np.uint8); cv2.polylines(an, [poly.round().astype(np.int32)], False, 1, thickness=2 * (BAND + 10) + 1); an = an.astype(bool) & ~bm
    gm = cv2.blur(np.where(an, g, 0), (41, 41)) / np.maximum(cv2.blur(an.astype(float), (41, 41)), 1e-6); mask = bm & (g < gm - DARK)
    pink = (Rimg[..., 2].astype(int) - Rimg[..., 1] > 30); supF = an & pink & np.isfinite(Zf); supS = an & pink & np.isfinite(Zs)
    cF, sdF, nF, totF = surface_fit(Zf, supF); cS, sdS, nS, totS = surface_fit(Zs, supS)
    ym, xm = np.nonzero(mask); am, bm_ = (xm - G.CX) / G.FX, (ym - G.CY) / G.FX
    zsurfF = Aq(am, bm_) @ cF; zsurfS = Aq(am, bm_) @ cS
    zpF = Zf[ym, xm]; zpS = Zs[ym, xm]; vS = np.isfinite(zpS)
    both = supF & supS
    r = {'sgbm_s': tsg, 'pad_sd_mm': {'FS': sdF, 'SGBM': sdS}, 'pad_support': {'FS': [nF, totF], 'SGBM': [nS, totS]}, 'pad_valid_frac_sgbm': float(supS.sum() / max(an.sum() * pink[an].mean(), 1)),
         'surf_diff_at_needle_FS_minus_SGBM_mm_median': float(np.median(zsurfF - zsurfS)), 'n_needle_px': int(mask.sum()), 'needle_hole_rate_sgbm': float(1 - vS.mean()),
         'wire_minus_surface_mm_median': {'FS': float(np.median(zpF - zsurfF)), 'SGBM': float(np.median(zpS[vS] - zsurfS[vS])) if vS.sum() > 10 else None},
         'pad_disagreement_abs_mm_median': float(np.median(np.abs(Zf[both] - Zs[both]))) if both.sum() else None, 'pad_diff_signed_FS_minus_SGBM_mm_median': float(np.median(Zf[both] - Zs[both])) if both.sum() else None}
    rows[t] = r
    print('%-14s pad sd FS %.2f SGBM %.2f | hole(needle) %.2f | wire-surf FS %+.2f SGBM %s | |FS-SGBM| pad %.2f signed %+.2f | surf diff @needle %+.2f' % (t, sdF, sdS, r['needle_hole_rate_sgbm'], r['wire_minus_surface_mm_median']['FS'], ('%+.2f' % r['wire_minus_surface_mm_median']['SGBM']) if r['wire_minus_surface_mm_median']['SGBM'] is not None else 'n/a', r['pad_disagreement_abs_mm_median'] or np.nan, r['pad_diff_signed_FS_minus_SGBM_mm_median'] or np.nan, r['surf_diff_at_needle_FS_minus_SGBM_mm_median']), flush=True)
agg = lambda f: [f(v) for v in rows.values() if f(v) is not None]
S = {'median_pad_sd_FS': float(np.median(agg(lambda v: v['pad_sd_mm']['FS']))), 'median_pad_sd_SGBM': float(np.median(agg(lambda v: v['pad_sd_mm']['SGBM']))),
     'median_needle_hole_rate_sgbm': float(np.median(agg(lambda v: v['needle_hole_rate_sgbm']))), 'median_wire_minus_surf_FS': float(np.median(agg(lambda v: v['wire_minus_surface_mm_median']['FS']))),
     'median_wire_minus_surf_SGBM': float(np.median(agg(lambda v: v['wire_minus_surface_mm_median']['SGBM']))) if agg(lambda v: v['wire_minus_surface_mm_median']['SGBM']) else None,
     'median_pad_abs_disagreement_mm': float(np.median(agg(lambda v: v['pad_disagreement_abs_mm_median']))), 'median_pad_signed_FS_minus_SGBM_mm': float(np.median(agg(lambda v: v['pad_diff_signed_FS_minus_SGBM_mm_median']))),
     'n_with_sgbm_wire': len(agg(lambda v: v['wire_minus_surface_mm_median']['SGBM']))}
print('SUMMARY', S)
json.dump({'per_placement': rows, 'summary': S}, open(OUT + '/exp3_results.json', 'w'), indent=1)
