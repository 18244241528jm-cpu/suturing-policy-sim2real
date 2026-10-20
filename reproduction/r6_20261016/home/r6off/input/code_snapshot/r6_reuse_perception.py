#!/usr/bin/env python3
"""R6 (user 12:5x): FS + arc fit run once per placement. For a later attempt on the same placement: take the new trial's snap0
(already captured by r6_trial_start.sh --nofs), check the needle shift against the placement's first frame (NCC patches on the clicked
polyline, same rule as before closing: >= 3 patches, <= 1.0 mm), and write a fresh perception.json that points to the stored fit."""
import argparse, json, os, sys, time, shutil, numpy as np, cv2
ap = argparse.ArgumentParser(); ap.add_argument('--trial', required=True); ap.add_argument('--from_trial', required=True); a = ap.parse_args()
FX = 1860.724932
src = json.load(open(a.from_trial + '/perception.json')); ref = src.get('placement_ref_snap') or src['frames'][0]['snap']
refg = cv2.imread(ref + '/right_rect.png', 0).astype(np.float32); cur = cv2.imread(a.trial + '/snap0/right_rect.png', 0).astype(np.float32)
pts = np.array(src['method']['clicks']['polyline_px'], float); rows = []
for (u, v) in pts:
    u, v = int(round(u)), int(round(v)); h = 12; Tp = refg[v - h:v + h + 1, u - h:u + h + 1]; W = cur[v - h - 30:v + h + 31, u - h - 30:u + h + 31]
    if Tp.shape != (25, 25) or W.shape != (85, 85): continue
    r = cv2.matchTemplate(W, Tp, cv2.TM_CCOEFF_NORMED); _, mx, _, (lx, ly) = cv2.minMaxLoc(r); rows.append({'uv': [u, v], 'ncc': float(mx), 'dx': lx - 30, 'dy': ly - 30})
good = [r for r in rows if r['ncc'] >= 0.7]
if len(good) >= 3:
    d = np.median([[r['dx'], r['dy']] for r in good], 0); mm = float(np.linalg.norm(d) * src['p_star_mm'][2] / FX)
else: d, mm = None, None
ok = mm is not None and mm <= 1.0
out = dict(src); out.update({'created': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'created_t': time.time(), 'reused_from': a.from_trial, 'placement_ref_snap': ref,
            'reuse_check': {'n_valid': len(good), 'shift_px': None if d is None else d.tolist(), 'shift_mm': mm, 'per_point': rows}, 'ok': bool(src['ok'] and ok)})
json.dump(out, open(a.trial + '/perception.json', 'w'), indent=1)
shutil.copy(a.from_trial + '/clicks.json', a.trial + '/clicks.json')
X = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(a.trial))), 'xfer', os.path.basename(a.trial.rstrip('/'))); os.makedirs(X + '/snap0', exist_ok=True)
for f in ('left_rect.png', 'right_rect.png', 'state.json'): shutil.copy(a.trial + '/snap0/' + f, X + '/snap0/' + f)
for f in ('perception.json', 'clicks.json', 'start.json'): shutil.copy(a.trial + '/' + f, X + '/' + f)
print('reuse check: shift %s mm from %d patches -> %s' % (None if mm is None else round(mm, 2), len(good), 'PERCEPTION OK (reused)' if out['ok'] else 'PERCEPTION FAILED (needle moved or unverifiable)'))
