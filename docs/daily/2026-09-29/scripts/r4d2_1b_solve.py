#!/usr/bin/env python3
"""R4 day-2 1-b: local camera(rect physical-left) <- PSM1 base registration + TCP, SE(3) and Sim(3), pre-drawn 25% test split.
Tip pixels: day2/1b_tip/tip_clicks.json. Disparity: NCC along the epipolar line on the 10-frame mean images (as r2_tip_solve2.py);
peaks with margin >= 0.10 are 'clear'; ambiguous TRAIN poses resolved with a model fitted on clear train poses only;
ambiguous TEST poses resolved with the final train-only model (test never enters any fit). Writes day2/1b_tip/registration.json."""
import json, math, os, numpy as np, cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
FX, CX, CY, FB = 1860.724932, 690.538345, 576.803638, 1860.724932 * 5.4476353950418295
B = 'records/logs/R4_20260928/day2/1b_tip'
man = json.load(open(B + '/manifest.json')); clk = json.load(open(B + '/tip_clicks.json')); TEST = set(man['test_poses'])
P = {}
for k, (x, y) in clk['tip_R'].items():
    i = int(k); Rm = cv2.imread(f'{B}/pose_{i:02d}/right_rect_mean.png', 0).astype(np.float32); Lm = cv2.imread(f'{B}/pose_{i:02d}/left_rect_mean.png', 0).astype(np.float32)
    h = 20; y0, y1 = max(y - h, 0), min(y + h + 1, 1024); x0, x1 = max(x - h, 0), min(x + h + 1, 1300); T = Rm[y0:y1, x0:x1]
    ds = np.arange(40, 260); c = np.nan_to_num(np.array([np.corrcoef(T.ravel(), Lm[y0:y1, x0 - d:x1 - d].ravel())[0, 1] if x0 - d >= 0 else np.nan for d in ds]), nan=-9)
    pk = sorted([j for j in range(1, len(c) - 1) if c[j] >= c[j - 1] and c[j] >= c[j + 1] and c[j] > 0.3], key=lambda j: -c[j])
    def sub(j):
        den = c[j - 1] - 2 * c[j] + c[j + 1]; return float(ds[j] + (0.5 * (c[j - 1] - c[j + 1]) / den if den < 0 else 0))
    fr = json.load(open(f'{B}/pose_{i:02d}/pose.json'))['frames']
    P[i] = dict(uv=(x, y), peaks=[(sub(j), float(c[j])) for j in pk[:4]], margin=float(c[pk[0]] - (c[pk[1]] if len(pk) > 1 else 0)),
                tl=np.mean([f['psm_local']['p'] for f in fr], 0) * 1000, Rl=Rot.from_quat(fr[0]['psm_local']['q_xyzw']).as_matrix())
X_of = lambda i, d: np.array([(P[i]['uv'][0] - CX) * FB / d / FX, (P[i]['uv'][1] - CY) * FB / d / FX, FB / d])

def fit(ids, dsel, sim):
    Xc = np.array([X_of(i, dsel[i]) for i in ids]); A = np.array([P[i]['tl'] for i in ids])
    ma, mb = A.mean(0), Xc.mean(0); U, _, Vt = np.linalg.svd((Xc - mb).T @ (A - ma)); Dg = np.eye(3); Dg[2, 2] = np.sign(np.linalg.det(U @ Vt)); R0 = U @ Dg @ Vt
    def res(x):
        R = Rot.from_rotvec(x[:3]).as_matrix(); s = math.exp(x[9]) if sim else 1.0
        return np.concatenate([s * R @ (P[i]['Rl'] @ x[6:9] + P[i]['tl']) + x[3:6] - Xc[k] for k, i in enumerate(ids)])
    r = least_squares(res, np.r_[Rot.from_matrix(R0).as_rotvec(), mb - R0 @ ma, 0, 0, 20, 0][:10 if sim else 9], method='lm')
    e = r.fun.reshape(-1, 3); cov = np.linalg.pinv(r.jac.T @ r.jac) * (e ** 2).sum() / max(e.size - len(r.x), 1)
    return r.x, np.linalg.norm(e, axis=1), np.sqrt(np.diag(cov))

def pred(x, i, sim):
    s = math.exp(x[9]) if sim else 1.0; return s * Rot.from_rotvec(x[:3]).as_matrix() @ (P[i]['Rl'] @ x[6:9] + P[i]['tl']) + x[3:6]

rms = lambda e: float(np.sqrt(np.mean(np.square(e))))
train = sorted(i for i in P if i not in TEST); test = sorted(i for i in P if i in TEST)
clear_tr = [i for i in train if P[i]['margin'] >= 0.10]; dsel = {i: P[i]['peaks'][0][0] for i in P}; choice = {}
x1, _, _ = fit(clear_tr, dsel, False)
for i in [i for i in train if P[i]['margin'] < 0.10]:
    dp = FB / pred(x1, i, False)[2]; best = min(P[i]['peaks'], key=lambda pk: abs(pk[0] - dp)); choice[i] = {'pred_disp': dp, 'peaks': P[i]['peaks'], 'chosen': best}; dsel[i] = best[0]
out = {'n_planned': len(man['plan']), 'test_planned': sorted(TEST), 'usable': sorted(P), 'unusable': clk['unusable'], 'train': train, 'test': test,
       'clear_train': clear_tr, 'ambiguous_train_resolved': {str(k): v for k, v in choice.items()}, 'models': {}}
for name, sim in (('SE3', False), ('Sim3', True)):
    x, e, sd = fit(train, dsel, sim); dt = dict(dsel); tchoice = {}
    for i in test:
        if P[i]['margin'] < 0.10:
            dp = FB / pred(x, i, sim)[2]; best = min(P[i]['peaks'], key=lambda pk: abs(pk[0] - dp)); dt[i] = best[0]; tchoice[str(i)] = {'pred_disp': dp, 'chosen': best}
    et = np.array([np.linalg.norm(pred(x, i, sim) - X_of(i, dt[i])) for i in test])
    T = np.eye(4); T[:3, :3] = (math.exp(x[9]) if sim else 1.0) * Rot.from_rotvec(x[:3]).as_matrix(); T[:3, 3] = x[3:6]
    out['models'][name] = {'train_rms_mm': rms(e), 'train_max_mm': float(e.max()), 'test_rms_mm': rms(et), 'test_max_mm': float(et.max()),
                           'test_err_mm': dict(zip(map(str, test), et.round(3).tolist())), 'train_err_mm': dict(zip(map(str, train), e.round(3).tolist())),
                           'tcp_mm': x[6:9].tolist(), 'tcp_1sigma_mm': sd[6:9].tolist(), 'scale': math.exp(x[9]) if sim else 1.0,
                           'T_camrect_PSM1local_mm (p_cam = T * [p_local;1])': T.tolist(), 'ambiguous_test_resolved': tchoice,
                           'test_err_vec_mm': {str(i): (pred(x, i, sim) - X_of(i, dt[i])).round(3).tolist() for i in test}}
# secondary: 20 random 25% splits over usable poses (disparities as chosen above for train; test ambiguity resolved per split model)
rng = np.random.default_rng(20260930); reps = []
allu = sorted(P); dall = dict(dsel)
for i in test: dall[i] = dsel[i] if P[i]['margin'] >= 0.10 else min(P[i]['peaks'], key=lambda pk: abs(pk[0] - FB / pred(fit(train, dsel, False)[0], i, False)[2]))[0]
for r in range(20):
    perm = rng.permutation(allu); te = sorted(perm[:math.ceil(0.25 * len(allu))].tolist()); tr = sorted(set(allu) - set(te)); row = {}
    for name, sim in (('SE3', False), ('Sim3', True)):
        x, _, _ = fit(tr, dall, sim); row[name] = rms([np.linalg.norm(pred(x, i, sim) - X_of(i, dall[i])) for i in te])
    reps.append(row)
out['secondary_20_random_splits'] = {k: np.percentile([r[k] for r in reps], [50, 25, 75]).round(3).tolist() for k in ('SE3', 'Sim3')}
out['disparity'] = {str(i): {'uv': P[i]['uv'], 'chosen_disp': dall[i], 'Z_mm': FB / dall[i], 'margin': P[i]['margin'], 'peaks': P[i]['peaks']} for i in allu}
out['R2_tcp_mm'] = [-2.70, 1.08, 23.81]
out['H_1b (SE3 test RMS < 2.80 mm)'] = out['models']['SE3']['test_rms_mm'] < 2.80
json.dump(out, open(B + '/registration.json', 'w'), indent=1)
for k, m in out['models'].items():
    print(k, 'train rms %.2f max %.2f | test rms %.2f max %.2f | tcp %s sd %s | s %.4f' % (m['train_rms_mm'], m['train_max_mm'], m['test_rms_mm'], m['test_max_mm'], np.round(m['tcp_mm'], 2), np.round(m['tcp_1sigma_mm'], 2), m['scale']))
print('usable', len(allu), 'train', len(train), 'test', len(test), 'clear train', len(clear_tr), 'amb train', list(choice), 'secondary', out['secondary_20_random_splits'], 'H_1b', out['H_1b (SE3 test RMS < 2.80 mm)'])
