#!/usr/bin/env python3
"""EXP4 re-assessment (pre_registration.json). Run (WSL): ~/.venvs/foundationstereo-5080/bin/python ~/r6off/exp4/exp4.py"""
import os, json, csv, numpy as np
from scipy.spatial.transform import Rotation as Rot
ROOT = os.path.expanduser('~/r6off'); IN = ROOT + '/input'; OUT = ROOT + '/exp4'
summ = {r['trial']: r for r in csv.DictReader(open(IN + '/trials_summary.csv'))}
unit = lambda v: np.asarray(v, float) / np.linalg.norm(v)
fold = lambda a: min(a, 180 - a)
ang = lambda a, b: float(np.degrees(np.arccos(np.clip(np.dot(unit(a), unit(b)), -1, 1))))
rows = []
for t in ['Q1_A', 'Q1_C', 'T1_C', 'T2_C', 'T3_C', 'T3b_C', 'T4_C']:
    pc = json.load(open(f'{IN}/trials/{t}/preclose_pose.json')); ts = json.load(open(f'{IN}/trials/{t}/trial_state.json'))
    tip = np.array(pc['tip']['cam_mm']); R = np.array(pc['tip']['R_cam']); nc = pc['needle_cam']
    p = np.array(nc['grasp_point_mm']); tg = unit(nc['tangent']); n = unit(nc['normal_toward_cam'])
    x, y, z = R[:, 0], R[:, 1], R[:, 2]
    # SurgicAI goal orientation (columns = gripper x,y,z in cam): gx=-y_g, gy=-x_g, gz=-z_g ; z_g = plane normal toward cam, y_g = +-tangent, x_g = y_g x z_g
    best = None
    for s in (+1, -1):
        yg = s * tg; zg = n; xg = np.cross(yg, zg)
        Rg = np.c_[-yg, -xg, -zg]
        g = float(np.degrees(Rot.from_matrix(R.T @ Rg).magnitude()))
        if best is None or g < best[0]: best = (g, s)
    hov_ang = ts['hover'].get('angles', {})
    d_tip = float(np.linalg.norm(tip - p))
    d_hover_pt = float(np.linalg.norm(tip - (p - 7.0 * z)))     # p backed off 7 mm along the tool -z (toward the camera side)
    rows.append({'trial': t, 'condition': summ[t]['condition'], 'counted': summ[t]['counted'], 'outcome': summ[t]['outcome'],
                 'tip_to_grasp_mm': d_tip, 'tip_to_hoverpoint7_mm': d_hover_pt, 'geodesic_to_surgicai_goal_deg': best[0], 'tangent_sign_used': best[1],
                 'x_tangent_deg': fold(ang(x, tg)), 'y_tangent_deg': fold(ang(y, tg)), 'angle_z_normal_deg': ang(z, -n), 'beta_deg': 90 - ang(z, -n),
                 'psi3_deg_day_hover': hov_ang.get('psi3_deg'), 'touch_extra_mm': float(summ[t]['touch_extra_mm']) if summ[t]['touch_extra_mm'] else None,
                 'pass_1mm': d_tip <= 1.0, 'pass_5mm': d_tip <= 5.0, 'pass_10mm': d_tip <= 10.0, 'pass_geo10': best[0] <= 10.0})
    r = rows[-1]; print('%-7s %-4s | tip->p* %5.1f mm | hover pt %5.1f | geo-to-SurgicAI %5.1f deg | x-tan %4.1f y-tan %4.1f z-n %4.1f beta %4.1f psi3 %s | %s' % (t, r['condition'], d_tip, d_hover_pt, best[0], r['x_tangent_deg'], r['y_tangent_deg'], r['angle_z_normal_deg'], r['beta_deg'], r['psi3_deg_day_hover'] and round(r['psi3_deg_day_hover'], 1), r['outcome'][:38]))
n = len(rows)
cnt = {k: sum(r[k] for r in rows) for k in ('pass_1mm', 'pass_5mm', 'pass_10mm', 'pass_geo10')}
print('counts of %d:' % n, cnt)
json.dump({'rows': rows, 'counts': cnt, 'n': n}, open(OUT + '/exp4_results.json', 'w'), indent=1, default=lambda o: o.item() if hasattr(o, 'item') else str(o))
