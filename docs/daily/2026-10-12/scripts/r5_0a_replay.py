#!/usr/bin/env python3
"""R5 0-a: replay grasp-v3 geometry on the two archived R4 day-2 grasps (read-only inputs).
For each grasp: the R4 grasp point (its arc fraction s), the v3 point s = 0.5, and psi from
 (i) R4's definition: image tangent vs the hard-coded image jaw axis (0.55, 0.83);
 (ii) image tangent vs the jaw axis measured in the frame (Hough on jaw edges);
 (iii) stereo 3D jaw axis projected into the needle plane;
 (iv) kinematic jaw axis (commanded q0; measured orientation was not logged in R4 grasp logs) via the calibration rotation.
usage: python3 r5_0a_replay.py --out records/logs/R5_<date>/0a_replay"""
import argparse, json, os, sys, numpy as np, cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import r5_grasp_v3 as G
ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
D = os.path.expanduser('~/suturing-policy-sim2real/records/logs/R4_20260928/day2')
cal = json.load(open(f'{D}/grid_calib/calibration_v3.json')); Rc = G.rotation_from_affine(cal['models']['affine']['M'])
q0 = json.load(open(f'{D}/grid_calib/manifest.json'))['q0']; jaw_kin = G.jaw_axis_kin(q0, Rc)
R4_AXIS = np.array([0.55, 0.83])
def tracks(path):
    out = {}
    for l in open(path):
        d = json.loads(l)
        if d.get('ev') == 'track': out[d['name']] = d
    return out
cases = {'g1': {'target': f'{D}/reach_target.json', 'dir': f'{D}/grasp', 'frames': ['pre_descend', 'correct_1'], 'r4_reported_psi': 25.0},
         'g2': {'target': f'{D}/grasp2_scene/grasp_target.json', 'dir': f'{D}/grasp_g2', 'frames': ['correct_1', 'dclosed_3', 'open_low', 'at_final'], 'r4_reported_psi': 57.5}}
res = {'jaw_axis_kin_cam': jaw_kin.tolist(), 'calib_rotation_source': 'polar of calibration_v3 affine', 'cases': {}}
for k, c in cases.items():
    tg = json.load(open(c['target'])); r = tg['arc']; p_r4 = np.array(tg['grasp_point_cam_mm']); s_r4 = G.nearest_on_arc(r, p_r4)
    tr = tracks(f"{c['dir']}/grasp_log.jsonl"); out = {'r4_reported_psi_deg': c['r4_reported_psi'], 's_r4': s_r4, 'points': {}}
    for lab, s in (('r4_point', s_r4), ('v3_s0.5', 0.5)):
        p, t, n = G.arc_point_tangent(r, s); psi_k, tilt_k = G.psi_inplane(jaw_kin, t, n)
        e = {'s': s, 'p_cam_mm': p.tolist(), 'dist_to_r4_point_mm': float(np.linalg.norm(p - p_r4)), 'psi_2d_r4axis_deg': G.psi_2d(R4_AXIS, p, t),
             'psi_kin_inplane_deg': psi_k, 'jaw_kin_tilt_from_normal_deg': tilt_k, 'frames': {}}
        for f in c['frames']:
            ref = cv2.imread(f"{c['dir']}/{f}_right.png", 0); oth = cv2.imread(f"{c['dir']}/{f}_left.png", 0)
            if ref is None: e['frames'][f] = 'missing'; continue
            tk = tr.get(f) or tr.get('dclosed_3') or tr.get('correct_1'); uv = tk['uv']; disp = G.FB / tk['X'][2]
            dl = G.dominant_line(ref, uv); st = G.jaw_axis_stereo(ref, oth, uv, disp)
            fr = {'track_uv_used': uv, 'track_from': tk['name']}
            if dl: fr.update(jaw_img_angle_deg=dl[0], n_seg=dl[2], psi_2d_measured_axis_deg=G.psi_2d([np.cos(np.radians(dl[0])), np.sin(np.radians(dl[0]))], p, t))
            if st:
                ps, tl = G.psi_inplane(np.array(st['dir_cam']), t, n)
                fr.update(psi_stereo_inplane_deg=ps, jaw_stereo_tilt_from_normal_deg=tl, stereo_plane_sin=st['plane_sin'], jaw_stereo_vs_kin_deg=float(np.degrees(np.arccos(min(1, abs(np.array(st['dir_cam']) @ jaw_kin))))))
            e['frames'][f] = fr
        out['points'][lab] = e
    res['cases'][k] = out
json.dump(res, open(f'{a.out}/replay.json', 'w'), indent=1)
for k, o in res['cases'].items():
    print(f"== {k}: R4 reported {o['r4_reported_psi_deg']}  s_r4={o['s_r4']:.2f}")
    for lab, e in o['points'].items():
        print(f"  {lab}: 2D(R4 axis) {e['psi_2d_r4axis_deg']:.1f} | kin in-plane {e['psi_kin_inplane_deg']:.1f} (jaw tilt {e['jaw_kin_tilt_from_normal_deg']:.0f})")
        for f, fr in e['frames'].items():
            if isinstance(fr, str): print('    ', f, fr); continue
            print(f"     {f:12s} img axis {fr.get('jaw_img_angle_deg', float('nan')):.1f}  2D(meas) {fr.get('psi_2d_measured_axis_deg', float('nan')):.1f}  stereo in-plane {fr.get('psi_stereo_inplane_deg', float('nan')):.1f} (tilt {fr.get('jaw_stereo_tilt_from_normal_deg', float('nan')):.0f}, sin {fr.get('stereo_plane_sin', float('nan')):.2f}, vs kin {fr.get('jaw_stereo_vs_kin_deg', float('nan')):.0f})")
