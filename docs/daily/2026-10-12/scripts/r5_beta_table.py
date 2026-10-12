#!/usr/bin/env python3
"""R5 (offline, for the sim side): angle between the PSM1 jaw axis and the needle-plane normal in the two R4 day-2 grasps.
Inputs (R4, read-only): grid_calib/manifest.json q0 (commanded tool orientation in ECM, used by every grasp move_cp),
grid_calib/calibration_v3.json (camera <- measured_cp[ECM]; rotation = polar of the affine linear part, or the SE3 fit),
needle normals from the stereo arc fits (reach_target.json for g1, grasp2_scene/grasp_target.json for g2).
jaw axis = tool z of q0 (dVRK tool frame: z along the jaws). beta_from_normal = angle(z, n) folded to [0, 90];
'elevation above the needle plane' = 90 - beta_from_normal.
Caveat: R4 grasp logs did not store the measured orientation, so the commanded q0 is used.
usage: python3 r5_beta_table.py --out records/logs/R5_<date>/beta_table"""
import argparse, json, os, sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import r5_grasp_v3 as G
ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
D = os.path.expanduser('~/suturing-policy-sim2real/records/logs/R4_20260928/day2')
cal = json.load(open(f'{D}/grid_calib/calibration_v3.json')); q0 = json.load(open(f'{D}/grid_calib/manifest.json'))['q0']
rots = {'affine_polar': G.rotation_from_affine(cal['models']['affine']['M']), 'SE3': np.array(cal['models']['SE3']['R'])}
Rt = Rot.from_quat(q0).as_matrix(); rows = []
for g, f in (('g1', 'reach_target.json'), ('g2', 'grasp2_scene/grasp_target.json')):
    r = json.load(open(f'{D}/{f}'))['arc']; n = G.unit(r['normal_toward_cam'])
    for rn, Rc in rots.items():
        z = Rc @ Rt[:, 2]; b = float(np.degrees(np.arccos(min(1, abs(z @ n)))))
        view = float(np.degrees(np.arccos(min(1, abs(n @ np.array([0, 0, 1.0]))))))
        rows.append({'grasp': g, 'calib_rotation': rn, 'needle_normal_cam': n.tolist(), 'jaw_z_cam': z.tolist(), 'jaw_vs_needle_normal_deg': b,
                     'jaw_elevation_above_needle_plane_deg': 90 - b, 'needle_normal_vs_camera_axis_deg': view, 'arc_fit_source': f})
out = {'q0_xyzw_ECM': q0, 'rows': rows, 'method': __doc__}
json.dump(out, open(f'{a.out}/beta_table.json', 'w'), indent=1)
with open(f'{a.out}/beta_table.md', 'w') as fh:
    fh.write('| 抓取 | 标定旋转 | 夹爪轴 vs 针平面法向 (°) | 夹爪轴离针平面的仰角 (°) | 针法向 vs 相机光轴 (°) |\n|---|---|---|---|---|\n')
    for r in rows: fh.write(f"| {r['grasp']} | {r['calib_rotation']} | {r['jaw_vs_needle_normal_deg']:.1f} | {r['jaw_elevation_above_needle_plane_deg']:.1f} | {r['needle_normal_vs_camera_axis_deg']:.1f} |\n")
print(open(f'{a.out}/beta_table.md').read())
