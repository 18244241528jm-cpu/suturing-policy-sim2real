#!/usr/bin/env python3
"""EXP2 offline analysis of dumped FP candidates against the human reference (pre_registration.json).
usage: analyze_fp.py <exp2 work dir with inputs/, meshes/, M0/ M2/ candidate dirs> [--selftest]"""
import os, sys, json, glob, math, numpy as np
ROOT = os.path.expanduser('~/r6off'); sys.path.insert(0, ROOT + '/input/code_snapshot'); import r5_grasp_v3 as G
unit = lambda v: np.asarray(v, float) / np.linalg.norm(v)


def wilson(k, n, z=1.96):
    if n == 0: return (float('nan'), float('nan'))
    p = k / n; d = 1 + z * z / n; c = p + z * z / (2 * n); h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


def mesh_models(W):
    par = json.load(open(W + '/meshes/mesh_params.json')); m0 = par['m0_arc_fit']
    u, v, c = np.array(par['basis_u']), np.array(par['basis_v']), np.array(par['centre_mm'])
    arc0 = {'centre_mm': m0['centre_mm'], 'normal_toward_cam': m0['normal_toward_cam'], 'R_mm': m0['R_mm'], 'arc_deg': m0['arc_deg'], '_u': m0['_u'], '_v': m0['_v'],
            '_a_start_deg': m0['_a_start_deg'], '_a_end_deg': m0['_a_end_deg']}
    arc2 = {'centre_mm': c.tolist(), 'normal_toward_cam': np.cross(u, v).tolist(), 'R_mm': par['R_mm'], 'arc_deg': par['arc_deg'], '_u': u.tolist(), '_v': v.tolist(),
            '_a_start_deg': par['angle_start_deg'], '_a_end_deg': par['angle_end_deg']}
    def ends(arc):
        cc, uu, vv = np.array(arc['centre_mm']), np.array(arc['_u']), np.array(arc['_v'])
        f = lambda a: cc + arc['R_mm'] * (np.cos(np.radians(a)) * uu + np.sin(np.radians(a)) * vv)
        return f(arc['_a_start_deg']), f(arc['_a_end_deg'])            # tail (arc start), tip (arc end)
    out = {}
    for name, arc in (('M0', arc0), ('M2', arc2)):
        tail, tip = ends(arc); out[name] = {'arc': arc, 'tail_mm': tail, 'tip_mm': tip}
    return out


def eval_frame(poses, scores, ref, mm):
    """poses (N,4,4) camera<-mesh, metres; ref = ref.json; mm = mesh_models()[name]. All distances in mm."""
    arc = mm['arc']; s = ref['s']; R = poses[:, :3, :3]; t = poses[:, :3, 3] * 1000.0
    p_m, tg_m, n_m = G.arc_point_tangent(arc, s)                                 # mesh frame, s from the tail (arc start)
    p_fp = np.einsum('nij,j->ni', R, p_m) + t; tg_fp = np.einsum('nij,j->ni', R, tg_m); n_fp = np.einsum('nij,j->ni', R, n_m)
    c_fp = np.einsum('nij,j->ni', R, np.array(arc['centre_mm'])) + t
    tail_fp = np.einsum('nij,j->ni', R, mm['tail_mm']) + t; tip_fp = np.einsum('nij,j->ni', R, mm['tip_mm']) + t
    chord_h = np.array(ref['chord_tail_to_tip_mm']); p_h = np.array(ref['p_star_mm']); n_h = unit(ref['normal_toward_cam']); c_h = np.array(ref['fit']['centre_mm'])
    t_h = unit(ref['tangent']); t_h = t_h if t_h @ chord_h > 0 else -t_h
    chord_fp = tip_fp - tail_fp
    cosf = lambda a, b: np.clip(np.einsum('ni,ni->n', a, b) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)), -1, 1)
    grasp = np.linalg.norm(p_fp - p_h, axis=1); centre = np.linalg.norm(c_fp - c_h, axis=1)
    tang = np.degrees(np.arccos(np.abs(np.clip(tg_fp @ t_h, -1, 1)))); norm = np.degrees(np.arccos(np.abs(np.clip(n_fp @ n_h, -1, 1))))
    flip = np.degrees(np.arccos(cosf(chord_fp, np.tile(chord_h, (len(poses), 1))))) > 90.0
    like = (grasp <= 2.0) & (tang <= 15.0) & (~flip)
    top = int(np.argmax(scores)); out = {'top1': {'index': top, 'score': float(scores[top]), 'grasp_err_mm': float(grasp[top]), 'tangent_deg': float(tang[top]), 'normal_deg': float(norm[top]), 'centre_err_mm': float(centre[top]),
                                                  'flip': bool(flip[top]), 'pass': bool(like[top])}, 'n_candidates': int(len(scores)), 'oracle_pass': bool(like.any()), 'n_reference_like': int(like.sum()), 'n_flipped': int(flip.sum())}
    sl = float(scores[like].max()) if like.any() else None; sf = float(scores[flip].max()) if flip.any() else None
    out['score_best_reference_like'] = sl; out['score_best_flipped'] = sf
    out['score_gap'] = (sl - sf) if (sl is not None and sf is not None) else None
    out['scores_exactly_equal'] = (sl is not None and sf is not None and sl == sf)
    out['best_candidate_grasp_err_mm'] = float(grasp.min())
    return out


def analyze(W, name, depth, mm, refs):
    d = f'{W}/{name}/candidates'; recs = {}
    for sid in refs:
        f = f'{d}/{sid}__{depth}.npz'
        if not os.path.exists(f): continue
        z = np.load(f); recs[sid] = eval_frame(z['poses_camera'], z['scores'], refs[sid], mm)
    return recs


def summarize(recs, per_frame_meta=None):
    n = len(recs); k = sum(r['top1']['pass'] for r in recs.values()); ko = sum(r['oracle_pass'] for r in recs.values())
    kf = sum(r['top1']['flip'] for r in recs.values())
    gaps = [r['score_gap'] for r in recs.values() if r['score_gap'] is not None]
    pl = {}
    for sid, r in recs.items(): pl.setdefault(sid.rsplit('_snap', 1)[0], []).append(r['top1']['pass'])
    both = sum(all(v) and len(v) == 2 for v in pl.values())
    return {'n_frames': n, 'top1_pass': k, 'top1_pass_ci95': wilson(k, n), 'oracle_pass': ko, 'top1_flipped': kf, 'placements_both_frames_pass': both, 'n_placements': len(pl),
            'median_grasp_err_mm': float(np.median([r['top1']['grasp_err_mm'] for r in recs.values()])) if n else None,
            'median_tangent_deg': float(np.median([r['top1']['tangent_deg'] for r in recs.values()])) if n else None,
            'median_centre_err_mm': float(np.median([r['top1']['centre_err_mm'] for r in recs.values()])) if n else None,
            'score_gap_defined': len(gaps), 'score_gap_median': float(np.median(gaps)) if gaps else None,
            'scores_exactly_equal_frames': [sid for sid, r in recs.items() if r['scores_exactly_equal']],
            'gap_undefined_frames': [sid for sid, r in recs.items() if r['score_gap'] is None]}


def load_refs(W):
    return {os.path.basename(p): json.load(open(p + '/ref.json')) for p in sorted(glob.glob(W + '/inputs/*'))}


def main(W):
    mms = mesh_models(W); refs = load_refs(W); allres = {}
    for name in ('M0', 'M2'):
        for depth in ('raw', 'fix'):
            recs = analyze(W, name, depth, mms[name], refs)
            if not recs: continue
            allres[f'{name}+D-{depth}'] = {'summary': summarize(recs), 'frames': recs}
            s = allres[f'{name}+D-{depth}']['summary']
            print(f'{name}+D-{depth}: n={s["n_frames"]} top1 pass {s["top1_pass"]}/{s["n_frames"]} (CI {s["top1_pass_ci95"][0]:.2f}-{s["top1_pass_ci95"][1]:.2f}) oracle {s["oracle_pass"]} flipped top1 {s["top1_flipped"]} '
                  f'median grasp err {s["median_grasp_err_mm"]:.2f} mm tangent {s["median_tangent_deg"]:.1f} deg placements both-pass {s["placements_both_frames_pass"]}/{s["n_placements"]}')
    json.dump(allres, open(W + '/exp2_results.json', 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
    return allres


def selftest(W):
    """synthetic candidates built from the human reference: true pose, a swapped-ends (flipped) pose, a random pose; checks the metric code."""
    import tempfile, shutil
    from scipy.spatial.transform import Rotation as Rot
    mms = mesh_models(W); refs = load_refs(W); sid = 'Q1_C_snap0'; ref = refs[sid]; mm = mms['M2']; arc = mm['arc']
    c_m = np.array(arc['centre_mm']); u, v = np.array(arc['_u']), np.array(arc['_v']); a_s, a_e = arc['_a_start_deg'], arc['_a_end_deg']
    rad = lambda a: np.cos(np.radians(a)) * u + np.sin(np.radians(a)) * v
    # mesh frame triad from tail/tip radials ; human triad from the fit end points
    ft = np.array(ref['tail_end_mm']); fp_ = np.array(ref['tip_end_mm']); c_h = np.array(ref['fit']['centre_mm'])
    a_m, b_m = unit(rad(a_s)), unit(rad(a_e)); a_h, b_h = unit(ft - c_h), unit(fp_ - c_h)
    def triad(a, b): n = unit(np.cross(a, b)); return np.c_[a, np.cross(n, a), n]
    Rtrue = triad(a_h, b_h) @ triad(a_m, b_m).T; ttrue = c_h - Rtrue @ c_m
    # centre of the mesh arc does not coincide with human centre in radius/arc terms (arcs differ) -> measure how close the construction is
    def T(R_, t_): X = np.eye(4); X[:3, :3] = R_; X[:3, 3] = t_ / 1000.0; return X
    mid_axis = unit(rad(0.5 * (a_s + a_e)))                                         # symmetry axis (centre -> apex) in the mesh arc plane
    Rflip_m = Rot.from_rotvec(np.pi * mid_axis).as_matrix()                         # 180 deg about the apex axis: swaps tail and tip
    Rf = Rtrue @ Rflip_m; tf = c_h - Rf @ c_m
    rng = np.random.default_rng(0); Rr = Rot.random(random_state=1).as_matrix()
    poses = np.stack([T(Rtrue, ttrue), T(Rf, tf), T(Rr, c_h + rng.normal(0, 3, 3))]); scores = np.array([0.5, 0.9, 0.1])
    r = eval_frame(poses, scores, ref, mm)
    print('selftest (top-1 is the FLIP with the highest score):', json.dumps({k: r[k] for k in ('top1', 'oracle_pass', 'n_reference_like', 'n_flipped', 'score_gap', 'best_candidate_grasp_err_mm')}, default=float))
    r2 = eval_frame(poses, np.array([0.9, 0.5, 0.1]), ref, mm); print('selftest (top-1 is the TRUE pose): pass', r2['top1']['pass'], 'grasp err %.2f mm tangent %.2f deg flip %s gap %.2f' % (r2['top1']['grasp_err_mm'], r2['top1']['tangent_deg'], r2['top1']['flip'], r2['score_gap']))
    r3 = eval_frame(poses, np.array([0.7, 0.7, 0.1]), ref, mm); print('selftest equal scores: equal flag', r3['scores_exactly_equal'], 'gap', r3['score_gap'])
    assert r['top1']['flip'] and not r['top1']['pass'] and r['oracle_pass'] and r['n_flipped'] >= 1
    assert r2['top1']['pass'] is True or r2['top1']['grasp_err_mm'] < 5
    assert r3['scores_exactly_equal']
    print('SELFTEST OK')


if __name__ == '__main__':
    W = os.path.expanduser(sys.argv[1]) if len(sys.argv) > 1 else os.path.expanduser('~/r6off_exp2')
    selftest(W) if '--selftest' in sys.argv else main(W)
