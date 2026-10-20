#!/usr/bin/env python3
"""EXP2 GPU driver (inside the foundationpose:blackwell container): FoundationPose register on the real R6 frames, dump all candidates.
Modelled on depth_audit_stage_a/dump_exp0c_candidates.py (same estimator construction, iteration 5, seed 2718 once). No ground truth here:
the comparison with the human reference is done offline by analyze_fp.py.
Writes only under --out (WSL home). Load-log lines go to /crashfix/load_log.txt (= C:\\crashfix\\load_log.txt)."""
import argparse, json, os, sys, time, random, datetime
import cv2, numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--bank', required=True); ap.add_argument('--mesh', required=True); ap.add_argument('--mesh-name', required=True)
ap.add_argument('--depths', nargs='+', default=['raw', 'fix']); ap.add_argument('--out', required=True)
ap.add_argument('--iteration', type=int, default=5); ap.add_argument('--seed', type=int, default=2718); ap.add_argument('--min-mask-pixels', type=int, default=20)
ap.add_argument('--mem-fraction', type=float, default=0.6); ap.add_argument('--only', nargs='*', default=None, help='sample ids to run (default all)')
ap.add_argument('--crop', nargs=2, type=int, default=None, help='crop W H around the mask centre (full-res pixels kept, K principal point shifted)')
ap.add_argument('--utc-offset-hours', type=float, default=0.0)
ap.add_argument('--loadlog', default='/crashfix/load_log.txt'); ap.add_argument('--stage', default='EXP2')
a = ap.parse_args()
sys.path.insert(0, '/project/depth_audit_stage_a'); sys.path.insert(0, '/workspace')


def now():
    return (datetime.datetime.utcnow() + datetime.timedelta(hours=a.utc_offset_hours)).strftime('%Y-%m-%d %H:%M:%S')   # container has no tzdata


def ll(msg):
    with open(a.loadlog, 'a') as f:
        f.write(f'{now()} {a.stage} {msg}\n'); f.flush(); os.fsync(f.fileno())


import torch
torch.cuda.set_per_process_memory_fraction(a.mem_fraction)
import nvdiffrast.torch as dr, trimesh
from estimater import PoseRefinePredictor, ScorePredictor
from occlusion_eval import make_estimator
from run_fp_p7_psm_tracker import load_mesh
random.seed(a.seed); np.random.seed(a.seed); torch.manual_seed(a.seed); torch.cuda.manual_seed_all(a.seed)
os.makedirs(a.out, exist_ok=True)
mesh = load_mesh(a.mesh, trimesh)
est = make_estimator(mesh, ScorePredictor(), PoseRefinePredictor(), dr.RasterizeCudaContext(), os.path.join(a.out, 'fp_debug'), 0)
to_original = est.get_tf_to_centered_mesh().detach().cpu().numpy()
rows = [json.loads(l) for l in open(a.bank) if l.strip()]
if a.only: rows = [r for r in rows if r['sample_id'] in a.only]
jobs = [(r, d) for r in rows for d in a.depths]
ll(f'START mesh={a.mesh_name} frames={len(rows)} depths={a.depths} jobs={len(jobs)} mem_fraction={a.mem_fraction}')
meta = os.path.join(a.out, 'per_frame.jsonl')
done = set()
if os.path.exists(meta):
    done = {(json.loads(l)['sample_id'], json.loads(l)['depth']) for l in open(meta) if l.strip()}
t_all = time.time()
for i, (r, dkind) in enumerate(jobs, 1):
    sid = r['sample_id']
    if (sid, dkind) in done: continue
    d = r['dir']
    rgb = cv2.cvtColor(cv2.imread(d + '/rgb.png'), cv2.COLOR_BGR2RGB)
    depth = np.load(d + f'/depth_{dkind}.npy').astype(np.float32); depth[(~np.isfinite(depth)) | (depth < 0.001)] = 0.0
    mask = cv2.imread(d + '/mask.png', 0) > 0
    K = np.array(json.load(open(d + '/ref.json'))['K'], float); crop0 = None
    if a.crop:
        cw, ch = a.crop; H, Wd = mask.shape; ys_, xs_ = np.nonzero(mask)
        x0 = int(np.clip(round(xs_.mean() - cw / 2), 0, Wd - cw)); y0 = int(np.clip(round(ys_.mean() - ch / 2), 0, H - ch)); crop0 = [x0, y0, cw, ch]
        rgb = np.ascontiguousarray(rgb[y0:y0 + ch, x0:x0 + cw]); depth = np.ascontiguousarray(depth[y0:y0 + ch, x0:x0 + cw]); mask = np.ascontiguousarray(mask[y0:y0 + ch, x0:x0 + cw])
        K = K.copy(); K[0, 2] -= x0; K[1, 2] -= y0
    rec = {'sample_id': sid, 'depth': dkind, 'mesh': a.mesh_name, 'crop_x0y0wh': crop0, 'mask_pixels': int(mask.sum()), 'mask_pixels_with_depth': int((mask & (depth > 0)).sum()), 'status': 'below_mask_gate'}
    if int(mask.sum()) >= a.min_mask_pixels:
        torch.cuda.reset_peak_memory_stats(); t0 = time.time()
        try:
            est.register(K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=a.iteration)
            torch.cuda.synchronize()
            poses = est.poses.detach().cpu().numpy() @ to_original
            scores = est.scores.detach().cpu().numpy().reshape(-1)
            os.makedirs(os.path.join(a.out, 'candidates'), exist_ok=True)
            np.savez_compressed(os.path.join(a.out, 'candidates', f'{sid}__{dkind}.npz'), poses_camera=poses, scores=scores, K=K)
            top = int(np.argmax(scores))
            rec.update({'status': 'candidate_dumped', 'n_candidates': int(len(scores)), 'top1_index': top, 'top1_score': float(scores[top]),
                        'wall_s': time.time() - t0, 'peak_alloc_gib': torch.cuda.max_memory_allocated() / 2**30, 'peak_reserved_gib': torch.cuda.max_memory_reserved() / 2**30})
        except Exception as e:
            rec.update({'status': 'register_error', 'error': repr(e)[:300], 'wall_s': time.time() - t0})
    with open(meta, 'a') as f:
        f.write(json.dumps(rec) + '\n'); f.flush(); os.fsync(f.fileno())
    print(f'FP {i}/{len(jobs)} {sid} {dkind} {rec["status"]} wall={rec.get("wall_s", 0):.1f}s peak={rec.get("peak_alloc_gib", 0):.2f}GiB', flush=True)
    if i % 10 == 0 or i == len(jobs) or i <= 3: ll(f'PROGRESS mesh={a.mesh_name} {i}/{len(jobs)} last={sid}/{dkind} {rec["status"]}')
ll(f'END mesh={a.mesh_name} wall={time.time()-t_all:.0f}s')
