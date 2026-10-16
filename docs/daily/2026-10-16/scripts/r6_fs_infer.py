"""R6 copy of r4_fs_infer.py: only change = torch.load(weights_only=False) for torch>=2.6 (official FS checkpoint).

Loads the official 23-51-11 ViT-L model once and runs every left/right pair in
<in_dir>/{left,right}/*.png (same stem). Writes <out_dir>/disp/<stem>.npy
(float32 disparity in ORIGINAL-resolution pixels) and <out_dir>/meta.json with
per-frame timing and peak VRAM.

Resolution policy (logged): images are resized by --scale s before the network;
the returned disparity is resized back to full res and multiplied by 1/s, so the
caller always uses full-res K and Z = f*B/d.
"""
import argparse, json, os, sys, time, types
sys.modules.setdefault("open3d", types.ModuleType("open3d"))  # point clouds unused
code_dir = os.path.dirname(os.path.realpath(__file__)); sys.path.insert(0, code_dir)
import cv2, imageio, numpy as np, torch
from omegaconf import OmegaConf
from core.utils.utils import InputPadder
from core.foundation_stereo import FoundationStereo

ap = argparse.ArgumentParser()
ap.add_argument("--in_dir", required=True)
ap.add_argument("--out_dir", required=True)
ap.add_argument("--ckpt", default=f"{code_dir}/pretrained_models/23-51-11/model_best_bp2.pth")
ap.add_argument("--scale", type=float, default=1.0)
ap.add_argument("--valid_iters", type=int, default=32)
ap.add_argument("--hiera", type=int, default=0)
a = ap.parse_args()
torch.autograd.set_grad_enabled(False); torch.manual_seed(0); np.random.seed(0)
cfg = OmegaConf.load(os.path.join(os.path.dirname(a.ckpt), "cfg.yaml"))
cfg.setdefault("vit_size", "vitl"); cfg.valid_iters = a.valid_iters
model = FoundationStereo(cfg); ck = torch.load(a.ckpt, map_location="cpu", weights_only=False)
model.load_state_dict(ck["model"]); model.cuda().eval()
os.makedirs(f"{a.out_dir}/disp", exist_ok=True)
stems = sorted(os.path.splitext(f)[0] for f in os.listdir(f"{a.in_dir}/left") if f.endswith(".png"))
rows = []
for st in stems:
    L = imageio.imread(f"{a.in_dir}/left/{st}.png")[..., :3]; R = imageio.imread(f"{a.in_dir}/right/{st}.png")[..., :3]
    H, W = L.shape[:2]
    if a.scale != 1.0:
        L = cv2.resize(L, None, fx=a.scale, fy=a.scale); R = cv2.resize(R, None, fx=a.scale, fy=a.scale)
    h, w = L.shape[:2]
    t0 = torch.as_tensor(L).cuda().float()[None].permute(0, 3, 1, 2)
    t1 = torch.as_tensor(R).cuda().float()[None].permute(0, 3, 1, 2)
    pad = InputPadder(t0.shape, divis_by=32, force_square=False); t0, t1 = pad.pad(t0, t1)
    torch.cuda.synchronize(); torch.cuda.reset_peak_memory_stats(); tt = time.time()
    with torch.cuda.amp.autocast(True):
        d = model.run_hierachical(t0, t1, iters=a.valid_iters, test_mode=True, small_ratio=0.5) if a.hiera \
            else model.forward(t0, t1, iters=a.valid_iters, test_mode=True)
    torch.cuda.synchronize(); dt = time.time() - tt
    d = pad.unpad(d.float()).cpu().numpy().reshape(h, w)
    if a.scale != 1.0:
        d = cv2.resize(d, (W, H), interpolation=cv2.INTER_LINEAR) / a.scale
    np.save(f"{a.out_dir}/disp/{st}.npy", d.astype(np.float32))
    rows.append({"stem": st, "net_hw": [h, w], "infer_s": round(dt, 3), "peak_vram_mib": round(torch.cuda.max_memory_allocated() / 2**20),
                 "disp_p5_p50_p95": np.percentile(d, [5, 50, 95]).round(2).tolist()})
    print(json.dumps(rows[-1]), flush=True)
json.dump({"ckpt": a.ckpt, "global_step": int(ck.get("global_step", -1)), "scale": a.scale, "valid_iters": a.valid_iters, "hiera": a.hiera,
           "disp_convention": "full-res pixels, left=physical LEFT camera; Z = f*B/d with full-res f", "frames": rows},
          open(f"{a.out_dir}/meta.json", "w"), indent=1)
