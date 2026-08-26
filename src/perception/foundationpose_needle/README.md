# Running FoundationPose on the needle

Two things live here: a headless runner, and one known-good scene to prove an install
works before you trust it on new data.

Upstream FoundationPose is **not** vendored. It is pinned by commit in
[`docs/MODEL_ASSETS.md`](../../../docs/MODEL_ASSETS.md)
(`a1b694b83e633c2cb6115b9063d940a687759392`) and cloned per the top-level README. A
vendored copy would drift from that pin, and the NVIDIA Source Code License governs
redistribution of their code.

---

## Why `run_demo_headless.py` exists

Upstream `run_demo.py` calls `cv2.imshow()` to display each frame. Inside the
FoundationPose Docker image, over WSLg, or on any box without an attached display, that
throws and the run dies before producing anything. This runner performs the same
`register` on frame 0 and `track_one` afterwards, then writes the overlay to
`<out_dir>/track_vis/*.png` and the pose to `<out_dir>/ob_in_cam/*.txt` instead of
showing a window.

It is otherwise a thin wrapper over the upstream API and follows the structure of
upstream `run_demo.py`; the estimator, its iteration counts, and the reader are all
upstream. It adds `--max_frames` so a smoke test does not walk an entire sequence.

Copy it next to the upstream code, since it imports `estimater` and `datareader` from
there:

```bash
cp run_demo_headless.py "$HOME/surgicai_external/FoundationPose/"
```

---

## The scene here is simulation, not the real cart

`needle_data/` was captured from the AMBF `surgical_robotics_challenge` scene, not from
the dVRK. Its intrinsics say so:

```
cam_K.txt   fx = fy = 350.807   cx = 320   cy = 240
```

The real endoscope is around `fx = 1634.9`. Use this scene to answer *"is my
FoundationPose install working at all"*, and nothing else. It has a measured ground
truth, so the answer is checkable rather than a vibe.

```bash
cd "$HOME/surgicai_external/FoundationPose"
python run_demo_headless.py \
  --mesh_file        <repo>/src/perception/foundationpose_needle/needle_data/mesh/needle_clean.ply \
  --test_scene_dir   <repo>/src/perception/foundationpose_needle/needle_data \
  --est_refine_iter 5 \
  --max_frames 1
```

Compare `debug/ob_in_cam/000000.txt` against `needle_data/gt/T_RN_cv.txt`, which is the
needle pose in the OpenCV camera frame. A healthy install lands within a few millimetres:
the recorded result for this frame is **1.04 mm** translation error (per-axis
0.26 / 0.71 / 0.71 mm) and **5.81°** rotation. Tens of millimetres, or an orientation
roughly 180° off, means the install or the mesh is wrong -- not that the needle is hard.

Ground truth files, all 4x4:

| File | Meaning |
|---|---|
| `gt/T_RN_cv.txt` | needle in the OpenCV camera frame -- **compare against this** |
| `gt/T_WN.txt` | needle in the AMBF world frame |
| `gt/T_WR.txt` | camera in the AMBF world frame |

---

## Building a scene from real dVRK captures

`run_demo_headless.py` is data-agnostic: it reads whatever `--test_scene_dir` you hand
it, through upstream's `YcbineoatReader`. That reader wants exactly this layout:

```
<scene_dir>/
├── cam_K.txt          3x3 intrinsics, whitespace separated
├── rgb/000000.png     8-bit colour
├── depth/000000.png   uint16 millimetres  (the reader divides by 1e3)
└── masks/000000.png   binary, non-zero on the object; only frame 0 is used
```

Three traps, all of which have bitten this project:

1. **Depth units.** `depth/*.png` must be uint16 **millimetres**. Handing it metres
   makes every pose land a thousand times too close.
2. **Intrinsics must match the images.** `cam_K.txt` from a different resolution or a
   different camera produces a confidently wrong pose, not an error.
3. **The mask is frame 0 only**, and FoundationPose trusts it completely. There is no
   needle detector on the real cart yet; `suturing_runtime`'s `operator_mask_publisher`
   exists because that mask is drawn by hand.

The real captures under `~/FoundationPose/occl_real*` do **not** use this layout -- they
are `run_XX/frame_YYY/` directories consumed by the bespoke `occlusion_eval_real.py`, so
this runner cannot read them without conversion.

---

## Meshes

| File | Use |
|---|---|
| `mesh/needle_clean.ply` | **the canonical one** |
| `mesh/Needle.obj` | the AMBF source geometry |
| `mesh/needle_clean_tagged.ply` | falsified experiment, see below |
| `mesh/needle_clean_grad.ply` | falsified experiment, see below |

`needle_clean.ply` is AMBF's `Needle.OBJ` re-exported, geometry unchanged (the manifest
in `depth_audit_stage_a/results/stage_d/mesh_variants/variant_manifest.json` records the
source sha256, and the face count matches at 1690). The re-export exists because
`Needle.mtl` declares a material with no `map_Kd`, so trimesh yields a `SimpleMaterial`
with no `.image` and FoundationPose's `make_mesh_tensors` takes its texture branch and
crashes. Forcing `ColorVisuals(vertex_colors=grey)` and writing a PLY routes it down the
vertex-colour branch instead. That is all "clean" means; the vertex count rising from 847
to 4120 is per-face splitting for the colour, not added detail.

**The tagged and gradient meshes were falsified.** Off-axis colour tags were meant to
break the needle's rotational symmetry and cut the monocular flip rate. Measured, they
made it worse -- 35% to 46-54% -- so the tag route cannot retire stereo. They are kept so
the negative result stays reproducible. Do not reach for them as an improvement.
