# P5a metric Depth Anything model asset

Place the validated checkpoint here as `best.pth`, or keep it in lab storage and
set an absolute `checkpoint_path` in `ros2_ws/src/suturing_runtime/config/jhu_real.yaml`.

Required contract:

```text
filename: best.pth
bytes:    4014768993
sha256:   fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88
model:    Depth Anything V2 ViT-L, forced-square 518x518, FP32
```

The file is intentionally ignored by normal Git. GitHub blocks ordinary Git
objects above 100 MiB; this checkpoint is about 3.8 GiB and also exceeds the
2 GiB per-file Git LFS limit of GitHub Free/Pro. Do not rename another model to
`best.pth`: the ROS node hashes the complete file before loading it.

The Depth-Anything-V2 source repository remains an external dependency. Point
`depth_anything_repo` at a checkout that contains
`depth_anything_v2/dpt.py`.
