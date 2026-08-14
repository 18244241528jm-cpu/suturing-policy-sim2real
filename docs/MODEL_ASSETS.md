# External model and data assets

Large or third-party assets are deliberately excluded from Git.

## Required for the validated DA/FP path

| Asset | Expected value |
|---|---|
| New DA checkpoint | `best.pth` |
| DA SHA256 | `fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88` |
| DA architecture | Depth Anything V2 ViT-L, 518x518, FP32 |
| FoundationPose commit | `a1b694b83e633c2cb6115b9063d940a687759392` |
| Original FP image digest | `sha256:2445efc3a681a71233299444847471d3d861307b8e061a847356ab34a26af096` |

The DA checkpoint is about 3.8 GB and is not stored in Git. Obtain it from the
project owner or lab storage, then verify the full SHA256 before use.

## Released RL assets

| File | SHA256 |
|---|---|
| `models/rl/m3_measured_r3_100k.zip` | `0407987e296d78b8b63ccf49c16e35395b00cf8d4ebc4cfe857b57f3381f2a2f` |
| `models/rl/r6_unified_single_goal_yaw15_seed1_final.zip` | `6286a88c21f04abfbc4b0747a87a67bc2c5dcba17f710692c6b5138f7776e525` |

The default demo path uses the staged D2 controller. RL is an experimental
alternative. Do not silently substitute another checkpoint; record its SHA,
training contract, number of transitions and evaluation mode.

The historical SIM-S4 evaluator still initializes the policy stack even when
the D2 action is selected, so exact SIM-S4 reproduction requires the M3-100k
checkpoint path in `SIM_S4_MODEL`. The controller algorithm itself does not use
the network output.

## Released frozen SIM-S3 bank

`data/reference/sim_s3_20260811/` contains the reset snapshots, DA metadata,
gated FP result and deployment-proxy candidate arrays needed to rebuild the
SIM-S4 goal banks. It enables controller replay without rerunning DA/FP. It
does not contain RGB/depth images and therefore cannot be used to claim that
the perception computation was reproduced.

## Simulator assets

Needle and PSM CAD meshes come from the Surgical Robotics Challenge scene.
AMBF binaries and world files are built/obtained from their upstream projects.
This repository does not relicense those assets.

## Data needed for exact statistical reproduction

The small reports and frozen bank preserve the main Reach input contract. Exact
Wilson intervals, visual re-analysis and DA image-level metrics still require
the original per-episode JSONL/CSV and RGB/depth frame bundles. Request the
corresponding dated result bundle and verify its `MANIFEST.sha256`.
