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

## Optional RL asset

The default demo path uses the staged D2 controller. RL is an experimental
alternative. The evaluated M3 checkpoints and expert trajectories are not in
this lightweight repository. Do not silently substitute another checkpoint;
record its SHA, training contract, number of transitions and evaluation mode.

The historical SIM-S4 evaluator still initializes the policy stack even when
the D2 action is selected, so exact SIM-S4 reproduction requires the M3-100k
checkpoint path in `SIM_S4_MODEL`. The controller algorithm itself does not use
the network output.

## Simulator assets

Needle and PSM CAD meshes come from the Surgical Robotics Challenge scene.
AMBF binaries and world files are built/obtained from their upstream projects.
This repository does not relicense those assets.

## Data needed for exact statistical reproduction

The small reports in `docs/evidence/` preserve conclusions, but exact Wilson
intervals and paired re-analysis require the original per-episode JSONL/CSV and
frame banks. Request the corresponding dated result bundle from the project
owner and verify its `MANIFEST.sha256`.
