# Setup and reproduction

## 1. Reference environment

- Ubuntu 22.04 or WSL2 Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- NVIDIA GPU and a FoundationPose-compatible CUDA container
- AMBF 3.0 plus the ROS communication plugin
- Surgical Robotics Challenge scene and meshes

Keep the public repository in a path without spaces, for example
`/home/surgicai/suturing-policy-sim2real`.

## 2. Install Python dependencies

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements/analysis.txt
```

ROS Python packages, PyKDL and AMBF clients must come from the corresponding
ROS/AMBF workspaces rather than PyPI.

## 3. Clone external repositories

```bash
mkdir -p ~/surgicai_external
cd ~/surgicai_external
git clone https://github.com/WPI-AIM/ambf.git
git clone https://github.com/surgical-robotics-ai/surgical_robotics_challenge.git
git clone https://github.com/NVlabs/FoundationPose.git
git clone https://github.com/DepthAnything/Depth-Anything-V2.git
```

FoundationPose experiments in the original environment used commit
`a1b694b83e633c2cb6115b9063d940a687759392`. Do not assume a newer checkout is
numerically identical.

## 4. Configure paths

```bash
cp configs/pipeline.env.example configs/pipeline.env
editor configs/pipeline.env
set -a; source configs/pipeline.env; set +a
```

Use a dedicated ROS domain and a uniquely named AMBF executable. Never use a
broad `pkill` command; stop only the PID started by your runner.

Render the archived AMBF launch with your local SRC path:

```bash
python scripts/prepare_ambf_launch.py \
  --src-root "$SRC_ROOT" \
  --output-dir "$HOME/.cache/surgicai/launch"
export SIM_S4_STEREO_LAUNCH="$HOME/.cache/surgicai/launch/depth_audit_stereo.launch.yaml"
```

## 5. Code-only smoke

```bash
python scripts/preflight.py --mode code-only
python -m unittest discover -s tests -v
python src/handeye/synthetic_self_test.py --output-root /tmp/surgicai_handeye_selftest
```

## 6. Asset verification

Place external model files as described in `docs/MODEL_ASSETS.md`, then run:

```bash
python scripts/preflight.py --mode simulation
```

The script checks required paths, checkpoint hashes and that the ROS domain is
explicitly set. A failed preflight is a stop condition.

## 7. Main simulation sequence

The main result depends on a validated first-frame needle bank. Reproduce in
this order:

1. capture paired fixed-ECM reset frames;
2. run DA inference with the frozen preprocessing contract;
3. run FP register and the support-plane/rest-orientation gate;
4. build a frozen external goal bank;
5. run the paired GT and deployment-proxy D2 Reach cells.

The included runners are exact research snapshots and therefore require the
external AMBF/SRC layouts described by environment variables. Start with one
episode and inspect every output before launching a formal matrix.

```bash
export ROS_DOMAIN_ID=330
export SIM_S4_RESULT_ROOT=$HOME/surgicai_runs/sim_s4_smoke
bash src/runners/run_sim_s4_deployment_proxy_reach.sh all
```

Expected artifacts include `episodes.jsonl`, `result.json`, `status.txt`, logs
and a `MANIFEST.sha256`. A ROS teardown exit code alone is not the acceptance
criterion; the runner checks completed episode artifacts.

## 8. Real-robot hand-eye: read-only first

```bash
source /opt/ros/humble/setup.bash
source /path/to/dvrk_ws/install/setup.bash
bash src/handeye/discover_topics.sh "$HOME/handeye_discovery"
```

Copy `src/handeye/config/example_session.yaml`, fill confirmed topics, frame
IDs, intrinsics, marker size and synchronization limits, then follow
`src/handeye/README.md`. Collection publishes no motion command.

## 9. Common silent failures

- A populated ROS topic list may be daemon cache, not a live simulator. Wait
  for a real PSM state message.
- High disparity-valid fraction does not prove stereo metric accuracy.
- A dense semantic mask is a simulator privilege, not an automatic mask.
- FP top-1 score is not calibrated probability of correctness.
- A high update rate does not prove freshness; measure pose age.
- Do not compare success rates across single-goal and 25-candidate semantics.
- Do not use training frames as DA/FP acceptance frames.
