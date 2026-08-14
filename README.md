# Suturing Policy Sim2Real

**中文新人入口：** [从零复现仿真](docs/zh/从零复现仿真.md) ·
[参数与排错](docs/zh/参数与排错.md) ·
[真机与仿真的区别](docs/zh/真机与仿真的区别.md)

This repository releases the project code and contracts for:

```text
ECM RGB -> metric depth -> needle mask -> FoundationPose -> physical gate
        -> frozen grasp goal -> PSM kinematics + hand-eye -> D2/RL controller
        -> Reach -> (unvalidated) physical close/lift
```

## What is actually validated

- [Measured] A frozen-goal deployment-proxy **Reach** result passed 29/30 paired
  AMBF episodes. This used the D2 staged controller, not RL actions.
- [Measured] The SIM-S3 first-frame physical gate accepted 40/40 and reduced
  proxy-mask raw flips from 1/40 to 0/40 after gating.
- [Disproved] Pure FoundationPose PSM tracking is not control-ready.
- [Not tested] Real dVRK hand-eye accuracy, automatic real-camera needle masks,
  physical jaw close/lift and needle retention.

Evidence is preserved in `docs/evidence/`. Read
[known limitations](docs/KNOWN_LIMITATIONS.md) before quoting a success rate.

## Repository layout

```text
src/SurgicAI/RL/       environment, TD3-HER-BC and evaluation contracts
src/perception/        stereo capture, DA adapter, FP gate and geometry
src/control/           D2 controller and SIM-S4 evaluator
src/handeye/           read-only collection, solve and overlay tools
src/runners/           portable SIM-S3/SIM-S4 research runners
models/rl/             released M3 and R6 project checkpoints
data/reference/        compact frozen SIM-S3 bank for Reach replay
configs/               host paths and frozen smoke/formal profiles
scripts/doctor.py      stage-coded setup audit; starts no simulator or robot
scripts/run_simulation.py  one entrypoint for code/S3/S4/full runs
scripts/inspect_results.py artifact completeness check
```

Third-party AMBF, Surgical Robotics Challenge, FoundationPose and Depth
Anything repositories remain external. The 3.8 GB DA checkpoint and the
FoundationPose image are also external; see [model assets](docs/MODEL_ASSETS.md).

## Ten-minute code-only check

Reference host: Ubuntu 22.04 or WSL2 Ubuntu 22.04, Python 3.10.

```bash
git clone https://github.com/18244241528jm-cpu/suturing-policy-sim2real.git
cd suturing-policy-sim2real
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements/analysis.txt

python scripts/doctor.py --profile code
python -m unittest discover -s tests -v
python src/handeye/synthetic_self_test.py --output-root "$HOME/surgicai_runs/handeye_selftest"
```

Expected endings are `DOCTOR_PASS`, `OK`, and `"passed": true`. The hand-eye
test is synthetic; it proves software conventions, not real robot accuracy.

## Simulation entrypoint

After following [the full setup guide](docs/zh/从零复现仿真.md):

```bash
cp configs/pipeline.env.example configs/pipeline.env
# Edit host paths, then source ROS in this shell.
source /opt/ros/humble/setup.bash
source "$HOME/ambf_ros_ws/install/setup.bash"

# Fast Reach replay from the released frozen SIM-S3 bank.
python scripts/run_simulation.py --stage s4 --profile smoke --goal both --controller d2

# Full render -> DA -> FP gate -> Reach computation.
python scripts/run_simulation.py --stage full --profile smoke --goal both --controller d2
```

Controlled replacements are explicit:

- `--depth gt|da`: privileged AMBF GT depth or learned metric depth;
- `--goal gt|fp|both`: bypass FP, use FP goal, or paired A/B;
- `--controller d2|rl`: staged servo or learned policy;
- `--profile smoke|formal`: 2-episode wiring check or frozen 40/30 contract.

Every stage writes `pipeline_status.json`. A failure is reported as a stable
code such as `D9-E60-S3` or `D9-E90-S4`; use the
[parameter and troubleshooting guide](docs/zh/参数与排错.md).

## ROS 2 real-robot runtime

The topic-first, startup-safe dVRK runtime is in
[`ros2_ws/src/suturing_runtime`](ros2_ws/src/suturing_runtime/README.md). It
ships with the JHU topic names supplied on 2026-08-13, a read-only type/message
preflight, a stable `/suturing/*` contract, TF-based Approach-goal generation,
freshness diagnostics, and a bounded single-Reach gateway. Robot output is
disabled by default and requires explicit acknowledgement, arm and execute
calls; it never commands jaw close or claims physical grasp.

## Safety boundary

Nothing here authorizes autonomous real-dVRK motion. Real operation adds camera
calibration, hand-eye, frame/unit validation, segmentation, latency, collision
limits, operator enable/stop and staged low-speed authorization. See
[simulation versus real robot](docs/zh/真机与仿真的区别.md).

## License

MIT for project-generated source code in this snapshot. Upstream projects,
meshes, model architectures and external checkpoints retain their own licenses.
