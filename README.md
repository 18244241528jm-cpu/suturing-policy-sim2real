# Suturing Policy Sim2Real

Reproducible research code for the pipeline

```text
ECM RGB -> metric depth -> needle mask -> FoundationPose -> physical gate
        -> frozen grasp goal -> PSM kinematics + hand-eye -> staged SE(3) servo
        -> Reach -> (future) physical close/lift
```

This repository is a compact, public-facing snapshot of Project 34. It keeps
the code that defines the interfaces and validated experiments, while large
checkpoints, simulator binaries, raw datasets and third-party repositories stay
outside Git.

## Read this first: what is and is not working

The validated simulation result is a **Reach sub-pipeline**, not autonomous
physical suturing:

- needle initialization: RGB -> new Depth Anything checkpoint -> FoundationPose
  -> support-plane/rest-orientation gate -> frozen goal;
- PSM state: kinematics plus a hand-eye transform, not pure visual PSM tracking;
- controller: staged D2 SE(3) servo;
- deployment-proxy Reach: 29/30 paired episodes;
- physical jaw close, lift and needle retention: not yet validated on the real robot.

Known negative results are part of the release:

- pure FoundationPose tracking of the PSM is not control-ready;
- 4 mm AMBF stereo degraded the validated DA depth when fused;
- an MHT/EKF selector did not materially beat the simple fixed fusion baseline;
- moving the camera near and normal to the phantom enlarged the needle but moved
  the DA model out of distribution, worsening needle depth p95 to 33.489 mm.

See [docs/KNOWN_LIMITATIONS.md](docs/KNOWN_LIMITATIONS.md) before interpreting
any success rate.

## Repository layout

```text
src/
  SurgicAI/RL/        Environment, reset, goal, TD3-HER-BC and evaluation contracts
  perception/         DA/FP bridges, geometry and first-frame safety gate
  control/            Staged SE(3) controller and isolated evaluator
  handeye/            Read-only real-robot collection, solve and overlay tools
  runners/            Reproducible simulation orchestration scripts
docs/
  ARCHITECTURE.md      Data flow, coordinate contracts and privilege boundaries
  SETUP.md             Installation and smoke-test instructions
  MODEL_ASSETS.md      External repositories, checkpoints and SHA256 values
  KNOWN_LIMITATIONS.md What has been measured, inferred and disproved
  evidence/            Small dated reports; no raw datasets or checkpoints
configs/
  pipeline.env.example Portable path and ROS-domain configuration
scripts/
  preflight.py         Checks code, external assets and unsafe missing inputs
tests/
  test_public_contract.py  Hardware-free contract checks
```

The original relative layout is intentionally retained under `src/`. Several
research scripts import the SurgicAI environment dynamically; a cosmetic
package rewrite would make the public code shorter but no longer reproduce the
tested runtime.

Existing lab onboarding material remains available:

- [完整Pipeline_从0到MTM控制仿真PSM.md](完整Pipeline_从0到MTM控制仿真PSM.md):
  environment setup, teleoperation, rosbag and basic PSM control;
- [policy_deployment_bundle/](policy_deployment_bundle/): historical dVRK
  adapter and deployment checklist;
- [checkpoint presentation/](checkpoint%20presentation/): earlier project
  presentation material.

## External dependencies

This repository does **not** vendor these projects:

- [AMBF](https://github.com/WPI-AIM/ambf)
- [Surgical Robotics Challenge](https://github.com/surgical-robotics-ai/surgical_robotics_challenge)
- [FoundationPose](https://github.com/NVlabs/FoundationPose)
- [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2)
- ROS 2 Humble and a compatible dVRK workspace

It also does not contain the 3.8 GB DA checkpoint, FoundationPose Docker image,
raw rosbag files or expert trajectories. Exact hashes and placement rules are
in [docs/MODEL_ASSETS.md](docs/MODEL_ASSETS.md).

## Quick start: hardware-free checks

Ubuntu 22.04 or WSL2 with Python 3.10 is the reference host.

```bash
git clone https://github.com/18244241528jm-cpu/suturing-policy-sim2real.git
cd suturing-policy-sim2real

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements/analysis.txt

python scripts/preflight.py --mode code-only
python -m unittest discover -s tests -v
python src/handeye/synthetic_self_test.py --output-root /tmp/surgicai_handeye_selftest
```

Expected endings:

```text
PUBLIC_PIPELINE_PREFLIGHT_OK mode=code-only
OK
"passed": true
```

The hand-eye self-test uses synthetic data. It verifies conventions and the
solver implementation; it is not a claim about real dVRK accuracy.

## Full simulation smoke test

Follow [docs/SETUP.md](docs/SETUP.md) to install the external repositories and
place the model assets. Copy the example configuration and edit only paths:

```bash
cp configs/pipeline.env.example configs/pipeline.env
set -a
source configs/pipeline.env
set +a

python scripts/preflight.py --mode simulation
bash src/runners/run_sim_s4_deployment_proxy_reach.sh all
```

The runner refuses to overwrite an existing result directory and requires a
clean, dedicated `ROS_DOMAIN_ID`. Do not share a domain with another AMBF run.

## Real robot safety boundary

Nothing in this repository authorizes autonomous motion on a real dVRK.

The hand-eye collector is read-only: an approved operator moves the robot and
presses Enter to save synchronized observations. Before any automatic command:

1. verify camera topics, camera intrinsics, frame IDs and units;
2. solve hand-eye and pass held-out overlays;
3. project measured PSM kinematics into the image and inspect the overlay;
4. validate needle DA/FP offline on real images;
5. authorize translation, orientation and approach separately at low speed.

The existing `policy_deployment_bundle/` is retained as historical dVRK adapter
material. It is not a complete needle-goal safety runner.

## Reproducing a result versus extending the project

- To reproduce the main Reach result, use SIM-S3 then SIM-S4 with the frozen
  assets listed in `docs/MODEL_ASSETS.md`.
- To evaluate another DA checkpoint, keep the same held-out frames and run the
  DA/FP acceptance code; never compare on its training frames.
- To change camera geometry, collect a new held-out set and revalidate metric
  depth before running FP or control.
- To add a probabilistic fusion model, first show that the FP candidate oracle
  contains a correct candidate. A selector cannot recover a pose absent from
  its candidate set.

## Citation and provenance

This is research software under active development. Every measured statement
in the docs points to a dated report in `docs/evidence/`. Upstream code remains
under its original license; see [LICENSE](LICENSE) and the upstream projects.

## License

MIT for the SurgicAI-derived code in this snapshot. Third-party repositories,
meshes, models and checkpoints are not relicensed by this repository.
