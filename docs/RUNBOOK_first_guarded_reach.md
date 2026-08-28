# Runbook: first guarded Reach on the real cart

Goal for the session: one operator-approved Cartesian Reach, proving the control and
calibration chain end to end.

FoundationPose now produces needle poses on the real cart, but in the **camera** frame,
and the hand-eye transform that would move them into ECM does not exist yet. The goal
therefore still comes from a PSM tip touch-off. `fp_goal_bridge` is ready for the camera
path the moment a hand-eye solution exists; see the checklist below.

The operator acknowledgement string is exactly:

```
I_HAVE_OPERATOR_AND_ESTOP
```

---

## Before you leave for the lab

| # | Do | Why |
|---|---|---|
| 1 | Copy `wide5/checkpoints/best_model.zip` into the OneDrive transfer folder | 68 MB is too large for git, and the cart can pull it from OneDrive instead of you improvising a transfer on site |
| 2 | Copy `measured_poseclose/final_model.zip` too | A second model is the only way to tell "the model is wrong" from "the calibration is wrong" once something misbehaves. Copying a file costs nothing; a return trip costs a session |
| 3 | Copy `planB_20260826/demos_wide5/collection_report.json` | It records the goal distribution the checkpoint was trained on. Without it you cannot answer "is the goal I am feeding it inside the training range", which is the first question when a policy behaves oddly |
| 4 | Note your sb3 and torch versions | `python3 -c "import stable_baselines3, torch; print(stable_baselines3.__version__, torch.__version__)"`. Verified working: sb3 2.7.1, torch 2.10.0. A major version gap on the cart means the checkpoint may not deserialise, and you want to know that before it is the blocker |

## On arrival, in this order

| # | Do | Why |
|---|---|---|
| 1 | `git status` **before** `git pull` | You edited the runtime on the cart. This branch has changed the frames, the goal routing and the executor guards; pulling over local edits is the one step that can lose work |
| 2 | `colcon build --packages-select suturing_runtime` | Parameters live in the installed share directory, so a pull alone does not take effect |
| 3 | Check torch and sb3 are present | Absent, the geometric servo still runs -- its torch import is lazy and no other node needs it. Only the policy path is blocked, which decides whether you start with `goal_servo_node` or `policy_servo_node` |
| 4 | `python3 scripts/workspace_audit.py --teach` | Read-only, cannot move the arm, and it warms up the pose stream. The box you get is grown from where the arm actually goes rather than carried over from the simulator |
| 5 | Record the FoundationPose topic name and its `frame_id` | They fill `fp_goal_bridge`'s `needle_pose_topic` and `required_needle_frame`. Nothing in the camera path can be configured until you have them |

## Bench rehearsal (do this before you travel)

Verified on 2026-08-25 under ROS 2 Humble; no robot and no simulator involved.

```bash
colcon build --packages-select suturing_runtime
```

These commands use `ros2 run` without `--params-file`, so the nodes fall back to their
code defaults and the frame here is `PSM1_psm_base_link`. That is deliberate: the generic
default belongs in the code, and the cart-specific `ECM` belongs in `jhu_real.yaml`. On
the real cart, launched through the launch files, everything is `ECM` instead.

In one shell, fake a 20 Hz pose stream, start both nodes, then hand the servo a goal
61 mm away -- the full Approach stroke, the case the executor rejects outright:

```bash
ros2 topic pub -r 20 /suturing/psm1/measured_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: PSM1_psm_base_link}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

```bash
ros2 run suturing_runtime goal_servo_node --ros-args -p enabled:=true
```

```bash
ros2 run suturing_runtime guarded_pose_executor --ros-args -p enable_output:=true -p operator_acknowledgement:=I_HAVE_OPERATOR_AND_ESTOP
```

```bash
ros2 topic pub --once /suturing/approach/goal_final geometry_msgs/msg/PoseStamped '{header: {frame_id: PSM1_psm_base_link}, pose: {position: {x: 0.0, y: 0.0, z: 0.061}, orientation: {w: 1.0}}}'
```

What a healthy run looks like:

| Check | Expected |
|---|---|
| Executor startup line | `D10_MOTION_BUDGET_OK single execute_once reaches 40 mm / 100 deg; caps are 10 mm / 30 deg` |
| Servo status | `SEGMENT_PUBLISHED ... "segment_translation_m": 0.008, "remaining_translation_m": 0.053` |
| `execute_once` | `accepted distance=0.008000m` — **not** `D10-E56-DISTANCE`, which is the whole point |
| `/PSM1/servo_cp` | steps of 0.1 mm, i.e. `max_linear_speed_m_s` 0.002 divided by `control_hz` 20 |

The mock pose never moves, so the executor keeps stepping and never reports `COMPLETE`;
that is the mock, not a fault.

**`D10-E52-STALE_MEASURED_POSE` right after launch is a startup transient** — the pose
stream and the node race during discovery. Wait a couple of seconds and call again. It
does not recur once the node has seen its first pose.

---

## Step 0 — read before you write (no motion possible)

Nothing in this step can move the arm; only the adapter runs.

```bash
ros2 run suturing_runtime dvrk_topic_adapter --ros-args --params-file jhu_real.yaml
```

```bash
ros2 topic echo /PSM1/measured_cp --once
```

### What this already answered, on 2026-08-26

`/PSM1/measured_cp` reports **`frame_id: ECM`**, not the PSM's own base: `set_base_frame`
has put PSM1's base at the ECM. The whole command chain is therefore configured for ECM,
and every goal you publish must be stamped `ECM`.

The decisive check was `/PSM1/setpoint_cp` against a stationary `/PSM1/measured_cp`:

```
frame_id       ECM  on both
position       agree to 0.0027 mm
orientation    agree to 0.0063 deg
```

Commands and measurements share one frame, so servoing on `measured_cp` and commanding
through `/PSM1/servo_cp` is consistent. There is no `/PSM1/local/servo_cp` on this cart:
`local/measured_cp` (frame `PSM1_base`, origin ~74 mm away) is for hand-eye calibration,
which wants `T_robot_base_from_control_point`, and must **not** be used to drive the
executor.

The shipped workspace box needed no change. The pose read at the parked position sits
inside it with 94 mm of z margin, and z is positive here -- the simulator's
`z ∈ [-0.25, 0.05]` convention does not apply in the ECM frame.

| Axis | Reading (m) | Box | Margin to lower bound |
|---|---|---|---|
| x | -0.0873 | [-0.20, 0.20] | +113 mm |
| y | +0.0648 | [-0.20, 0.20] | +265 mm |
| z | +0.0441 | [-0.05, 0.25] | **+94 mm** |

**Still verify the box over the working volume**, not just the parked pose: move the PSM
by hand to the needle, to the home pose, and to a midpoint, and echo `measured_cp` at
each. One pose inside the box proves nothing about the rest.

**Keep the ECM parked for the whole session.** The base frame is anchored to it, so
moving the ECM invalidates every goal captured before the move.

`psm_pose_selector.selection_mode: fk_only` is *not* needed today. It only unblocks the
pipeline supervisor, and `execute_once` never consults the supervisor.

---

## Step 1 — preview only, still no motion

```bash
ros2 launch suturing_runtime real_read_only.launch.py
```

`enable_output` stays false, so the executor reports `PREVIEW_ONLY` and constructs no
command publisher at all. Touch the PSM tip to the target, read `measured_cp`, and
publish that pose as the final goal:

```bash
ros2 topic pub --once /suturing/approach/goal_final geometry_msgs/msg/PoseStamped '{header: {frame_id: ECM}, pose: {position: {x: -0.0873, y: 0.0648, z: 0.0441}, orientation: {x: 0.0656, y: -0.1444, z: 0.9839, w: -0.0818}}}'
```

Substitute the numbers you just read, **including the orientation**. A placeholder
`{w: 1.0}` is the identity rotation, tens of degrees away from any real grasp pose, and
the servo will happily spend its whole rotation budget turning towards it.

Enable the servo and watch what it *would* command:

```bash
ros2 service call /suturing/goal_servo/enable std_srvs/srv/SetBool '{data: true}'
```

```bash
ros2 topic echo /suturing/execution/preview
```

Check `/suturing/goal_servo/status`: `segment_translation_m` must be ≤ 0.008 and
`remaining_translation_m` should shrink each time you call `advance`. Subtract the current
`measured_cp` from the preview pose: the difference must point at the goal. If the preview
walks away from the target, stop -- something upstream is still in the wrong frame.

Note the pipeline supervisor will sit in `WAITING_INPUTS` because perception is not
running. **That does not block execution**: `execute_once` checks arming, freshness,
frame, distance, rotation and workspace, and never consults the supervisor.

---

## Step 2 — armed, one segment at a time

```bash
ros2 launch suturing_runtime real_guarded.launch.py operator_acknowledgement:=I_HAVE_OPERATOR_AND_ESTOP
```

`enable_output` must be set **at launch**. Flipping it later with `ros2 param set` is
refused with `D10-E60-OUTPUT_NOT_CONSTRUCTED`, because the command publisher only exists
if the contract passed at startup.

Per segment, hand on the e-stop:

```bash
ros2 service call /suturing/execution/arm std_srvs/srv/SetBool '{data: true}'
```

```bash
ros2 service call /suturing/execution/execute_once std_srvs/srv/Trigger
```

The executor disarms itself on `COMPLETE`. Queue the next waypoint, then arm and execute
again:

```bash
ros2 service call /suturing/goal_servo/advance std_srvs/srv/Trigger
```

A 61 mm approach is roughly 8 segments at the 8 mm budget. Only after a full approach has
run clean should you consider `auto_advance: true`, which re-queues on `COMPLETE` and
removes the per-segment operator gate.

Abort at any time:

```bash
ros2 service call /suturing/execution/stop std_srvs/srv/Trigger
```

---

## Error codes, and what they actually mean

| Code | Cause | Fix |
|---|---|---|
| `D10-E50-OUTPUT_LOCKED` | `enable_output` false or acknowledgement string wrong | Relaunch with both set |
| `D10-E60-OUTPUT_NOT_CONSTRUCTED` | `enable_output` was changed after startup | Relaunch; never `ros2 param set` this one |
| `D10-E51-MISSING_POSE_OR_GOAL` | No measured pose, or no goal on `/suturing/approach/goal` | Check the servo published a waypoint |
| `D10-E52-STALE_MEASURED_POSE` | Pose older than `max_input_age_s` (0.5 s) | dVRK publishing rate, or the adapter died |
| `D10-E54-FRAME` | `frame_id` mismatch | Step 0 |
| `D10-E56-DISTANCE` | Goal further than 10 mm | The servo should prevent this; if it appears, the servo is disabled or something else is publishing to `/suturing/approach/goal` |
| `D10-E57-ROTATION` | Rotation past 30 deg | Same |
| `D10-E58-WORKSPACE` | Target outside the workspace box | **Almost certainly the z sign from Step 0** |
| `D10-E59-TIMEOUT` | Segment exceeded 20 s | 2 mm/s speed cap against an 8 mm segment needs ~4 s; a timeout means the arm is not tracking |
| `D11-E02-MISSING_POSE_OR_GOAL` | Servo has no final goal | Publish `/suturing/approach/goal_final` |
| `D11-E04-FRAME` | Servo frame mismatch | Step 0 |

---

## What is deliberately not in this session

- **Perception-driven goals.** `metric_da_depth.enabled` is false and FoundationPose is
  0/252 on real frames. Recording data is the objective; closing the loop is not.
- **The learned policy** is now runnable: `policy_servo_node` loads a checkpoint and
  emits waypoints exactly like `goal_servo_node`, so the two swap by launching one or the
  other. Loading needs one trick, already built in: these checkpoints were written by a
  custom `TD3_HER_BC` whose replay buffer lives in the `RL_algo` package, so a plain
  `TD3.load` raises `ModuleNotFoundError: No module named 'RL_algo'` on a machine without
  the training tree. The node replaces the training-only entries instead of installing
  the trainer on the cart.
- **Historic note.** Under the measured-contract lens the July `measured_poseclose`
  checkpoint scores 5/5 at 2.53 mm / 9.40 deg, but no policy node exists and the
  geometric servo outperforms it on every swept axis. Bring the checkpoint as a fallback,
  do not let it block the Reach.
- **`auto_advance`.** Off until a full manual approach has been observed.
