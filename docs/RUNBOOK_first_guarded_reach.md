# Runbook: first guarded Reach on the real cart

Goal for the session: one operator-approved Cartesian Reach to a hand-measured goal,
proving the control and calibration chain end to end. Perception is **not** on this
critical path — FoundationPose currently returns 0/252 on real frames, so the goal comes
from a PSM tip touch-off, not from the camera.

The operator acknowledgement string is exactly:

```
I_HAVE_OPERATOR_AND_ESTOP
```

---

## Step 0 — read before you write (no motion possible)

Nothing in this step can move the arm; only the adapter runs.

```bash
ros2 run suturing_runtime dvrk_topic_adapter --ros-args --params-file jhu_real.yaml
```

```bash
ros2 topic echo /PSM1/measured_cp --once
```

Record three things from that one message:

| What | Fills in | Why it matters |
|---|---|---|
| `header.frame_id` | `approach_goal_builder.required_needle_frame`, `psm_camera_bridge.camera_frame` | Both ship as `""`; a mismatch is `D10-E54-FRAME` |
| **sign of `pose.position.z`** | `guarded_pose_executor.workspace_min_m` / `max_m` | The shipped box is `z ∈ [-0.05, 0.25]` and marked `[假设]`. The simulator uses `z ∈ [-0.25, 0.05]` and the checkpoint goal sits at `z = -0.1187`. If the real base frame follows the simulator convention, **every** `execute_once` returns `D10-E58-WORKSPACE` until the box is corrected |
| magnitude of x/y/z | sanity | Metres, not centimetres. The goal servo converts internally; do not pre-scale |

Then set `psm_pose_selector.selection_mode: fk_only`. `vision_only` and `fused` stay
unavailable: `mesh_to_control_point_configured` is false and the quaternion placeholder
is `[0,0,0,0]` on purpose, so those modes have no calibration behind them.

---

## Step 1 — preview only, still no motion

```bash
ros2 launch suturing_runtime real_read_only.launch.py
```

`enable_output` stays false, so the executor reports `PREVIEW_ONLY` and constructs no
command publisher at all. Touch the PSM tip to the target, read `measured_cp`, and
publish that pose as the final goal:

```bash
ros2 topic pub --once /suturing/approach/goal_final geometry_msgs/msg/PoseStamped '{header: {frame_id: PSM1_psm_base_link}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

Enable the servo and watch what it *would* command:

```bash
ros2 service call /suturing/goal_servo/enable std_srvs/srv/SetBool '{data: true}'
```

```bash
ros2 topic echo /suturing/execution/preview
```

Check `/suturing/goal_servo/status`: `segment_translation_m` must be ≤ 0.008 and
`remaining_translation_m` should shrink each time you call `advance`. If the preview
walks away from the target, stop here — the frame or the z sign is wrong.

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
- **The learned policy.** Under the measured-contract lens the July `measured_poseclose`
  checkpoint scores 5/5 at 2.53 mm / 9.40 deg, but no policy node exists and the
  geometric servo outperforms it on every swept axis. Bring the checkpoint as a fallback,
  do not let it block the Reach.
- **`auto_advance`.** Off until a full manual approach has been observed.
