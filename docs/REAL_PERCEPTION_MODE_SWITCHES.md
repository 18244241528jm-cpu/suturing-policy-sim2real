# Real perception mode switches

This document describes the **real-robot interfaces**, not the AMBF evaluation path.  The runtime deliberately exposes ablations instead of forcing every prior into one opaque result.

## 1. Needle selection modes

Edit `ros2_ws/src/suturing_runtime/config/jhu_real.yaml`:

```yaml
needle_pose_selector:
  ros__parameters:
    selection_mode: fp_only
```

| Mode | FoundationPose candidates | flat constraint | OBJ-origin support height | planar mask+plane+CAD observation |
|---|---:|---:|---:|---:|
| `fp_only` | required | off | off | off |
| `flat` | required | on | off | off |
| `support` | required | on | on | off |
| `flat_planar` | required | on | off | on |
| `support_planar` | required | on | on | on |

The selected output is always one of the original FoundationPose poses.  Geometry can reject impossible candidates, but it never silently replaces FP with an AMBF pose or a planar estimate.

Required topics:

- `/suturing/needle/candidates`: canonical `suturing.fp_candidates.v1` JSON produced by `fp_candidate_adapter`.
- `/suturing/external/support_surface`: `std_msgs/String`, schema `suturing.support_surface.v1`.
- `/suturing/external/needle_planar_observation`: `std_msgs/String`, schema `suturing.needle_planar_observation.v1`.

The support message must contain a real camera-frame point, normal and two tangent axes:

```json
{
  "schema": "suturing.support_surface.v1",
  "stamp_ns": 0,
  "frame_id": "ACTUAL_RECTIFIED_CAMERA_OPTICAL_FRAME",
  "source": "aruco_board_plus_measured_board_to_phantom_transform",
  "point_camera_m": ["MEASURED", "MEASURED", "MEASURED"],
  "normal_camera": ["MEASURED", "MEASURED", "MEASURED"],
  "axis_x_camera": ["MEASURED", "MEASURED", "MEASURED"],
  "axis_y_camera": ["MEASURED", "MEASURED", "MEASURED"]
}
```

The planar message must be computed from the same image stamp as the FP candidates:

```json
{
  "schema": "suturing.needle_planar_observation.v1",
  "stamp_ns": 0,
  "frame_id": "ACTUAL_RECTIFIED_CAMERA_OPTICAL_FRAME",
  "source": "real_mask_plus_real_support_plane_plus_needle_CAD",
  "position_camera_m": ["MEASURED", "MEASURED", "MEASURED"],
  "quaternion_xyzw": ["MEASURED", "MEASURED", "MEASURED", "MEASURED"],
  "sigma_xyyaw": ["REAL_SIGMA_X_M", "REAL_SIGMA_Y_M", "REAL_SIGMA_YAW_DEG"]
}
```

These snippets are schemas, not deployable numbers.  The repository intentionally does not contain a synthetic support pose disguised as a real one.

## 2. PSM pose modes

```yaml
psm_pose_selector:
  ros__parameters:
    selection_mode: disabled
```

| Mode | Output source | Required real calibration |
|---|---|---|
| `disabled` | none | none; safe default |
| `vision_only` | FP top-score PSM mesh candidate transformed to the dVRK control point | measured `T_mesh_from_control_point` |
| `fk_only` | dVRK `measured_cp` transformed into the camera frame | validated `T_camera_from_PSM_base` in TF |
| `fused` | FP candidate selected by FK innovation, FP rank and explicit error models | both transforms plus measured FK/vision `sigma6` |

The fusion cost for candidate `i` is:

```text
innovation_i = Log6( inverse(FK_camera_control_point) * FP_camera_control_point_i )
cost_i = sum_j innovation_ij^2 / (sigma_FK_j^2 + sigma_FP_j^2)
       + fp_rank_weight * normalized_FP_rank_i
```

Translation uses metres and rotation-vector components use degrees.  This is not a Kalman filter: it is a same-frame multi-hypothesis selector.  Optional motion compensation advances a delayed visual pose using the relative motion measured by FK between image capture and publication.

The external PSM FP process publishes `suturing.fp_candidates.v1` on `/suturing/external/psm_candidates` and must additionally set:

```json
{
  "frame_id": "ACTUAL_RECTIFIED_CAMERA_OPTICAL_FRAME",
  "mesh_frame": "THE_EXACT_PSM_CAD_FRAME_USED_BY_FP",
  "stamp_ns": 0,
  "poses": [
    {"position_m": ["FP_OUTPUT", "FP_OUTPUT", "FP_OUTPUT"],
     "quaternion_xyzw": ["FP_OUTPUT", "FP_OUTPUT", "FP_OUTPUT", "FP_OUTPUT"],
     "score": "FP_OUTPUT"}
  ]
}
```

The placeholders above only illustrate field shape.  They are not valid numeric runtime values and cannot be mistaken for calibration.

## 3. Missing real measurements — do not bypass

The default `jhu_real.yaml` leaves the following values empty or deliberately invalid:

1. `psm_camera_bridge.camera_frame` and a validated TF `T_camera_from_PSM_base` from real camera registration.
2. The fixed `T_mesh_from_control_point` between the exact PSM CAD frame tracked by FP and dVRK `measured_cp`.
3. Held-out real error models `kinematic_sigma6_m_deg` and `vision_sigma6_m_deg`.
4. The rigid transform from a phantom/table ArUco board to the phantom surface coordinate system.
5. A real publisher for support-surface and planar needle observations.
6. An external FoundationPose bridge that publishes all PSM candidates with the source RGB timestamp.

`fk_only`, `vision_only`, and `fused` fail closed when their required data is absent.  Do not insert identity transforms merely to make the graph green.

## 4. ArUco versus camera registration

A known-size ArUco board in one image gives `T_camera_from_marker` through PnP: detected 2D corners are matched to known 3D marker corners using the matching camera matrix and distortion model.

That is not yet camera-to-PSM hand-eye calibration.  The official [jhu-dVRK camera registration](https://github.com/jhu-dvrk/dvrk_camera_registration) package places a small marker on a moving PSM, collects multiple camera/robot pose pairs and solves the camera-to-robot registration.  Its output can provide the validated TF required by `psm_camera_bridge`; use its replay and overlay validation before enabling motion.

A second board fixed to the phantom has a different role.  Once its rigid board-to-phantom transform is physically measured, each visible frame provides:

```text
T_camera_from_phantom = T_camera_from_board * T_board_from_phantom
```

It anchors the support surface after the ECM moves.  It does not by itself locate the needle.

## 5. Read-only verification

```bash
source /opt/ros/jazzy/setup.bash       # use humble instead if that is the installed dVRK ROS
cd "$HOME/suturing-policy-sim2real/ros2_ws"
colcon build --symlink-install --packages-select suturing_runtime
source install/setup.bash

ros2 launch suturing_runtime real_read_only.launch.py \
  config:="$HOME/suturing-policy-sim2real/ros2_ws/src/suturing_runtime/config/jhu_real.yaml"
```

In another terminal:

```bash
ros2 topic echo /suturing/needle/selector_status --qos-durability transient_local --once
ros2 topic echo /suturing/psm1/pose_selector_status --qos-durability transient_local --once
```

Default expected state:

```text
needle: READY, mode=fp_only, required_inputs=[foundationpose_candidates]
PSM: DISABLED, warning=no AMBF, identity hand-eye, or mesh/control-point fallback
```

Switch one mode at a time.  A missing real input must produce a `D18-E*` status and no selected pose.  That is a successful safety test, not a pipeline failure.
