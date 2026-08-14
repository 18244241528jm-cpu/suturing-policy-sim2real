# D3a real-robot hand-eye preflight kit

This directory contains a read-only ROS 2 collector plus offline validation,
ECM eye-in-hand, PSM eye-to-hand, held-out validation, and overlay tools.

Safety boundary: `capture_handeye_sample.py` creates subscribers only. It has
no publisher, motion action, power/home call, `servo_cp`, jaw, or hold command.
The operator or robot owner moves the arm manually/through the approved
teleoperation interface; this kit only takes a snapshot after Enter is pressed.

## Frame convention

Every matrix name is executable documentation:

```text
T_A_from_B maps coordinates expressed in B into A.
```

ECM eye-in-hand inputs and output:

```text
A_i = T_ecm_base_from_control_point_i       (measured_cp)
B_i = T_left_camera_from_static_marker_i    (PnP)
X   = T_control_point_from_left_camera      (solved)
A_i @ X @ B_i = constant T_ecm_base_from_static_marker
```

PSM eye-to-hand inputs and outputs:

```text
A_i = T_psm_base_from_control_point_i        (measured_cp)
B_i = T_left_camera_from_marker_i            (PnP)
X   = T_left_camera_from_psm_base             (solved)
Y   = T_control_point_from_marker              (solved mount)
B_i = X @ A_i @ Y
```

Every solution JSON also contains the inverse matrix. Held-out samples never
enter the solver.

## 0. Hardware-free acceptance test

From the `project34` root:

First verify that the selected interpreter has the contrib ArUco module:

```powershell
python -c "import cv2, numpy, scipy, yaml; assert hasattr(cv2, 'aruco'); print('OpenCV', cv2.__version__, 'ArUco OK')"
```

The collector supports both the legacy OpenCV contrib ArUco API
(`drawMarker`, module-level `detectMarkers`) and the newer API
(`generateImageMarker`, `ArucoDetector`). A plain `opencv-python` build without
`cv2.aruco` is rejected with an explicit error.

```powershell
$env:PYTHONIOENCODING='utf-8'
python scripts/real_robot_handeye/synthetic_self_test.py `
  --output-root environments/SurgicAI/records/logs/D3a_handeye_preflight_20260806/synthetic_self_test
```

Expected final field: `"passed": true`. This generates 4 sessions:

- ECM clean and noisy;
- PSM clean and noisy;
- each has 24 solve + 6 held-out poses;
- each has validation JSON, solution JSON, 6 held-out overlays, and images;
- the root contains `MANIFEST.sha256`.

`[测量]` The compatibility path was revalidated with WSL Python 3.10.12 and
OpenCV 4.5.4; all four sessions passed. The evidence is stored at
`/home/jiaming/d3a_reverify_20260806_codex_019fd9fe_r1/`.

## 1. Before the lab

Do not guess the local installation. Confirm and enter these values in a copy
of `config/example_session.yaml`:

1. left/right image and left `camera_info` topics;
2. ECM or PSM `measured_cp` topic and its exact ROS message type;
3. frame IDs from one real message;
4. real camera resolution and distortion;
5. ArUco dictionary, ID, and caliper-measured side length in meters;
6. checkerboard rows/columns/square size if used instead;
7. robot and camera configuration paths;
8. rigid marker-clamp installation photo path;
9. the lab-approved dVRK camera-registration repository URL and branch.

The repository URL/branch was not found locally and is intentionally not
invented. These tools do not depend on it for collection or data auditing.

## 2. Read-only topic discovery

```bash
source /opt/ros/humble/setup.bash
source /ACTUAL/DVRK/WORKSPACE/install/setup.bash
SESSION_PARENT="$HOME/surgicai_handeye_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$SESSION_PARENT"
bash scripts/real_robot_handeye/discover_topics.sh "$SESSION_PARENT"
```

Review every candidate with the robot owner. Copy confirmed topics to a new
YAML file. This step sends no robot command.

## 3. Collect ECM eye-in-hand

Physical arrangement: checkerboard/ArUco is rigidly fixed on the table; the
left camera moves with ECM. Collect 24 diverse solve poses first, then move to
6 new held-out poses that are not reused or relabeled.

```bash
python3 scripts/real_robot_handeye/capture_handeye_sample.py \
  --config "$SESSION_PARENT/confirmed_ecm.yaml" \
  --session-dir "$SESSION_PARENT/ecm_eye_in_hand" \
  --calibration-type ecm_eye_in_hand \
  --mode ros --count 30
```

Keep the board fully visible and add multi-axis rotations. The 24+6 counts and
coverage thresholds are `[假设]` engineering recommendations, not validated
real-robot acceptance limits.

## 4. Collect PSM eye-to-hand

Physical arrangement: ECM stays fixed; an ArUco/checkerboard is attached to
PSM with a rigid screw clamp. Adhesive that can flex is not acceptable.

```bash
python3 scripts/real_robot_handeye/capture_handeye_sample.py \
  --config "$SESSION_PARENT/confirmed_psm.yaml" \
  --session-dir "$SESSION_PARENT/psm_eye_to_hand" \
  --calibration-type psm_eye_to_hand \
  --mode ros --count 30
```

## 5. Validate before solving

```bash
python3 scripts/real_robot_handeye/validate_handeye_dataset.py \
  --config "$SESSION_PARENT/confirmed_ecm.yaml" \
  --session-dir "$SESSION_PARENT/ecm_eye_in_hand"

python3 scripts/real_robot_handeye/validate_handeye_dataset.py \
  --config "$SESSION_PARENT/confirmed_psm.yaml" \
  --session-dir "$SESSION_PARENT/psm_eye_to_hand"
```

The validator fails closed on resolution/camera-info mismatch, missing marker
size, excessive timestamp delta, non-meter units, non-finite/non-SE(3) poses,
duplicate poses, insufficient translation/rotation coverage, solve/held-out
overlap, or ambiguous frame-chain directions.

## 6. Solve and render held-out overlays

```bash
python3 scripts/real_robot_handeye/solve_ecm_camera_handeye.py \
  --session-dir "$SESSION_PARENT/ecm_eye_in_hand"
python3 scripts/real_robot_handeye/render_handeye_overlay.py \
  --session-dir "$SESSION_PARENT/ecm_eye_in_hand" \
  --solution "$SESSION_PARENT/ecm_eye_in_hand/ecm_solution.json" \
  --output-dir "$SESSION_PARENT/ecm_eye_in_hand/overlays"

python3 scripts/real_robot_handeye/solve_psm_camera_extrinsic.py \
  --session-dir "$SESSION_PARENT/psm_eye_to_hand"
python3 scripts/real_robot_handeye/render_handeye_overlay.py \
  --session-dir "$SESSION_PARENT/psm_eye_to_hand" \
  --solution "$SESSION_PARENT/psm_eye_to_hand/psm_solution.json" \
  --output-dir "$SESSION_PARENT/psm_eye_to_hand/overlays"
```

Do not accept a result based only on solve residual. Review held-out
translation, rotation, pixel reprojection, and all 6 overlays. A real-robot
pass threshold remains for Adnan/Ed to approve.

## 7. Sample contents

Each `samples/sample_NNN/` stores:

- left RGB and optional right RGB;
- full `camera_info` fields and timestamps;
- raw marker corners and PnP `T_camera_from_marker`;
- raw robot message fields and `T_robot_base_from_control_point`;
- image/robot timestamps, frame IDs, units, sync delta;
- marker dictionary/ID/physical size;
- session-level checkerboard, robot/camera configs, git commit, installation
  photo, operator notes, validity and rejection reason.

## 8. Absolute stop line

Do not send any automatic motion command during discovery, collection,
validation, solving, or overlay review. Even after both hand-eye solutions are
accepted, first perform PSM-kinematics projection overlay and offline needle
DA/FP acceptance. D2 translation, orientation, and approach each require a
separate human confirmation. Only then may the robot owner authorize a single
close/lift trial with a purpose-built safety runner. The existing
`Image_IL/dvrk_policy_adapter.py` is not that runner.
