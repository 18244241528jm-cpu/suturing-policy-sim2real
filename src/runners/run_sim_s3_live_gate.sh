#!/usr/bin/env bash
set -eo pipefail

PROJECT_MNT="${SIM_S3_PROJECT_MNT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
RESULT_ROOT="${SIM_S3_RESULT_ROOT:-$HOME/surgicai_runs/sim_s3}"
REPO="${SIM_S3_REPO:-$PROJECT_MNT/src/SurgicAI}"
AUD="${SIM_S3_PERCEPTION_ROOT:-$PROJECT_MNT/src/perception}"
AMBF_ROOT="${SIM_S3_AMBF_ROOT:-${AMBF_ROOT:-}}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-220}"
AMBF_EXECUTABLE="${SIM_S3_AMBF_EXECUTABLE:-$HOME/.cache/surgicai/ambf_sim_s3_domain${ROS_DOMAIN_ID}}"
STEREO_LAUNCH="${SIM_S3_STEREO_LAUNCH:-}"
FP_IMAGE="${SIM_S3_FP_IMAGE:-foundationpose:blackwell}"
FP_ROOT="${SIM_S3_FP_ROOT:-${FOUNDATIONPOSE_ROOT:-}}"
FP_MESH="${SIM_S3_FP_MESH:-${SRC_ROOT:-}/ADF/Phantoms/3D_MED/high_res/Needle_stage_d_v0.OBJ}"
DA_REPO="${SIM_S3_DA_REPO:-${DA_ROOT:-}}"
DA_CHECKPOINT="${SIM_S3_DA_CHECKPOINT:-${DA_CHECKPOINT:-}}"
EXPECTED_DA_SHA="${SIM_S3_EXPECTED_DA_SHA:-fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88}"
EPISODES="${SIM_S3_EPISODES:-40}"
EVAL_SEED="${SIM_S3_EVAL_SEED:-1}"
YAW_DEG="${SIM_S3_YAW_DEG:-15}"
X_MM="${SIM_S3_X_MM:-3}"
Y_MM="${SIM_S3_Y_MM:-3}"
DEPTH_SOURCE="${SIM_S3_DEPTH_SOURCE:-da}"
RUNTIME="${SIM_S3_RUNTIME:-$HOME/.cache/surgicai/sim_s3_runtime}"
ROS_SETUP="${SIM_S3_ROS_SETUP:-/opt/ros/humble/setup.bash}"
AMBF_ROS_SETUP="${SIM_S3_AMBF_ROS_SETUP:-$HOME/ambf_ros_ws/install/setup.bash}"

[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ && "$ROS_DOMAIN_ID" -ge 1 && "$ROS_DOMAIN_ID" -le 232 ]] || { echo "Use an isolated ROS_DOMAIN_ID in 1..232" >&2; exit 2; }
[[ "$EPISODES" =~ ^[1-9][0-9]*$ ]] || { echo "SIM_S3_EPISODES must be a positive integer" >&2; exit 2; }
[[ "$DEPTH_SOURCE" == da || "$DEPTH_SOURCE" == gt ]] || { echo "SIM_S3_DEPTH_SOURCE must be da or gt" >&2; exit 2; }
[[ -n "$AMBF_ROOT" && -d "$AMBF_ROOT" ]] || { echo "Set SIM_S3_AMBF_ROOT or AMBF_ROOT" >&2; exit 2; }
[[ -s "$STEREO_LAUNCH" ]] || { echo "Set SIM_S3_STEREO_LAUNCH to the rendered launch YAML" >&2; exit 2; }
[[ -d "$FP_ROOT" ]] || { echo "Set SIM_S3_FP_ROOT or FOUNDATIONPOSE_ROOT" >&2; exit 2; }
[[ -s "$FP_MESH" ]] || { echo "Set SIM_S3_FP_MESH or SRC_ROOT" >&2; exit 2; }
if [[ "$DEPTH_SOURCE" == da ]]; then
  [[ -d "$DA_REPO" ]] || { echo "Set SIM_S3_DA_REPO or DA_ROOT" >&2; exit 2; }
  [[ -s "$DA_CHECKPOINT" ]] || { echo "Set SIM_S3_DA_CHECKPOINT or DA_CHECKPOINT" >&2; exit 2; }
fi
[[ ! -e "$RESULT_ROOT" ]] || { echo "Refusing overwrite: $RESULT_ROOT" >&2; exit 3; }
mkdir -p "$RESULT_ROOT" "$RUNTIME"
source "$ROS_SETUP"
source "$AMBF_ROS_SETUP"
export ROS_DOMAIN_ID DISPLAY="${DISPLAY:-:0}" MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-d3d12}"
export PYTHONPATH="$REPO:$REPO/RL:$AUD:${PYTHONPATH:-}"
export AMBF_PLUGINS_PATH="$AMBF_ROOT/core/build/ambf_plugins:$(dirname "$AMBF_ROS_SETUP")/ros_comm_plugin/lib"
export LD_LIBRARY_PATH="$AMBF_ROOT/core/build/lib:${LD_LIBRARY_PATH:-}"

topics="$(timeout 8 ros2 topic list 2>&1 | grep -Ev '^/(parameter_events|rosout)$' || true)"
[[ -z "$topics" ]] || { printf '%s\n' "$topics" >"$RESULT_ROOT/domain_preflight_topics.txt"; echo "ROS domain $ROS_DOMAIN_ID is not clean" >&2; exit 4; }
printf 'ROS_DOMAIN_ID=%s\nstatus=clean\n' "$ROS_DOMAIN_ID" >"$RESULT_ROOT/domain_preflight_topics.txt"
if [[ "$DEPTH_SOURCE" == da ]]; then
  actual_sha="$(sha256sum "$DA_CHECKPOINT" | awk '{print $1}')"
  [[ "$actual_sha" == "$EXPECTED_DA_SHA" ]] || { echo "DA SHA mismatch: $actual_sha" >&2; exit 5; }
fi

python3 - "$RESULT_ROOT/perturbation_plan_frozen.json" "$EPISODES" <<'PY'
import json, numpy as np, sys
from pathlib import Path
rng=np.random.default_rng(20260811); ops=['erosion','dilation','none']; rows=[]
for i in range(int(sys.argv[2])):
 rows.append({'frame_id':f'frame_{i:06d}','dx_px':int(rng.integers(-3,4)),
              'dy_px':int(rng.integers(-3,4)),'morphology':ops[int(rng.integers(0,3))],
              'kernel_radius_px':int(rng.integers(0,4))})
Path(sys.argv[1]).write_text(json.dumps({'schema':'SurgicAI.SIM-S3.frozen_mask_perturbations.v1',
 'written_before_capture':True,'seed':20260811,'rows':rows},indent=2)+'\n')
PY

if [[ ! -x "$AMBF_EXECUTABLE" ]]; then
  cp "$AMBF_ROOT/core/build/bin/ambf_simulator" "$AMBF_EXECUTABLE"
  chmod +x "$AMBF_EXECUTABLE"
fi
AMBF_PID=""; CAMERA_PID=""
cleanup() {
  if [[ -n "$CAMERA_PID" ]] && kill -0 "$CAMERA_PID" 2>/dev/null; then kill "$CAMERA_PID" 2>/dev/null || true; wait "$CAMERA_PID" 2>/dev/null || true; fi
  if [[ -n "$AMBF_PID" ]] && kill -0 "$AMBF_PID" 2>/dev/null; then
    kill "$AMBF_PID" 2>/dev/null || true
    for _ in $(seq 1 30); do kill -0 "$AMBF_PID" 2>/dev/null || break; sleep 0.1; done
    kill -KILL "$AMBF_PID" 2>/dev/null || true; wait "$AMBF_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

(cd "$AMBF_ROOT/core/build" && exec env LIBGL_ALWAYS_SOFTWARE=1 "$AMBF_EXECUTABLE" \
  --launch_file "$STEREO_LAUNCH" -l 0,1,2,3,4,5 -p 1000 -t 1 -s 5 \
  --override_max_comm_freq 200 --override_min_comm_freq 200 -g 1) >"$RESULT_ROOT/ambf_capture.log" 2>&1 &
AMBF_PID=$!
ready=0
for _ in $(seq 1 120); do
  kill -0 "$AMBF_PID" 2>/dev/null || break
  if timeout 2 ros2 topic echo --once /ambf/env/phantom/Needle/State >/dev/null 2>&1; then ready=1; break; fi
  sleep 1
done
[[ "$ready" == 1 ]] || { tail -100 "$RESULT_ROOT/ambf_capture.log" >&2; exit 6; }
sleep 3

python3 -u "$AUD/capture_p9a_camera_daemon.py" \
  --request-dir "$RESULT_ROOT/requests" --out "$RESULT_ROOT/rendered_reset_frames" \
  --expected-frames "$EPISODES" >"$RESULT_ROOT/camera_capture.log" 2>&1 &
CAMERA_PID=$!
python3 -u "$AUD/capture_p9a_reset_bank.py" --episodes "$EPISODES" --eval-seed "$EVAL_SEED" \
  --yaw-deg "$YAW_DEG" --x-mm "$X_MM" --y-mm "$Y_MM" --ros-domain-id "$ROS_DOMAIN_ID" \
  --request-dir "$RESULT_ROOT/requests" --output "$RESULT_ROOT/reset_bank.json" \
  >"$RESULT_ROOT/reset_bank.log" 2>&1
wait "$CAMERA_PID"; CAMERA_PID=""; cleanup; AMBF_PID=""

if [[ "$DEPTH_SOURCE" == da ]]; then
  python3 -u "$AUD/infer_p5c_gate_da.py" --repo-root "$DA_REPO" \
    --checkpoint "$DA_CHECKPOINT" --capture-dir "$RESULT_ROOT/rendered_reset_frames/L" \
    --prediction-dir "$RESULT_ROOT/new_da_depth" --out "$RESULT_ROOT/da_result.json" \
    --device cuda >"$RESULT_ROOT/da.log" 2>&1
else
  python3 -u "$AUD/use_gt_depth.py" --capture-dir "$RESULT_ROOT/rendered_reset_frames/L" \
    --prediction-dir "$RESULT_ROOT/new_da_depth" --out "$RESULT_ROOT/da_result.json" \
    >"$RESULT_ROOT/da.log" 2>&1
fi

if [[ -n "${SIM_S3_FP_DOCKER_ARGS:-}" ]]; then
  read -r -a FP_GPU_ARGS <<<"$SIM_S3_FP_DOCKER_ARGS"
elif [[ -e /dev/dxg ]]; then
  FP_GPU_ARGS=(--device=/dev/dxg -v /usr/lib/wsl:/usr/lib/wsl -e LD_LIBRARY_PATH=/usr/lib/wsl/lib)
else
  FP_GPU_ARGS=(--gpus all)
fi
docker run --rm --ipc host "${FP_GPU_ARGS[@]}" -v "$FP_ROOT:/workspace" -v "$AUD:/audit:ro" \
  -v "$RESULT_ROOT:/results" -v "$FP_MESH:/mesh/needle.obj:ro" -w /audit "$FP_IMAGE" \
  /opt/venv/bin/python /audit/run_fp_sim_s3_live_gate.py \
  --foundationpose-root /workspace --dataset /results/rendered_reset_frames \
  --reset-bank /results/reset_bank.json --prediction-dir /results/new_da_depth \
  --perturbation-plan /results/perturbation_plan_frozen.json --mesh /mesh/needle.obj \
  --out-dir /results/fp_gate --iteration 5 --seed 2718 --normal-gate-deg 20 \
  --height-gate-mm 5 >"$RESULT_ROOT/foundationpose.log" 2>&1

(cd "$RESULT_ROOT" && find . -type f ! -name MANIFEST.sha256 -print0 | sort -z | xargs -0 sha256sum >MANIFEST.sha256)
echo "SIM_S3_RAW_COMPLETE $RESULT_ROOT"
