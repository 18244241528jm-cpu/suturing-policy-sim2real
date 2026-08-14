#!/usr/bin/env bash
set -eo pipefail

MODE="${1:-smoke}"
EPISODES="${2:-1}"
RESULT_ROOT="${3:-/home/jiaming/p5c_runs/20260801}"
REPO="${P5C_REPO:-/home/jiaming/SurgicAI-edheadd-clean}"
PROJECT_MNT="${P5C_PROJECT_MNT:-/mnt/c/Users/30518/OneDrive - Johns Hopkins/Desktop/cis2/project34}"
AUD="$PROJECT_MNT/depth_audit_stage_a"
AMBF_ROOT="${P5C_AMBF_ROOT:-$PROJECT_MNT/environments/ambf-ambf-3.0}"
AMBF_EXECUTABLE="${P5C_AMBF_EXECUTABLE:-/home/jiaming/p5c_runtime/ambf_p5c_domain217}"
STEREO_LAUNCH="${P5C_STEREO_LAUNCH:-$AUD/depth_audit_stereo.launch.yaml}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-217}"
FP_IMAGE="${P5C_FP_IMAGE:-foundationpose:blackwell}"
FP_ROOT="${P5C_FP_ROOT:-/home/jiaming/FoundationPose}"
FP_MESH="${P5C_FP_MESH:-/home/jiaming/surgical_robotics_challenge/ADF/Phantoms/3D_MED/high_res/Needle_stage_d_v0.OBJ}"
P9A_ROOT="${P5C_P9A_ROOT:-/home/jiaming/p9a_runs/20260728}"
DA_REPO="${P5C_DA_REPO:-/home/jiaming/p5a_da_trainer/Depth-Anything-V2-xiangrui}"
DA_CHECKPOINT="${P5C_DA_CHECKPOINT:-/home/jiaming/p5a_da_training/20260729/p5a_diverse_clean_vitl_fp32/best.pth}"
EXPECTED_DA_SHA="fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88"
MODEL_100K="$REPO/RL/Evaluation_model/Approach/TD3_HER_BC/measured_r3_20260727/checkpoints/rl_model_100000_steps.zip"
PRIVATE_EVAL="/home/jiaming/p5c_runtime/Model_evaluation_p5c.py"
MAX_OUTPUT_HZ="${P5C_MAX_OUTPUT_HZ:-0}"
LABEL_SUFFIX="${P5C_LABEL_SUFFIX:-}"
ACK_TIMESLOT="${P5C_ACK_TIMESLOT:-0}"
TASK_ID="${P5C_TASK_ID:-P5c}"
CAPTURE_HZ="${P5C_CAPTURE_HZ:-3.35}"

mkdir -p "$RESULT_ROOT" /home/jiaming/p5c_runtime
source /opt/ros/humble/setup.bash
source /home/jiaming/ambf_ros_ws/install/setup.bash
export ROS_DOMAIN_ID DISPLAY="${DISPLAY:-:0}" MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-d3d12}"
export PYTHONPATH="$REPO:$REPO/RL:$AUD:${PYTHONPATH:-}"
export AMBF_PLUGINS_PATH="$AMBF_ROOT/core/build/ambf_plugins:/home/jiaming/ambf_ros_ws/install/ros_comm_plugin/lib"
export LD_LIBRARY_PATH="$AMBF_ROOT/core/build/lib:${LD_LIBRARY_PATH:-}"

actual_sha="$(sha256sum "$DA_CHECKPOINT" | awk '{print $1}')"
[[ "$actual_sha" == "$EXPECTED_DA_SHA" ]] || { echo "DA SHA mismatch: $actual_sha" >&2; exit 5; }
python3 "$PROJECT_MNT/scripts/prepare_p5c_runtime_eval.py" --source "$REPO/RL/Model_evaluation.py" --out "$PRIVATE_EVAL"
if [[ ! -x "$AMBF_EXECUTABLE" ]]; then
  cp "$AMBF_ROOT/core/build/bin/ambf_simulator" "$AMBF_EXECUTABLE"
  chmod +x "$AMBF_EXECUTABLE"
fi

cat >"$RESULT_ROOT/pre_registration.json" <<EOF
{"schema":"SurgicAI.${TASK_ID}.preregistration.v1","written_before_smoke":true,"date":"2026-08-02","arm":"yaw15_new_DA_M3_100k","paired_control":"P9b/yaw15_live_M3_100k/GT_depth","scheduler":{"fp_ack_timeslot":$ACK_TIMESLOT,"max_output_hz":$MAX_OUTPUT_HZ,"capture_hz":$CAPTURE_HZ},"gates":{"G1":{"translation_mm_lte":5.0,"rotation_deg_lte":15.0,"non_flip":true},"G2_init":{"seconds_lte":10.0},"G2_live_stop":{"policy_consumed_fp_p50_hz_gte":1.5},"G2_live_target":{"policy_consumed_fp_p50_hz_gte":3.0},"pose_age_target":{"policy_step_p95_ms_lte":500.0},"G3":{"rotation_jump_reject_deg":90.0,"flips":"report"},"G4":{"environment_termination_success_gte":14,"denominator":20},"G5":{"reset_rejection_expected_near":"0/20"}},"da_checkpoint_sha256":"$actual_sha"}
EOF

ACTIVE_AMBF_PID=""; ACTIVE_CAPTURE_PID=""; ACTIVE_DA_PID=""; ACTIVE_DOCKER_PID=""; ACTIVE_CONTAINER=""; ACTIVE_STOP_FILE=""
stop_bridge() {
  [[ -n "$ACTIVE_STOP_FILE" ]] && touch "$ACTIVE_STOP_FILE"
  for name in ACTIVE_CAPTURE_PID ACTIVE_DA_PID; do
    pid="${!name}"; [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true; printf -v "$name" '%s' ""
  done
  [[ -n "$ACTIVE_CONTAINER" ]] && docker stop -t 5 "$ACTIVE_CONTAINER" >/dev/null 2>&1 || true
  [[ -n "$ACTIVE_DOCKER_PID" ]] && wait "$ACTIVE_DOCKER_PID" 2>/dev/null || true
  ACTIVE_DOCKER_PID=""; ACTIVE_CONTAINER=""; ACTIVE_STOP_FILE=""
}
cleanup() {
  stop_bridge
  if [[ -n "$ACTIVE_AMBF_PID" ]]; then
    kill "$ACTIVE_AMBF_PID" 2>/dev/null || true; sleep 1
    kill -KILL "$ACTIVE_AMBF_PID" 2>/dev/null || true; wait "$ACTIVE_AMBF_PID" 2>/dev/null || true
  fi
  ACTIVE_AMBF_PID=""
}
trap cleanup EXIT INT TERM

launch_ambf() {
  local log="$1"
  (cd "$AMBF_ROOT/core/build" && exec env LIBGL_ALWAYS_SOFTWARE=1 "$AMBF_EXECUTABLE" \
    --launch_file "$STEREO_LAUNCH" -l 0,1,2,3,4,5 -p 1000 -t 1 -s 5 \
    --override_max_comm_freq 200 --override_min_comm_freq 200 -g 1) >"$log" 2>&1 &
  ACTIVE_AMBF_PID=$!; sleep 20
  kill -0 "$ACTIVE_AMBF_PID" 2>/dev/null || { tail -100 "$log" >&2; return 4; }
}

start_bridge() {
  local cell="$1" container="$2" stream="$1/stream" stop="$1/stop"
  local da_extra=() fp_extra=()
  if [[ "$ACK_TIMESLOT" == "1" ]]; then
    da_extra+=(--fp-ack-file "$cell/fp_ack.json")
    fp_extra+=(--ack-file "$cell/fp_ack.json")
  fi
  mkdir -p "$stream" "$cell/fp_debug" "$cell/da_debug"; ACTIVE_STOP_FILE="$stop"
  python3 -u "$AUD/capture_p9b_live_stream.py" --control-file "$cell/control.json" \
    --stream-dir "$stream" --rate-hz "$CAPTURE_HZ" --stop-file "$stop" >"$cell/capture.log" 2>&1 & ACTIVE_CAPTURE_PID=$!
  python3 -u "$AUD/run_p5c_da_live_depth.py" --repo-root "$DA_REPO" --checkpoint "$DA_CHECKPOINT" \
    --stream-dir "$stream" --jsonl "$cell/da_stream.jsonl" --debug-dir "$cell/da_debug" \
    --stop-file "$stop" --device cuda --max-output-hz "$MAX_OUTPUT_HZ" "${da_extra[@]}" >"$cell/da.log" 2>&1 & ACTIVE_DA_PID=$!
  docker run --rm --ipc host --name "$container" --device=/dev/dxg -v /usr/lib/wsl:/usr/lib/wsl \
    -e LD_LIBRARY_PATH=/usr/lib/wsl/lib -v "$FP_ROOT:/workspace" -v "$AUD:/audit:ro" \
    -v "$RESULT_ROOT:$RESULT_ROOT" -v "$FP_MESH:/mesh/needle.obj:ro" -w /audit "$FP_IMAGE" \
    /opt/venv/bin/python /audit/run_fp_p5c_da_live_tracker.py --foundationpose-root /workspace \
    --stream-dir "$stream" --pose-file "$cell/live_pose.json" --jsonl "$cell/fp_stream.jsonl" \
    --stop-file "$stop" --mesh /mesh/needle.obj --debug-dir "$cell/fp_debug" \
    --iteration 5 --seed 2718 --rotation-jump-reject-deg 90 "${fp_extra[@]}" >"$cell/foundationpose.log" 2>&1 &
  ACTIVE_DOCKER_PID=$!; ACTIVE_CONTAINER="$container"
}

run_cell() {
  local label="$1" count="$2"
  local cell="$RESULT_ROOT/$label" bank="$P9A_ROOT/yaw15/external_goal_bank.json"
  [[ ! -e "$cell" ]] || { echo "Refusing overwrite: $cell" >&2; return 3; }
  mkdir -p "$cell"; cp "$RESULT_ROOT/pre_registration.json" "$cell/pre_registration.json"
  launch_ambf "$cell/ambf.log"; start_bridge "$cell" "p5c_d${ROS_DOMAIN_ID}_${label}"
  local started="$(date +%s)" rc pid complete=0; set +e
  (cd "$REPO" && exec python3 -u "$PRIVATE_EVAL" --algorithm TD3_HER_BC --task_name Approach \
    --reward_type sparse --model-path "$MODEL_100K" --train-seeds 1 --num-episodes "$count" \
    --eval_seed 1 --trans_error 1 --angle_error 10 --variant base_env --measured-success-reward \
    --command-state-clamp --deterministic-eval --needle-settle-steps 60 --needle-settle-interval-s 0.1 \
    --needle-random-x-mm 3 --needle-random-y-mm 3 --needle-random-rz-deg 15 \
    --external-goal-bank "$bank" --external-goal-source fp --external-pairing-trans-tol-mm 1.0 \
    --external-pairing-rot-tol-deg 0.5 --no-external-snapshot-lock \
    --live-fp-pose-file "$cell/live_pose.json" --live-fp-control-file "$cell/control.json" \
    --live-fp-initial-wait-s 120 --max-consecutive-reset-invalid 0 --results-json "$cell/result.json" \
    --episode-jsonl "$cell/episodes.jsonl" --log-level INFO --log-file "$cell/eval.log") >>"$cell/eval.log" 2>&1 & pid=$!
  while kill -0 "$pid" 2>/dev/null; do
    if [[ -s "$cell/result.json" ]] && python3 - "$cell/result.json" "$count" <<'PY'
import json,sys
p=json.load(open(sys.argv[1])); raise SystemExit(0 if len(p.get("episode_records",[]))==int(sys.argv[2]) else 1)
PY
    then complete=1; kill "$pid" 2>/dev/null || true; break; fi
    if ! kill -0 "$ACTIVE_DA_PID" 2>/dev/null || ! kill -0 "$ACTIVE_DOCKER_PID" 2>/dev/null; then kill "$pid" 2>/dev/null || true; break; fi
    if (( $(date +%s)-started > 10800 )); then kill "$pid" 2>/dev/null || true; break; fi
    sleep 2
  done
  for _ in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
  kill -KILL "$pid" 2>/dev/null || true
  wait "$pid"; rc=$?; set -e; stop_bridge; cleanup
  printf 'label=%s\nepisodes=%s\nwall_s=%s\nexit_code=%s\nartifact_complete=%s\nros_domain_id=%s\nmax_output_hz=%s\n' \
    "$label" "$count" "$(($(date +%s)-started))" "$rc" "$complete" "$ROS_DOMAIN_ID" "$MAX_OUTPUT_HZ" >"$cell/status.txt"
  printf 'ack_timeslot=%s\ntask_id=%s\ncapture_hz=%s\n' "$ACK_TIMESLOT" "$TASK_ID" "$CAPTURE_HZ" >>"$cell/status.txt"
  [[ -s "$cell/result.json" ]] || return 6
}

case "$MODE" in
  smoke) run_cell "smoke_latest_${MAX_OUTPUT_HZ}hz${LABEL_SUFFIX}" "$EPISODES" ;;
  formal) run_cell "yaw15_newDA_latest_${MAX_OUTPUT_HZ}hz_M3_100k${LABEL_SUFFIX}" "$EPISODES" ;;
  *) echo "MODE must be smoke or formal" >&2; exit 2 ;;
esac
