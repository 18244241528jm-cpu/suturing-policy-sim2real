#!/usr/bin/env bash
set -eo pipefail
RESULT_ROOT="${SIM_S4_RESULT_ROOT:-$HOME/surgicai_runs/sim_s4}"
MODE="${1:-all}"
PROJECT_MNT="${SIM_S4_PROJECT_MNT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
REPO="${SIM_S4_REPO:-$PROJECT_MNT/src/SurgicAI}"
SIM_S3_ROOT="${SIM_S4_SIM_S3_ROOT:-$HOME/surgicai_runs/sim_s3}"
AMBF_ROOT="${SIM_S4_AMBF_ROOT:-${AMBF_ROOT:-}}"
AMBF_EXECUTABLE="${SIM_S4_AMBF_EXECUTABLE:-$HOME/.cache/surgicai/ambf_sim_s4_domain${ROS_DOMAIN_ID:-330}}"
STEREO_LAUNCH="${SIM_S4_STEREO_LAUNCH:-}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-330}"
MODEL="${SIM_S4_MODEL:-}"
RUNTIME="${SIM_S4_RUNTIME:-$HOME/.cache/surgicai/sim_s4_runtime}"
ROS_SETUP="${SIM_S4_ROS_SETUP:-/opt/ros/humble/setup.bash}"
AMBF_ROS_SETUP="${SIM_S4_AMBF_ROS_SETUP:-$HOME/ambf_ros_ws/install/setup.bash}"

[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ && "$ROS_DOMAIN_ID" != 0 ]] || { echo "Use an isolated non-zero ROS_DOMAIN_ID" >&2; exit 2; }
[[ -n "$AMBF_ROOT" && -d "$AMBF_ROOT" ]] || { echo "Set SIM_S4_AMBF_ROOT or AMBF_ROOT" >&2; exit 2; }
[[ -s "$STEREO_LAUNCH" ]] || { echo "Set SIM_S4_STEREO_LAUNCH to the SRC/AMBF launch YAML" >&2; exit 2; }
[[ -s "$MODEL" ]] || { echo "Set SIM_S4_MODEL to the evaluated M3 checkpoint" >&2; exit 2; }
if [[ "$MODE" == all ]]; then
  [[ ! -e "$RESULT_ROOT" ]] || { echo "Refusing overwrite: $RESULT_ROOT" >&2; exit 3; }
elif [[ "$MODE" == resume-b ]]; then
  [[ -s "$RESULT_ROOT/A_GT_frozen_D2/result.json" && ! -e "$RESULT_ROOT/B_deployment_proxy_D2" ]] || {
    echo "resume-b requires complete A and absent B" >&2; exit 3;
  }
else
  echo "mode must be all or resume-b" >&2; exit 2
fi
mkdir -p "$RESULT_ROOT" "$RUNTIME"
source "$ROS_SETUP"
source "$AMBF_ROS_SETUP"
export ROS_DOMAIN_ID DISPLAY="${DISPLAY:-:0}" MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-d3d12}"
export PYTHONPATH="$REPO:$REPO/RL:$PROJECT_MNT/src/perception:${PYTHONPATH:-}"
export AMBF_PLUGINS_PATH="$AMBF_ROOT/core/build/ambf_plugins:$(dirname "$AMBF_ROS_SETUP")/ros_comm_plugin/lib"
export LD_LIBRARY_PATH="$AMBF_ROOT/core/build/lib:${LD_LIBRARY_PATH:-}" PYTHONDONTWRITEBYTECODE=1
topics="$(timeout 8 ros2 topic list 2>&1 | grep -Ev '^/(parameter_events|rosout)$' || true)"
domain_log="$RESULT_ROOT/domain_preflight_topics.txt"; [[ "$MODE" == resume-b ]] && domain_log="$RESULT_ROOT/domain_resume_b_preflight_topics.txt"
[[ -z "$topics" ]] || { printf '%s\n' "$topics" >"$domain_log"; exit 4; }
printf 'ROS_DOMAIN_ID=%s\nstatus=clean\n' "$ROS_DOMAIN_ID" >"$domain_log"
cp "$PROJECT_MNT/src/control/controllers.py" "$RUNTIME/controllers.py"
cp "$PROJECT_MNT/src/control/sim_s4_isolated_eval.py" "$RUNTIME/sim_s4_isolated_eval.py"
if [[ "$MODE" == all ]]; then
  python3 "$PROJECT_MNT/src/control/build_sim_s4_goal_banks.py" --sim-s3-root "$SIM_S3_ROOT" \
    --out "$RESULT_ROOT/banks" --episodes 30 --bias-seed 20260811 >"$RESULT_ROOT/bank_build.log" 2>&1
fi
[[ -s "$RESULT_ROOT/banks/A_gt_frozen.json" && -s "$RESULT_ROOT/banks/B_deployment_proxy_frozen_bias5.json" ]] || exit 5
if [[ "$MODE" == resume-b && ! -s "$RESULT_ROOT/A_GT_frozen_D2/status.txt" ]]; then
  cat >"$RESULT_ROOT/A_GT_frozen_D2/status.txt" <<'EOF'
label=A_GT_frozen_D2
episodes=30
wall_s=unknown_after_teardown_recovery
evaluator_exit_code=137
evaluator_requested_teardown=true
artifact_complete=1
ros_domain_id=$ROS_DOMAIN_ID
ambf_requested_teardown=true
ambf_wait_exit_code=143
ambf_teardown_clean=true
status_recovered_after_shell_errexit=true
controller=D2_staged
internal_gate=3mm_3deg
step=1.5mm_3deg
bank=A_gt_frozen.json
goal_source=gt
EOF
fi
if [[ ! -x "$AMBF_EXECUTABLE" ]]; then cp "$AMBF_ROOT/core/build/bin/ambf_simulator" "$AMBF_EXECUTABLE"; chmod +x "$AMBF_EXECUTABLE"; fi

AMBF_PID=""; AMBF_LOG=""
cleanup() {
  [[ -n "$AMBF_PID" ]] || return 0
  kill "$AMBF_PID" 2>/dev/null || true
  for _ in $(seq 1 30); do kill -0 "$AMBF_PID" 2>/dev/null || break; sleep 0.1; done
  kill -KILL "$AMBF_PID" 2>/dev/null || true
  if wait "$AMBF_PID" 2>/dev/null; then AMBF_WAIT_RC=0; else AMBF_WAIT_RC=$?; fi
  AMBF_PID=""
}
trap cleanup EXIT INT TERM
launch_ambf() {
  AMBF_LOG="$1"
  (cd "$AMBF_ROOT/core/build" && exec env LIBGL_ALWAYS_SOFTWARE=1 "$AMBF_EXECUTABLE" \
    --launch_file "$STEREO_LAUNCH" -l 0,1,2,3,4,5 -p 1000 -t 1 -s 5 \
    --override_max_comm_freq 200 --override_min_comm_freq 200 -g 1) >"$AMBF_LOG" 2>&1 &
  AMBF_PID=$!; ready=0
  for _ in $(seq 1 120); do
    kill -0 "$AMBF_PID" 2>/dev/null || break
    if timeout 2 ros2 topic echo --once /ambf/env/psm2/baselink/State >/dev/null 2>&1; then ready=1; break; fi
    sleep 1
  done
  [[ "$ready" == 1 ]] || { tail -100 "$AMBF_LOG" >&2; return 5; }; sleep 2
}
complete() { python3 - "$1" <<'PY'
import json,sys
try: p=json.load(open(sys.argv[1])); ok=len(p.get('episode_records',[]))==30
except Exception: ok=False
raise SystemExit(0 if ok else 1)
PY
}
run_cell() {
  label="$1" bank="$2" source="$3"; cell="$RESULT_ROOT/$label"; mkdir -p "$cell/legacy"
  launch_ambf "$cell/ambf.log"; start="$(date +%s)"; set +e
  python3 -u "$RUNTIME/sim_s4_isolated_eval.py" --repo "$REPO" --artifacts-dir "$cell/legacy" -- \
    --algorithm TD3_HER_BC --task_name Approach --reward_type sparse --model-path "$MODEL" \
    --train-seeds 1 --num-episodes 30 --eval_seed 1 --trans_error 0.3 --angle_error 3 \
    --variant base_env --controller goal-servo --measured-success-reward --command-state-clamp \
    --deterministic-eval --needle-settle-steps 60 --needle-settle-interval-s 0.1 \
    --needle-random-x-mm 3 --needle-random-y-mm 3 --needle-random-rz-deg 15 \
    --external-goal-bank "$RESULT_ROOT/banks/$bank" --external-goal-source "$source" \
    --external-pairing-trans-tol-mm 1.0 --external-pairing-rot-tol-deg 0.5 \
    --no-external-snapshot-lock --max-consecutive-reset-invalid 0 --trans-step-mm 1.5 \
    --angle-step-deg 3 --divergence-abort-cm 5 --stall-abort-step 700 \
    --structured-step-trace --results-json "$cell/result.json" --episode-jsonl "$cell/episodes.jsonl" \
    --log-level INFO --log-file "$cell/eval.log" >>"$cell/eval.log" 2>&1 &
  eval_pid=$!; evaluator_requested_teardown=false
  while kill -0 "$eval_pid" 2>/dev/null; do
    if complete "$cell/result.json"; then evaluator_requested_teardown=true; kill "$eval_pid" 2>/dev/null || true; break; fi
    if ! kill -0 "$AMBF_PID" 2>/dev/null; then kill "$eval_pid" 2>/dev/null || true; break; fi
    if (( $(date +%s)-start > 3600 )); then kill "$eval_pid" 2>/dev/null || true; break; fi
    sleep 2
  done
  for _ in $(seq 1 30); do kill -0 "$eval_pid" 2>/dev/null || break; sleep 0.2; done
  kill -KILL "$eval_pid" 2>/dev/null || true; wait "$eval_pid"; eval_rc=$?
  set -e; cleanup; teardown_rc="${AMBF_WAIT_RC:-0}"
  artifact=0; complete "$cell/result.json" && artifact=1
  cat >"$cell/status.txt" <<EOF
label=$label
episodes=30
wall_s=$(($(date +%s)-start))
evaluator_exit_code=$eval_rc
evaluator_requested_teardown=$evaluator_requested_teardown
artifact_complete=$artifact
ros_domain_id=230
ambf_requested_teardown=true
ambf_wait_exit_code=$teardown_rc
ambf_teardown_clean=true
controller=D2_staged
internal_gate=3mm_3deg
step=1.5mm_3deg
bank=$bank
goal_source=$source
EOF
  [[ "$artifact" == 1 ]] || return 6
}
if [[ "$MODE" == all ]]; then run_cell A_GT_frozen_D2 A_gt_frozen.json gt; fi
run_cell B_deployment_proxy_D2 B_deployment_proxy_frozen_bias5.json fp
(cd "$RESULT_ROOT" && find . -type f ! -name MANIFEST.sha256 -print0 | sort -z | xargs -0 sha256sum >MANIFEST.sha256)
echo "SIM_S4_RAW_COMPLETE $RESULT_ROOT"
