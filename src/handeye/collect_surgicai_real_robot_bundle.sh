#!/usr/bin/env bash
# SurgicAI real-robot read-only evidence and hand-eye data collector.
#
# Usage on the dVRK computer:
#   chmod +x collect_surgicai_real_robot_bundle.sh
#   ./collect_surgicai_real_robot_bundle.sh PSM1 quick   # about 4-6 minutes
#   ./collect_surgicai_real_robot_bundle.sh PSM1 full    # 30 Enter-anchored poses
#
# Optional output location:
#   SURGICAI_OUTPUT_ROOT=/media/USB ./collect_surgicai_real_robot_bundle.sh PSM1
#
# Safety contract:
#   - This script only lists, subscribes, echoes, and records ROS 2 topics.
#   - It never publishes, powers, homes, holds, or commands ECM/PSM/jaws.
#   - A trained robot owner must perform all approved teleoperation.

set -u
set -o pipefail

ARM="${1:-PSM1}"
case "$ARM" in
  PSM1|PSM2) ;;
  *)
    printf 'ERROR: arm must be PSM1 or PSM2, got: %s\n' "$ARM" >&2
    exit 2
    ;;
esac

MODE="${2:-quick}"
case "$MODE" in
  quick|full) ;;
  *)
    printf 'ERROR: mode must be quick or full, got: %s\n' "$MODE" >&2
    exit 2
    ;;
esac

# Source the lab stack even when a base `ros2` command is already on PATH.
# The JHU host in the 2026-08-12 capture uses Jazzy; older project hosts used
# Humble. Prefer an already selected distro, then Jazzy, then Humble.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  for ros_setup in /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do
    if [[ -f "$ros_setup" ]]; then
      # shellcheck disable=SC1090
      source "$ros_setup"
      break
    fi
  done
fi

for setup_file in \
  "$HOME/ros2_ws/install/setup.bash" \
  "$HOME/cisst_ws/install/setup.bash" \
  "$HOME/camera_registration_ws/install/setup.bash"; do
  if [[ -f "$setup_file" ]]; then
    # shellcheck disable=SC1090
    source "$setup_file"
  fi
done

if ! command -v ros2 >/dev/null 2>&1; then
  printf '%s\n' \
    'ERROR: ros2 is unavailable.' \
    'Source the lab ROS 2 and dVRK workspaces, then rerun this file.' >&2
  exit 3
fi

OUTPUT_ROOT="${SURGICAI_OUTPUT_ROOT:-$HOME}"
STAMP="$(date +%Y%m%dT%H%M%S)"
SESSION_DIR="$OUTPUT_ROOT/surgicai_real_robot_${ARM}_${MODE}_${STAMP}"
ARCHIVE_BASE="$OUTPUT_ROOT/surgicai_real_robot_${ARM}_${MODE}_${STAMP}"

mkdir -p \
  "$SESSION_DIR/system" \
  "$SESSION_DIR/topics/info" \
  "$SESSION_DIR/topics/messages" \
  "$SESSION_DIR/topics/rates" \
  "$SESSION_DIR/params" \
  "$SESSION_DIR/config_candidates/copied" \
  "$SESSION_DIR/bags" \
  "$SESSION_DIR/handeye_events" \
  "$SESSION_DIR/physical"

exec 3>&1 4>&2
exec > >(tee -a "$SESSION_DIR/run.log") 2>&1

printf '\n%s\n' '============================================================'
printf 'SurgicAI READ-ONLY real-robot collection\n'
printf 'Arm: %s\nMode: %s\nSession: %s\n' "$ARM" "$MODE" "$SESSION_DIR"
printf '%s\n' 'NO ROS publisher or robot command is created by this script.'
printf '%s\n\n' 'A robot owner must move the arm using approved teleoperation.'

LEFT_RAW=/jhu_daVinci/left/image_raw
RIGHT_RAW=/jhu_daVinci/right/image_raw
LEFT_RECT_COMP=/jhu_daVinci/left/image_rect/compressed
RIGHT_RECT_COMP=/jhu_daVinci/right/image_rect/compressed
LEFT_INFO=/jhu_daVinci/left/camera_info
RIGHT_INFO=/jhu_daVinci/right/camera_info

topic_exists() {
  ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

safe_name() {
  printf '%s' "$1" | sed 's#^/##; s#[/: ]#_#g'
}

record_topic_info() {
  local topic="$1"
  local name
  name="$(safe_name "$topic")"
  if topic_exists "$topic"; then
    timeout 15 ros2 topic info -v "$topic" \
      > "$SESSION_DIR/topics/info/${name}.txt" 2>&1 || true
  else
    printf '%s\n' "$topic" >> "$SESSION_DIR/topics/missing_topics.txt"
  fi
}

record_one_message() {
  local topic="$1"
  local name
  name="$(safe_name "$topic")"
  if topic_exists "$topic"; then
    timeout 20 ros2 topic echo --once "$topic" \
      > "$SESSION_DIR/topics/messages/${name}.txt" 2>&1 || true
  fi
}

record_rate() {
  local topic="$1"
  local name
  name="$(safe_name "$topic")"
  if topic_exists "$topic"; then
    local duration=12
    [[ "$MODE" == quick ]] && duration=5
    timeout --signal=INT --kill-after=3 "$duration" ros2 topic hz "$topic" \
      > "$SESSION_DIR/topics/rates/${name}.txt" 2>&1 || true
  fi
}

{
  printf 'utc_start=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf 'local_start=%s\n' "$(date -Is)"
  printf 'arm=%s\n' "$ARM"
  printf 'mode=%s\n' "$MODE"
  printf 'hostname=%s\n' "$(hostname)"
  printf 'user=%s\n' "$(id -un)"
  printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-unset}"
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-unset}"
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-unset}"
  printf 'output=%s\n' "$SESSION_DIR"
} > "$SESSION_DIR/session.env"

{
  date -Is
  uname -a
  lsb_release -a 2>/dev/null || true
  hostnamectl 2>/dev/null || true
  lscpu 2>/dev/null || true
  free -h 2>/dev/null || true
  df -h "$OUTPUT_ROOT" 2>/dev/null || true
  nvidia-smi 2>/dev/null || true
  python3 --version 2>&1 || true
  python3 - <<'PY' 2>&1 || true
try:
    import cv2
    print("opencv_version=", cv2.__version__)
    print("opencv_has_aruco=", hasattr(cv2, "aruco"))
except Exception as exc:
    print("opencv_probe_error=", repr(exc))
PY
} > "$SESSION_DIR/system/host_and_software.txt"

ps -eo user,pid,ppid,lstart,args --width 500 \
  > "$SESSION_DIR/system/processes_all.txt" 2>&1 || true
pgrep -af 'dvrk_system|decklink|camera.*registration|hand.?eye|extrinsic|ros2' \
  > "$SESSION_DIR/system/processes_relevant.txt" 2>&1 || true
env | sort > "$SESSION_DIR/system/environment.txt"
if [[ "$MODE" == full ]]; then
  timeout 35 ros2 doctor --report > "$SESSION_DIR/system/ros2_doctor.txt" 2>&1 || true
else
  printf 'Skipped in quick mode.\n' > "$SESSION_DIR/system/ros2_doctor.txt"
fi
ros2 pkg list | sort > "$SESSION_DIR/system/ros2_packages.txt" 2>&1 || true
ros2 pkg executables | sort > "$SESSION_DIR/system/ros2_executables.txt" 2>&1 || true
ros2 node list | sort > "$SESSION_DIR/topics/nodes.txt" 2>&1 || true
ros2 topic list | sort > "$SESSION_DIR/topics/topics.txt" 2>&1 || true

printf 'Capturing ROS graph metadata...\n'

TOPICS=(
  "$LEFT_RAW"
  "$RIGHT_RAW"
  "$LEFT_RECT_COMP"
  "$RIGHT_RECT_COMP"
  "$LEFT_INFO"
  "$RIGHT_INFO"
  "/$ARM/measured_cp"
  "/$ARM/local/measured_cp"
  "/$ARM/measured_cv"
  "/$ARM/measured_js"
  "/$ARM/jaw/measured_js"
  "/$ARM/setpoint_cp"
  "/$ARM/operating_state"
  "/$ARM/goal_reached"
  "/ECM/measured_cp"
  "/ECM/local/measured_cp"
  "/ECM/measured_cv"
  "/ECM/measured_js"
  "/ECM/setpoint_cp"
  "/ECM/operating_state"
  "/SUJ/$ARM/measured_cp"
  "/SUJ/$ARM/local/measured_cp"
  "/SUJ/$ARM/measured_js"
  "/SUJ/ECM/measured_cp"
  "/SUJ/ECM/local/measured_cp"
  "/SUJ/ECM/measured_js"
  "/tf"
  "/tf_static"
)

: > "$SESSION_DIR/topics/missing_topics.txt"
for topic in "${TOPICS[@]}"; do
  record_topic_info "$topic"
done

REQUIRED=(
  "$LEFT_RAW"
  "$RIGHT_RAW"
  "$LEFT_INFO"
  "$RIGHT_INFO"
  "/$ARM/measured_cp"
)

required_missing=0
for topic in "${REQUIRED[@]}"; do
  if ! topic_exists "$topic"; then
    printf 'REQUIRED TOPIC MISSING: %s\n' "$topic"
    required_missing=1
  fi
done

if [[ "$required_missing" -ne 0 ]]; then
  printf '%s\n' \
    'STOP: required camera/robot topics are missing.' \
    'No bag was recorded. Bring back this partial directory for diagnosis.'
  exit 4
fi

printf 'Saving representative messages and frame IDs...\n'
for topic in \
  "$LEFT_INFO" "$RIGHT_INFO" \
  "/$ARM/measured_cp" "/$ARM/local/measured_cp" "/$ARM/measured_cv" \
  "/$ARM/measured_js" "/$ARM/jaw/measured_js" "/$ARM/operating_state" \
  "/ECM/measured_cp" "/ECM/local/measured_cp" "/ECM/measured_js" \
  "/SUJ/$ARM/measured_cp" "/SUJ/ECM/measured_cp" "/tf_static"; do
  record_one_message "$topic"
done

printf 'Measuring topic rates; this takes about one minute...\n'
for topic in "$LEFT_RAW" "$RIGHT_RAW" "/$ARM/measured_cp" "/$ARM/measured_cv"; do
  record_rate "$topic"
done

if [[ "$MODE" == full ]]; then
  printf 'Saving node parameter dumps...\n'
  while IFS= read -r node; do
    [[ -n "$node" ]] || continue
    node_name="$(safe_name "$node")"
    timeout 15 ros2 param dump "$node" \
      > "$SESSION_DIR/params/${node_name}.yaml" 2>&1 || true
  done < "$SESSION_DIR/topics/nodes.txt"
else
  printf 'Skipped in quick mode.\n' > "$SESSION_DIR/params/README.txt"
fi

printf 'Finding robot/camera/registration configuration files...\n'
if [[ "$MODE" == full ]]; then
  for root in "$HOME/ros2_ws" "$HOME" /opt; do
    [[ -d "$root" ]] || continue
    timeout 30 find "$root" -maxdepth 7 -type f \
      \( -iname '*dvrk*.json' -o -iname 'system-*.json' \
         -o -iname '*camera*.yaml' -o -iname '*camera*.yml' \
         -o -iname '*stereo*.yaml' -o -iname '*stereo*.yml' \
         -o -iname '*handeye*' -o -iname '*hand-eye*' \
         -o -iname '*extrinsic*.yaml' -o -iname '*extrinsic*.json' \
         -o -iname '*registration*.yaml' -o -iname '*registration*.json' \) \
      -size -10M -print 2>/dev/null || true
  done | sort -u > "$SESSION_DIR/config_candidates/files.txt"
else
  {
    find "$HOME/camera_registration_ws/calibration_results" -maxdepth 2 -type f \
      \( -iname '*.json' -o -iname '*.yaml' -o -iname '*.yml' -o -iname '*.txt' \) \
      -size -10M -print 2>/dev/null || true
    find "$HOME/ros2_ws/install/dvrk_video" -maxdepth 8 -type f \
      \( -iname '*.yaml' -o -iname '*.yml' \) -size -10M -print 2>/dev/null || true
  } | sort -u > "$SESSION_DIR/config_candidates/files.txt"
fi

grep -Eo "/[^[:space:]\"']+\\.(json|yaml|yml)" \
  "$SESSION_DIR/system/processes_all.txt" 2>/dev/null \
  | sort -u > "$SESSION_DIR/config_candidates/paths_from_processes.txt" || true

while IFS= read -r config_path; do
  [[ -f "$config_path" ]] || continue
  config_name="$(printf '%s' "$config_path" | sed 's#^/##; s#/#__#g')"
  cp -a "$config_path" "$SESSION_DIR/config_candidates/copied/$config_name" || true
done < "$SESSION_DIR/config_candidates/paths_from_processes.txt"

while IFS= read -r config_path; do
  [[ -f "$config_path" ]] || continue
  config_name="$(printf '%s' "$config_path" | sed 's#^/##; s#/#__#g')"
  cp -a "$config_path" "$SESSION_DIR/config_candidates/copied/$config_name" || true
done < "$SESSION_DIR/config_candidates/files.txt"

printf '\n%s\n' 'Physical metadata (press Enter to record UNKNOWN if unavailable).'
read -r -p 'Marker type/dictionary (example DICT_4X4_50): ' MARKER_DICT || MARKER_DICT=UNKNOWN
read -r -p 'Marker ID: ' MARKER_ID || MARKER_ID=UNKNOWN
read -r -p 'Measured BLACK-SQUARE side length in mm: ' MARKER_SIZE_MM || MARKER_SIZE_MM=UNKNOWN
read -r -p 'Marker mount description: ' MARKER_MOUNT || MARKER_MOUNT=UNKNOWN
read -r -p 'ECM fixed and locked? (yes/no): ' ECM_FIXED || ECM_FIXED=UNKNOWN
read -r -p 'Approx. camera-to-needle working distance in mm: ' WORKING_DISTANCE_MM || WORKING_DISTANCE_MM=UNKNOWN
read -r -p 'Robot config path confirmed by owner: ' ROBOT_CONFIG || ROBOT_CONFIG=UNKNOWN
read -r -p 'Camera config path confirmed by owner: ' CAMERA_CONFIG || CAMERA_CONFIG=UNKNOWN
read -r -p 'Operator/robot-owner names: ' OPERATORS || OPERATORS=UNKNOWN
read -r -p 'Optional installation photo path to copy: ' INSTALL_PHOTO || INSTALL_PHOTO=''

{
  printf 'marker_dictionary=%s\n' "${MARKER_DICT:-UNKNOWN}"
  printf 'marker_id=%s\n' "${MARKER_ID:-UNKNOWN}"
  printf 'marker_black_square_size_mm=%s\n' "${MARKER_SIZE_MM:-UNKNOWN}"
  printf 'marker_mount=%s\n' "${MARKER_MOUNT:-UNKNOWN}"
  printf 'ecm_fixed=%s\n' "${ECM_FIXED:-UNKNOWN}"
  printf 'working_distance_mm=%s\n' "${WORKING_DISTANCE_MM:-UNKNOWN}"
  printf 'robot_config=%s\n' "${ROBOT_CONFIG:-UNKNOWN}"
  printf 'camera_config=%s\n' "${CAMERA_CONFIG:-UNKNOWN}"
  printf 'operators=%s\n' "${OPERATORS:-UNKNOWN}"
  printf 'installation_photo_source=%s\n' "${INSTALL_PHOTO:-MISSING}"
} > "$SESSION_DIR/physical/metadata.txt"

if [[ -n "${INSTALL_PHOTO:-}" && -f "$INSTALL_PHOTO" ]]; then
  cp -a "$INSTALL_PHOTO" "$SESSION_DIR/physical/"
fi

RAW_SECONDS=5
[[ "$MODE" == quick ]] && RAW_SECONDS=3
RAW_BAG_DIR="$SESSION_DIR/bags/raw_static_${RAW_SECONDS}s"
printf '\nRAW BASELINE (%s seconds)\n' "$RAW_SECONDS"
printf '%s\n' \
  'Make the ECM and both PSMs stationary, with the needle and marker visible.' \
  'This short bag uses uncompressed images and may be about 1-3 GB.'
read -r -p "Press Enter to record the ${RAW_SECONDS}-second raw baseline..." _

RAW_TOPICS=(
  "$LEFT_RAW" "$RIGHT_RAW" "$LEFT_INFO" "$RIGHT_INFO"
  "/$ARM/measured_cp" "/$ARM/local/measured_cp" "/$ARM/measured_cv"
  "/$ARM/measured_js" "/$ARM/jaw/measured_js"
  "/ECM/measured_cp" "/ECM/local/measured_cp"
  "/SUJ/$ARM/measured_cp" "/SUJ/ECM/measured_cp" "/tf"
)

raw_existing=()
for topic in "${RAW_TOPICS[@]}"; do
  topic_exists "$topic" && raw_existing+=("$topic")
done

timeout --signal=INT --kill-after=8 "$RAW_SECONDS" \
  ros2 bag record -o "$RAW_BAG_DIR" "${raw_existing[@]}" || true
ros2 bag info "$RAW_BAG_DIR" \
  > "$SESSION_DIR/bags/raw_static_${RAW_SECONDS}s_info.txt" 2>&1 || true

printf '\n%s\n' 'HAND-EYE MOTION BAG'
printf '%s\n' \
  "A rigid marker must be attached near the distal $ARM tool." \
  'ECM must remain fixed for the complete recording.' \
  'The robot owner moves the arm by approved teleoperation.' \
  'At every pose: stop completely, keep marker visible, then press Enter.' \
  'Use x/y/z translations and roll/pitch/yaw rotations; do not remain planar.' \
  'Samples 0-23 are solve poses; samples 24-29 are new held-out poses.'
read -r -p 'Press Enter when the owner, E-stop, camera, marker, and disk are ready...' _

MOTION_TOPICS=(
  "$LEFT_RECT_COMP" "$RIGHT_RECT_COMP" "$LEFT_INFO" "$RIGHT_INFO"
  "/$ARM/measured_cp" "/$ARM/local/measured_cp" "/$ARM/measured_cv"
  "/$ARM/measured_js" "/$ARM/jaw/measured_js" "/$ARM/setpoint_cp"
  "/$ARM/operating_state" "/$ARM/goal_reached"
  "/ECM/measured_cp" "/ECM/local/measured_cp" "/ECM/measured_cv"
  "/ECM/measured_js" "/ECM/setpoint_cp" "/ECM/operating_state"
  "/PSM1/measured_cp" "/PSM1/local/measured_cp" "/PSM1/measured_cv"
  "/PSM1/measured_js" "/PSM1/jaw/measured_js"
  "/PSM2/measured_cp" "/PSM2/local/measured_cp" "/PSM2/measured_cv"
  "/PSM2/measured_js" "/PSM2/jaw/measured_js"
  "/SUJ/$ARM/measured_cp" "/SUJ/$ARM/local/measured_cp" "/SUJ/$ARM/measured_js"
  "/SUJ/ECM/measured_cp" "/SUJ/ECM/local/measured_cp" "/SUJ/ECM/measured_js"
  "/tf" "/tf_static"
)

motion_existing=()
for topic in "${MOTION_TOPICS[@]}"; do
  topic_exists "$topic" && motion_existing+=("$topic")
done

if [[ "$MODE" == quick ]]; then
  printf '%s\n' \
    'QUICK MODE: no 30 Enter presses.' \
    'For the next 90 seconds, visit about 24 distinct poses.' \
    'Hold each pose still for about 1.5 seconds; keep the marker visible.' \
    'Use x/y/z plus roll/pitch/yaw. Safety is more important than speed.'
  read -r -p 'Press Enter to begin the 90-second SOLVE segment...' _
  printf 'solve_start_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)" \
    > "$SESSION_DIR/handeye_events/quick_phase_times.txt"
  timeout --signal=INT --kill-after=8 90 \
    ros2 bag record -o "$SESSION_DIR/bags/handeye_solve_90s" "${motion_existing[@]}" || true
  printf 'solve_end_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)" \
    >> "$SESSION_DIR/handeye_events/quick_phase_times.txt"
  ros2 bag info "$SESSION_DIR/bags/handeye_solve_90s" \
    > "$SESSION_DIR/bags/handeye_solve_90s_info.txt" 2>&1 || true

  printf '%s\n' \
    'Now use 6 NEW held-out poses that were not used in the solve segment.' \
    'Hold each new pose for about 2-3 seconds.'
  read -r -p 'Press Enter to begin the 30-second HELD-OUT segment...' _
  printf 'heldout_start_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)" \
    >> "$SESSION_DIR/handeye_events/quick_phase_times.txt"
  timeout --signal=INT --kill-after=8 30 \
    ros2 bag record -o "$SESSION_DIR/bags/handeye_heldout_30s" "${motion_existing[@]}" || true
  printf 'heldout_end_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)" \
    >> "$SESSION_DIR/handeye_events/quick_phase_times.txt"
  ros2 bag info "$SESSION_DIR/bags/handeye_heldout_30s" \
    > "$SESSION_DIR/bags/handeye_heldout_30s_info.txt" 2>&1 || true
else
  BAG_PID=''
  stop_motion_bag() {
    if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
      kill -INT "$BAG_PID" 2>/dev/null || true
      wait "$BAG_PID" 2>/dev/null || true
    fi
    BAG_PID=''
  }
  trap stop_motion_bag INT TERM EXIT

  ros2 bag record -o "$SESSION_DIR/bags/handeye_motion" "${motion_existing[@]}" &
  BAG_PID=$!
  sleep 3

  printf 'index\tsplit\tutc_iso\tunix_time_ns\tarm\n' \
    > "$SESSION_DIR/handeye_events/events.tsv"

  for index in $(seq 0 29); do
    if [[ "$index" -lt 24 ]]; then
      split=solve
    else
      split=heldout
    fi
    printf '\nPose %02d/29 [%s]: move, settle, verify marker visibility.\n' "$index" "$split"
    read -r -p 'Press Enter to mark this pose...' _
    utc_iso="$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
    unix_ns="$(date +%s%N)"
    printf '%s\t%s\t%s\t%s\t%s\n' \
      "$index" "$split" "$utc_iso" "$unix_ns" "$ARM" \
      >> "$SESSION_DIR/handeye_events/events.tsv"
    timeout 10 ros2 topic echo --once "/$ARM/measured_cp" \
      > "$SESSION_DIR/handeye_events/pose_$(printf '%02d' "$index")_${split}_${ARM}_measured_cp.txt" \
      2>&1 || true
    timeout 10 ros2 topic echo --once "/ECM/measured_cp" \
      > "$SESSION_DIR/handeye_events/pose_$(printf '%02d' "$index")_${split}_ECM_measured_cp.txt" \
      2>&1 || true
  done

  stop_motion_bag
  trap - INT TERM EXIT

  ros2 bag info "$SESSION_DIR/bags/handeye_motion" \
    > "$SESSION_DIR/bags/handeye_motion_info.txt" 2>&1 || true
fi

{
  printf 'utc_end=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf 'local_end=%s\n' "$(date -Is)"
  printf 'status=collection_finished\n'
  printf 'safety=read_only_no_publishers\n'
  printf 'arm=%s\n' "$ARM"
  printf 'mode=%s\n' "$MODE"
  if [[ "$MODE" == quick ]]; then
    printf 'handeye_samples=offline_select_stationary_frames_from_bags\n'
    printf 'solve_bag=%s\n' "$SESSION_DIR/bags/handeye_solve_90s"
    printf 'heldout_bag=%s\n' "$SESSION_DIR/bags/handeye_heldout_30s"
  else
    printf 'handeye_event_count=30\n'
    printf 'solve_event_count=24\n'
    printf 'heldout_event_count=6\n'
    printf 'motion_bag=%s\n' "$SESSION_DIR/bags/handeye_motion"
  fi
  printf 'raw_bag=%s\n' "$RAW_BAG_DIR"
} > "$SESSION_DIR/COLLECTION_SUMMARY.txt"

# Stop appending to run.log before hashing it. Otherwise the final console
# messages would invalidate its manifest entry after MANIFEST.sha256 is made.
exec 1>&3 2>&4
sleep 1

printf 'Generating SHA256 manifest...\n'
(
  cd "$SESSION_DIR" || exit 1
  find . -type f ! -name MANIFEST.sha256 -print0 \
    | sort -z \
    | xargs -0 sha256sum > MANIFEST.sha256
)

printf 'Creating portable archive; this can take several minutes...\n'
if command -v zstd >/dev/null 2>&1; then
  ARCHIVE_PATH="${ARCHIVE_BASE}.tar.zst"
  tar -C "$(dirname "$SESSION_DIR")" \
    --use-compress-program='zstd -T0 -3' \
    -cf "$ARCHIVE_PATH" "$(basename "$SESSION_DIR")"
else
  ARCHIVE_PATH="${ARCHIVE_BASE}.tar.gz"
  tar -C "$(dirname "$SESSION_DIR")" \
    -czf "$ARCHIVE_PATH" "$(basename "$SESSION_DIR")"
fi
sha256sum "$ARCHIVE_PATH" > "${ARCHIVE_PATH}.sha256"

printf '\n%s\n' '============================================================'
printf '%s\n' 'COLLECTION COMPLETE'
printf 'Directory: %s\n' "$SESSION_DIR"
printf 'Archive:   %s\n' "$ARCHIVE_PATH"
printf 'SHA file:  %s\n' "${ARCHIVE_PATH}.sha256"
printf '%s\n' \
  'Bring back the archive and its .sha256 file.' \
  'Also bring the physical installation photo if it was not copied.' \
  'Do not run autonomous motion based only on this collection.'
