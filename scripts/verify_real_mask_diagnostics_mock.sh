#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="${1:-$HOME/d15_build_20260819}"
OUTPUT_ROOT="${2:-$HOME/d15_verify_20260819}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-231}"

# ROS setup scripts are not nounset-clean on every distro.
set +u
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u

extra_topics="$(ros2 topic list 2>/dev/null | grep -Ev '^/(parameter_events|rosout)$' || true)"
if [[ -n "${extra_topics}" ]]; then
  echo "D15-E901 selected ROS domain is not clean:" >&2
  echo "${extra_topics}" >&2
  exit 2
fi

mkdir -p "${OUTPUT_ROOT}"
ros2 launch suturing_runtime mock_read_only.launch.py >"${OUTPUT_ROOT}/runtime.log" 2>&1 &
runtime_pid=$!
ros2 run suturing_runtime operator_mask_publisher --ros-args \
  -p output_root:="${OUTPUT_ROOT}/operator_masks" >"${OUTPUT_ROOT}/operator_mask.log" 2>&1 &
mask_pid=$!
cleanup() {
  kill "${mask_pid}" 2>/dev/null || true
  kill "${runtime_pid}" 2>/dev/null || true
  wait "${mask_pid}" 2>/dev/null || true
  wait "${runtime_pid}" 2>/dev/null || true
}
trap cleanup EXIT
sleep 4

ros2 run suturing_runtime diagnostic_bundle --ros-args \
  -p output_dir:="${OUTPUT_ROOT}/bundle" -p duration_s:=6.0

python3 - "${OUTPUT_ROOT}/bundle/SUMMARY.json" <<'PY'
import json
import sys

summary = json.load(open(sys.argv[1], encoding="utf-8"))
first = summary["first_unresolved_stage"]
assert summary["motion_commands_published"] == 0
assert first and first["stage"] == "R6_NEEDLE_GATE", first
assert all(item["state"] == "OBSERVED" for item in summary["stages"][:6])
assert all(item["state"] == "BLOCKED_BY_EARLIER_STAGE" for item in summary["stages"][7:])
print("D15_MOCK_DIAGNOSTIC_PASS first_unresolved=R6_NEEDLE_GATE motion_commands=0")
PY

session_dir="$(find "${OUTPUT_ROOT}/operator_masks" -mindepth 1 -maxdepth 1 -type d | head -1)"
python3 - "${session_dir}" <<'PY'
import json
import sys
from pathlib import Path

import cv2
import numpy as np

session = Path(sys.argv[1])
metadata = json.loads((session / "source.json").read_text(encoding="utf-8"))
mask = np.zeros((metadata["height"], metadata["width"]), dtype=np.uint8)
mask[0, 0] = 255
assert cv2.imwrite(str(session / "needle_mask.png"), mask)
PY
ros2 service call /suturing/operator_mask/publish_file std_srvs/srv/Trigger '{}'
test -s "${session_dir}/needle_mask_normalized.png"
test -s "${session_dir}/mask_overlay.png"
echo "D15_MOCK_MASK_PASS session=${session_dir}"
