#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "D15-E901-ROS_NOT_SOURCED: source /opt/ros/humble/setup.bash and the built workspace" >&2
  exit 2
fi

OUTPUT_DIR="${1:-$HOME/surgicai_diagnostics/$(date +%Y%m%d_%H%M%S)}"
DURATION_S="${2:-20}"
mkdir -p "$OUTPUT_DIR"

echo "Read-only collection. It publishes zero robot motion commands."
echo "Output: $OUTPUT_DIR"
ros2 run suturing_runtime diagnostic_bundle --ros-args \
  -p output_dir:="$OUTPUT_DIR" \
  -p duration_s:="$DURATION_S" \
  -p trigger_capture:=true

echo "Collection complete. Send these first:"
echo "  $OUTPUT_DIR/SHARE_THIS_FIRST.md"
echo "  $OUTPUT_DIR/SUMMARY.json"
