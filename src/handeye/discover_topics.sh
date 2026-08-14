#!/usr/bin/env bash
set -euo pipefail

# Read-only discovery only.  This script does not publish, power, home, or move.
SESSION_DIR="${1:?usage: discover_topics.sh SESSION_DIR}"
mkdir -p "$SESSION_DIR/logs"

ros2 node list | tee "$SESSION_DIR/logs/nodes.txt"
ros2 topic list | sort | tee "$SESSION_DIR/logs/topics.txt"
ros2 topic list | grep -Ei 'image|camera_info|measured_cp|measured_js|jaw|operating_state' \
  | tee "$SESSION_DIR/logs/handeye_topic_candidates.txt"

while IFS= read -r topic; do
  safe_name=$(printf '%s' "$topic" | tr '/' '_')
  ros2 topic info -v "$topic" > "$SESSION_DIR/logs/topic_${safe_name}.txt" || true
done < "$SESSION_DIR/logs/handeye_topic_candidates.txt"

printf '%s\n' \
  'No motion command was sent.' \
  'Copy confirmed names and message types into config/example_session.yaml.' \
  'Then record one message and topic hz manually under operator supervision.'

