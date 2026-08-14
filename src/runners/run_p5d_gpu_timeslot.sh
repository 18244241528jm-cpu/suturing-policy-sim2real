#!/usr/bin/env bash
set -eo pipefail

# P5d is the P5c closed-loop contract with one additional scheduler invariant:
# DA cannot start the next inference until FP ACKs the previous paired RGBD.
MODE="${1:-smoke}"
EPISODES="${2:-1}"
RESULT_ROOT="${3:-/home/jiaming/p5d_runs/20260802}"
PROJECT_MNT="${P5D_PROJECT_MNT:-/mnt/c/Users/30518/OneDrive - Johns Hopkins/Desktop/cis2/project34}"

export P5C_TASK_ID=P5d
export P5C_ACK_TIMESLOT=1
export P5C_MAX_OUTPUT_HZ=0
export P5C_LABEL_SUFFIX="${P5D_LABEL_SUFFIX:-_ack_timeslot}"
export P5C_CAPTURE_HZ="${P5D_CAPTURE_HZ:-6.0}"
export P5C_AMBF_EXECUTABLE="${P5D_AMBF_EXECUTABLE:-/home/jiaming/p5d_runtime/ambf_p5d_domain219}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-219}"
mkdir -p "$(dirname "$P5C_AMBF_EXECUTABLE")"

exec bash "$PROJECT_MNT/scripts/run_p5c_closed_loop_completion.sh" \
  "$MODE" "$EPISODES" "$RESULT_ROOT"
