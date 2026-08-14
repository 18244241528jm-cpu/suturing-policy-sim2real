#!/usr/bin/env bash
set -eo pipefail

PROJECT_MNT="${P5C_PROJECT_MNT:-/mnt/c/Users/30518/OneDrive - Johns Hopkins/Desktop/cis2/project34}"
CANONICAL="$PROJECT_MNT/environments/SurgicAI"
MIRROR="${P5C_REPO:-/home/jiaming/SurgicAI-edheadd-clean}"
DOMAIN="${ROS_DOMAIN_ID:-217}"
CHECKPOINT="${P5C_DA_CHECKPOINT:-/home/jiaming/p5a_da_training/20260729/p5a_diverse_clean_vitl_fp32/best.pth}"
EXPECTED="fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88"

files=(
  RL/Model_evaluation.py
  RL/Approach_env.py
  RL/subtask_env.py
  RL/needle_reset_ranges.py
  RL/utils/scene_manager.py
  RL/utils/gym_manager.py
  RL/utils/needle.py
)

all_same=true
for file in "${files[@]}"; do
  canonical_sha="$(sha256sum "$CANONICAL/$file" | cut -d' ' -f1)"
  mirror_sha="$(sha256sum "$MIRROR/$file" | cut -d' ' -f1)"
  if [[ "$canonical_sha" == "$mirror_sha" ]]; then
    printf 'SAME %s %s\n' "$file" "$canonical_sha"
  else
    printf 'DIFF %s canonical=%s mirror=%s\n' "$file" "$canonical_sha" "$mirror_sha"
    all_same=false
  fi
done

actual="$(sha256sum "$CHECKPOINT" | cut -d' ' -f1)"
printf 'CHECKPOINT %s %s\n' "$actual" "$CHECKPOINT"
[[ "$actual" == "$EXPECTED" ]] || {
  echo "CHECKPOINT_SHA_MISMATCH expected=$EXPECTED actual=$actual" >&2
  exit 5
}
[[ "$all_same" == true ]] || exit 6

source /opt/ros/humble/setup.bash
topics="$(ROS_DOMAIN_ID="$DOMAIN" ros2 topic list 2>/dev/null || true)"
printf 'ROS_DOMAIN_ID %s\n' "$DOMAIN"
printf '%s\n' "$topics" | sed '/^$/d' | sed 's/^/TOPIC /'
unexpected="$(printf '%s\n' "$topics" | grep -Ev '^(/parameter_events|/rosout)?$' || true)"
[[ -z "$unexpected" ]] || {
  echo "DOMAIN_NOT_CLEAN domain=$DOMAIN" >&2
  exit 7
}

nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv,noheader | sed 's/^/GPU /'
echo P5C_PREFLIGHT_OK
