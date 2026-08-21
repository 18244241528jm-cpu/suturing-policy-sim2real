#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 BUNDLE_ROOT NEEDLE_MESH OUTPUT_ROOT FOUNDATIONPOSE_ROOT" >&2
  exit 64
fi

BUNDLE_ROOT=$(realpath "$1")
NEEDLE_MESH=$(realpath "$2")
OUTPUT_ROOT=$(realpath -m "$3")
FOUNDATIONPOSE_ROOT=$(realpath "$4")
FP_IMAGE=${SIM_S3_FP_IMAGE:-foundationpose:blackwell}
SCRIPT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

[[ -d "$BUNDLE_ROOT" ]] || { echo "D16-E701 bundle root missing: $BUNDLE_ROOT" >&2; exit 2; }
[[ -f "$NEEDLE_MESH" ]] || { echo "D16-E702 needle mesh missing: $NEEDLE_MESH" >&2; exit 2; }
[[ -f "$FOUNDATIONPOSE_ROOT/estimater.py" ]] || { echo "D16-E703 FP root invalid: $FOUNDATIONPOSE_ROOT" >&2; exit 2; }
docker image inspect "$FP_IMAGE" >/dev/null
mkdir -p "$OUTPUT_ROOT"
[[ -z "$(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 -print -quit)" ]] || {
  echo "D16-E704 refusing non-empty output: $OUTPUT_ROOT" >&2; exit 2;
}

if [[ -n "${D16_FP_DOCKER_ARGS:-}" ]]; then
  read -r -a FP_GPU_ARGS <<<"$D16_FP_DOCKER_ARGS"
elif [[ -e /dev/dxg ]]; then
  # WSL2 exposes CUDA through dxg and the host driver libraries.  --gpus all
  # can enumerate an incompatible container toolkit and fail with CUDA 500.
  FP_GPU_ARGS=(--device=/dev/dxg -v /usr/lib/wsl:/usr/lib/wsl -e LD_LIBRARY_PATH=/usr/lib/wsl/lib)
else
  FP_GPU_ARGS=(--gpus all)
fi

docker run --rm --ipc=host "${FP_GPU_ARGS[@]}" \
  -v "$FOUNDATIONPOSE_ROOT:/workspace:ro" \
  -v "$BUNDLE_ROOT:/d16_bundles:ro" \
  -v "$NEEDLE_MESH:/d16_mesh/needle.obj:ro" \
  -v "$OUTPUT_ROOT:/d16_output" \
  -v "$SCRIPT_ROOT/src/perception:/d16_code:ro" \
  "$FP_IMAGE" bash -lc \
  'cd /workspace && python /d16_code/run_foundationpose_offline.py \
     --foundationpose-root /workspace --bundle-root /d16_bundles \
     --mesh /d16_mesh/needle.obj --out-root /d16_output --iteration 5 --seed 2718'
