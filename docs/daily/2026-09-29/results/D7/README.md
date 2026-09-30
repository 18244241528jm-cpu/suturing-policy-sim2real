# EXP8 checkpoints for real-camera depth inference

This bundle contains the frozen EXP8-main and EXP8-ctrl `best.pth` checkpoints, the inference script, and every non-pip Python module reached by its local import chain. It does **not** include training data or evaluation images.

## Contents

- `checkpoints/EXP8-main/best.pth`: real-K, re-aimed EXP8-main (`reaim_realK`).
- `checkpoints/EXP8-ctrl/best.pth`: old-K, re-aimed EXP8-ctrl (`reaim_oldK`).
- `code/infer_exp0c_metric_da.py` and `code/run_da_a1_checkpoint_eval.py`.
- `depth_anything_v2/`: the Depth Anything V2 local source package used by EXP8 inference, including its `dinov2_layers` and `util` modules.
- `SHA256SUMS`: hashes of every checkpoint and Python source file in this bundle (and this README).

## Verified source environment

- Ubuntu 22.04 / WSL2, Python 3.10.12.
- PyTorch 2.10.0+cu128 (CUDA 12.8), torchvision 0.25.0, NumPy 1.26.4, OpenCV (`cv2`) 4.5.4.
- `xformers` was **not installed** in the verified environment; the bundled DINOv2 code uses its fallback path.
- Python standard-library modules are used by the scripts but need no separate installation.

PyTorch/torchvision must be installed as a compatible pair for the real machine's OS, CUDA runtime, and GPU driver. The external runtime dependencies are `torch`, `torchvision`, `numpy`, and `cv2` (for example `opencv-python` or a system OpenCV package). The bundled `depth_anything_v2` and `run_da_a1_checkpoint_eval.py` are **local source**, not pip requirements.

## Inference

Run from the extracted bundle directory. The bank is newline-delimited JSON with a unique `sample_id` and an absolute `rgb` image path per row. The current script also reads a `depth` path per row; for real images without GT depth, set it to a nonexistent path so error metrics are skipped. Optional `semantic_mask` is only used when a GT depth file exists. Output paths must not already exist; the script refuses to overwrite them.

```bash
python3 code/infer_exp0c_metric_da.py \
  --repo-root "$PWD" \
  --checkpoint checkpoints/EXP8-main/best.pth \
  --bank /absolute/path/to/real_bank.jsonl \
  --prediction-dir /absolute/path/to/predictions_exp8_main \
  --output /absolute/path/to/exp8_main_inference.json \
  --device cuda \
  --preprocess square
```

To run the control, use `checkpoints/EXP8-ctrl/best.pth` and distinct output paths. **Always pass `--preprocess square`**: this is the EXP8 training/inference contract; `keep_aspect` is a separate diagnostic mode and must not be substituted for real-camera evaluation. The predictions are metric depth in **metres**, saved as one `.npy` file per frame. The checkpoint's stored `input_size` (518), encoder (`vitl`), and metric head parameters are loaded by the script; no test-time GT alignment is applied.

Verify the extracted files before use:

```bash
sha256sum -c SHA256SUMS
```

Source provenance: `depth_audit_stage_a/infer_exp0c_metric_da.py` and `depth_audit_stage_a/run_da_a1_checkpoint_eval.py` from SurgicAI; `depth_anything_v2/` from `/home/jiaming/p5a_da_trainer/Depth-Anything-V2-xiangrui/depth_anything_v2`; checkpoints from `/home/jiaming/p5a_da_training/20260929_exp8/{reaim_realK,reaim_oldK}/exp8_needle_weighted_q040_3ep/best.pth`.
