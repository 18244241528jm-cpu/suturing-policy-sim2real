#!/usr/bin/env python3
"""P5c latest-frame-only DA producer with immutable RGBD pairing."""

from __future__ import annotations

import argparse
import json
import os
import queue
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from run_p5_da_live_depth import (
    append_jsonl,
    atomic_json,
    atomic_npy,
    load_model,
    preprocess,
    save_first_frame_debug,
    sha256,
)


EXPECTED_SHA = "fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88"


def atomic_png(path: Path, image: np.ndarray) -> None:
    temporary = path.with_name(path.stem + ".tmp" + path.suffix)
    if not cv2.imwrite(str(temporary), image):
        raise RuntimeError(f"Failed to write {temporary}")
    os.replace(temporary, path)


class LatestFrameReader(threading.Thread):
    """Load a coherent capture into RAM and retain only the newest sample."""

    def __init__(self, stream_dir: Path, pending: queue.Queue, stop: threading.Event):
        super().__init__(daemon=True)
        self.stream_dir = stream_dir
        self.pending = pending
        self.stop = stop
        self.last_key = None
        self.loaded = 0
        self.dropped = 0

    def run(self) -> None:
        source = self.stream_dir / "latest.json"
        while not self.stop.is_set():
            try:
                first = json.loads(source.read_text(encoding="utf-8"))
                key = (int(first["episode"]), int(first["sequence"]))
                if key == self.last_key:
                    time.sleep(0.002)
                    continue
                rgb = cv2.imread(first["rgb"], cv2.IMREAD_COLOR)
                depth = np.load(first["depth"], allow_pickle=False).astype(np.float32)
                mask_u8 = cv2.imread(first["mask"], cv2.IMREAD_GRAYSCALE)
                second = json.loads(source.read_text(encoding="utf-8"))
                second_key = (int(second["episode"]), int(second["sequence"]))
            except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError, TypeError):
                time.sleep(0.002)
                continue
            if rgb is None or mask_u8 is None or second_key != key:
                continue
            sample = {
                "key": key,
                "metadata": first,
                "rgb": rgb,
                "gt_depth": depth,
                "mask_u8": mask_u8,
                "reader_snapshot_time_ns": time.time_ns(),
            }
            try:
                self.pending.put_nowait(sample)
            except queue.Full:
                try:
                    self.pending.get_nowait()
                    self.dropped += 1
                except queue.Empty:
                    pass
                self.pending.put_nowait(sample)
            self.loaded += 1
            self.last_key = key


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--stream-dir", type=Path, required=True)
    parser.add_argument("--jsonl", type=Path, required=True)
    parser.add_argument("--debug-dir", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--max-output-hz", type=float, default=0.0)
    parser.add_argument("--output-ring-size", type=int, default=16)
    parser.add_argument(
        "--fp-ack-file",
        type=Path,
        help="Optional P5d handshake: wait for FP to ACK each publication before the next DA inference.",
    )
    args = parser.parse_args()

    actual_sha = sha256(args.checkpoint)
    if actual_sha != EXPECTED_SHA:
        raise RuntimeError(f"P5a checkpoint SHA mismatch: {actual_sha}")
    args.debug_dir.mkdir(parents=True, exist_ok=True)
    args.jsonl.parent.mkdir(parents=True, exist_ok=True)
    paired_dir = args.stream_dir / "p5c_paired"
    paired_dir.mkdir(parents=True, exist_ok=True)
    model, input_size, saved_args = load_model(args.repo_root, args.checkpoint, args.device)
    latest_out = args.stream_dir / "latest_da.json"
    pending: queue.Queue = queue.Queue(maxsize=1)
    stop = threading.Event()
    reader = LatestFrameReader(args.stream_dir, pending, stop)
    reader.start()
    published = 0
    last_publish_monotonic = 0.0
    last_published_key = None
    last_ack_wait_s = 0.0
    print(
        f"P5C_DA_READY sha={actual_sha} encoder={saved_args['encoder']} "
        f"input={input_size} precision={saved_args['precision']} queue=latest-only",
        flush=True,
    )
    try:
        with torch.inference_mode():
            while not args.stop_file.exists():
                if args.fp_ack_file is not None and last_published_key is not None:
                    ack_wait_started = time.monotonic()
                    while not args.stop_file.exists():
                        try:
                            ack = json.loads(args.fp_ack_file.read_text(encoding="utf-8"))
                            ack_key = (int(ack["episode"]), int(ack["sequence"]))
                        except (FileNotFoundError, json.JSONDecodeError, KeyError):
                            ack_key = None
                        if ack_key == last_published_key:
                            break
                        time.sleep(0.002)
                    last_ack_wait_s = time.monotonic() - ack_wait_started
                    if args.stop_file.exists():
                        break
                try:
                    sample = pending.get(timeout=0.05)
                except queue.Empty:
                    continue
                if args.max_output_hz > 0.0:
                    wait_s = 1.0 / args.max_output_hz - (time.monotonic() - last_publish_monotonic)
                    if wait_s > 0:
                        time.sleep(wait_s)
                    while True:
                        try:
                            sample = pending.get_nowait()
                        except queue.Empty:
                            break
                key = sample["key"]
                rgb_bgr = sample["rgb"]
                gt_depth = sample["gt_depth"]
                mask_u8 = sample["mask_u8"]
                started_ns = time.time_ns()
                tensor = preprocess(rgb_bgr, input_size).to(args.device)
                torch.cuda.synchronize() if str(args.device).startswith("cuda") else None
                inference_start_ns = time.time_ns()
                square = model(tensor)[0, 0].float().cpu().numpy()
                torch.cuda.synchronize() if str(args.device).startswith("cuda") else None
                inference_end_ns = time.time_ns()
                da_depth = cv2.resize(
                    square, (rgb_bgr.shape[1], rgb_bgr.shape[0]), interpolation=cv2.INTER_LINEAR
                ).astype(np.float32)

                slot = published % args.output_ring_size
                prefix = paired_dir / f"slot_{slot:02d}"
                rgb_path = prefix.with_name(prefix.name + "_rgb.png")
                mask_path = prefix.with_name(prefix.name + "_mask.png")
                gt_path = prefix.with_name(prefix.name + "_gt.npy")
                da_path = prefix.with_name(prefix.name + "_da.npy")
                atomic_png(rgb_path, rgb_bgr)
                atomic_png(mask_path, mask_u8)
                atomic_npy(gt_path, gt_depth)
                atomic_npy(da_path, da_depth)
                processed_ns = time.time_ns()
                metadata = sample["metadata"]
                payload = dict(metadata)
                payload.update({
                    "rgb": str(rgb_path),
                    "mask": str(mask_path),
                    "depth": str(da_path),
                    "gt_depth": str(gt_path),
                    "depth_source": "p5a_new_da",
                    "pairing_contract": "p5c_immutable_rgb_mask_gt_da_same_capture",
                    "da_checkpoint_sha256": actual_sha,
                    "reader_snapshot_time_ns": int(sample["reader_snapshot_time_ns"]),
                    "da_started_time_ns": started_ns,
                    "da_inference_start_time_ns": inference_start_ns,
                    "da_inference_end_time_ns": inference_end_ns,
                    "da_processed_time_ns": processed_ns,
                    "da_inference_s": (inference_end_ns - inference_start_ns) / 1.0e9,
                    "da_total_s": (processed_ns - started_ns) / 1.0e9,
                    "capture_to_da_start_ms": (started_ns - int(metadata["capture_time_ns"])) / 1.0e6,
                    "capture_to_da_publish_ms": (processed_ns - int(metadata["capture_time_ns"])) / 1.0e6,
                    "reader_loaded_total": reader.loaded,
                    "reader_dropped_total": reader.dropped,
                    "output_ring_slot": slot,
                    "fp_ack_timeslot_enabled": args.fp_ack_file is not None,
                    "previous_fp_ack_wait_s": last_ack_wait_s,
                })
                atomic_json(latest_out, payload)
                append_jsonl(args.jsonl, payload)
                if key[1] == 0:
                    save_first_frame_debug(
                        args.debug_dir, key[0], rgb_bgr, gt_depth, da_depth, mask_u8 > 0
                    )
                published += 1
                last_published_key = key
                last_publish_monotonic = time.monotonic()
                print(
                    f"P5C_DA episode={key[0]} seq={key[1]} "
                    f"infer={(inference_end_ns-inference_start_ns)/1e9:.3f}s "
                    f"age={(processed_ns-int(metadata['capture_time_ns']))/1e6:.1f}ms "
                    f"dropped={reader.dropped}",
                    flush=True,
                )
    finally:
        stop.set()
        reader.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
