#!/usr/bin/env python3
"""Convert a side-by-side stereo video into an ORB-SLAM3 image sequence."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import cv2


def split_stereo_frame(frame, swap_eyes: bool = False):
    """Return independent left/right images from an even-width SBS frame."""
    if frame is None or frame.ndim not in (2, 3) or frame.shape[1] < 2:
        raise ValueError("invalid stereo frame")
    width = int(frame.shape[1])
    if width % 2:
        frame = frame[:, : width - 1]
        width -= 1
    first = frame[:, : width // 2].copy()
    second = frame[:, width // 2 :].copy()
    return (second, first) if swap_eyes else (first, second)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Split a side-by-side video into synchronized stereo images."
    )
    parser.add_argument("input", help="input side-by-side video")
    parser.add_argument(
        "--output", required=True, help="new or empty dataset directory"
    )
    parser.add_argument(
        "--swap-eyes",
        action="store_true",
        help="store the raw second half as LEFT (RERVISION default)",
    )
    parser.add_argument("--every", type=int, default=1, help="keep every Nth frame")
    parser.add_argument("--max-frames", type=int, default=0, help="0 keeps all frames")
    parser.add_argument("--start-seconds", type=float, default=0.0)
    parser.add_argument("--png-compression", type=int, default=3, choices=range(10))
    return parser.parse_args()


def prepare_dataset(args: argparse.Namespace) -> dict:
    source = Path(args.input).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"input video not found: {source}")
    if args.every < 1 or args.max_frames < 0 or args.start_seconds < 0:
        raise ValueError(
            "--every must be positive; time/frame limits cannot be negative"
        )
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"output directory is not empty: {output}")

    left_dir = output / "left"
    right_dir = output / "right"
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f"OpenCV cannot open: {source}")
    source_fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not source_fps or source_fps <= 0:
        source_fps = 30.0
    capture.set(cv2.CAP_PROP_POS_MSEC, args.start_seconds * 1000.0)

    timestamps: list[float] = []
    source_indices: list[int] = []
    seen = 0
    saved = 0
    image_size = None
    parameters = [cv2.IMWRITE_PNG_COMPRESSION, args.png_compression]
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            source_index = int(capture.get(cv2.CAP_PROP_POS_FRAMES)) - 1
            if seen % args.every:
                seen += 1
                continue
            left, right = split_stereo_frame(frame, args.swap_eyes)
            name = f"{saved:06d}.png"
            if not cv2.imwrite(str(left_dir / name), left, parameters):
                raise RuntimeError(f"failed to write left/{name}")
            if not cv2.imwrite(str(right_dir / name), right, parameters):
                raise RuntimeError(f"failed to write right/{name}")
            timestamp = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000.0
            if timestamp <= 0:
                timestamp = source_index / source_fps
            timestamps.append(timestamp)
            source_indices.append(source_index)
            image_size = [int(left.shape[1]), int(left.shape[0])]
            saved += 1
            seen += 1
            if args.max_frames and saved >= args.max_frames:
                break
    finally:
        capture.release()

    if not saved:
        raise RuntimeError("the input contains no readable frames")
    with (output / "timestamps.txt").open("w", encoding="utf-8", newline="\n") as file:
        for timestamp in timestamps:
            file.write(f"{timestamp:.9f}\n")
    manifest = {
        "format": "visual-slam-stereo-dataset-v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source": str(source),
        "source_fps": source_fps,
        "frame_count": saved,
        "image_size": image_size,
        "swap_eyes": bool(args.swap_eyes),
        "sample_every": args.every,
        "source_frame_first": source_indices[0],
        "source_frame_last": source_indices[-1],
        "left": "left",
        "right": "right",
        "timestamps": "timestamps.txt",
    }
    with (output / "dataset.json").open("w", encoding="utf-8", newline="\n") as file:
        json.dump(manifest, file, indent=2, ensure_ascii=False)
        file.write("\n")
    return manifest


def main() -> int:
    args = parse_args()
    try:
        manifest = prepare_dataset(args)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}")
        return 2
    print(
        f"dataset ready: {Path(args.output).resolve()} "
        f"frames={manifest['frame_count']} size={manifest['image_size']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
