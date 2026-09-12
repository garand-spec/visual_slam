#!/usr/bin/env python3
"""Capture a mono or stereo fisheye camera on Jetson/USB/RTSP."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

WINDOW_NAME = "fire-monitor-stereo-camera"


def jetson_argus_available() -> bool:
    if not Path("/dev/media0").exists() or shutil.which("gst-inspect-1.0") is None:
        return False
    result = subprocess.run(
        ["gst-inspect-1.0", "nvarguscamerasrc"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def make_jetson_pipeline(
    sensor_id: int, width: int, height: int, fps: int, flip_method: int
) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"format=NV12, framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        "video/x-raw, format=BGRx ! videoconvert ! "
        "video/x-raw, format=BGR ! appsink drop=1 max-buffers=1 sync=false"
    )


def open_camera(args: argparse.Namespace) -> tuple[cv2.VideoCapture, str]:
    source = args.source.lower()
    if source == "auto":
        if Path(f"/dev/video{args.camera_index}").exists():
            source = "v4l2"
        elif jetson_argus_available():
            source = "jetson"
        else:
            source = "v4l2"

    if source == "jetson":
        pipeline = make_jetson_pipeline(
            args.sensor_id, args.width, args.height, args.fps, args.flip_method
        )
        print(f"Using Jetson GStreamer pipeline:\n{pipeline}")
        return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER), "jetson"

    if source == "v4l2":
        print(f"Using V4L2 camera index {args.camera_index}")
        capture = cv2.VideoCapture(args.camera_index, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        capture.set(cv2.CAP_PROP_FPS, args.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture, "v4l2"

    print(f"Using camera source: {args.source}")
    return cv2.VideoCapture(args.source), args.source


def split_stereo(
    frame: np.ndarray, layout: str, swap_eyes: bool
) -> tuple[np.ndarray, np.ndarray]:
    if layout == "side-by-side":
        split_at = frame.shape[1] // 2
        if frame.shape[1] % 2:
            raise ValueError(f"stereo width must be even, got {frame.shape[1]}")
        left, right = frame[:, :split_at], frame[:, split_at:]
    else:
        split_at = frame.shape[0] // 2
        if frame.shape[0] % 2:
            raise ValueError(f"stereo height must be even, got {frame.shape[0]}")
        left, right = frame[:split_at, :], frame[split_at:, :]
    return (right, left) if swap_eyes else (left, right)


def load_calibration(path: Optional[Path]) -> Optional[dict]:
    if path is None:
        return None
    with path.open("r", encoding="utf-8") as file:
        calibration = json.load(file)
    for eye in ("left", "right"):
        if (
            eye not in calibration
            or "K" not in calibration[eye]
            or "D" not in calibration[eye]
        ):
            raise ValueError(f"calibration must contain {eye}.K and {eye}.D")
    return calibration


def undistort_fisheye(image: np.ndarray, calibration: dict, eye: str) -> np.ndarray:
    params = calibration[eye]
    height, width = image.shape[:2]
    camera_matrix = np.asarray(params["K"], dtype=np.float64)
    distortion = np.asarray(params["D"], dtype=np.float64).reshape(4, 1)
    balance = float(params.get("balance", 0.0))
    new_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        camera_matrix, distortion, (width, height), np.eye(3), balance=balance
    )
    map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
        camera_matrix, distortion, np.eye(3), new_matrix, (width, height), cv2.CV_16SC2
    )
    return cv2.remap(
        image,
        map_x,
        map_y,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )


def add_label(image: np.ndarray, text: str) -> np.ndarray:
    result = image.copy()
    cv2.putText(
        result,
        text,
        (20, 38),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return result


def make_display(
    left: np.ndarray, right: np.ndarray, eye: str, layout: str
) -> np.ndarray:
    if eye == "left":
        return add_label(left, "LEFT")
    if eye == "right":
        return add_label(right, "RIGHT")
    left_view = add_label(left, "LEFT")
    right_view = add_label(right, "RIGHT")
    return (
        np.vstack((left_view, right_view))
        if layout == "top-bottom"
        else np.hstack((left_view, right_view))
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", default="auto", help="auto, jetson, v4l2, video file, or RTSP URL"
    )
    parser.add_argument("--camera-index", type=int, default=0, help="USB/V4L2 index")
    parser.add_argument("--sensor-id", type=int, default=0, help="Jetson CSI sensor id")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--flip-method", type=int, default=0, choices=range(8))
    parser.add_argument("--mode", choices=("mono", "stereo"), default="stereo")
    parser.add_argument(
        "--stereo-layout",
        choices=("side-by-side", "top-bottom"),
        default="side-by-side",
    )
    parser.add_argument("--eye", choices=("both", "left", "right"), default="both")
    parser.add_argument(
        "--swap-eyes", action="store_true", help="Swap the left/right channel order"
    )
    parser.add_argument(
        "--calibration", type=Path, help="JSON fisheye calibration file"
    )
    parser.add_argument("--output", type=Path, help="Save displayed/combined frames")
    parser.add_argument("--output-left", type=Path, help="Save left-eye frames")
    parser.add_argument("--output-right", type=Path, help="Save right-eye frames")
    parser.add_argument(
        "--headless", action="store_true", help="Do not open a preview window"
    )
    parser.add_argument("--max-frames", type=int, help="Stop after this many frames")
    return parser.parse_args()


def make_writer(
    path: Optional[Path], frame: np.ndarray, fps: int
) -> Optional[cv2.VideoWriter]:
    if path is None:
        return None
    path.parent.mkdir(parents=True, exist_ok=True)
    height, width = frame.shape[:2]
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    if not writer.isOpened():
        raise RuntimeError(f"unable to open output file: {path}")
    return writer


def main() -> int:
    args = parse_args()
    try:
        calibration = load_calibration(args.calibration)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"ERROR: invalid calibration: {error}", file=sys.stderr)
        return 2
    if args.calibration and args.mode != "stereo":
        print("ERROR: fisheye calibration requires --mode stereo", file=sys.stderr)
        return 2

    capture, backend = open_camera(args)
    if not capture.isOpened():
        print(
            "ERROR: unable to open the camera. Check connection, permissions, and source.",
            file=sys.stderr,
        )
        return 2

    display = not args.headless and bool(os.environ.get("DISPLAY"))
    if not args.headless and not display:
        print("DISPLAY is not set; running without a preview window.")

    output_writer: Optional[cv2.VideoWriter] = None
    left_writer: Optional[cv2.VideoWriter] = None
    right_writer: Optional[cv2.VideoWriter] = None
    frame_count = 0
    failed_reads = 0
    start_time = time.monotonic()
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                failed_reads += 1
                if failed_reads >= 30:
                    print("ERROR: camera stopped producing frames.", file=sys.stderr)
                    return 3
                time.sleep(0.05)
                continue
            failed_reads = 0

            try:
                if args.mode == "stereo":
                    left, right = split_stereo(
                        frame, args.stereo_layout, args.swap_eyes
                    )
                    if calibration is not None:
                        left = undistort_fisheye(left, calibration, "left")
                        right = undistort_fisheye(right, calibration, "right")
                    view = make_display(left, right, args.eye, args.stereo_layout)
                else:
                    left = right = frame
                    view = add_label(frame, "MONO")
            except (ValueError, cv2.error) as error:
                print(f"ERROR: frame processing failed: {error}", file=sys.stderr)
                return 4

            frame_count += 1
            elapsed = max(time.monotonic() - start_time, 1e-6)
            current_fps = frame_count / elapsed
            cv2.putText(
                view,
                f"backend={backend} frames={frame_count} fps={current_fps:.1f}",
                (20, view.shape[0] - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            if output_writer is None:
                output_writer = make_writer(args.output, view, args.fps)
                left_writer = make_writer(args.output_left, left, args.fps)
                right_writer = make_writer(args.output_right, right, args.fps)
            if output_writer is not None:
                output_writer.write(view)
            if left_writer is not None:
                left_writer.write(left)
            if right_writer is not None:
                right_writer.write(right)

            if display:
                cv2.imshow(WINDOW_NAME, view)
                if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                    break
            if args.max_frames is not None and frame_count >= args.max_frames:
                break
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        capture.release()
        for writer in (output_writer, left_writer, right_writer):
            if writer is not None:
                writer.release()
        if display:
            cv2.destroyAllWindows()

    print(f"Captured {frame_count} frame(s), mode={args.mode}, eye={args.eye}.")
    for path in (args.output, args.output_left, args.output_right):
        if path is not None:
            print(f"Saved video to {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
