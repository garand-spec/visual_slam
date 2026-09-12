#!/usr/bin/env python3
import argparse
import ctypes
import os
import queue
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_DIR = os.path.join(
    os.path.dirname(SCRIPT_DIR), "vendor", "rervision_single_imu", "python"
)
sys.path.insert(0, SDK_DIR)
from scam_sdk import CamFormat, ScamSDK  # noqa: E402


class LatestFrameQueue:
    def __init__(self):
        self.q = queue.Queue(maxsize=1)
        self.dropped = 0

    def put(self, item):
        try:
            self.q.put_nowait(item)
        except queue.Full:
            try:
                self.q.get_nowait()
            except queue.Empty:
                pass
            self.dropped += 1
            try:
                self.q.put_nowait(item)
            except queue.Full:
                self.dropped += 1

    def get(self, timeout=0.2):
        return self.q.get(timeout=timeout)


class DetectionState:
    def __init__(self):
        self.lock = threading.Lock()
        self.left_found = False
        self.left_corners = None
        self.right_found = False
        self.right_corners = None
        self.updated_at = 0.0


def copy_stereo_bgr(frame, swap_eyes: bool = False):
    if frame.format != CamFormat.FORMAT_RGB24 or not frame._raw_data_ptr_val:
        return None
    width, height = int(frame.width), int(frame.height)
    count = width * height * 3
    raw = np.ctypeslib.as_array(
        (ctypes.c_uint8 * count).from_address(frame._raw_data_ptr_val)
    ).reshape((height, width, 3))
    bgr = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
    half = width // 2
    first = bgr[:, :half].copy()
    second = bgr[:, half : half * 2].copy()
    return (second, first) if swap_eyes else (first, second)


def detect_board(image, pattern, detect_width=1280):
    original_width = image.shape[1]
    scale = min(1.0, float(detect_width) / float(original_width))
    if scale < 1.0:
        small = cv2.resize(
            image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
        )
    else:
        small = image
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    candidates = [gray, cv2.equalizeHist(gray)]
    sb_flags = (
        cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    )
    for candidate in candidates:
        found, corners = cv2.findChessboardCornersSB(candidate, pattern, flags=sb_flags)
        if found:
            if scale < 1.0:
                corners = corners / scale
            return True, corners

    classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags=classic_flags)
    if found:
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)
        if scale < 1.0:
            corners = corners / scale
        return True, corners
    return False, None


def draw_view(image, found, corners, pattern, label):
    view = cv2.resize(image, (640, 400), interpolation=cv2.INTER_AREA)
    if found and corners is not None:
        scale_x = view.shape[1] / image.shape[1]
        scale_y = view.shape[0] / image.shape[0]
        scaled = corners.copy()
        scaled[:, :, 0] *= scale_x
        scaled[:, :, 1] *= scale_y
        cv2.drawChessboardCorners(view, pattern, scaled, True)
    cv2.putText(
        view,
        f"{label}: {'FOUND' if found else 'searching'}",
        (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 220, 0) if found else (0, 100, 255),
        2,
        cv2.LINE_AA,
    )
    return view


def main():
    parser = argparse.ArgumentParser(
        description="Capture synchronized fisheye calibration pairs"
    )
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--format-index", type=int, default=0)
    parser.add_argument(
        "--board-cols",
        type=int,
        default=11,
        help="inner corners horizontally; 12 squares means 11 corners",
    )
    parser.add_argument(
        "--board-rows",
        type=int,
        default=8,
        help="inner corners vertically; 9 squares means 8 corners",
    )
    parser.add_argument(
        "--output-dir",
        default=os.path.join(SCRIPT_DIR, "..", "data", "calibration", "raw"),
    )
    parser.add_argument(
        "--swap-eyes",
        action="store_true",
        help="store the raw second half as left_* (matches the swapped runtime config)",
    )
    parser.add_argument(
        "--detect-width",
        type=int,
        default=1280,
        help="width used by the background detector",
    )
    parser.add_argument(
        "--detect-interval",
        type=float,
        default=0.25,
        help="seconds between background board searches",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="stop automatically after this many seconds; 0 means until q",
    )
    args = parser.parse_args()

    pattern = (args.board_cols, args.board_rows)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.abspath(os.path.join(args.output_dir, "run_" + stamp))
    os.makedirs(output_dir, exist_ok=True)

    frames = LatestFrameQueue()
    detect_queue = queue.Queue(maxsize=1)
    detect_stop = threading.Event()
    detection = DetectionState()
    callback_errors = []

    def on_frame(frame):
        try:
            copied = copy_stereo_bgr(frame, args.swap_eyes)
            if copied is not None:
                frames.put(copied)
        except Exception as exc:
            callback_errors.append(str(exc))

    def detector_loop():
        while not detect_stop.is_set():
            try:
                left, right = detect_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                left_found, left_corners = detect_board(
                    left, pattern, args.detect_width
                )
                right_found, right_corners = detect_board(
                    right, pattern, args.detect_width
                )
                with detection.lock:
                    detection.left_found = left_found
                    detection.left_corners = left_corners
                    detection.right_found = right_found
                    detection.right_corners = right_corners
                    detection.updated_at = time.monotonic()
            except Exception as exc:
                callback_errors.append(f"detector: {exc}")

    detector = threading.Thread(
        target=detector_loop, name="checkerboard-detector", daemon=True
    )
    detector.start()

    sdk = ScamSDK()
    opened = False
    saved = 0
    processed = 0
    start_loop = time.monotonic()
    try:
        if not sdk.initialize():
            raise RuntimeError("SCAM_Initialize failed")
        devices = sdk.enum_devices()
        if args.device >= len(devices):
            raise RuntimeError(
                f"device {args.device} unavailable; enumerated {len(devices)}"
            )
        formats = sdk.get_device_formats(args.device)
        if not (0 <= args.format_index < len(formats)):
            raise RuntimeError(
                f"format index {args.format_index} unavailable; got {len(formats)} formats"
            )
        sdk.set_device_format(args.device, args.format_index)
        sdk.set_image_format(args.device, CamFormat.FORMAT_RGB24)
        sdk.open_device(args.device, on_frame)
        opened = True

        cv2.namedWindow("Fisheye Calibration - LEFT | RIGHT", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Fisheye Calibration - LEFT | RIGHT", 1280, 460)
        print("Calibration capture started.")
        print("Place one rigid checkerboard in view of both lenses.")
        print(
            "Press c to save a pair when both boards show FOUND; press q or ESC to finish."
        )
        print(f"Output: {output_dir}")
        deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
        last_detection_submit = 0.0
        latest_pair = None

        while deadline is None or time.monotonic() < deadline:
            try:
                left, right = frames.get(timeout=0.2)
            except queue.Empty:
                continue
            processed += 1
            latest_pair = (left, right)
            now = time.monotonic()
            if now - last_detection_submit >= max(0.05, args.detect_interval):
                try:
                    detect_queue.put_nowait((left, right))
                    last_detection_submit = now
                except queue.Full:
                    pass

            with detection.lock:
                fresh = (now - detection.updated_at) <= max(
                    1.0, args.detect_interval * 4.0
                )
                left_found = fresh and detection.left_found
                left_corners = detection.left_corners
                right_found = fresh and detection.right_found
                right_corners = detection.right_corners

            left_view = draw_view(left, left_found, left_corners, pattern, "LEFT")
            right_view = draw_view(right, right_found, right_corners, pattern, "RIGHT")
            combined = np.hstack((left_view, right_view))
            cv2.putText(
                combined,
                f"saved pairs: {saved} | c=capture q=quit",
                (12, 450),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("Fisheye Calibration - LEFT | RIGHT", combined)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("c"):
                if not (left_found and right_found) or latest_pair is None:
                    print("Both boards must be FOUND before capture.")
                    continue
                save_left, save_right = latest_pair
                cv2.imwrite(
                    os.path.join(output_dir, f"left_{saved:04d}.png"), save_left
                )
                cv2.imwrite(
                    os.path.join(output_dir, f"right_{saved:04d}.png"), save_right
                )
                saved += 1
                print(f"saved pair {saved}")
    finally:
        detect_stop.set()
        detector.join(timeout=2.0)
        if opened:
            try:
                sdk.close_device(args.device)
            except Exception:
                pass
        try:
            sdk.release()
        except Exception:
            pass
        cv2.destroyAllWindows()

    elapsed = max(time.monotonic() - start_loop, 1e-6)
    print(f"processed_frames={processed}")
    print(f"processing_fps={processed / elapsed:.2f}")
    print(f"queue_dropped={frames.dropped}")
    print(f"saved_pairs={saved}")
    print(f"output_dir={output_dir}")
    if callback_errors:
        print(f"callback_errors={callback_errors[-3:]}", file=sys.stderr)


if __name__ == "__main__":
    main()
