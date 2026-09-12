#!/usr/bin/env python3
import argparse
import glob
import json
import os

import cv2
import numpy as np


def detect(path, pattern):
    image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        return None, None
    found, corners = cv2.findChessboardCornersSB(
        image, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    )
    if not found:
        return image.shape[::-1], None
    return image.shape[::-1], corners.astype(np.float64)


def make_object_points(pattern, square_size):
    cols, rows = pattern
    points = np.zeros((1, cols * rows, 3), np.float64)
    points[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points *= square_size
    return points


def calibrate_one(object_points, image_points, image_size):
    K = np.eye(3, dtype=np.float64)
    D = np.zeros((4, 1), dtype=np.float64)
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-6)
    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        object_points,
        image_points,
        image_size,
        K,
        D,
        None,
        None,
        flags=flags,
        criteria=criteria,
    )
    return float(rms), K, D


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate synchronized stereo fisheye images"
    )
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--board-cols", type=int, default=11)
    parser.add_argument("--board-rows", type=int, default=8)
    parser.add_argument(
        "--square-size",
        type=float,
        required=True,
        help="checkerboard square size in meters, e.g. 0.025",
    )
    parser.add_argument("--output", default="configs/fisheye_calibration.json")
    parser.add_argument("--min-views", type=int, default=15)
    args = parser.parse_args()

    pattern = (args.board_cols, args.board_rows)
    left_paths = sorted(glob.glob(os.path.join(args.input_dir, "left_*.png")))
    right_paths = sorted(glob.glob(os.path.join(args.input_dir, "right_*.png")))
    right_by_name = {os.path.basename(p): p for p in right_paths}

    object_template = make_object_points(pattern, args.square_size)
    objpoints = []
    left_points = []
    right_points = []
    image_size = None
    used = []
    rejected = []

    for left_path in left_paths:
        name = os.path.basename(left_path).replace("left_", "right_", 1)
        right_path = right_by_name.get(name)
        if right_path is None:
            rejected.append((name, "missing_pair"))
            continue
        left_size, left_corners = detect(left_path, pattern)
        right_size, right_corners = detect(right_path, pattern)
        if left_corners is None or right_corners is None:
            rejected.append((name, "board_not_found_both_views"))
            continue
        if left_size != right_size:
            rejected.append((name, "image_size_mismatch"))
            continue
        if image_size is None:
            image_size = left_size
        if left_size != image_size:
            rejected.append((name, "inconsistent_image_size"))
            continue
        objpoints.append(object_template.copy())
        left_points.append(left_corners.reshape(1, -1, 2))
        right_points.append(right_corners.reshape(1, -1, 2))
        used.append(name)

    if image_size is None or len(used) < args.min_views:
        raise RuntimeError(
            f"only {len(used)} valid stereo views; need at least {args.min_views}. "
            "Capture more varied poses with the board visible in both lenses."
        )

    left_rms, K1, D1 = calibrate_one(objpoints, left_points, image_size)
    right_rms, K2, D2 = calibrate_one(objpoints, right_points, image_size)

    R = np.eye(3, dtype=np.float64)
    T = np.zeros((3, 1), dtype=np.float64)
    stereo_flags = cv2.fisheye.CALIB_FIX_INTRINSIC
    stereo_criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-6)
    stereo_rms, K1, D1, K2, D2, R, T, _, _ = cv2.fisheye.stereoCalibrate(
        objpoints,
        left_points,
        right_points,
        K1,
        D1,
        K2,
        D2,
        image_size,
        R,
        T,
        flags=stereo_flags,
        criteria=stereo_criteria,
    )

    R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
        K1,
        D1,
        K2,
        D2,
        image_size,
        R,
        T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        newImageSize=image_size,
        balance=0.0,
        fov_scale=1.0,
    )

    result = {
        "model": "opencv_fisheye",
        "image_size": [int(image_size[0]), int(image_size[1])],
        "board": {
            "inner_corners": [args.board_cols, args.board_rows],
            "square_size_m": args.square_size,
        },
        "views_used": len(used),
        "rejected_views": len(rejected),
        "rms": {"left": left_rms, "right": right_rms, "stereo": float(stereo_rms)},
        "left": {"K": K1.tolist(), "D": D1.reshape(-1).tolist()},
        "right": {"K": K2.tolist(), "D": D2.reshape(-1).tolist()},
        "right_T_left": {"R": R.tolist(), "T_m": T.reshape(-1).tolist()},
        "rectification": {
            "R1": R1.tolist(),
            "R2": R2.tolist(),
            "P1": P1.tolist(),
            "P2": P2.tolist(),
            "Q": Q.tolist(),
        },
        "used_pairs": used,
        "rejected_examples": rejected[:20],
    }
    output_path = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2)

    print(
        json.dumps(
            {
                "output": output_path,
                "views_used": len(used),
                "rms_left_px": left_rms,
                "rms_right_px": right_rms,
                "rms_stereo_px": float(stereo_rms),
                "translation_m": T.reshape(-1).tolist(),
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
