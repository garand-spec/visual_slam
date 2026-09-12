#!/usr/bin/env python3
"""Validate project assets, calibration files and optional SLAM run outputs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import tarfile
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass
class Check:
    name: str
    ok: bool
    detail: str
    required: bool = True


def _finite_positive(value) -> bool:
    return value is not None and math.isfinite(float(value)) and float(value) > 0


def validate_orb_yaml(path: Path) -> list[Check]:
    checks: list[Check] = []
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        return [Check(path.name, False, "cannot open OpenCV YAML")]
    try:
        required_scalars = [
            "Camera1.fx",
            "Camera1.fy",
            "Camera1.cx",
            "Camera1.cy",
            "Camera2.fx",
            "Camera2.fy",
            "Camera2.cx",
            "Camera2.cy",
            "Camera.width",
            "Camera.height",
            "Camera.fps",
            "ORBextractor.nFeatures",
        ]
        bad = [
            key
            for key in required_scalars
            if not _finite_positive(storage.getNode(key).real())
        ]
        checks.append(
            Check(
                path.name + ":intrinsics",
                not bad,
                "valid" if not bad else "invalid/missing: " + ", ".join(bad),
            )
        )
        transform = storage.getNode("Stereo.T_c1_c2").mat()
        transform_ok = (
            transform is not None
            and transform.shape == (4, 4)
            and np.isfinite(transform).all()
            and np.allclose(transform[3], [0, 0, 0, 1], atol=1e-5)
        )
        baseline = float(np.linalg.norm(transform[:3, 3])) if transform_ok else 0.0
        checks.append(
            Check(
                path.name + ":extrinsics",
                bool(transform_ok and baseline > 1e-4),
                f"baseline={baseline:.4f} m"
                if transform_ok
                else "invalid 4x4 transform",
            )
        )
    finally:
        storage.release()
    return checks


def validate_json_calibration(path: Path) -> list[Check]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        size = data["image_size"]
        parts_ok = len(size) == 2 and min(size) > 0
        for eye in ("left", "right"):
            k = np.asarray(data[eye]["K"], dtype=float)
            d = np.asarray(data[eye]["D"], dtype=float)
            parts_ok = parts_ok and k.shape == (3, 3) and d.size == 4
            parts_ok = parts_ok and np.isfinite(k).all() and np.isfinite(d).all()
        extrinsics = data.get("right_T_left", data.get("stereo"))
        if extrinsics is None:
            raise KeyError("right_T_left")
        rotation = np.asarray(extrinsics["R"], dtype=float)
        translation = np.asarray(
            extrinsics.get("T_m", extrinsics.get("T")), dtype=float
        ).reshape(-1)
        parts_ok = parts_ok and rotation.shape == (3, 3) and translation.size == 3
        baseline = float(np.linalg.norm(translation))
        return [
            Check(
                path.name,
                bool(parts_ok and baseline > 1e-4),
                f"{size[0]}x{size[1]}, baseline={baseline:.4f} m",
            )
        ]
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        return [Check(path.name, False, str(error))]


def ply_vertex_count(path: Path) -> int:
    with path.open("rb") as file:
        for raw in file:
            line = raw.decode("ascii", errors="strict").strip()
            if line.startswith("element vertex "):
                return int(line.rsplit(" ", 1)[1])
            if line == "end_header":
                break
    raise ValueError("PLY has no vertex declaration")


def validate_run(run_dir: Path) -> list[Check]:
    checks: list[Check] = []
    required = ["poses.csv", "tracking_health.csv", "map.ply", "CameraTrajectory.txt"]
    for name in required:
        path = run_dir / name
        checks.append(
            Check(
                "run:" + name,
                path.is_file() and path.stat().st_size > 0,
                f"{path.stat().st_size} bytes" if path.is_file() else "missing",
            )
        )
    poses = run_dir / "poses.csv"
    if poses.is_file():
        with poses.open(encoding="utf-8", newline="") as file:
            rows = list(csv.DictReader(file))
        finite_rows = sum(
            all(
                math.isfinite(float(row[key]))
                for key in ("tx", "ty", "tz", "qx", "qy", "qz", "qw")
            )
            for row in rows
        )
        checks.append(
            Check(
                "run:trajectory",
                bool(rows and finite_rows == len(rows)),
                f"{len(rows)} finite poses",
            )
        )
    health = run_dir / "tracking_health.csv"
    if health.is_file():
        with health.open(encoding="utf-8", newline="") as file:
            rows = list(csv.DictReader(file))
        ok_rows = sum(row.get("state_name") in {"OK", "OK_KLT"} for row in rows)
        ratio = ok_rows / len(rows) if rows else 0.0
        timings = [float(row["track_ms"]) for row in rows if row.get("track_ms")]
        mean_ms = sum(timings) / len(timings) if timings else math.inf
        checks.append(
            Check(
                "run:tracking-health",
                ratio >= 0.5,
                f"ok={ratio:.1%}, mean_track={mean_ms:.1f} ms",
            )
        )
    for name in ("map.ply", "dense_map.ply"):
        path = run_dir / name
        if path.is_file():
            try:
                count = ply_vertex_count(path)
                checks.append(
                    Check(
                        "run:" + name + ":vertices",
                        count > 0,
                        f"{count} vertices",
                        required=name == "map.ply",
                    )
                )
            except (OSError, ValueError, UnicodeError, struct.error) as error:
                checks.append(
                    Check(
                        "run:" + name + ":vertices",
                        False,
                        str(error),
                        required=name == "map.ply",
                    )
                )
    return checks


def newest_run(root: Path) -> Path | None:
    runtime = root / "data" / "runtime" / "orbslam3"
    candidates = [path for path in runtime.glob("run_*") if path.is_dir()]
    return max(candidates, key=lambda path: path.name) if candidates else None


def validate_project(root: Path, run_dir: Path | None = None) -> list[Check]:
    required_files = [
        "README.md",
        "README_SLAM.md",
        "environment.yml",
        "requirements-tools.txt",
        "pyrightconfig.json",
        ".vscode/c_cpp_properties.json",
        ".vscode/settings.json",
        "src/orbslam3_rervision.cpp",
        "src/orbslam3_stereo_dataset.cpp",
        "run_visual_slam_3d.sh",
        "run_stereo_dataset.sh",
        "configs/orbslam3_rervision_stereo_inertial.yaml",
        "configs/orbslam3_rervision_stereo_inertial_swapped.yaml",
    ]
    checks = [
        Check(
            "file:" + name,
            (root / name).is_file(),
            "present" if (root / name).is_file() else "missing",
        )
        for name in required_files
    ]
    vocabulary = root / "vendor" / "ORB_SLAM3" / "Vocabulary" / "ORBvoc.txt"
    archive = vocabulary.with_suffix(vocabulary.suffix + ".tar.gz")
    archive_ok = False
    if archive.is_file():
        try:
            with tarfile.open(archive, "r:gz") as file:
                archive_ok = any(
                    Path(member.name).name == "ORBvoc.txt"
                    for member in file.getmembers()
                )
        except tarfile.TarError:
            pass
    checks.append(
        Check(
            "ORB vocabulary",
            vocabulary.is_file() or archive_ok,
            "extracted"
            if vocabulary.is_file()
            else "valid archive"
            if archive_ok
            else "missing",
        )
    )
    for name in (
        "orbslam3_rervision_stereo_inertial.yaml",
        "orbslam3_rervision_stereo_inertial_swapped.yaml",
    ):
        checks.extend(validate_orb_yaml(root / "configs" / name))
    checks.extend(
        validate_json_calibration(
            root / "configs" / "fisheye_calibration_20260830.json"
        )
    )
    if run_dir is not None:
        checks.extend(validate_run(run_dir))
    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    default_root = Path(__file__).resolve().parents[1]
    parser.add_argument("--root", type=Path, default=default_root)
    parser.add_argument(
        "--run", type=Path, help="also validate one output run directory"
    )
    parser.add_argument(
        "--latest", action="store_true", help="validate newest local run"
    )
    args = parser.parse_args()
    root = args.root.resolve()
    run_dir = (
        args.run.resolve() if args.run else newest_run(root) if args.latest else None
    )
    if args.latest and run_dir is None:
        print("FAIL latest run: none found")
        return 1
    checks = validate_project(root, run_dir)
    for check in checks:
        label = "PASS" if check.ok else "WARN" if not check.required else "FAIL"
        print(f"{label:4} {check.name}: {check.detail}")
    failures = [check for check in checks if check.required and not check.ok]
    print(
        f"\n{len(checks) - len(failures)}/{len(checks)} checks passed; required failures={len(failures)}"
    )
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
