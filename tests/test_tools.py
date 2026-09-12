from pathlib import Path
from argparse import Namespace
import tempfile
import unittest

import cv2
import numpy as np

from tools.prepare_stereo_dataset import prepare_dataset, split_stereo_frame
from tools.validate_project import ply_vertex_count, validate_project


ROOT = Path(__file__).resolve().parents[1]


class StereoSplitTests(unittest.TestCase):
    def test_split_and_swap(self):
        frame = np.zeros((3, 8, 3), dtype=np.uint8)
        frame[:, :4] = 10
        frame[:, 4:] = 20
        left, right = split_stereo_frame(frame)
        self.assertTrue(np.all(left == 10))
        self.assertTrue(np.all(right == 20))
        left, right = split_stereo_frame(frame, swap_eyes=True)
        self.assertTrue(np.all(left == 20))
        self.assertTrue(np.all(right == 10))

    def test_odd_column_is_safely_trimmed(self):
        frame = np.zeros((2, 9), dtype=np.uint8)
        left, right = split_stereo_frame(frame)
        self.assertEqual(left.shape, (2, 4))
        self.assertEqual(right.shape, (2, 4))

    def test_video_to_dataset_end_to_end(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            video_path = root / "stereo.avi"
            writer = cv2.VideoWriter(
                str(video_path), cv2.VideoWriter_fourcc(*"MJPG"), 10.0, (128, 48)
            )
            self.assertTrue(writer.isOpened())
            for index in range(8):
                frame = np.zeros((48, 128, 3), dtype=np.uint8)
                cv2.circle(frame, (15 + index * 2, 24), 8, (255, 255, 255), -1)
                cv2.circle(frame, (79 + index * 2, 24), 8, (180, 180, 180), -1)
                writer.write(frame)
            writer.release()
            output = root / "dataset"
            manifest = prepare_dataset(
                Namespace(
                    input=str(video_path),
                    output=str(output),
                    swap_eyes=True,
                    every=2,
                    max_frames=0,
                    start_seconds=0.0,
                    png_compression=1,
                )
            )
            self.assertEqual(manifest["frame_count"], 4)
            self.assertEqual(manifest["image_size"], [64, 48])
            self.assertEqual(len(list((output / "left").glob("*.png"))), 4)
            self.assertEqual(
                len((output / "timestamps.txt").read_text().splitlines()), 4
            )


class ValidationTests(unittest.TestCase):
    def test_project_assets_and_calibration(self):
        failures = [
            check for check in validate_project(ROOT) if check.required and not check.ok
        ]
        self.assertEqual(
            [], failures, "\n".join(f"{x.name}: {x.detail}" for x in failures)
        )

    def test_ply_vertex_header(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.ply"
            path.write_text(
                "ply\nformat ascii 1.0\nelement vertex 42\nend_header\n",
                encoding="ascii",
            )
            self.assertEqual(ply_vertex_count(path), 42)


if __name__ == "__main__":
    unittest.main()
