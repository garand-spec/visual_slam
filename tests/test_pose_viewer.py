import json
import math
from pathlib import Path
import tempfile
import unittest

from tools.pose_viewer import inverse_pose, read_frames, read_cloud, select_cloud, Store


class PoseViewerTests(unittest.TestCase):
    def test_empty_dense_cloud_falls_back_to_sparse(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            header='ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
            (root/'dense_map.ply').write_text(header.format(0))
            (root/'map.ply').write_text(header.format(1)+'1 2 3\n')
            self.assertEqual(select_cloud(root)['source'],'map.ply')
            (root/'dense_map.ply').write_text('incomplete export')
            self.assertEqual(select_cloud(root)['points'],[[1,2,3]])
            (root/'dense_map.ply').write_text(header.format(1)+'4 5 6\n')
            self.assertEqual(select_cloud(root)['source'],'dense_map.ply')

    def test_absent_cloud_is_reported_without_fake_points(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(select_cloud(Path(d))['points'],[])

    def test_late_cloud_export_invalidates_recording_cache(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);run=root/'data/runtime/orbslam3/run_20260913_200139'
            run.mkdir(parents=True);store=Store(root)
            self.assertFalse(store.recording(run.name)['cloud_available'])
            (run/'map.ply').write_text('ply\nformat ascii 1.0\nelement vertex 0\nend_header\n')
            self.assertTrue(store.recording(run.name)['cloud_available'])

    def test_inversion_rotates_translation_not_just_negation(self):
        # Rcw is +90 degrees about Z. Camera centre is -Rcw.T * [1,0,0].
        s = math.sqrt(.5)
        p, q = inverse_pose([1, 0, 0], [0, 0, s, s])
        for actual, expected in zip(p, [0, 1, 0]):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(q[2], -s)

    def test_invalid_quaternions_are_rejected(self):
        for q in ([0, 0, 0, 0], [float('nan'), 0, 0, 1]):
            with self.assertRaises(ValueError):
                inverse_pose([0, 0, 0], q)

    def test_live_partial_rows_lost_states_and_map_ids(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'poses.csv'
            p.write_text('frame,timestamp_s,state,tx,ty,tz,qx,qy,qz,qw,map_id\n'
                         '0,1,2,1,0,0,0,0,0,1,3\n'
                         '1,2,4,0,0,0,0,0,0,1,3\n'
                         '2,3,2,2,0,0,0,0,0,1,4\n'
                         '3,4,2,1,')
            data = read_frames(p)
            self.assertEqual(len(data['frames']), 3)
            self.assertEqual(data['frames'][0]['p'], [-1, 0, 0])
            self.assertFalse(data['frames'][1]['valid'])
            self.assertEqual(data['frames'][2]['map_id'], 4)

    def test_corrupt_row_breaks_trajectory_and_old_schema_works(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'poses.csv'
            p.write_text('frame,timestamp_s,state,tx,ty,tz,qx,qy,qz,qw\n'
                         '0,1,2,0,0,0,0,0,0,1\n'
                         '1,2,2,broken,0,0,0,0,0,1\n'
                         '2,3,2,1,0,0,0,0,0,1\n')
            data = read_frames(p)
            self.assertFalse(data['map_ids_recorded'])
            self.assertEqual(data['invalid_rows'], 1)
            self.assertFalse(data['frames'][1]['valid'])

    def test_paths_cannot_escape_runtime(self):
        with tempfile.TemporaryDirectory() as d:
            store = Store(Path(d))
            for path in ['../../etc', '../run_20260912_192042', '/etc/passwd']:
                with self.assertRaises(ValueError):
                    store.run(path)

    def test_ok_without_map_support_is_not_a_position(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'poses.csv'
            p.write_text('frame,timestamp_s,state,map_points,tx,ty,tz,qx,qy,qz,qw\n'
                         '0,1,2,0,0,0,0,0,0,0,1\n')
            self.assertFalse(read_frames(p)['frames'][0]['valid'])

    def test_finished_telemetry_never_reports_active(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            run = root / 'data/runtime/orbslam3/run_20260912_192042'
            run.mkdir(parents=True)
            (run / 'viewer_state.json').write_text(json.dumps({
                'lifecycle': 'finished', 'unix_ms': 0, 'pid': 0}))
            self.assertFalse(Store(root).live(run.name)['active'])

    def test_ascii_point_cloud_sampling(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'map.ply'
            p.write_text('ply\nformat ascii 1.0\nelement vertex 5\n'
                         'property float x\nproperty float y\nproperty float z\nend_header\n'
                         + ''.join(f'{i} 0 0 220 220 220\n' for i in range(5)))
            self.assertEqual(read_cloud(p, 2), [[0, 0, 0], [3, 0, 0]])


if __name__ == '__main__':
    unittest.main()
