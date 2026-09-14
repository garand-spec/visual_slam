import csv
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np
from tools.reconstruct_scene import depth_surface, fuse_surfaces, pose_matrix, reconstruct, resolve_poses


CAL = dict(width=12, height=12, fx=100, fy=100, cx=5.5, cy=5.5,
           depth_scale=1000, rectified_to_camera=np.eye(3).reshape(-1).tolist())


def fixture(run):
    (run/'depth_frames').mkdir(parents=True)
    (run/'depth_frames/calibration.json').write_text(json.dumps(CAL))
    rows=[]
    for i in range(3):
        rows.append(dict(frame=i, timestamp_s=i+1, state=4 if i==2 else 2,
                         map_points=100, tx=-i*10, ty=0, tz=0, qx=0, qy=0, qz=0, qw=1,
                         map_id=i, tracked_features=0 if i==2 else 100, source_segment=i))
        cv2.imwrite(str(run/f'depth_frames/{i}_depth.png'), np.full((12,12),1000,np.uint16))
        cv2.imwrite(str(run/f'depth_frames/{i}_gray.png'), np.full((12,12),180,np.uint8))
    for name in ['poses.csv','optimized_poses.csv']:
        with (run/name).open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    with (run/'depth_frames/frames.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['timestamp_s','depth','image']);writer.writeheader()
        writer.writerows(dict(timestamp_s=i+1,depth=f'{i}_depth.png',image=f'{i}_gray.png') for i in range(3))


class SurfaceTests(unittest.TestCase):
    def test_partial_final_coverage_uses_one_consistent_snapshot_epoch(self):
        base=dict(frame=0,timestamp_s=1,state=2,map_id=0,map_version=8,source_segment=0,
                  tracked_features=100,tx=0,ty=0,tz=0,qx=0,qy=0,qz=0,qw=1)
        second={**base,'frame':1,'timestamp_s':2,'tx':-1}
        captures=[{**r,'depth':'depth.png','image':'gray.png'} for r in (base,second)]
        resolved,metadata,usable,audit=resolve_poses([base,second],[{**second,'tx':999}],captures)
        self.assertEqual(resolved[1]['map_id'],resolved[2]['map_id'])
        self.assertEqual(resolved[2]['tx'],-1)
        self.assertEqual(resolved[2]['pose_source'],'online_snapshot')
        self.assertEqual(audit['final_pose_matched_depth_frames'],1)

    def test_reset_and_no_overlapping_final_poses_recovers_separate_epochs(self):
        with tempfile.TemporaryDirectory() as d:
            run=Path(d);fixture(run)
            with (run/'poses.csv').open() as f:online=list(csv.DictReader(f))
            for row in online:row['map_id']='0'
            with (run/'poses.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=list(online[0]));w.writeheader();w.writerows(online)
            (run/'optimized_poses.csv').write_text('timestamp_s,map_id,tx,ty,tz,qx,qy,qz,qw\n')
            captures=[{**r,'map_version':10+i,'depth':f'{i}_depth.png','image':f'{i}_gray.png'} for i,r in enumerate(online)]
            with (run/'depth_frames/frames.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=list(captures[0]));w.writeheader();w.writerows(captures)
            result=reconstruct(run,voxel=.005)
            self.assertEqual(result['state'],'ready')
            self.assertEqual(result['final_pose_matched_depth_frames'],0)
            self.assertEqual(result['online_snapshot_depth_frames'],2)
            self.assertEqual(len(result['models']),2)
            self.assertEqual({m['map_version'] for m in result['models']},{10,11})
            self.assertTrue(all(m['pose_source']=='online_snapshot' for m in result['models']))
            self.assertTrue(all(m['map_id']!=0 for m in result['models']))
            with (run/'model_poses.csv').open() as f:trajectory=list(csv.DictReader(f))
            self.assertEqual(trajectory[0]['pose_source'],'online_snapshot')
            self.assertEqual(trajectory[2]['state'],'4')
            # Embedded snapshots remain usable even if the producer's main CSV is lost.
            (run/'poses.csv').unlink();(run/'optimized_poses.csv').unlink()
            self.assertEqual(reconstruct(run,voxel=.005)['state'],'ready')

    def test_unmatched_legacy_depth_is_not_misreported_as_missing_depth(self):
        with tempfile.TemporaryDirectory() as d:
            run=Path(d);fixture(run)
            (run/'optimized_poses.csv').write_text('timestamp_s,map_id,tx,ty,tz,qx,qy,qz,qw\n')
            result=reconstruct(run)
            self.assertEqual(result['reason'],'pose_association_failed')
            self.assertEqual(result['captured_depth_frames'],3)

    def test_partial_journal_tail_does_not_hide_completed_depth_pairs(self):
        with tempfile.TemporaryDirectory() as d:
            run=Path(d);fixture(run)
            with (run/'depth_frames/frames.csv').open('a') as f:f.write('4,unfinished')
            result=reconstruct(run,voxel=.005)
            self.assertEqual(result['state'],'ready')
            self.assertEqual(result['captured_depth_frames'],3)
            self.assertEqual(result['discarded_csv_rows'],1)

    def test_plane_and_camera_to_world_transform(self):
        depth=np.full((12,12),1000,np.uint16);gray=np.full((12,12),180,np.uint8)
        xyz,faces,_=depth_surface(depth,gray,CAL,np.eye(3),np.array([1,2,3]),3)
        self.assertEqual(len(faces),18)
        np.testing.assert_allclose(xyz[:,2],4)
        np.testing.assert_allclose(xyz[0],[.945,1.945,4])
        s=2**-.5
        rotation,translation=pose_matrix(dict(qx=0,qy=0,qz=s,qw=s,tx=1,ty=0,tz=0))
        np.testing.assert_allclose(translation,[0,1,0],atol=1e-12)

    def test_occlusion_and_missing_depth_do_not_get_bridged(self):
        depth=np.full((12,12),1000,np.uint16);depth[:,6:]=3000;depth[0,:]=0
        xyz,faces,_=depth_surface(depth,np.ones_like(depth,np.uint8),CAL,np.eye(3),np.zeros(3),1)
        self.assertGreater(len(faces),0)
        self.assertTrue(np.all(np.ptp(xyz[faces,2],axis=1)==0))
        self.assertTrue(np.all(xyz[:,2]>0))

    def test_overlapping_observations_weld_without_duplicate_faces(self):
        surface=depth_surface(np.full((12,12),1000,np.uint16),np.ones((12,12),np.uint8),CAL,np.eye(3),np.zeros(3),3)
        xyz,faces,_=fuse_surfaces([surface,surface],.005)
        self.assertEqual(len(xyz),len(surface[0]));self.assertEqual(len(faces),18)
        self.assertTrue(np.all(faces>=0));self.assertLess(faces.max(),len(xyz))

    def test_final_maps_remain_separate_and_lost_frames_excluded(self):
        with tempfile.TemporaryDirectory() as d:
            run=Path(d);fixture(run);result=reconstruct(run,voxel=.005)
            self.assertEqual(result['state'],'ready')
            self.assertEqual({m['map_id'] for m in result['models']},{0,1})
            self.assertEqual(result['selected_depth_frames'],2)
            model=json.loads((run/'models/scene_map_1.json').read_text())
            self.assertGreater(min(v[0] for v in model['vertices']),9)
            self.assertTrue((run/'models/scene_map_1.obj').read_text().startswith('# Observed'))
            with (run/'model_poses.csv').open() as f: rows=list(csv.DictReader(f))
            self.assertEqual(rows[2]['state'],'4')
            self.assertEqual(rows[1]['source_segment'],'1')

    def test_old_recording_reports_missing_evidence(self):
        with tempfile.TemporaryDirectory() as d:
            result=reconstruct(Path(d))
            self.assertEqual(result['state'],'unavailable');self.assertEqual(result['models'],[])


if __name__=='__main__':unittest.main()
