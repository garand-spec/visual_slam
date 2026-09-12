#!/usr/bin/env python3
"""Build observed surface meshes from stereo depth and final SLAM poses.

No completion of unseen surfaces, no cross-map stitching, no network dependencies.
Requires NumPy and OpenCV; normal Jetson SLAM tool environment already supplies both.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np


def atomic_json(path: Path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    os.replace(temp, path)


def pose_matrix(row):
    q = np.array([float(row[k]) for k in ('qx','qy','qz','qw')], dtype=np.float64)
    t = np.array([float(row[k]) for k in ('tx','ty','tz')], dtype=np.float64)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or not np.isfinite(t).all() or not 1e-8 < norm < 1e8:
        raise ValueError('invalid pose')
    x,y,z,w = q/norm
    r = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return r.T, -r.T @ t


def depth_surface(depth, gray, calibration, rotation, translation, stride=3):
    """Triangulate adjacent supported image samples, rejecting depth edges."""
    if depth.dtype != np.uint16 or depth.shape != gray.shape or depth.ndim != 2:
        raise ValueError('depth must be uint16 millimetres matching the grayscale image')
    if depth.shape != (calibration['height'], calibration['width']):
        raise ValueError('depth dimensions do not match calibration')
    vv, uu = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride,::stride].astype(np.float64)/calibration['depth_scale']
    valid = (z > 0) & np.isfinite(z)
    xyz = np.stack(((uu-calibration['cx'])*z/calibration['fx'],
                    (vv-calibration['cy'])*z/calibration['fy'], z),axis=-1).reshape(-1,3)
    rrc = np.asarray(calibration['rectified_to_camera']).reshape(3,3)
    world = xyz @ rrc.T @ rotation.T + translation
    ids = np.arange(z.size).reshape(z.shape)
    a,b,c,d = ids[:-1,:-1],ids[:-1,1:],ids[1:,:-1],ids[1:,1:]
    faces = np.concatenate((np.stack((a,c,b),-1).reshape(-1,3),
                            np.stack((b,c,d),-1).reshape(-1,3)))
    faces = faces[valid.reshape(-1)[faces].all(axis=1)]
    if not len(faces): return np.empty((0,3)),np.empty((0,3),np.int32),np.empty(0)
    depths = z.reshape(-1)[faces]
    edges = np.stack([np.linalg.norm(world[faces[:,i]]-world[faces[:,(i+1)%3]],axis=1) for i in range(3)],axis=1)
    keep = (np.ptp(depths,axis=1) <= .04+.03*depths.min(axis=1)) & (edges.max(axis=1) <= .06+.05*depths.min(axis=1))
    faces = faces[keep]
    used, inverse = np.unique(faces, return_inverse=True)
    return world[used], inverse.reshape(-1,3).astype(np.int32), gray[::stride,::stride].reshape(-1)[used]


def fuse_surfaces(surfaces, voxel=.02):
    vertices,faces,colors=[],[],[]
    offset=0
    for xyz,tri,color in surfaces:
        if not len(tri):continue
        vertices.append(xyz);faces.append(tri+offset);colors.append(color);offset+=len(xyz)
    if not vertices:return np.empty((0,3)),np.empty((0,3),np.int32),np.empty(0)
    xyz=np.concatenate(vertices);tri=np.concatenate(faces);gray=np.concatenate(colors)
    keys=np.floor(xyz/voxel).astype(np.int64)
    _, inverse, counts=np.unique(keys,axis=0,return_inverse=True,return_counts=True)
    merged=np.column_stack([np.bincount(inverse,weights=xyz[:,i])/counts for i in range(3)])
    colors=np.rint(np.bincount(inverse,weights=gray)/counts).clip(0,255).astype(np.uint8)
    tri=inverse[tri]
    tri=tri[(tri[:,0]!=tri[:,1])&(tri[:,0]!=tri[:,2])&(tri[:,1]!=tri[:,2])]
    if not len(tri):return np.empty((0,3)),np.empty((0,3),np.int32),np.empty(0)
    _, unique=np.unique(np.sort(tri,axis=1),axis=0,return_index=True)
    tri=tri[unique]
    areas=np.linalg.norm(np.cross(merged[tri[:,1]]-merged[tri[:,0]],merged[tri[:,2]]-merged[tri[:,0]]),axis=1)
    tri=tri[areas>1e-9]
    used, inverse=np.unique(tri,return_inverse=True)
    return merged[used], inverse.reshape(-1,3).astype(np.int32), colors[used]


def export_mesh(directory, map_id, vertices, faces, colors):
    stem=f'scene_map_{map_id}'
    with (directory/(stem+'.ply.tmp')).open('w',encoding='ascii') as f:
        f.write(f'ply\nformat ascii 1.0\ncomment observed stereo surfaces; optimized map {map_id}\n'
                f'element vertex {len(vertices)}\nproperty float x\nproperty float y\nproperty float z\n'
                f'property uchar red\nproperty uchar green\nproperty uchar blue\nelement face {len(faces)}\n'
                'property list uchar int vertex_indices\nend_header\n')
        for p,g in zip(vertices,colors):f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {g} {g} {g}\n')
        for a,b,c in faces:f.write(f'3 {a} {b} {c}\n')
    os.replace(directory/(stem+'.ply.tmp'),directory/(stem+'.ply'))
    with (directory/(stem+'.obj.tmp')).open('w',encoding='ascii') as f:
        f.write('# Observed stereo surfaces in final SLAM map coordinates, metres\n')
        for p in vertices:f.write(f'v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
        for a,b,c in faces:f.write(f'f {a+1} {b+1} {c+1}\n')
    os.replace(directory/(stem+'.obj.tmp'),directory/(stem+'.obj'))
    # Bounded preview; full-resolution files remain available for download.
    preview_vertices,preview_faces,preview_colors=vertices,faces,colors
    preview_voxel=.02
    while len(preview_faces)>80000:
        preview_voxel*=max(1.25,float(np.sqrt(len(preview_faces)/80000)))
        preview_vertices,preview_faces,preview_colors=fuse_surfaces(
            [(vertices,faces,colors)],preview_voxel)
    atomic_json(directory/(stem+'.json'),{'map_id':map_id,'vertices':preview_vertices.round(6).tolist(),
                'faces':preview_faces.tolist(),'gray':preview_colors.tolist(),
                'full_vertices':len(vertices),'full_faces':len(faces)})
    return {'map_id':map_id,'vertices':len(vertices),'faces':len(faces),'file':stem+'.ply',
            'obj':stem+'.obj','preview':stem+'.json'}


def reconstruct(run: Path, stride=3, voxel=.02, max_frames=500):
    run=run.resolve();output=run/'models';output.mkdir(exist_ok=True)
    started=time.time()
    status={'state':'building','models':[],'message':'正在按最终位姿构建观测表面'}
    atomic_json(output/'status.json',status)
    calibration_file=run/'depth_frames/calibration.json'
    frames_file=run/'depth_frames/frames.csv'
    optimized_file=run/'optimized_poses.csv'
    if not all(p.is_file() for p in (calibration_file,frames_file,optimized_file)):
        status.update(state='unavailable',message='该记录缺少逐帧深度或最终位姿，无法可靠重建表面；请用新版重新采集。')
        atomic_json(output/'status.json',status);return status
    calibration=json.loads(calibration_file.read_text())
    with optimized_file.open() as f:optimized=list(csv.DictReader(f))
    with (run/'poses.csv').open() as f:online=list(csv.DictReader(f))
    with frames_file.open() as f:captures=list(csv.DictReader(f))
    # Only originally supported frames may use a final optimized reference pose.
    original={round(float(r['timestamp_s']),6):r for r in online}
    corrected={}
    for row in optimized:
        stamp=round(float(row['timestamp_s']),6);base=original.get(stamp)
        if base and int(base['state']) in (2,5) and int(base.get('tracked_features',0))>0:
            corrected[stamp]={**base,**row}
    # Preserve original frame order and explicit source segments in the displayed trajectory.
    fields=['frame','timestamp_s','state','map_points','tx','ty','tz','qx','qy','qz','qw','map_id','tracked_features','source_segment']
    trajectory=[]
    for base in online:
        stamp=round(float(base['timestamp_s']),6)
        if stamp in corrected:
            row={**corrected[stamp],'frame':base['frame']};trajectory.append(row)
        else:
            trajectory.append({**base,'state':4 if int(base['state']) in (2,5) else base['state'],'tracked_features':0})
    with (run/'model_poses.csv.tmp').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(trajectory)
    os.replace(run/'model_poses.csv.tmp',run/'model_poses.csv')
    samples=[r for r in captures if round(float(r['timestamp_s']),6) in corrected]
    if len(samples)>max_frames:samples=[samples[i] for i in np.linspace(0,len(samples)-1,max_frames,dtype=int)]
    groups={};skipped=0;points_budget=0
    for index,row in enumerate(samples):
        stamp=round(float(row['timestamp_s']),6);pose=corrected[stamp];map_id=int(pose['map_id'])
        depth_path=(run/'depth_frames'/row['depth']).resolve();gray_path=(run/'depth_frames'/row['image']).resolve()
        if depth_path.parent!=(run/'depth_frames').resolve() or gray_path.parent!=(run/'depth_frames').resolve():
            raise ValueError('depth path outside recording')
        depth=cv2.imread(str(depth_path),cv2.IMREAD_UNCHANGED);gray=cv2.imread(str(gray_path),cv2.IMREAD_GRAYSCALE)
        if depth is None or gray is None:skipped+=1;continue
        rotation,translation=pose_matrix(pose)
        surface=depth_surface(depth,gray,calibration,rotation,translation,stride)
        points_budget+=len(surface[0])
        if points_budget>8000000:skipped+=len(samples)-index;break
        groups.setdefault(map_id,[]).append(surface)
        if index%20==0:
            atomic_json(output/'status.json',{**status,'processed_depth_frames':index+1,'total_depth_frames':len(samples)})
    models=[]
    for map_id,surfaces in groups.items():
        vertices,faces,colors=fuse_surfaces(surfaces,voxel)
        if len(faces):models.append(export_mesh(output,map_id,vertices,faces,colors))
    models.sort(key=lambda m:m['faces'],reverse=True)
    status.update(state='ready' if models else 'unavailable',models=models,
                  primary_map=models[0]['map_id'] if models else None,
                  message='表面模型已生成；未观测区域保留空缺' if models else '没有足够的可靠深度表面，请保持双目可见、缓慢移动后重新采集。',
                  source='stereo depth + final optimized camera poses',coordinate='native final SLAM map; metres',
                  selected_depth_frames=len(samples),skipped_depth_frames=skipped,
                  stride=stride,voxel_m=voxel,elapsed_s=round(time.time()-started,3))
    atomic_json(output/'status.json',status)
    return status


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--stride',type=int,default=3)
    parser.add_argument('--voxel',type=float,default=.02)
    parser.add_argument('--max-frames',type=int,default=500)
    args=parser.parse_args()
    if args.stride<1 or args.voxel<=0 or args.max_frames<1:parser.error('parameters must be positive')
    try:
        result=reconstruct(args.run,args.stride,args.voxel,args.max_frames)
        print(json.dumps(result,ensure_ascii=False));return 0 if result['state']=='ready' else 3
    except Exception as error:
        output=args.run/'models';output.mkdir(exist_ok=True)
        atomic_json(output/'status.json',{'state':'failed','models':[],'message':str(error)})
        print(f'Model reconstruction failed: {error}');return 1


if __name__=='__main__':raise SystemExit(main())
