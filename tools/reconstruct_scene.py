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


def export_mesh(directory, map_id, vertices, faces, colors, provenance=None):
    provenance=provenance or {"pose_source":"optimized","source_map_id":map_id}
    quality=provenance["pose_source"]
    stem=f'scene_map_{map_id}'
    with (directory/(stem+'.ply.tmp')).open('w',encoding='ascii') as f:
        f.write(f'ply\nformat ascii 1.0\ncomment observed stereo surfaces; {quality} group {map_id}\n'
                f'element vertex {len(vertices)}\nproperty float x\nproperty float y\nproperty float z\n'
                f'property uchar red\nproperty uchar green\nproperty uchar blue\nelement face {len(faces)}\n'
                'property list uchar int vertex_indices\nend_header\n')
        for p,g in zip(vertices,colors):f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {g} {g} {g}\n')
        for a,b,c in faces:f.write(f'3 {a} {b} {c}\n')
    os.replace(directory/(stem+'.ply.tmp'),directory/(stem+'.ply'))
    with (directory/(stem+'.obj.tmp')).open('w',encoding='ascii') as f:
        f.write(f'# Observed stereo surfaces; {quality} coordinates, metres\n')
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
                'full_vertices':len(vertices),'full_faces':len(faces),**provenance})
    return {'map_id':map_id,'vertices':len(vertices),'faces':len(faces),'file':stem+'.ply',
            'obj':stem+'.obj','preview':stem+'.json',**provenance}


def stamp(row):
    return round(float(row['timestamp_s']),6)


def supported(row):
    try:
        if int(row.get('state',-1)) not in (2,5) or int(row.get('tracked_features',0))<=0:return False
        pose_matrix(row)
        return int(row.get('map_id',-1))>=0
    except (ValueError,KeyError,TypeError):return False


def resolve_poses(online, optimized, captures):
    """Prefer final coordinates; retain complete fallback epochs in separate groups.

    Never mix online and final poses in one mesh, even when map IDs are reused.
    Embedded capture poses also work when a crash leaves the main CSV incomplete.
    """
    original={stamp(r):r for r in online}
    bases={}
    epochs={}
    segments={}
    for capture in captures:
        t=stamp(capture);base=original.get(t,capture)
        if not supported(base):continue
        if capture.get('map_id','')!='' and int(capture['map_id'])!=int(base['map_id']):continue
        if capture.get('frame','')!='' and base.get('frame','')!='' and int(capture['frame'])!=int(base['frame']):continue
        snapshot={**base,**{k:capture[k] for k in ('tx','ty','tz','qx','qy','qz','qw') if k in capture}}
        if not supported(snapshot):continue
        bases[t]=snapshot
        if capture.get('map_version','')!='':
            epoch=(int(capture.get('map_id',base['map_id'])),int(capture['map_version']))
            epochs.setdefault(epoch,[]).append(t)
            if base.get('source_segment','')!='':
                segments.setdefault((int(base['map_id']),int(base['source_segment'])),set()).add(epoch)
    corrected={}
    for row in optimized:
        t=stamp(row);base=original.get(t,bases.get(t))
        if base and supported(base):
            merged={**base,**row,'frame':base.get('frame',row.get('frame',0)),'pose_source':'optimized'}
            if supported(merged):corrected[t]=merged
    resolved=dict(corrected)
    metadata={int(r['map_id']):{'pose_source':'optimized','source_map_id':int(r['map_id'])} for r in corrected.values()}
    next_id=max([int(r.get('map_id',-1)) for r in online+optimized+captures]+[-1])+1
    fallback={}
    for epoch,times in sorted(epochs.items()):
        if any(t not in corrected for t in times):
            fallback[epoch]=next_id
            metadata[next_id]={'pose_source':'online_snapshot','source_map_id':epoch[0],'map_version':epoch[1],
                'warning':'采集位姿保留结果，未完成最终优化；独立地图，不与优化地图拼接。'}
            next_id+=1
    for base in online:
        if not supported(base):continue
        epoch=None
        if base.get('map_version','')!='':epoch=(int(base['map_id']),int(base['map_version']))
        elif base.get('source_segment','')!='':
            choices=segments.get((int(base['map_id']),int(base['source_segment'])),set())
            if len(choices)==1:epoch=next(iter(choices))
        if epoch in fallback:
            resolved[stamp(base)]={**base,'map_id':fallback[epoch],'pose_source':'online_snapshot'}
    for epoch,times in epochs.items():
        if epoch in fallback:
            for t in times:resolved[t]={**bases[t],'map_id':fallback[epoch],'pose_source':'online_snapshot'}
    usable=[r for r in captures if stamp(r) in resolved and stamp(r) in bases]
    return resolved,metadata,usable,{'captured_depth_frames':len(captures),
        'final_pose_matched_depth_frames':sum(stamp(r) in corrected for r in captures),
        'usable_depth_frames':len(usable),
        'unmatched_depth_frames':len(captures)-len(usable)}


def reconstruct(run: Path, stride=3, voxel=.02, max_frames=500):
    run=run.resolve();output=run/'models';output.mkdir(exist_ok=True)
    started=time.time()
    status={'state':'building','models':[],'message':'正在匹配深度和位姿并构建观测表面'}
    atomic_json(output/'status.json',status)
    calibration_file=run/'depth_frames/calibration.json';frames_file=run/'depth_frames/frames.csv'
    if not all(p.is_file() for p in (calibration_file,frames_file)):
        status.update(state='unavailable',reason='missing_depth_archive',message='该记录缺少逐帧深度，无法重建表面；请重新采集。')
        atomic_json(output/'status.json',status);return status
    calibration=json.loads(calibration_file.read_text())
    discarded_rows=0
    def rows(path):
        nonlocal discarded_rows
        if not path.exists():return []
        text=path.read_text()
        if text and not text.endswith('\n'):
            text=text[:text.rfind('\n')+1];discarded_rows+=1
        result=[]
        for row in csv.DictReader(text.splitlines()):
            try:
                if not np.isfinite(float(row['timestamp_s'])):raise ValueError('timestamp')
                if None in row.values():raise ValueError('incomplete row')
                result.append(row)
            except (KeyError,TypeError,ValueError):discarded_rows+=1
        return result
    captures=rows(frames_file);online=rows(run/'poses.csv');optimized=rows(run/'optimized_poses.csv')
    if not online:online=[r for r in captures if supported(r)]
    resolved,metadata,samples,audit=resolve_poses(online,optimized,captures)
    status.update(audit,discarded_csv_rows=discarded_rows)
    # Round-robin groups so smaller maps are not starved by a long first map.
    grouped={}
    for r in samples:grouped.setdefault(int(resolved[stamp(r)]['map_id']),[]).append(r)
    allocations={k:0 for k in grouped}
    for _ in range(min(max_frames,len(samples))):
        candidates=[k for k in grouped if allocations[k]<len(grouped[k])]
        key=min(candidates,key=lambda k:(allocations[k],k));allocations[key]+=1
    samples=[]
    for k,group in grouped.items():
        n=allocations[k]
        if n:samples.extend(group[i] for i in np.linspace(0,len(group)-1,n,dtype=int))
    samples.sort(key=stamp)
    effective_stride=max(stride,int(np.ceil(np.sqrt(calibration['width']*calibration['height']*max(1,len(samples))/8000000))))
    groups={};skipped=0;points_budget=0;integrated=0
    for index,row in enumerate(samples):
        pose=resolved[stamp(row)];map_id=int(pose['map_id'])
        depth_path=(run/'depth_frames'/row['depth']).resolve();gray_path=(run/'depth_frames'/row['image']).resolve()
        if depth_path.parent!=(run/'depth_frames').resolve() or gray_path.parent!=(run/'depth_frames').resolve():raise ValueError('depth path outside recording')
        depth=cv2.imread(str(depth_path),cv2.IMREAD_UNCHANGED);gray=cv2.imread(str(gray_path),cv2.IMREAD_GRAYSCALE)
        if depth is None or gray is None:skipped+=1;continue
        rotation,translation=pose_matrix(pose)
        surface=depth_surface(depth,gray,calibration,rotation,translation,effective_stride)
        points_budget+=len(surface[0])
        if points_budget>8000000:skipped+=len(samples)-index;break
        if len(surface[1]):groups.setdefault(map_id,[]).append(surface);integrated+=1
        if index%20==0:atomic_json(output/'status.json',{**status,'processed_depth_frames':index+1,'total_depth_frames':len(samples)})
    models=[]
    for map_id,surfaces in groups.items():
        vertices,faces,colors=fuse_surfaces(surfaces,voxel)
        if len(faces):models.append(export_mesh(output,map_id,vertices,faces,colors,metadata[map_id]))
    models.sort(key=lambda m:m['faces'],reverse=True)
    fallback_count=sum(resolved[stamp(r)]['pose_source']=='online_snapshot' for r in samples)
    # Publish the aligned trajectory only after mesh export succeeds.
    if models:
        fields=['frame','timestamp_s','state','map_points','tx','ty','tz','qx','qy','qz','qw','map_id','map_version','tracked_features','source_segment','pose_source']
        trajectory=[]
        for base in online:
            t=stamp(base)
            if t in resolved:trajectory.append({**resolved[t],'frame':base.get('frame',0)})
            else:trajectory.append({**base,'state':4 if int(base['state']) in (2,5) else base['state'],'tracked_features':0})
        with (run/'model_poses.csv.tmp').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(trajectory)
        os.replace(run/'model_poses.csv.tmp',run/'model_poses.csv')
    reason='ready' if models else 'no_depth_frames' if not captures else 'pose_association_failed' if not samples else 'no_valid_surfaces'
    message=('表面模型已生成；含未优化采集位姿保留的独立地图' if fallback_count else '表面模型已生成；未观测区域保留空缺') if models else {
        'no_depth_frames':'没有归档深度帧，请检查跟踪和深度采集状态。',
        'pose_association_failed':'已有深度，但缺少可配对的可信位姿或地图版本；请检查数据完整性。',
        'no_valid_surfaces':'深度与位姿已配对，但没有可用三角面；请检查深度质量或文件读取错误。'}[reason]
    status.update(state='ready' if models else 'unavailable',reason=reason,models=models,
                  primary_map=models[0]['map_id'] if models else None,message=message,
                  source='stereo depth + per-group pose provenance',coordinate='separate map groups; metres',
                  selected_depth_frames=len(samples),online_snapshot_depth_frames=fallback_count,
                  integrated_depth_frames=integrated,skipped_depth_frames=skipped,
                  stride=effective_stride,requested_stride=stride,voxel_m=voxel,elapsed_s=round(time.time()-started,3))
    archive_status=run/'depth_archive_status.json'
    if archive_status.exists():status['archive']=json.loads(archive_status.read_text())
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
