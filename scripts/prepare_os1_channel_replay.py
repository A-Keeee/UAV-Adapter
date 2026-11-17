#!/usr/bin/env python3
"""Test-only OS1-64 channel sparsification of a closed recorded PointCloud2.

Match measured endpoints against the pinned simulated ray lines, including
their rotating origins. The navigation nodes never import this module or read
the pattern. This produces replay evidence, not a physical flight result.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import tempfile

import numpy as np
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message,serialize_message
from sensor_msgs.msg import PointCloud2


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--fragment',type=Path,help='Closed .db3.zstd containing /autonomy/raw_cloud.')
    source.add_argument('--input-cloud',type=Path,help='Previously extracted serialized PointCloud2.')
    prefix=Path(os.environ.get('MID360_RGL_DIR',str(Path.home()/'.local/opt/gazebo_sim_mid360_rgl-v0.2.0')))
    parser.add_argument('--pattern',type=Path,default=prefix/'lidar_patterns/OusterOS1_64.mat3x4f')
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    if args.fragment:
        if not args.fragment.name.endswith('.db3.zstd'):raise RuntimeError('Use a closed compressed recording')
        with tempfile.TemporaryDirectory(prefix='os1_channels_') as folder:
            db=Path(folder)/'recording.db3'
            with db.open('wb') as stream:subprocess.run(['zstd','-dq','-c',str(args.fragment)],stdout=stream,check=True)
            connection=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
            row=connection.execute("SELECT messages.data FROM messages JOIN topics ON messages.topic_id=topics.id WHERE topics.name='/autonomy/raw_cloud' ORDER BY messages.timestamp LIMIT 1").fetchone()
            connection.close()
            if row is None:raise RuntimeError('No raw point cloud in the selected fragment')
            data=row[0]
    else:data=args.input_cloud.read_bytes()
    message=deserialize_message(data,PointCloud2);fields={field.name:field for field in message.fields}
    endian='>' if message.is_bigendian else '<'
    if any(name not in fields or fields[name].datatype not in (7,8) for name in 'xyz'):raise RuntimeError('Expected floating point XYZ')
    dtype=np.dtype(dict(names=list('xyz'),formats=[endian+('f4' if fields[n].datatype==7 else 'f8') for n in 'xyz'],
        offsets=[fields[n].offset for n in 'xyz'],itemsize=message.point_step))
    rows=np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,strides=(message.row_step,message.point_step))
    xyz=np.stack([rows[n].ravel() for n in 'xyz'],axis=1).astype(float)
    poses=np.fromfile(args.pattern,dtype='<f4').astype(float).reshape(-1,3,4)
    directions=poses[:,:,2];directions/=np.linalg.norm(directions,axis=1)[:,None];origins=poses[:,:,3]
    angles,channels=np.unique(np.round(np.arcsin(directions[:,2]),5),return_inverse=True)
    if len(angles)!=64:raise RuntimeError('Pattern is not the pinned 64-channel scanner')
    tree=cKDTree(directions);assignments=np.full(len(xyz),-1,dtype=int);errors=[]
    valid=np.flatnonzero(np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)>.5))
    if not len(valid):raise RuntimeError('No valid recorded endpoints')
    for begin in range(0,len(valid),2000):
        ids=valid[begin:begin+2000];p=xyz[ids]
        _,near=tree.query(p/np.linalg.norm(p,axis=1)[:,None],k=128)
        direction=directions[near];offset=p[:,None,:]-origins[near]
        along=np.sum(offset*direction,axis=2)
        residual=np.linalg.norm(offset-along[:,:,None]*direction,axis=2);residual[along<0]=np.inf
        chosen=np.argmin(residual,axis=1);error=residual[np.arange(len(ids)),chosen]
        assignments[ids]=channels[near[np.arange(len(ids)),chosen]];errors.extend(error.tolist())
    errors=np.asarray(errors)
    if errors.max()>.001:raise RuntimeError('Recorded cloud does not match the known simulated ray lines')
    selected=valid[assignments[valid]%4==0]
    raw=bytes(message.data);width=message.width;step=message.point_step;stride=message.row_step
    message.data=b''.join(raw[(i//width)*stride+(i%width)*step:(i//width)*stride+(i%width+1)*step] for i in selected)
    message.height=1;message.width=len(selected);message.row_step=step*message.width
    filtered=serialize_message(message)
    (args.output/'original.msg').write_bytes(data);(args.output/'sparse.msg').write_bytes(filtered)
    result=dict(source=str((args.fragment or args.input_cloud).resolve()),pattern_channels=64,
        measured_channels=len(np.unique(assignments[valid])),matched_points=len(valid),retained_points=len(selected),
        retained_channels=len(np.unique(assignments[selected])),retained_channel_indices=list(range(0,64,4)),
        retained_fraction=float(len(selected)/len(valid)),maximum_ray_distance_m=float(errors.max()),
        p99_ray_distance_m=float(np.percentile(errors,99)),original_cloud_sha256=hashlib.sha256(data).hexdigest(),
        sparse_cloud_sha256=hashlib.sha256(filtered).hexdigest(),pattern_sha256=hashlib.sha256(args.pattern.read_bytes()).hexdigest(),
        method='Endpoint to known simulated ray-line matching, including rotating emitter origins; retain every fourth channel',
        navigation_received_model_parameters=False,flight_acceptance=False,interface_replay_verified=False)
    (args.output/'channel_selection.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
