#!/usr/bin/env python3
"""Unarmed, scoring-only check that RGL rays hit the scene's physics surfaces."""
import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np
import trimesh
from gz.transport13 import Node
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
from autonomy_geometry import world_collision_mesh


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--world-file', required=True, type=Path)
    p.add_argument('--lidar', required=True, choices=['mid360', 'avia', 'os1_64'])
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--timeout', type=float, default=120)
    args = p.parse_args()
    mesh, planes = world_collision_mesh(args.world_file)
    proximity = trimesh.proximity.ProximityQuery(mesh)
    clouds = []; lock = threading.Lock(); node = Node()

    def receive(m):
        fields = {f.name: f.offset for f in m.field}
        if not {'x', 'y', 'z'} <= fields.keys() or not m.width*m.height:
            return
        dtype = np.dtype(dict(names=['x','y','z'], formats=['<f4']*3,
                              offsets=[fields[a] for a in ['x','y','z']], itemsize=m.point_step))
        rows = [np.frombuffer(m.data, dtype=dtype, count=m.width, offset=y*m.row_step)
                for y in range(m.height)]
        points = np.concatenate([np.column_stack([r[a] for a in ['x','y','z']]) for r in rows])
        points = points[np.isfinite(points).all(axis=1)]
        with lock:
            if len(clouds)<3 and len(points): clouds.append(points[::max(1,len(points)//2000)].copy())

    topic = f'/{args.lidar}/points/world'
    node.subscribe(PointCloudPacked, topic, receive)
    end = time.monotonic()+args.timeout
    while time.monotonic()<end:
        with lock: ready = len(clouds)>=3
        if ready: break
        time.sleep(.1)
    with lock: samples = list(clouds)
    # Stop callbacks before Python tears down the callback closure and arrays.
    node.unsubscribe(topic)
    if len(samples)<3: raise RuntimeError('Three world-frame RGL measurements were not received')
    points = np.concatenate(samples)
    distance = proximity.on_surface(points)[1]
    for z in planes: distance = np.minimum(distance, np.abs(points[:,2]-z))
    # Graphics can protrude outside a simplified collision hull; those returns
    # are conservative too. The dominant physical surfaces must still match.
    result = dict(simulated_lidar=args.lidar, frames=len(samples), sampled_returns=len(points),
                  median_surface_distance_m=float(np.median(distance)),
                  p95_surface_distance_m=float(np.percentile(distance,95)),
                  returns_within_5cm=float(np.mean(distance<.05)),
                  physics_surface_echo_pass=bool(np.median(distance)<.03 and np.mean(distance<.05)>.90),
                  unarmed_check=True, navigation_received_truth=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    if not result['physics_surface_echo_pass']: raise SystemExit(1)


if __name__=='__main__': main()
