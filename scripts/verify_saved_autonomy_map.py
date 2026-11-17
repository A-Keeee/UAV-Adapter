#!/usr/bin/env python3
"""Independently verify the saved exploration OctoMap against the scoring ROI."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import trimesh
from autonomy_evaluator import Evaluator
from autonomy_geometry import ROOT, world_collision_mesh


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mission_directory', type=Path)
    args = p.parse_args(); folder = args.mission_directory.resolve()
    run = json.loads((folder/'run.json').read_text())
    evaluation = json.loads((folder/'evaluation/evaluation.json').read_text())
    world = Path(run['scene'].get('world_file',ROOT/'assets/gazebo_world'/f'{run["scene"]["world"]}.sdf'))
    if hashlib.sha256(world.read_bytes()).hexdigest()!=run['scene']['world_sha256']:
        raise RuntimeError('World differs from the recorded trial')
    if not evaluation.get('truth_to_odom_translation'):
        raise RuntimeError('Independent preflight frame alignment is unavailable')
    output = folder/'evaluation/saved_map_check'; output.mkdir(exist_ok=True)
    mesh, _ = world_collision_mesh(world)
    inspector = SimpleNamespace(args=SimpleNamespace(output=output,mission_result=folder/'result.json'),
        proximity=trimesh.proximity.ProximityQuery(mesh), known=set(),
        offset=np.asarray(evaluation['truth_to_odom_translation']),saved_map_verified=False,coverage_source=None)
    coverage = Evaluator.coverage(inspector)
    artifact = folder/'warehouse.bt'
    result = dict(coverage=coverage,coverage_pass=coverage is not None and coverage>=.9,
        saved_map_verified=inspector.saved_map_verified,source=inspector.coverage_source,
        map_file=str(artifact),map_sha256=hashlib.sha256(artifact.read_bytes()).hexdigest(),
        world_sha256=run['scene']['world_sha256'],navigation_received_truth=False)
    (output/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    if not result['saved_map_verified'] or not result['coverage_pass']: raise SystemExit(1)


if __name__=='__main__': main()
