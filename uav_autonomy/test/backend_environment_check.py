#!/usr/bin/env python3
"""Exercise a terminal containing both native overlays, with real ROS setup."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from ego_environment import PATH_VARIABLES, without_ego_overlays

# Native soft-wall tuning must not silently shrink/shift the hard flight bounds.
from autonomy_session import execution_height_bounds
import yaml
forest = yaml.safe_load((ROOT/'uav_autonomy/config/forest.yaml').read_text())['autonomy']['ros__parameters']
v2 = yaml.safe_load((ROOT/'uav_autonomy/config/ego_v2.yaml').read_text())['ego_planner_node']['ros__parameters']
assert execution_height_bounds(forest, v2, 'v2') == [1.5, 2.3]
assert 1.5 < v2['grid_map/virtual_ground'] < forest['takeoff_height'] < v2['grid_map/virtual_ceil'] < 2.3
assert execution_height_bounds({}, dict(v2, **{'grid_map/virtual_ground':1.4,'grid_map/virtual_ceil':2.4}), 'v2') == [1.5, 2.3]
inherited = dict(os.environ)
for name in PATH_VARIABLES:
    inherited[name] = os.pathsep.join([
        str(Path.home()/'.cache/gazebo_ego_swarm/install/traj_utils/lib'),
        str(Path.home()/'.cache/gazebo_ego_v2/install/traj_utils/lib'),
        inherited.get(name, '')])
inherited['AUTONOMY_ENVIRONMENT_TEST'] = 'preserved'
clean = without_ego_overlays(inherited)
assert clean['AUTONOMY_ENVIRONMENT_TEST'] == 'preserved'
assert all('/gazebo_ego_' not in clean[name] for name in PATH_VARIABLES)
for backend in ('swarm', 'v2'):
    overlay = Path.home()/f'.cache/gazebo_ego_{backend}/install'
    assert (overlay/'local_setup.bash').exists(), 'Build both EGO backends first'
    commands = ['source /opt/ros/humble/setup.bash',
                'source '+shlex.quote(str(ROOT/'install/local_setup.bash')),
                'source '+shlex.quote(str(overlay/'local_setup.bash')),
                '/usr/bin/python3 -c '+shlex.quote(
                    'import os,json; print(json.dumps({k:os.environ.get(k, "") for k in '+repr(PATH_VARIABLES)+'}))')]
    result = subprocess.run(['bash', '-c', ' && '.join(commands)], env=clean,
                            capture_output=True, text=True, check=True)
    loaded = json.loads(result.stdout.splitlines()[-1])
    assert str(overlay) in loaded['AMENT_PREFIX_PATH'], 'Selected backend was not loaded'
    other = 'v2' if backend == 'swarm' else 'swarm'
    assert all(f'/gazebo_ego_{other}/' not in value for value in loaded.values()), loaded
    handshake = [*commands[:3], '/usr/bin/python3 '+shlex.quote(str(
        ROOT/'third_party/ego_bridge/test/rejection_handshake.py'))+' --backend '+backend]
    subprocess.run(['bash','-c',' && '.join(handshake)],env=clean,check=True)
overlay = Path.home()/'.cache/gazebo_ego_swarm/install'
progress = ['source /opt/ros/humble/setup.bash',
            'source '+shlex.quote(str(ROOT/'install/local_setup.bash')),
            'source '+shlex.quote(str(overlay/'local_setup.bash')),
            '/usr/bin/python3 '+shlex.quote(str(ROOT/'third_party/ego_bridge/test/native_progress_check.py'))]
for speed in ('.18','.5'):
    subprocess.run(['bash','-c',' && '.join(progress)+' '+speed],env=clean,check=True)
# The native v2 long-range guide must tolerate recorded transverse hover
# derivatives without changing the actual local MINCO start state.
overlay = Path.home()/'.cache/gazebo_ego_v2/install'
hover_check = ['source /opt/ros/humble/setup.bash',
               'source '+shlex.quote(str(ROOT/'install/local_setup.bash')),
               'source '+shlex.quote(str(overlay/'local_setup.bash')),
               '/usr/bin/python3 '+shlex.quote(str(ROOT/'third_party/ego_bridge/test/v2_hover_goal_check.py'))]
subprocess.run(['bash','-c',' && '.join(hover_check)],env=clean,check=True)
# Actual native-map witness from a rejected full-flight stopping tube, plus
# subvoxel queries on all axes. Exercise the shared map library as deployed.
map_check = hover_check[:-1] + ['/usr/bin/python3 '+shlex.quote(str(
    ROOT/'third_party/ego_bridge/test/v2_stopping_map_check.py'))]
subprocess.run(['bash','-c',' && '.join(map_check)],env=clean,check=True)
print('Contaminated parent environment: exactly one selected native overlay, both backends PASS')
# Check only geometry; this command expands SDF but never starts a simulator.
subprocess.run([str(Path.home()/'.cache/gazebo_autonomy/eval_venv/bin/python'),
                str(ROOT/'uav_autonomy/test/spawn_clearance_check.py')],env=clean,check=True)
