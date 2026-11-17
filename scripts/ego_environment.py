#!/usr/bin/env python3
"""Keep inherited EGO overlays out of independent builds and processes."""
import os
from pathlib import Path
import sys

PATH_VARIABLES = ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                  'LD_LIBRARY_PATH', 'PYTHONPATH', 'PATH', 'ROS_PACKAGE_PATH',
                  'PKG_CONFIG_PATH')


def without_ego_overlays(environment):
    result = dict(environment)
    roots = [str(Path.home()/('.cache/gazebo_ego_'+name)) for name in ('swarm', 'v2')]
    for name in PATH_VARIABLES:
        if name not in result:
            continue
        result[name] = os.pathsep.join(value for value in result[name].split(os.pathsep)
            if not any(value == root or value.startswith(root+os.sep) for root in roots))
    return result


if __name__ == '__main__':
    command = sys.argv[1:]
    if command[:1] == ['--']:
        command = command[1:]
    if not command:
        raise SystemExit('Expected a command after --')
    os.execvpe(command[0], command, without_ego_overlays(os.environ))
