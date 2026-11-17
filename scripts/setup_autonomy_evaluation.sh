#!/usr/bin/env bash
set -euo pipefail
prefix="$HOME/.cache/gazebo_autonomy/eval_venv"
if [[ ! -x $prefix/bin/python ]]; then
    /usr/bin/python3 -m venv --system-site-packages "$prefix"
fi
"$prefix/bin/pip" install numpy==1.24.4 scipy==1.10.1 pycollada==0.9.2 trimesh==4.5.3 rtree==1.3.0 psutil==5.9.0
