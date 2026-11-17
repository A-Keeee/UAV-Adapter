#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
set +u
source /opt/ros/humble/setup.bash
set -u
cd -- "$repo_dir"
/usr/bin/python3 "$repo_dir/scripts/ego_environment.py" -- colcon build --build-base "$HOME/.cache/gazebo_sim_autonomy/build" --symlink-install \
    --packages-up-to uav_autonomy --executor sequential "$@" --cmake-args -DCMAKE_BUILD_TYPE=Release
