#!/usr/bin/env bash
# rosidl_adapter in Humble reads generated paths as ASCII; keep its build tree
# outside a potentially non-ASCII/moved workspace. Source/install stay here.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
workspace_key=$(printf '%s' "$repo_dir" | sha256sum | cut -c1-12)
build_dir="${GAZEBO_BUILD_CACHE:-$HOME/.cache/gazebo_sim_planner/build}"
if ! /usr/bin/python3 -c 'import sys; sys.exit(not sys.argv[1].isascii())' "$build_dir"; then
    echo "GAZEBO_BUILD_CACHE must be an ASCII path for ROS Humble message generation." >&2
    exit 1
fi
# Isolate different copies of the workspace while retaining this one's cache.
cache_source="$build_dir/.gazebo_workspace"
if [[ -f $cache_source ]] && [[ $(cat "$cache_source") != "$repo_dir" ]]; then
    build_dir="${build_dir}_${workspace_key}"
fi
mkdir -p -- "$build_dir"
printf '%s\n' "$repo_dir" >"$build_dir/.gazebo_workspace"
set +u
source /opt/ros/humble/setup.bash
set -u
cd -- "$repo_dir"
colcon build --build-base "$build_dir" --symlink-install \
    --packages-up-to uav_path_planner uav_sim_visualization \
    "$@" --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Build cache: $build_dir"
echo "Next: source install/setup.bash"
