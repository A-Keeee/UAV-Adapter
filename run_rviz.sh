#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
set +u
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
set -u
if ! ros2 pkg prefix uav_sim_visualization >/dev/null 2>&1; then
    echo "Build the viewer first: colcon build --symlink-install --packages-select uav_sim_visualization" >&2
    exit 1
fi
exec ros2 launch uav_sim_visualization sensors.launch.py "$@"
