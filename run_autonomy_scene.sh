#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
px4_dir=${PX4_DIR:-"$HOME/PX4-Autopilot"}
set +u
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
source "$px4_dir/build/px4_sitl_default/rootfs/gz_env.sh"
set -u
export PATH="$HOME/.local/bin:$PATH"
export PX4_DIR="$px4_dir"
export PX4_GZ_MODELS="$repo_dir/assets/gazebo_models"
export GZ_SIM_RESOURCE_PATH="$repo_dir/assets/gazebo_models:$repo_dir/assets/vendor/warehouse/models:$repo_dir/assets/vendor/forest:${GZ_SIM_RESOURCE_PATH:-}"
export SDF_PATH="$GZ_SIM_RESOURCE_PATH"
rgl_dir=${MID360_RGL_DIR:-"$HOME/.local/opt/gazebo_sim_mid360_rgl-v0.2.0"}
export GZ_SIM_SYSTEM_PLUGIN_PATH="$rgl_dir/RGLServerPlugin:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export RGL_PATTERNS_DIR="$rgl_dir/lidar_patterns"
exec /usr/bin/python3 "$repo_dir/scripts/autonomy_scene.py" "$@"
