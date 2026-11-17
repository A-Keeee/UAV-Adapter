#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
backend=${1:-all}
[[ $backend == all || $backend == swarm || $backend == v2 ]] || { echo 'Expected all, swarm or v2' >&2; exit 2; }
set +u
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
set -u
build_backend() {
    local name=$1 src=$2
    shift 2
    local cache="$HOME/.cache/gazebo_ego_$name"
    mkdir -p "$cache"
    # Each backend is built in its own process/environment. Their duplicate
    # upstream package names must never be loaded in one overlay chain.
    (cd "$repo_dir" && MAKEFLAGS=-j2 /usr/bin/python3 "$repo_dir/scripts/ego_environment.py" -- colcon --log-base "$cache/log" build --base-paths "$src" "$repo_dir/third_party/ego_bridge" "$@" \
        --build-base "$cache/build" --install-base "$cache/install" \
        --executor sequential \
        --packages-up-to ego_planner ego_ganyu_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "-DEGO_BACKEND_V2=$([[ $name == v2 ]] && echo ON || echo OFF)")
}
if [[ $backend == all || $backend == swarm ]]; then
    build_backend swarm "$repo_dir/third_party/ego_swarm/src"
fi
if [[ $backend == all || $backend == v2 ]]; then
    build_backend v2 "$repo_dir/third_party/ego_v2/swarm-playground/main_ws/src" "$repo_dir/third_party/ego_v2/ros2_adapter"
fi
