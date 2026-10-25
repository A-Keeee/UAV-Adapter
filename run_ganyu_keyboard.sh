#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
keyboard_args=()
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    echo "Usage: $0 [--external-gcs] [--ros-args ...controller parameters...]"
    echo "Start the existing obstacle scene in another terminal first. Press t to take off."
    exit 0
fi
if [[ ${1:-} == --external-gcs ]]; then keyboard_args+=(--external-gcs); shift; fi
[[ -t 0 ]] || { echo "Run in an interactive terminal (keyboard input is required)." >&2; exit 1; }
node_bin="$repo_dir/install/uav_ganyu_control/lib/uav_ganyu_control/ganyu_controller"
[[ -x $node_bin ]] || { echo "Build uav_ganyu_control first; see uav_ganyu_control/README.md" >&2; exit 1; }
exec 8>"${XDG_RUNTIME_DIR:-/tmp}/gazebo_sim_ganyu_${UID}.lock"
flock -n 8 || { echo "Another GanYu keyboard session is running." >&2; exit 1; }
set +u
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
set -u
mkdir -p "$repo_dir/log/ganyu_control"
run_dir=$(mktemp -d "$repo_dir/log/ganyu_control/$(date +%Y%m%d_%H%M%S)_XXXX")
ln -sfn -- "$run_dir" "$repo_dir/log/ganyu_control/latest"
setsid "$node_bin" "$@" >"$run_dir/controller.log" 2>&1 < /dev/null &
node_pid=$!
printf '%s\n' "$node_pid" >"$run_dir/controller.pid"
cleanup() {
    local result=$?
    trap - EXIT INT TERM
    kill -TERM -- "-$node_pid" 2>/dev/null || true
    wait "$node_pid" 2>/dev/null || true
    echo "Controller log retained: $run_dir/controller.log"
    exit "$result"
}
trap cleanup EXIT
# Keep the controller alive while the foreground keyboard requests and confirms landing.
trap ':' INT TERM
sleep 0.5
kill -0 "$node_pid" 2>/dev/null || { cat "$run_dir/controller.log"; exit 1; }
/usr/bin/python3 "$repo_dir/uav_ganyu_control/scripts/keyboard.py" "${keyboard_args[@]}"
