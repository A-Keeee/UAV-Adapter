#!/usr/bin/env bash
# Start GanYu plus an A*/B-spline mission in the already running Walls scene.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
auto_start=false
rviz=false
mass=2.125307692307692
lidar_args=()
config="$repo_dir/uav_path_planner/config/waypoints.yaml"
while (($#)); do
    case "$1" in
        --start) auto_start=true; shift ;;
        --rviz) rviz=true; shift ;;
        --mid360) mass=2.390307692307692; lidar_args=(-p lidar_enabled:=true); shift ;;
        --config) [[ -f ${2:-} ]] || { echo "--config requires a waypoint YAML file" >&2; exit 2; }; config=$(realpath -- "$2"); shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--start] [--rviz] [--mid360] [--config FILE]"
            echo "Start ./run_obstacle_scene.sh first. Default only previews the planned routes."
            echo "--start: automatically take off, track all waypoints, then land."
            echo "--mid360: match the vehicle mass and enable live lidar stop/replan avoidance (no mapping)."
            echo "Ctrl+C requests landing and waits for disarming before stopping GanYu."
            exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
exec 8>"${XDG_RUNTIME_DIR:-/tmp}/gazebo_sim_ganyu_${UID}.lock"
flock -n 8 || { echo "Another GanYu session is running; stop its terminal first." >&2; exit 1; }
set +u
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
set -u
node_bin="$repo_dir/install/uav_ganyu_control/lib/uav_ganyu_control/ganyu_controller"
mission_bin="$repo_dir/install/uav_path_planner/lib/uav_path_planner/waypoint_mission"
[[ -x $node_bin && -x $mission_bin ]] || { echo "Build packages first; see uav_path_planner/README.md" >&2; exit 1; }
height=$("$mission_bin" --takeoff-height "$config")
mkdir -p "$repo_dir/log/ganyu_planner"
run_dir=$(mktemp -d "$repo_dir/log/ganyu_planner/$(date +%Y%m%d_%H%M%S)_XXXX")
ln -sfn -- "$run_dir" "$repo_dir/log/ganyu_planner/latest"
cp -- "$config" "$run_dir/waypoints.yaml"
pids=()
cleanup() {
    local result=$?
    trap - EXIT INT TERM
    for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
    echo "Planning / tracking logs: $run_dir"
    exit "$result"
}
trap cleanup EXIT
# The foreground mission handles signals, requests LAND and waits for disarming.
trap ':' INT TERM
setsid "$node_bin" --ros-args -p reference_source:=planner -p "mass:=$mass" \
    -p "takeoff_height:=$height" >"$run_dir/controller.log" 2>&1 < /dev/null &
pids+=("$!")
# Timestamped TF is also needed for lidar avoidance without RViz.
setsid ros2 launch uav_sim_visualization sensors.launch.py rviz:="$rviz" \
    rviz_config:="$repo_dir/uav_path_planner/config/planning.rviz" \
    >"$run_dir/rviz.log" 2>&1 < /dev/null &
pids+=("$!")
echo "auto_start=$auto_start | mass=$mass kg | waypoints=$config"
echo "Logs: $run_dir"
if ! $auto_start; then echo "PREVIEW only. Add --start to execute the mission."; fi
"$mission_bin" --ros-args --params-file "$config" -p "auto_start:=$auto_start" \
    -p "result_file:=$run_dir/result.json" "${lidar_args[@]}" > >(tee "$run_dir/mission.log") 2>&1
