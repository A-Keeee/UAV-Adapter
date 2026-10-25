#!/usr/bin/env bash
# Single-vehicle PX4 / Harmonic scene. Existing worlds, parameters and logs stay intact.
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
px4_dir=${PX4_DIR:-"$HOME/PX4-Autopilot"}
headless=false
check_only=false
ready_timeout=90
while (($#)); do
    case "$1" in
        --headless) headless=true; shift ;;
        --check) check_only=true; shift ;;
        --timeout)
            [[ ${2:-} =~ ^[1-9][0-9]*$ ]] || { echo "--timeout requires positive seconds" >&2; exit 2; }
            ready_timeout=$2; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--headless] [--check] [--timeout SECONDS]"
            echo "Default: open Walls scene with one unarmed x500_depth and verify ROS data."
            echo "--check: exit and stop this session after verification. PX4_DIR overrides the PX4 checkout."
            exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done

export PATH="$HOME/.local/bin:$PATH"
for tool in gz MicroXRCEAgent setsid flock grep pgrep timeout /usr/bin/python3; do
    command -v "$tool" >/dev/null || { echo "Missing dependency: $tool" >&2; exit 1; }
done
px4_bin="$px4_dir/build/px4_sitl_default/bin/px4"
[[ -x $px4_bin ]] || { echo "PX4 SITL is not built: $px4_bin" >&2; exit 1; }
[[ -f $repo_dir/install/setup.bash ]] || { echo "Build this ROS 2 workspace first." >&2; exit 1; }

# Refuse conflicting simulators instead of killing someone else's simulation.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/gazebo_sim_obstacle_${UID}.lock"
flock -n 9 || { echo "Another obstacle scene session is running." >&2; exit 1; }
if pgrep -u "$UID" -x px4 >/dev/null; then
    echo "PX4 is already running. Stop its owning terminal before starting this scene." >&2
    exit 1
fi
if timeout 5 gz topic -l 2>/dev/null | grep -E '^/world/.*/clock$' >/dev/null; then
    echo "A Gazebo world is already running. Stop it before starting this scene." >&2
    exit 1
fi
/usr/bin/python3 - <<'PY'
import socket
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    try:
        sock.bind(('0.0.0.0', 8888))
    except OSError as exc:
        raise SystemExit(f'UDP 8888 is in use; stop the existing XRCE agent first: {exc}')
PY

set +u
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
source "$px4_dir/build/px4_sitl_default/rootfs/gz_env.sh"
set -u
/usr/bin/python3 -c 'from px4_msgs.msg import VehicleStatus; import rclpy; from PIL import Image'
if ! $headless && [[ -z ${DISPLAY:-}${WAYLAND_DISPLAY:-} ]]; then
    echo "No desktop display. Use --headless." >&2
    exit 1
fi

world="$repo_dir/assets/gazebo_world/walls_demo.sdf"
gz sdf -k "$world" >/dev/null
mkdir -p "$repo_dir/log/obstacle_scene"
run_dir=$(mktemp -d "$repo_dir/log/obstacle_scene/$(date +%Y%m%d_%H%M%S)_XXXX")
mkdir "$run_dir/px4"
ln -sfn -- "$run_dir" "$repo_dir/log/obstacle_scene/latest"
printf '%s\n' "$$" >"$run_dir/launcher.pid"
pids=()
cleanup() {
    local result=$?
    trap - EXIT INT TERM
    echo "Stopping this scene's processes. Logs retained: $run_dir"
    for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
    sleep 2
    for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start_process() {
    local label=$1
    shift
    setsid "$@" >"$run_dir/$label.log" 2>&1 < /dev/null &
    pids+=("$!")
    printf '%s %s\n' "$label" "$!" >>"$run_dir/processes.txt"
}

cat >"$run_dir/bridge.yaml" <<'YAML'
- ros_topic_name: /clock
  gz_topic_name: /world/walls_demo/clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
- ros_topic_name: /camera/image
  gz_topic_name: /world/walls_demo/model/x500_depth_1/link/camera_link/sensor/IMX214/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
- ros_topic_name: /camera/camera_info
  gz_topic_name: /world/walls_demo/model/x500_depth_1/link/camera_link/sensor/IMX214/camera_info
  ros_type_name: sensor_msgs/msg/CameraInfo
  gz_type_name: gz.msgs.CameraInfo
  direction: GZ_TO_ROS
YAML
sha256sum "$world" "$repo_dir/assets/gazebo_world/grass_world.sdf" "$repo_dir/run_swarm.sh" >"$run_dir/source.sha256"
gz sim --versions >"$run_dir/gazebo_version.txt"
git -C "$px4_dir" describe --always --tags >"$run_dir/px4_version.txt"
printf 'World: walls_demo; vehicle: x500_depth_1; spawn ENU: (0,0,0); PX4 system ID: 2\nLogs: %s\n' "$run_dir"

start_process agent MicroXRCEAgent udp4 -p 8888
server_args=(gz sim -r -s -v 3 "$world")
if $headless; then server_args+=(--headless-rendering); fi
start_process gazebo "${server_args[@]}"
if ! $headless; then
    start_process gui gz sim -g --gui-config "$repo_dir/assets/gazebo_world/walls_demo.gui.config"
fi
start_process px4 env PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=walls_demo \
    PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_x500_depth PX4_GZ_MODEL_POSE=0,0,0 \
    PX4_PARAM_EKF2_HGT_REF=1 \
    "$px4_bin" -d -i 1 -w "$run_dir/px4" "$px4_dir/build/px4_sitl_default/etc"
start_process bridge ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=$run_dir/bridge.yaml"

if ! /usr/bin/python3 "$repo_dir/scripts/check_obstacle_scene.py" --timeout "$ready_timeout" --output "$run_dir"; then
    for label in gazebo px4 bridge; do
        echo "Last messages: $label"
        tail -n 15 "$run_dir/$label.log"
    done
    exit 1
fi
echo "READY: world, four obstacle walls, PX4 position/odometry, RGB image, camera info and clock verified."
echo "Vehicle remains unarmed. No controller or flight task has been started."
if $check_only; then exit 0; fi
echo "Keep this terminal open. Ctrl+C stops only this scene."
while sleep 2; do
    for pid in "${pids[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "A scene process exited. Inspect $run_dir" >&2
            exit 1
        fi
    done
done
