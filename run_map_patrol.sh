#!/usr/bin/env bash
# Request patrol in the running exploration session, retaining its map/origin.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
set +u
source /opt/ros/humble/setup.bash
set -u
exec /usr/bin/python3 "$repo_dir/scripts/request_map_patrol.py" "$@"
