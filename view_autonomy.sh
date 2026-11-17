#!/usr/bin/env bash
# Read-only RViz for either unknown-environment mission, in the same ROS domain.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
set +u
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
set -u
exec /usr/bin/python3 "$repo_dir/scripts/ego_environment.py" -- \
  /usr/bin/python3 "$repo_dir/scripts/view_autonomy.py" "$@"
