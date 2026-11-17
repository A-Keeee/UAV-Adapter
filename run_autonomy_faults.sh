#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
set +u
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
set -u
exec /usr/bin/python3 "$repo_dir/scripts/run_autonomy_faults.py" "$@"
