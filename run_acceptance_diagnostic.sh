#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
exec /usr/bin/python3 "$repo_dir/log/crash_analysis_20261114/capture_acceptance.py" "$@"
