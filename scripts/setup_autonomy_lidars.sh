#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
"$repo_dir/scripts/setup_mid360_sim.sh"
prefix=${MID360_RGL_DIR:-"$HOME/.local/opt/gazebo_sim_mid360_rgl-v0.2.0"}
revision=d4bf3cf36fe4a363a56df1bec2ce3809720db563
for item in 'LivoxAvia.mat3x4f:5e16715d655ae7e6f6e05daaccd0186c0e03cdd3a9ad47d11c696136189904b4' 'OusterOS1_64.mat3x4f:00c79b0524ea81b7a672a377e1f098c4a7e01233034f34eb627b307534b89bd1'; do
    name=${item%%:*}; digest=${item#*:}
    destination="$prefix/lidar_patterns/$name"
    if ! printf '%s  %s\n' "$digest" "$destination" | sha256sum -c - >/dev/null 2>&1; then
        cache="$HOME/.cache/gazebo_autonomy/$name"
        mkdir -p -- "$(dirname -- "$cache")"
        if ! printf '%s  %s\n' "$digest" "$cache" | sha256sum -c - >/dev/null 2>&1; then
            curl -fL --retry 3 --connect-timeout 15 --max-time 300 "https://raw.githubusercontent.com/RobotecAI/RGLGazeboPlugin/$revision/lidar_patterns/$name" -o "$cache"
        fi
        printf '%s  %s\n' "$digest" "$cache" | sha256sum -c -
        cp -- "$cache" "$destination"
    fi
done
# Extend the existing checked installation; legacy setup remains usable.
(cd "$prefix" && sha256sum RGLServerPlugin/*.so RGLVisualize/*.so lidar_patterns/*.mat3x4f LICENSE.RGLGazeboPlugin VERSION >manifest.sha256)
echo "MID-360, Avia and OS1-64 scan patterns installed."
