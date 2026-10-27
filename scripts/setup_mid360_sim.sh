#!/usr/bin/env bash
# Install pinned RGL binaries and the MID-360 pattern into a dedicated user directory.
set -euo pipefail
prefix=${MID360_RGL_DIR:-"$HOME/.local/opt/gazebo_sim_mid360_rgl-v0.2.0"}
cache=${MID360_DOWNLOAD_CACHE:-"$HOME/.cache/gazebo_sim_mid360"}
revision=d4bf3cf36fe4a363a56df1bec2ce3809720db563
for tool in curl unzip sha256sum nvidia-smi; do
    command -v "$tool" >/dev/null || { echo "Missing dependency: $tool" >&2; exit 1; }
done
nvidia-smi --query-gpu=name --format=csv,noheader
if [[ -f $prefix/manifest.sha256 ]]; then
    (cd "$prefix" && sha256sum -c manifest.sha256 >/dev/null)
    echo "MID-360 RGL dependency is ready: $prefix"
    exit 0
fi
[[ ! -e $prefix ]] || { echo "Directory already exists without a valid manifest: $prefix" >&2; exit 1; }
mkdir -p "$cache" "$(dirname -- "$prefix")"
download() {
    local name=$1 digest=$2 url=$3
    if ! (cd "$cache" && printf '%s  %s\n' "$digest" "$name" | sha256sum -c - >/dev/null 2>&1); then
        curl -fL --retry 3 --connect-timeout 15 --max-time 900 "$url" -o "$cache/$name"
    fi
    (cd "$cache" && printf '%s  %s\n' "$digest" "$name" | sha256sum -c -)
}
download RGLGazeboPlugin_ubuntu22.zip c89d7b88cefaf79702b91701708bc9356496fca3048c6db0db6d265c46e6c84f \
    https://github.com/RobotecAI/RGLGazeboPlugin/releases/download/v0.2.0-harmonic/RGLGazeboPlugin_ubuntu22.zip
download LivoxMid360.mat3x4f d7b3322f303f9a83268180931e730b5f361592ef1ab6f1d56c166aadf47153fd \
    "https://raw.githubusercontent.com/RobotecAI/RGLGazeboPlugin/$revision/lidar_patterns/LivoxMid360.mat3x4f"
download LICENSE.RGLGazeboPlugin 3f1b4d99668b72cc1fad2e0e370734cac1d5b4982a421cb153d4fee92643497e \
    "https://raw.githubusercontent.com/RobotecAI/RGLGazeboPlugin/$revision/LICENSE"
stage=$(mktemp -d "${prefix}.tmp.XXXXXX")
trap 'rm -rf -- "$stage"' EXIT
unzip -q "$cache/RGLGazeboPlugin_ubuntu22.zip" -d "$stage"
mkdir "$stage/lidar_patterns"
cp "$cache/LivoxMid360.mat3x4f" "$stage/lidar_patterns/"
cp "$cache/LICENSE.RGLGazeboPlugin" "$stage/"
printf 'RGLGazeboPlugin release: v0.2.0-harmonic\nPattern source commit: %s\n' "$revision" >"$stage/VERSION"
(cd "$stage" && sha256sum RGLServerPlugin/*.so RGLVisualize/*.so lidar_patterns/*.mat3x4f LICENSE.RGLGazeboPlugin VERSION >manifest.sha256)
mv -- "$stage" "$prefix"
trap - EXIT
echo "Installed MID-360 RGL dependency: $prefix"
echo "No system libraries, GPU drivers or PX4 source files were changed."
