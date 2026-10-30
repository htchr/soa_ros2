#!/usr/bin/env bash
#
# Build the full colcon workspace.
#
# The external packages from docker/soa.repos are already built into the image.
# This compiles the bind-mounted soa_ros2 (and its feetech_ros2_driver submodule)
# on top of that underlay. Runs once on first container start (container.sh) and
# as the devcontainer postCreateCommand — re-run it any time to rebuild.
set -euo pipefail

WS="/home/ubuntu/colcon_ws"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

cd "$WS"

# soa_ros2 is a multi-package repo with no package.xml at its root, so colcon
# recurses into it. Stale artifacts from a previous *native* build at the repo
# root (build/ install/ log/) contain package.xml copies that would collide with
# the real sources; the huggingface cache is large and irrelevant. Mark them so
# colcon skips them. All are git-ignored, so these markers are invisible to git.
for d in build install log huggingface; do
    if [ -d "$WS/src/soa_ros2/$d" ]; then
        touch "$WS/src/soa_ros2/$d/COLCON_IGNORE" 2>/dev/null || true
    fi
done

echo "==> Resolving ROS dependencies (rosdep) ..."
sudo apt-get update
rosdep update || true
# soa_ros2's own dependencies (the external repos' deps are already installed).
rosdep install --from-paths src --ignore-src -r -y || true

echo "==> Building workspace (colcon build --symlink-install) ..."
colcon build --symlink-install

# shellcheck disable=SC1091
source "$WS/install/setup.bash"

# Marker used by container.sh to detect the first run.
touch "/home/ubuntu/.soa_built"
echo "==> Workspace build complete."
