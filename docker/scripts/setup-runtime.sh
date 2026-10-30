#!/usr/bin/env bash
#
# Per-start runtime fixes. Must run every time the container starts because they
# touch live host devices (/sys, /proc, /dev) that only exist while running.
# Applied by container.sh on `start`/`enter` and by the devcontainer
# postStartCommand. All steps are best-effort and idempotent.
set -uo pipefail

# --- XDG runtime dir for GUI apps like rerun/rviz ------------------------------
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-ubuntu}"
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true

# --- USB latency fix for serial devices (feetech servos, etc.) -----------------
for tty in /sys/bus/usb-serial/devices/*/latency_timer; do
    if [ -f "$tty" ]; then
        echo 1 | sudo tee "$tty" >/dev/null 2>&1 || true
    fi
done

# --- Increase USB/network buffer sizes for better stability --------------------
sudo sh -c 'echo 16777216 > /proc/sys/net/core/rmem_max' 2>/dev/null || true
sudo sh -c 'echo 16777216 > /proc/sys/net/core/wmem_max' 2>/dev/null || true

# --- V4L2 camera permissions (helps with camera timeouts) ----------------------
for video_dev in /dev/video*; do
    if [ -c "$video_dev" ]; then
        sudo chmod 666 "$video_dev" 2>/dev/null || true
    fi
done

# --- Link the huggingface cache to the persistent (bind-mounted) project dir ---
# Anything written to ~/.cache/huggingface lands in soa_ros2/huggingface on the
# host, so downloaded models/datasets survive container restarts.
PERSISTENT_HF_DIR="/home/ubuntu/colcon_ws/src/soa_ros2/huggingface"
HF_CACHE_DIR="/home/ubuntu/.cache/huggingface"

mkdir -p "$PERSISTENT_HF_DIR" "$(dirname "$HF_CACHE_DIR")"

if [ -L "$HF_CACHE_DIR" ]; then
    # Already a symlink — repoint it if it aims somewhere else.
    if [ "$(readlink "$HF_CACHE_DIR")" != "$PERSISTENT_HF_DIR" ]; then
        rm -f "$HF_CACHE_DIR"
        ln -s "$PERSISTENT_HF_DIR" "$HF_CACHE_DIR"
    fi
elif [ -d "$HF_CACHE_DIR" ]; then
    # Real directory — migrate its contents, then replace it with the symlink.
    cp -rn "$HF_CACHE_DIR"/* "$PERSISTENT_HF_DIR"/ 2>/dev/null || true
    rm -rf "$HF_CACHE_DIR"
    ln -s "$PERSISTENT_HF_DIR" "$HF_CACHE_DIR"
else
    ln -s "$PERSISTENT_HF_DIR" "$HF_CACHE_DIR"
fi

echo "==> Runtime device fixes applied."
