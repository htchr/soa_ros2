# syntax=docker/dockerfile:1
#
# Single image for the Standard Open Arm (soa_ros2) project.
# Used both from the terminal (docker/container.sh + docker-compose.yml) and as a
# VS Code devcontainer (.devcontainer/devcontainer.json -> the same compose file).
#
# WARNING: this image is large and takes a long time to build. It installs a full
# ROS 2 Humble desktop, MoveIt, RealSense, lerobot + CUDA torch, and pre-builds all
# of the external ROS packages listed in docker/soa.repos.

ARG CUDA_VERSION=12.4.1
ARG OS_VERSION=22.04
FROM nvidia/cuda:${CUDA_VERSION}-base-ubuntu${OS_VERSION}

# ---- build-time configuration -------------------------------------------------
ARG ROS_DISTRO=humble
ARG USERNAME=ubuntu
# Match these to the host user so files created in the container are owned by the
# host user (no `sudo chown`/`chmod` needed to edit them from outside). container.sh
# and docker-compose.yml pass the host uid/gid in automatically.
ARG USER_UID=1000
ARG USER_GID=1000
ENV ROS_DISTRO=${ROS_DISTRO}

# Retry flaky apt mirrors and never prompt during the build.
RUN echo 'Acquire::Retries "3";' > /etc/apt/apt.conf.d/80retries
ENV DEBIAN_FRONTEND=noninteractive

# ---- locale + timezone --------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends locales tzdata \
 && locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ---- base toolchain + python + graphics/usb libs (lerobot runtime deps) --------
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake pkg-config ninja-build \
      git curl wget gnupg2 lsb-release sudo software-properties-common \
      python3 python3-pip python3-venv python3-dev python-is-python3 \
      libglib2.0-0 libgl1-mesa-glx libegl1-mesa libvulkan1 mesa-vulkan-drivers ffmpeg \
      libxcb-cursor0 libusb-1.0-0-dev speech-dispatcher libgeos-dev portaudio19-dev \
      libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 \
 && rm -rf /var/lib/apt/lists/*

# ---- ROS 2 apt source ---------------------------------------------------------
RUN curl -L -s -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.$(lsb_release -cs)_all.deb" \
 && apt-get update && apt-get install -y /tmp/ros2-apt-source.deb \
 && rm -f /tmp/ros2-apt-source.deb && rm -rf /var/lib/apt/lists/*

# ---- ROS 2 desktop + build tooling --------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-desktop-full \
      ros-dev-tools python3-argcomplete python3-rosdep \
      python3-colcon-common-extensions python3-vcstool \
      bash-completion gdb openssh-client vim nano less tmux \
 && rm -rf /var/lib/apt/lists/*

# ---- project ROS dependencies (mirrors soa_ros2/build.sh) ---------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-moveit \
      ros-${ROS_DISTRO}-usb-cam \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-apriltag-ros \
      ros-${ROS_DISTRO}-topic-tools \
      ros-${ROS_DISTRO}-warehouse-ros \
 && rm -rf /var/lib/apt/lists/*
# RealSense is a wildcard package set, kept in its own layer.
RUN apt-get update && apt-get install -y --no-install-recommends \
      "ros-${ROS_DISTRO}-realsense2-*" \
 && rm -rf /var/lib/apt/lists/*

# ---- NVIDIA / GUI runtime environment -----------------------------------------
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    QT_X11_NO_MITSHM=1 \
    MUJOCO_GL=egl

# ---- lerobot + CUDA torch (installed into the system interpreter that ROS uses) -
# lerobot[all] pulls a CPU/other torch; reinstall the cu128 wheels afterwards to
# match the old docker/setup.sh behaviour. ultralytics is required by yolo_ros.
RUN python3 -m pip install --no-cache-dir -U pip \
 && python3 -m pip install --no-cache-dir \
      'lerobot[all]' pyrealsense2 ultralytics \
 && (python3 -m pip uninstall -y torch torchvision torchaudio || true) \
 && python3 -m pip install --no-cache-dir \
      torch==2.7.1 torchvision==0.22.1 torchaudio==2.7.1 \
      --index-url https://download.pytorch.org/whl/cu128

# ---- create the non-root `ubuntu` user matched to the host uid/gid ------------
RUN if ! getent group ${USER_GID} >/dev/null 2>&1; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
 && if ! id -u ${USER_UID} >/dev/null 2>&1; then \
        useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}; \
    fi \
 && usermod -aG sudo,video,plugdev,dialout ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# rosdep needs to be initialised once as root.
RUN rosdep init || true

# ---- everything below runs as the unprivileged user ---------------------------
USER ${USERNAME}
WORKDIR /home/${USERNAME}/colcon_ws
RUN rosdep update

# Clone the external ROS packages (single source of truth: docker/soa.repos) and
# pre-build them into the image so only soa_ros2 is left to build at first start.
# soa_ros2 itself (and its feetech_ros2_driver submodule) is bind-mounted at runtime.
COPY --chown=${USER_UID}:${USER_GID} soa.repos /home/${USERNAME}/colcon_ws/soa.repos
RUN mkdir -p src && vcs import src < soa.repos
RUN sudo apt-get update \
 && rosdep install --from-paths src --ignore-src -r -y \
 && sudo rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build --symlink-install --event-handlers console_direct+

# ---- interactive shell setup --------------------------------------------------
RUN printf '%s\n' \
      "source /opt/ros/${ROS_DISTRO}/setup.bash" \
      "[ -f /home/${USERNAME}/colcon_ws/install/setup.bash ] && source /home/${USERNAME}/colcon_ws/install/setup.bash" \
      'export PATH="$HOME/.local/bin:$PATH"' \
      >> /home/${USERNAME}/.bashrc

# Runtime env vars that must apply to every process live in docker-compose.yml.
ENV DEBIAN_FRONTEND=
CMD ["sleep", "infinity"]
