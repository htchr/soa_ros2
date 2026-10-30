# soa_ros2 Docker environment

A single container image that provides the complete Standard Open Arm development
environment — ROS 2 Humble, MoveIt, RealSense, `lerobot` + CUDA PyTorch, and every
external ROS package the project depends on, pre-built and ready to use.

You can use it two ways, both driven by the **same** configuration in this
directory:

- **Terminal** — `docker/container.sh` + `docker/docker-compose.yml`
- **VS Code devcontainer** — `.devcontainer/devcontainer.json` references the same
  compose file, so "Reopen in Container" gives you an identical environment.

> ⚠️ **The first build takes a long time** (tens of minutes to well over an hour,
> depending on your machine and network). It installs a full ROS 2 desktop, MoveIt,
> RealSense, `lerobot[all]` with CUDA PyTorch, and compiles all of the external ROS
> packages. Start it and go get a coffee. Subsequent starts are fast.

---

## Files (single source of truth)

Everything that defines the container lives here in `docker/`:

| File | Purpose |
| --- | --- |
| `Dockerfile` | The one image: CUDA + ROS 2 Humble + MoveIt/RealSense + lerobot/torch, clones & pre-builds the external repos, creates the `ubuntu` user. |
| `docker-compose.yml` | Runtime config — mounts, devices, GPU, environment, privileged/host-network. Referenced by both `container.sh` and the devcontainer. |
| `soa.repos` | `vcstool` manifest — the list of external ROS packages cloned into the workspace. Edit this to add/remove dependencies. |
| `scripts/build-workspace.sh` | First-run `rosdep` + `colcon build` of the mounted `soa_ros2`. |
| `scripts/setup-runtime.sh` | Per-start USB latency / buffer / v4l2 fixes and the huggingface cache symlink. |
| `container.sh` | The terminal CLI wrapper around docker compose. |
| `.env` | Default `HOST_UID`/`HOST_GID` for the devcontainer build. |

The devcontainer at `../.devcontainer/devcontainer.json` intentionally contains no
container configuration of its own — it points at `docker-compose.yml` and runs the
same two scripts above.

---

## Prerequisites

1. **Docker** with the Compose plugin (`docker compose`).
2. **NVIDIA Container Toolkit** (for GPU access). See "No NVIDIA GPU?" below if you
   don't have one.
3. **Submodules initialised** so `feetech_ros2_driver` is present in the mount:
   ```bash
   git submodule update --init --recursive
   ```

---

## Quick start (terminal)

From the repository root:

```bash
# 1. Build the image (this is the very long step — first time only)
./docker/container.sh build

# 2. Start the container. On the first start it also builds the ROS workspace
#    (compiles soa_ros2 on top of the pre-built external packages).
./docker/container.sh start

# 3. Open a shell inside it
./docker/container.sh enter
```

You'll land in `/home/ubuntu/colcon_ws/src/soa_ros2` with ROS 2 and the workspace
already sourced. Try:

```bash
ros2 launch soa_bringup soa_bringup.launch.py
```

### `container.sh` commands

| Command | What it does |
| --- | --- |
| `build` | Build the image. Pass extra args through, e.g. `build --pull`. |
| `start` | Create/start the container, apply device fixes, and build the workspace on first run. |
| `enter` | Open an interactive shell (starts the container first if needed). |
| `stop` | Stop the container but keep it around for a fast next start. |
| `rebuild` | Rebuild the image from scratch (`--no-cache`), recreate the container, rebuild the workspace. |
| `down` | Remove the container entirely. |
| `status` | Show container status (`docker compose ps`). |
| `logs` | Follow the container logs. |

---

## Quick start (VS Code devcontainer)

1. Install the **Dev Containers** extension.
2. Open the repository in VS Code.
3. Run **"Dev Containers: Reopen in Container"**.

VS Code builds/starts the same compose service, runs `build-workspace.sh` once
(`postCreateCommand`) and `setup-runtime.sh` on every start (`postStartCommand`),
and opens the workspace at `/home/ubuntu/colcon_ws/src/soa_ros2`.

> The terminal (`container.sh`) and the devcontainer share the **image**, but run as
> separate container instances. Use one workflow at a time.

---

## Workspace layout inside the container

```
/home/ubuntu/colcon_ws/
├── build/  install/  log/          # colcon output (lives in the container)
└── src/
    ├── lerobot-robot-rosetta/       ┐
    ├── lerobot-teleoperator-rosetta/│
    ├── rosetta/                     │
    ├── rosetta_interfaces/          │  cloned from docker/soa.repos
    ├── pymoveit2/                   │  and pre-built into the image
    ├── warehouse_ros_mongo/         │  (ros2 branch)
    ├── yolo_ros/                    │
    ├── ros2_handeye_calibration/    │
    ├── aruco_tracker_ros2/          ┘
    └── soa_ros2/                    ← this repo, bind-mounted (live edits)
        └── feetech_ros2_driver/     ← soa_ros2's own submodule, via the mount
```

---

## How persistence & permissions work

- **Your code** (`soa_ros2`) is bind-mounted from the host, so every edit you make
  inside the container is saved on the host and survives exit.
- The container's **`ubuntu` user is created with your host uid/gid** (auto-detected
  by `container.sh`; defaults to `1000` for the devcontainer via `.env`). Files
  created in the container are therefore owned by you on the host — no `sudo chmod`
  needed. If your host user isn't `1000`, set `HOST_UID`/`HOST_GID` in `.env` before
  opening the devcontainer.
- **Huggingface cache**: `~/.cache/huggingface` inside the container is symlinked to
  `soa_ros2/huggingface` (which is git-ignored), so downloaded models and datasets
  persist on the host.
- **Build artifacts** (`build/`, `install/`, `log/`) live inside the container, not
  on the host. They persist across `stop`/`start`; a `rebuild` (or `down`) discards
  them and they are rebuilt.

---

## Rebuilding after code changes

Your source is live-mounted, so just recompile inside the container:

```bash
cd /home/ubuntu/colcon_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means Python edits take effect without rebuilding.

---

## Device / hardware fixes

`scripts/setup-runtime.sh` runs on every start (it needs the live host devices) and
applies the same fixes as the old `setup.sh`:

- USB-serial **latency timer → 1** (feetech servos and other serial devices),
- larger **USB/network buffers** (`rmem_max`/`wmem_max`),
- **v4l2 camera** permissions (`chmod 666 /dev/video*`).

The container runs `privileged` with `network_mode: host` and mounts `/dev` so USB
servos and cameras are visible.

---

## GPU / display notes

### No NVIDIA GPU?

The compose file requests all GPUs via the NVIDIA Container Toolkit. On a machine
without an NVIDIA GPU, comment out the `deploy:` block in `docker-compose.yml`:

```yaml
    # deploy:
    #   resources:
    #     reservations:
    #       devices:
    #         - driver: nvidia
    #           count: all
    #           capabilities: [gpu]
```

(Note that `lerobot`/YOLO inference still expect CUDA to be genuinely useful.)

### GUI apps (rviz, rerun, MoveIt)

`container.sh` runs `xhost +local:root` for you and `DISPLAY` is passed through. If a
GUI window fails to open, run on the host:

```bash
xhost +local:root
```

---

## Troubleshooting

- **`could not select device driver "nvidia"`** — the NVIDIA Container Toolkit isn't
  installed/configured, or you're on a GPU-less machine. Install the toolkit or
  comment out the `deploy:` block (see above).
- **Files owned by the wrong user on the host** — your host uid/gid isn't `1000`.
  Set `HOST_UID`/`HOST_GID` in `.env`, then `./docker/container.sh rebuild`.
- **`feetech_ros2_driver` is empty / won't build** — run
  `git submodule update --init --recursive` on the host and start again.
- **The image build fails on one external package** — a package in `soa.repos` may
  need an extra apt/pip dependency. Add it to the relevant section of the
  `Dockerfile` and rebuild. The build is intentionally strict so problems surface
  early.
