# grunt_docker • Product Requirements Doc (PRD) v0.1

**Purpose**
Unify all ROS 2 + Docker configurations for the *Grunt* sentry platform into one repo (this repo), while documenting non‑containerized setups that participate in the same robot graph. Optimize for clarity for new contributors and repeatable, cross‑arch builds.

---
## 1) Executive summary

### The System Architecture
The Grunt sentry platform consists of **autonomous mobile robots** operating in field environments and **remote operator workstations** providing visualization, teleoperation, and computational offload. These systems form a **unified ROS 2 graph** communicating over a private VPN (ZeroTier), enabling:

- **Robots in the field** running core autonomy, navigation, and sensor fusion
- **Windows/WSL2 workstations** running visualization (RViz2, Foxglove) and development tools
- **Cloud/edge compute** (future) for ML inference and heavy processing

### The Docker Challenge
While robots traditionally run ROS 2 **natively on Ubuntu** for direct hardware access and real-time performance, operator workstations face different constraints:

- **Windows 11 primary environment** for most developers/operators
- **Frequent dependency changes** as packages are added/removed during development
- **Multiple ROS distro versions** (Humble stable, Jazzy migration testing)
- **Cross-architecture development** (x86_64 workstations, ARM64 Jetson/RPi robots)

Docker provides **dependency isolation and reproducibility** for operator workstations while keeping robots on **native installs for performance**. This repo documents **both approaches** and the patterns to make them interoperate.

### Deployment Model

**Robots (autonomous, field-deployed):**
- **Barney** (x86_64 NUC, Ubuntu 22.04 + Humble): Primary development platform, all nodes native
- **Betty** (Jetson Orin Nano, Ubuntu 22.04 + Humble): GPU-accelerated perception, hybrid native+container
- **BamBam** (RPi 5, Ubuntu 24.04 + Jazzy): Migration testbed for Jazzy distro

**Operator Workstations (Windows 11 + WSL2):**
- **Visualization containers**: RViz2, RQT, PlotJuggler via WSLg (Wayland)
- **Web-based tools**: Foxglove Bridge, Vizanti (headless containers)
- **Development environments**: VSCode devcontainers with full ROS 2 toolchain

### Network Topology
- **Single ROS_DOMAIN_ID** shared across all participants (robots + workstations)
- **ZeroTier VPN** provides L2 connectivity across physical networks
- **Fast DDS** middleware (default); exploring **Zenoh** for Jazzy migrations
- **Namespace convention**: `/<robot>/<subsystem>` (e.g., `/barney/nav`, `/betty/camera`)

### Priorities (v0.1)
1. **WSL2 visualization stacks** - containerized RViz2/RQT with WSLg/Wayland support
2. **Betty bring-up** - Jetson containers with NVIDIA runtime, OAK-D-Lite camera
3. **Shared networking docs** - ZeroTier setup, DDS tuning, domain/namespace conventions
4. **Barney documentation** - native install patterns (no containerization yet)

---
## 2) Platforms & roles
| Host | CPU/GPU | OS | ROS | Role | Container stance |
|---|---|---|---|---|---|
| **Barney** | x86_64 iGPU | Ubuntu 22.04 | Humble | Primary dev robot | Native (host) for core nodes; optional per‑subsystem containers later |
| **Betty** | ARM64 + Jetson GPU | Ubuntu 22.04 (JetPack 6.2) | Humble initially | GPU accel; OAK‑D‑Lite | Mix of host + containers. Use NVIDIA Container Toolkit. |
| **BamBam** | RPi 5 (ARM64) | Ubuntu 24.04 | Jazzy | Migration testbed | Documented; optional isolated ROS graph if needed |
| **Windows 11 workstations (WSL2)** | x86_64 + some GPU | WSL2 Ubuntu 24.04 | Humble/Jazzy client tools | Visualization, dev tools, occasional compute | Prefer containers for GUI stacks; WSLg Wayland.

---
## 3) Packages likely in play (per Barney today)
**audio_common, by_your_command, grunt, realsense-ros, RTK_GPS_NTRIP, whisper_ros, bno055, p2os2, roarm_ws_em0, vizanti**
Note: Devices may be swapped per robot (e.g., OAK‑D‑Lite vs RealSense). Repo will provide container stubs or host notes for each.

---
## 4) Networking & graph conventions
- **ZeroTier** for private overlay. All hosts join the same network; per‑host setup remains in ops docs.
- **ROS_DOMAIN_ID** shared by default. Optionally segment BamBam or per‑sim runs with distinct domain IDs.
- **Namespaces**: `/<robot>/<subsystem>` (e.g., `/barney/nav`, `/betty/camera`).
- **TF frames**: Consistent base (`base_link`, `odom`, `map`, `camera_*`).
- **QoS defaults**: Follow Humble profiles; sensor data topics use `SensorDataQoS` when available. Provide rosbag2 QoS override templates.

---
## 5) Repository layout (proposed)
```
/ (repo root)
├─ docs/                      # newcomer-friendly walkthroughs
│  ├─ getting-started.md
│  ├─ networking-zerotier.md
│  ├─ ros-graph-conventions.md
│  ├─ wsl2-visualization.md
│  ├─ dds-fastdds-tuning.md
│  ├─ jazzy-zenoh-notes.md
│  ├─ devices/
│  │  ├─ realsense-d455.md
│  │  ├─ oakd-lite.md
│  │  └─ webcam-c920.md
│  └─ robots/
│     ├─ barney.md
│     ├─ betty.md
│     └─ bambam.md
├─ compose/                   # docker compose stacks
│  ├─ viz/                    # visualization stacks for WSL2 + Linux
│  │  ├─ rviz.yaml
│  │  ├─ rqt.yaml
│  │  ├─ plotjuggler.yaml
│  │  ├─ foxglove-bridge.yaml
│  │  └─ vizanti.yaml
│  ├─ betty/                  # Jetson-targeted services
│  │  ├─ camera-oakd.yaml
│  │  ├─ perception.yaml
│  │  └─ tools.yaml
│  ├─ common/                 # shared services
│  │  ├─ ros-bridge.yaml      # (optional) rosbridge, foxglove-bridge
│  │  ├─ bagging.yaml         # rosbag2 recorder/player
│  │  └─ diagnostics.yaml
│  └─ sim/
│     └─ gazebo-garden.yaml   # host or containerized bridge + sim
├─ images/                    # Dockerfiles (multi-arch where useful)
│  ├─ base/                   # minimal ROS bases (humble, jazzy)
│  │  ├─ Dockerfile              # Multi-stage: base → overlay → dev (ARG ROS_DISTRO)
│  │  ├─ entrypoint.sh           # Workspace sourcing script
│  │  └─ dependencies.repos      # External ROS packages to build
│  ├─ viz/
│  │  ├─ Dockerfile.rviz
│  │  ├─ Dockerfile.rqt
│  │  ├─ Dockerfile.plotjuggler
│  │  ├─ Dockerfile.foxglove-bridge
│  │  └─ Dockerfile.vizanti
│  ├─ devices/
│  │  ├─ Dockerfile.realsense
│  │  └─ Dockerfile.oakd
│  └─ jetson/
│     ├─ Dockerfile.oakd.jetson   # FROM l4t-base, nvidia runtime
│     └─ Dockerfile.perception
├─ env/                       # .env templates; domain ids, network, DISPLAY/WAYLAND
│  ├─ env.example
│  ├─ env.wsl
│  └─ env.jetson
├─ udev/                      # example udev rules + helper scripts
│  └─ grunt_udev_rules.sh     # docs link back to grunt repo
├─ tools/
│  ├─ colcon-mixin-profiles/
│  ├─ rosbag2-qos-profiles/
│  └─ scripts/                # helper launchers, health checks
└─ samples/                   # copy‑paste recipes for humans
   ├─ wslg-rviz.md
   ├─ jetson-oakd.md
   └─ rosbag-rotate.md
```

---
## 6) Image strategy & tags
- **Base images**
  - Humble / Jazzy from OSRF where possible (`osrf/ros:*` for desktops + tools).
  - Jetson: start FROM `nvcr.io/nvidia/l4t-base` (JetPack 6.2), add ROS via apt or build.
- **Multi-arch**: Build x86_64 + arm64 using `docker buildx` for non‑Jetson images. Jetson images built natively on device (or via cross tools later).
- **Tagging**: `pondersome/grunt_<role>:<distro>-<arch>` e.g., `grunt_viz:humble-amd64`, `grunt_oakd:humble-l4t`.
- **Runtime**: `--gpus all` for NVIDIA; `--device` mounts for cameras; grant `/dev/bus/usb` when needed.

---
## 6.5) Workspace Layering Convention

Images use a **three-tier workspace strategy**:

1. **`/ros2_ws`** (base image, immutable)
   - External dependencies from `dependencies.repos`
   - Built once, cached in image layers
   - Example: p2os2, grunt core packages

2. **`/overlay_ws` or `/sim_ws`** (overlay image, semi-stable)
   - Robot-specific packages (navigation, perception)
   - Rebuilt when robot code changes
   - Example: grunt_bringup, simulation worlds

3. **`~/dev_ws`** (bind-mounted, ephemeral)
   - Active development code
   - Mounted from host via `-v ./grunt_ws:/home/dev/dev_ws`
   - Built inside container with `colcon build`

**Sourcing order** (in entrypoint.sh):
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash
source /overlay_ws/install/setup.bash  # if exists
# Dev workspace sourced manually by developer
```

---
## 6.6) User Account Strategy

**Build-time (all stages):**
- Default user: `root` (required for apt, system modifications)

**Runtime users by deployment:**

| Image Stage | User | UID | Purpose |
|-------------|------|-----|---------|
| `base` | `root` | 0 | System services, not for interactive use |
| `dev` | `dev` | 1000 | Interactive development, matches host UID |
| `robot` (future) | `robot` | 1001 | Production robot nodes, limited privileges |

**Dev image specifics:**
- Remove default `ubuntu` user (UID 1000 conflict)
- Create `dev` user with `sudo` access
- Match host UID/GID for bind-mount permissions
- Add to `fuse` group for AppImage support

---
## 6.7) Python Dependency Isolation

To avoid `--break-system-packages` polluting system Python:

**System Python** (`/usr/bin/python3`):
- ROS 2 runtime packages only
- No pip installs after base image creation

**Build tools venv** (`/opt/venv`):
- vcstool, rosdep, colcon-common-extensions
- Activated in Dockerfile: `ENV PATH="/opt/venv/bin:$PATH"`
- Used for workspace builds

**Workspace-specific venvs** (future):
- Per-workspace Python dependencies (ML/vision)
- Example: `/ros2_ws/venv` for TensorFlow/PyTorch

---
## 7) Compose stacks (first wave)
### 7.1 Visualization on Windows 11 + WSL2 (WSLg/Wayland Primary)

**Goal:** One-liner `docker compose -f compose/viz/rviz.yaml up` runs RViz2 with Wayland.

**Mounts (WSLg/Wayland):**
```yaml
volumes:
  - /mnt/wslg:/mnt/wslg
environment:
  - WAYLAND_DISPLAY=${WAYLAND_DISPLAY}
  - XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}
  - PULSE_SERVER=${PULSE_SERVER}
```

**Optional X11 fallback** (for apps without Wayland support):
```yaml
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:ro
environment:
  - DISPLAY=${DISPLAY}
```

**Network:** Use `network_mode: host` for DDS discovery (requires Docker-in-WSL, not Docker Desktop).

### 7.2 Foxglove Bridge & Vizanti
- Headless containers exposing web ports (e.g., 8765 for foxglove‑bridge; 8080 for Vizanti proxy).
- Optionally co‑locate with rosbridge for widest compatibility.

### 7.3 rosbag2 services
- `bagging.yaml` includes record/play with optional QoS override file mounts. Log rotation via `--max-bag-size` / `--max-splits` and cron‑like sidecar.

### 7.4 Gazebo Garden (Humble)
- Provide `sim/gazebo-garden.yaml` that launches `ros_gz_bridge` + examples. Note: Garden on Humble is possible but may require unofficial packages; document caveats and link upstream guidance.

### 7.5 Betty (Jetson) camera/OAK‑D
- Two variants:
  1) **Host‑native** DepthAI + ROS nodes, plus containerized tools.
  2) **Containerized** DepthAI pipeline (with `/dev/bus/usb`, `--privileged` or udev rules) + ROS wrapper.
- Provide compose targeting the NVIDIA runtime and JetPack 6.2.

---
## 8) Device access & udev
- Standardize predictable symlinks (`/dev/grunt_*`).
- In containers: pass devices explicitly (e.g., `--device=/dev/grunt_imu`) or map `/dev/bus/usb`. Avoid fully‑privileged unless required.
- Include `udev/grunt_udev_rules.sh` mirror with pointers back to canonical scripts in the **grunt** repo.

---
## 9) DDS/transport profiles
- Default **Fast DDS**. Provide `config/fastrtps_profiles.xml` examples for:
  - interface whitelisting (bind to ZeroTier),
  - participant discovery tweaks for multi‑LAN,
  - lowered traffic for constrained devices.
- **Jazzy + zenoh** notes: add `rmw_zenoh` quickstart and router patterns for multi‑site graphs.

---
## 10) Registry & build infra (scaffolding)
- **Registry**: GHCR (`ghcr.io/pondersome/*`) for all multi-arch images. See `docs/ghcr-setup.md` for authentication setup.
- **Build**: scripts for local `buildx bake`; future:
  - GH Actions matrix (x86_64) + on‑device Jetson builds; caching via `actions/cache` or registry cache.

---
## 11) Logging & storage
- rosbag2 rotation defaults; write to host bind volume `~/grunt_data/bags` (create on first run).
- Per‑service log volumes with size caps; optional Loki/Grafana later.

---
## 12) Security & secrets (out of scope)
- Assume env‑injection available at runtime. Provide `.env.example` with placeholders.

---
## 13) Contributor experience
- **Docs first**: every compose/image has a short README with copy‑paste run lines.
- **High verbosity** in Dockerfiles and compose comments.
- **Cookbooks** in `samples/` for common tasks (WSLg RViz, Jetson OAK‑D, rosbag capture, Foxglove).

---
## 14) Acceptance criteria (v0.1)
- ✅ `compose/viz/rviz.yaml` runs RViz on Windows 11 WSL2 (Wayland).
- ✅ `compose/viz/foxglove-bridge.yaml` exposes a working WebSocket bridge to a live graph.
- ✅ `compose/betty/camera-oakd.yaml` runs on Jetson with OAK‑D‑Lite connected and publishes at least one camera topic.
- ✅ Docs exist for ZeroTier join + ROS_DOMAIN_ID + namespace conventions.
- ✅ Barney host‑native instructions linked; no regression to current workflows.

---
## 15) Risks & gotchas
- **WSLg Wayland socket quirks** (missing `wayland-0` in `XDG_RUNTIME_DIR`; provide auto‑symlink snippet).
- **DDS multicast across VPNs**: Fast DDS may need profile tuning to stay on ZeroTier and avoid broadcast storms.
- **Jetson driver / CUDA** inside containers: must match JetPack; use NVIDIA Container Toolkit and L4T images.
- **Garden on Humble**: works with caveats/unofficial packages; contributors should prefer supported combos.

---
## 16) Roadmap (next)
1) Implement first‑wave compose files & base images.
2) Add `docs/` walkthroughs for WSL2 + RViz, Foxglove, Vizanti.
3) Bring up Betty camera pipeline (host & container variants).
4) Add rosbag2 QoS override templates + scripts.
5) Evaluate zenoh on BamBam (Jazzy) with a minimal router.

---
## 17) Helpful commands (to be refined)
```bash
# Enable buildx and create a multi-arch builder
docker buildx create --use --name gruntx || docker buildx use gruntx

# Build a generic viz image for Humble (x86_64 + arm64)
DOCKER_BUILDKIT=1 docker buildx build \
  -f images/viz/Dockerfile.rviz \
  --platform linux/amd64,linux/arm64 \
  -t pondersome/grunt_viz:humble --push .

# Run RViz via compose on WSL2
cd compose/viz && docker compose -f rviz.yaml up --build
```

---
## 18) Appendix: compose snippets (illustrative)
### A) RViz on WSLg (Wayland)
```yaml
# compose/viz/rviz.yaml
services:
  rviz:
    image: pondersome/grunt_viz:humble
    environment:
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
      # WSLg/Wayland (primary)
      - WAYLAND_DISPLAY=${WAYLAND_DISPLAY}
      - XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}
      - PULSE_SERVER=${PULSE_SERVER}
    volumes:
      # WSLg Wayland socket
      - /mnt/wslg:/mnt/wslg
      # Optional: X11 fallback
      # - /tmp/.X11-unix:/tmp/.X11-unix:ro
    network_mode: host
    command: ["rviz2"]  # entrypoint.sh handles sourcing
```

### B) Foxglove Bridge (headless)
```yaml
# compose/viz/foxglove-bridge.yaml
services:
  foxglove:
    image: ghcr.io/foxglove/foxglove-bridge:latest
    environment:
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    network_mode: host
    command: ["/foxglove-bridge","--port","8765"]
```

### C) Jetson OAK‑D‑Lite (DepthAI) on Humble
```yaml
# compose/betty/camera-oakd.yaml
services:
  oakd:
    image: pondersome/grunt_oakd:humble-l4t
    runtime: nvidia
    environment:
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    devices:
      - /dev/bus/usb:/dev/bus/usb
    network_mode: host
    command: ["bash","-lc","source /opt/ros/humble/setup.bash && ros2 launch depthai_ros_driver camera.launch.py"]
```

---
## 19) External links to include in READMEs (curated)
*(Add these into the relevant docs pages with short explanations for newcomers.)*
- ROS 2 Humble/Jazzy docs; QoS primer; rosbag2 QoS overrides.
- Docker Compose v2 reference.
- WSLg GUI apps guide & troubleshooting notes for Wayland sockets.
- NVIDIA Container Toolkit + L4T images for Jetson; JetPack 6.2 docs.
- Gazebo Garden + ros_gz bridge (Humble caveats noted).
- DepthAI (OAK‑D) + ROS 2 driver.
- Foxglove Bridge; Vizanti.

---
*End of PRD v0.1*
