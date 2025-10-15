# grunt_docker

**Container infrastructure for the Grunt sentry robot platform**

This repository provides Docker images and compose configurations for running ROS 2 Humble and Jazzy workloads across x86_64 and ARM64 platforms, with emphasis on **operator workstations (Windows 11 + WSL2)** and **development environments**.

---

## Quick Start

> **Note**: This guide currently focuses on **Windows 11 + WSL2** as the primary development platform. The containers also work on **native Ubuntu Linux** and **cloud instances** - those configurations just use standard Docker practices without WSLg. See [Platform Support](#platform-support) below.

### Prerequisites (WSL2)

- **WSL2 with Ubuntu 24.04** (Windows 11)
- **Docker CE** (native Docker in WSL2, not Docker Desktop)
  - See [docs/native-docker-wsl2-setup.md](docs/native-docker-wsl2-setup.md) for installation
- **WSLg** enabled (GUI support via Wayland)
- **ZeroTier VPN** installed on WSL2 (for robot network access)

### Launch RViz2

```bash
# Pull the dev image (includes MoveIt2, Nav2, RealSense, audio_common dependencies)
docker pull ghcr.io/pondersome/grunt:humble-dev

# Launch RViz2 with WSLg (Wayland GUI)
docker compose -f compose/viz/rviz.yaml up
```

### Launch RQT

```bash
docker compose -f compose/viz/rqt.yaml up
```

### Interactive Bash Session

```bash
# For development and debugging
docker compose -f compose/viz/bash.yaml up
```

---

## Platform Support

The images and compose files work across multiple platforms:

| Platform | Status | GUI Support | Notes |
|----------|--------|-------------|-------|
| **Windows 11 + WSL2** | **Primary focus** | WSLg (Wayland) | Current documentation emphasizes this platform |
| **Native Ubuntu 22.04/24.04** | Supported | Wayland (default) or X11 | Compose files work as-is, environment passthrough automatic |
| **Other Linux distros** | Supported | X11 (typical) | May require xhost configuration for X11 forwarding |
| **Cloud instances (headless)** | Supported | None | For headless services (no RViz/RQT), use bash.yaml or custom compose |
| **macOS** | Not tested | XQuartz (X11) | Should work with X11 forwarding, not documented |
| **Jetson (ARM64)** | Planned (Phase 4) | Wayland or X11 | Native Docker on JetPack 6.2 |

### Running on Native Ubuntu Linux

The same images and compose files work on native Ubuntu without modification:

```bash
# Install Docker CE (same as WSL2)
# See docs/native-docker-wsl2-setup.md for instructions

# Ubuntu 22.04/24.04 use Wayland by default - just works
docker compose -f compose/viz/rviz.yaml up

# X11 session (older distros or manual X11 selection)
export DISPLAY=:0
xhost +local:docker
docker compose -f compose/viz/rviz.yaml up
```

The compose files mount both Wayland sockets and X11 paths, with environment passthrough for both `WAYLAND_DISPLAY` and `DISPLAY`, so they work on modern Ubuntu (Wayland) and older/non-Ubuntu distros (X11).

**Note**: On native Ubuntu, `/mnt/wslg` won't exist (WSL2-specific), but the compose file gracefully handles missing mounts.

### Running Headless (Cloud Instances)

For headless environments (no GUI), use bash.yaml for CLI-only access:

```bash
# Headless container for ROS 2 CLI tools
docker compose -f compose/viz/bash.yaml up

# Or run containers without GUI tools
docker run -it --rm --network=host \
  ghcr.io/pondersome/grunt:humble-dev \
  bash
```

Future phases will add headless web-based tools (Foxglove Bridge, Vizanti) better suited for cloud deployments.

### Why WSL2 Focus?

The current documentation emphasizes WSL2 because:
- **Primary operator platform** for this project (Windows 11 workstations)
- **WSLg requires specific setup** (socket mounts, environment variables)
- **Native Linux is simpler** (standard Docker + X11, well-documented elsewhere)
- **Most complex case first** - if it works on WSL2, it works elsewhere

Native Linux users can follow the same workflow but ignore WSLg-specific sections.

---

## Architecture Overview

### Deployment Model

The Grunt platform uses a **hybrid containerization strategy**:

**Robots (field-deployed, autonomous):**
- **Native ROS 2 installs** for direct hardware access and real-time performance
- **Barney** (x86_64 NUC): Ubuntu 22.04 + Humble, all nodes native
- **Betty** (Jetson Orin Nano): Ubuntu 22.04 + Humble, hybrid native+container for GPU workloads
- **BamBam** (RPi 5): Ubuntu 24.04 + Jazzy, migration testbed

**Operator Workstations (Windows 11 + WSL2):**
- **Containerized visualization**: RViz2, RQT, PlotJuggler via WSLg
- **Development environments**: Multi-distro support (Humble/Jazzy) with bind-mounted workspaces
- **Web-based tools**: Foxglove Bridge, Vizanti (headless containers)

### Network Topology

- **Single ROS_DOMAIN_ID** shared across all participants (robots + workstations)
- **ZeroTier VPN** provides Layer 2 connectivity across physical networks
- **Fast DDS** middleware (default, with optional Cyclone DDS)
- **Namespace convention**: `/<robot>/<subsystem>` (e.g., `/barney/nav`, `/betty/camera`)

---

## Image Strategy

### Base Image (Multi-Stage Dockerfile)

The `base/Dockerfile` uses a **multi-stage build** with two stages:

#### 1. Base Stage (`ghcr.io/pondersome/grunt:humble`)

**Purpose**: Core ROS 2 environment for deployment and basic operations

**Includes**:
- ROS 2 Humble/Jazzy Desktop (from `osrf/ros:*-desktop`)
- Gazebo Harmonic (via gz-harmonic vendor packages)
- Core dependencies: Cyclone DDS, xacro, RViz2, image_transport
- External packages from `dependencies.repos` built into `/ros2_ws`
- Groot2 AppImage (Behavior Tree visualization)
- Python virtual environment (`/opt/venv`) with colcon, vcstool, rosdep

**User**: `dev` (UID 1000, matches typical WSL2 user for bind-mount permissions)

#### 2. Dev Stage (`ghcr.io/pondersome/grunt:humble-dev`)

**Purpose**: Development environment with additional packages for manipulation, navigation, and perception

**Adds to base**:
- **MoveIt2** - manipulation and motion planning (required for roarm_description)
- **Nav2** - autonomous navigation stack (path planning, behavior trees, collision avoidance)
- **Intel RealSense SDK** - camera drivers (userspace libraries, not kernel modules)
- **ROS 2 Control** - controller_manager, ros2_control, ros2_controllers
- **Audio Common dependencies** - PortAudio, ALSA, GStreamer (for by_your_command voice control)
- **Optional MoveIt2 packages** - visual tools, example configs, python helpers

**Dev Layer Inventory**:

| Category | Baked into Image (dev stage) | Built from Source (workspaces) |
|----------|-------------------------------|--------------------------------|
| **Manipulation** | MoveIt2 core (`ros-humble-moveit`) | roarm_description, roarm_ws_em0 |
| **Navigation** | Nav2 full stack (`ros-humble-navigation2`, `nav2-bringup`) | Custom nav configurations |
| **Perception** | RealSense SDK (librealsense2-dev, librealsense2-utils) | realsense-ros wrapper |
| **Control** | ros2_control, ros2_controllers, controller_manager | Custom controllers |
| **Audio** | PortAudio, ALSA, GStreamer plugins | audio_common, by_your_command, whisper_ros |
| **Core** | ROS 2 Desktop, Gazebo Harmonic, Cyclone DDS | grunt, p2os2, bno055 |

### Multi-Distro Support

Both stages support **Humble and Jazzy** via build argument:

```bash
# Build Humble dev image
docker buildx build --build-arg ROS_DISTRO=humble --target dev -t grunt:humble-dev .

# Build Jazzy dev image
docker buildx build --build-arg ROS_DISTRO=jazzy --target dev -t grunt:jazzy-dev .
```

Compose files use `${ROS_DISTRO:-humble}` to default to Humble but allow override:

```bash
# Use Jazzy instead
ROS_DISTRO=jazzy docker compose -f compose/viz/rviz.yaml up
```

### Multi-Architecture Builds

Images support **x86_64 (amd64)** and **ARM64** via Docker buildx:

```bash
# Build multi-arch and push to GHCR
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=humble \
  --target dev \
  -t ghcr.io/pondersome/grunt:humble-dev \
  --push \
  -f base/Dockerfile .
```

See [docs/ghcr-setup.md](docs/ghcr-setup.md) for GHCR authentication and buildx setup.

---

## Workspace Strategy

Images use a **three-tier workspace layering convention**:

### 1. `/ros2_ws` (Base Image, Immutable)

- External dependencies from `dependencies.repos` (p2os2, grunt core packages)
- Built once during image creation, cached in Docker layers
- Sourced automatically in `entrypoint.sh`

### 2. `~/dev_ws` and `~/sim_ws` (Bind-Mounted, Persistent)

- Active development code, bind-mounted from host WSL2 filesystem
- **Host path**: `~/ros2/{humble,jazzy}/{dev_ws,sim_ws}`
- **Container path**: `/home/dev/dev_ws`, `/home/dev/sim_ws`
- Built inside container with `colcon build`
- Persists across container restarts

**Example workspace setup**:

```bash
# On WSL2 host, create distro-specific workspaces
mkdir -p ~/ros2/humble/{dev_ws,sim_ws}/src
mkdir -p ~/ros2/jazzy/{dev_ws,sim_ws}/src

# Clone your development repos
cd ~/ros2/humble/dev_ws/src
git clone https://github.com/pondersome/grunt.git
git clone https://github.com/pondersome/roarm_ws_em0.git
# ... other repos

# Build inside container
docker compose -f compose/viz/bash.yaml up
# Inside container:
cd ~/dev_ws
colcon build --symlink-install
```

### Sourcing Order

The `entrypoint.sh` automatically sources workspaces in correct order:

1. `/opt/ros/${ROS_DISTRO}/setup.bash` (ROS base install)
2. `/ros2_ws/install/setup.bash` (external dependencies)
3. `~/dev_ws/install/setup.bash` (if exists, development workspace)
4. `~/sim_ws/install/setup.bash` (if exists, simulation workspace)

---

## DDS Configuration

### Default: Multicast (rviz.yaml, rqt.yaml)

RViz and RQT containers use **default Fast DDS multicast** for maximum discovery:

- Discovers all topics on the ROS graph (local + remote)
- Works best for visualization tools that need full graph visibility
- Requires no manual peer configuration

### Unicast Mode (bash.yaml)

The bash container uses **unicast with explicit peers** for cross-VPN discovery:

- Configured via `FASTRTPS_DEFAULT_PROFILES_FILE=/dds_config/fastrtps_unicast.xml`
- Explicitly lists ZeroTier peers for discovery
- More restrictive but works reliably across NAT/VPN

**Configuration file**: `config/dds/fastrtps_unicast.xml`

See [docs/wsl2-visualization.md](docs/wsl2-visualization.md) for DDS troubleshooting.

---

## Compose Files

All compose files are located in `compose/viz/` and support multi-distro via environment variable:

| File | Purpose | DDS Mode | GUI |
|------|---------|----------|-----|
| `bash.yaml` | Interactive debug shell | Unicast | Yes (WSLg) |
| `rviz.yaml` | RViz2 visualization | Multicast | Yes (WSLg) |
| `rqt.yaml` | RQT GUI tools | Multicast | Yes (WSLg) |

### Common Features

All compose files:
- Use `network_mode: host` for DDS discovery
- Mount `/mnt/wslg` for WSLg/Wayland GUI support
- Bind-mount `~/ros2/${ROS_DISTRO:-humble}/dev_ws` and `sim_ws`
- Run as `dev` user (UID 1000) for proper file permissions
- Support `${ROS_DISTRO:-humble}` environment variable override

### Example Usage

```bash
# Default Humble
docker compose -f compose/viz/rviz.yaml up

# Override to Jazzy
ROS_DISTRO=jazzy docker compose -f compose/viz/rviz.yaml up

# Multiple containers simultaneously
docker compose -f compose/viz/bash.yaml -f compose/viz/rviz.yaml up
```

---

## Why Docker?

Docker has well-documented issues with ROS 2 ([ROS Docker: 6 reasons why they are not a good fit](https://ubuntu.com/blog/ros-docker)), particularly for direct hardware access and real-time performance. We agree with these concerns and use a **hybrid approach**:

**Where we DON'T use Docker:**
- Robot core nodes (navigation, localization, sensor fusion) - **native install**
- Real-time control loops requiring deterministic timing
- Direct hardware I/O (motors, IMUs, cameras on robots)

**Where we DO use Docker:**
- **Operator workstations** (Windows 11 + WSL2) for visualization and development
- **Development environments** with multi-distro testing (Humble/Jazzy)
- **Dependency isolation** during rapid prototyping
- **Headless services** (Foxglove Bridge, rosbag recorders, web tools)

Docker provides **dependency reproducibility** for operator workstations while keeping robots on **native installs for performance**.

---

## Repository Layout

```
grunt_docker/
├── base/
│   ├── Dockerfile              # Multi-stage: base + dev
│   ├── entrypoint.sh           # Workspace sourcing script
│   └── dependencies.repos      # External ROS packages
│
├── compose/
│   └── viz/                    # Visualization containers (WSL2)
│       ├── bash.yaml           # Interactive shell (unicast DDS)
│       ├── rviz.yaml           # RViz2 (multicast DDS)
│       └── rqt.yaml            # RQT (multicast DDS)
│
├── config/
│   ├── dds/
│   │   └── fastrtps_unicast.xml  # Unicast DDS profile for cross-VPN
│   └── rviz/
│       └── default.rviz          # Default RViz config
│
├── docs/
│   ├── ghcr-setup.md           # GHCR authentication & buildx
│   ├── native-docker-wsl2-setup.md  # Docker CE installation
│   ├── wsl2-visualization.md   # WSLg troubleshooting
│   └── ROADMAP.md              # Implementation roadmap
│
├── specs/
│   ├── grunt_docker_prd_v_0.md  # Product Requirements Doc
│   └── CRITIQUE_AND_BUILDX_GUIDE.md  # Architecture analysis
│
└── README.md                   # This file
```

---

## Documentation

- **[docs/docker-commands-cheatsheet.md](docs/docker-commands-cheatsheet.md)** - Quick reference for all Docker, buildx, and compose commands
- **[docs/native-docker-wsl2-setup.md](docs/native-docker-wsl2-setup.md)** - Docker CE installation on WSL2 (required for `buildx` and multi-arch builds)
- **[docs/wsl2-visualization.md](docs/wsl2-visualization.md)** - WSLg troubleshooting, DDS configuration, ZeroTier setup
- **[docs/dev-workflow.md](docs/dev-workflow.md)** - Dev layer pattern, workspace management, and development workflow
- **[docs/ghcr-setup.md](docs/ghcr-setup.md)** - GitHub Container Registry authentication and multi-arch build workflow
- **[docs/ROADMAP.md](docs/ROADMAP.md)** - Implementation roadmap and acceptance criteria
- **[specs/grunt_docker_prd_v_0.md](specs/grunt_docker_prd_v_0.md)** - Product Requirements Document with detailed architecture

---

## Contributing

### Building Images Locally

```bash
# Build base image (Humble, x86_64 only)
docker build --build-arg ROS_DISTRO=humble --target base -t grunt:humble -f base/Dockerfile .

# Build dev image (Humble, x86_64 only)
docker build --build-arg ROS_DISTRO=humble --target dev -t grunt:humble-dev -f base/Dockerfile .

# Build multi-arch (requires buildx)
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=humble \
  --target dev \
  -t ghcr.io/pondersome/grunt:humble-dev \
  --push \
  -f base/Dockerfile .
```

### Testing Changes

```bash
# Test locally built image
docker compose -f compose/viz/bash.yaml up

# Inside container, verify ROS environment
ros2 topic list
ros2 node list

# Test workspace builds
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Troubleshooting

### ROS 2 Daemon Timeout

If you see timeout warnings when running `ros2` commands:

```bash
# Inside container
ros2 daemon stop
ros2 daemon start
```

This is a known issue ([ros2/ros2#1531](https://github.com/ros2/ros2/issues/1531)) in Docker where the daemon can hang on first use.

### WSLg GUI Not Working

Verify WSLg sockets are mounted:

```bash
# On WSL2 host
echo $WAYLAND_DISPLAY
echo $XDG_RUNTIME_DIR
ls /mnt/wslg/
```

If missing, ensure WSLg is enabled in WSL2:

```bash
wsl --update
wsl --shutdown
# Restart WSL2
```

### DDS Discovery Issues

If containers can't see robot topics:

1. **Check ZeroTier**: Verify ZeroTier interface exists in WSL2 and container
   ```bash
   # On host and in container
   ifconfig | grep zt
   ```

2. **Check DDS profile**: bash.yaml uses unicast (requires peer list), rviz.yaml uses multicast
   ```bash
   # Inside container
   echo $FASTRTPS_DEFAULT_PROFILES_FILE
   cat /dds_config/fastrtps_unicast.xml
   ```

3. **Test with multicast**: Remove DDS profile env var to use default multicast
   ```yaml
   # In compose file, comment out:
   # - FASTRTPS_DEFAULT_PROFILES_FILE=/dds_config/fastrtps_unicast.xml
   ```

See [docs/wsl2-visualization.md](docs/wsl2-visualization.md) for detailed DDS troubleshooting.

---

## Known Issues

- **RViz frame rate slow on WSLg** - Wayland rendering overhead, investigating optimization
- **Camera/image view in RViz** - Some image transport plugins may not work with WSLg, use RQT image_view as workaround
- **ROS 2 daemon hangs** - Known Docker issue, manual restart required (see above)

---

## License

See the main [grunt repository](https://github.com/pondersome/grunt) for license information.

---

## Related Repositories

- **[grunt](https://github.com/pondersome/grunt)** - Main robot code (Barney native install)
- **[roarm_ws_em0](https://github.com/pondersome/roarm_ws_em0)** - Arm manipulation packages
- **[by_your_command](https://github.com/pondersome/by_your_command)** - Voice control interface

---

## Acknowledgments

Configuration guidance from:
- [Updated Guide: Docker and ROS 2](https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/)
- [turtlebot3_behavior_demos](https://github.com/sea-bass/turtlebot3_behavior_demos/tree/main/docker)
- [leorover_gazebo_sim_docker](https://github.com/pondersome/leorover_gazebo_sim_docker)
