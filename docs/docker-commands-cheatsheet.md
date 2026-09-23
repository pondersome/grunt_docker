# Docker Commands Cheat Sheet

Quick reference for common Docker, buildx, and compose commands used in this project.

---

## Table of Contents

- [Image Building](#image-building)
- [Image Management](#image-management)
- [Docker Compose](#docker-compose)
- [Vizanti](#vizanti)
- [Hal Audio](#hal-audio)
- [Running Containers Directly](#running-containers-directly)
- [Multi-Arch Setup](#multi-arch-setup)
- [Registry Operations](#registry-operations)
- [Debugging & Inspection](#debugging--inspection)

---

## Image Building

### Build Single Architecture (Local Testing)

```bash
# Build base stage for Jazzy (x86_64 only)
docker build \
  --build-arg ROS_DISTRO=jazzy \
  --target base \
  -t grunt:jazzy \
  -f base/Dockerfile .

# Build dev stage for Jazzy (x86_64 only)
docker build \
  --build-arg ROS_DISTRO=jazzy \
  --target dev \
  -t grunt:jazzy-dev \
  -f base/Dockerfile .

# Build legacy Humble dev stage
docker build \
  --build-arg ROS_DISTRO=humble \
  --target dev \
  -t grunt:humble-dev \
  -f base/Dockerfile .
```

### Build with Bake (preferred)

`docker-bake.hcl` encodes the full build matrix, so releases are one
command instead of repeated `buildx build` invocations:

```bash
# Local single-arch build of base + dev (loads into local Docker)
docker buildx bake

# Multi-arch release: build and push base + dev for amd64 + arm64,
# tagged :jazzy / :jazzy-dev plus a date tag
DATE_TAG=$(date +%Y%m%d) docker buildx bake --builder grunt-builder release --push

# Preview what a target will do without building
docker buildx bake --print release
```

### Build Multi-Architecture (raw buildx)

```bash
# Build and push multi-arch base stage (Jazzy)
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=jazzy \
  --target base \
  -t ghcr.io/pondersome/grunt:jazzy \
  --push \
  -f base/Dockerfile .

# Build and push multi-arch dev stage (Jazzy)
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=jazzy \
  --target dev \
  -t ghcr.io/pondersome/grunt:jazzy-dev \
  --push \
  -f base/Dockerfile .

# Build multi-arch for legacy Humble
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=humble \
  --target dev \
  -t ghcr.io/pondersome/grunt:humble-dev \
  --push \
  -f base/Dockerfile .

# Build locally without pushing (load to local Docker)
docker buildx build \
  --platform linux/amd64 \
  --build-arg ROS_DISTRO=jazzy \
  --target dev \
  -t grunt:jazzy-dev \
  --load \
  -f base/Dockerfile .
```

**Note**: `--push` requires registry authentication. Use `--load` for local testing (single architecture only).

---

## Image Management

### Pull Images

```bash
# Pull base image (Jazzy)
docker pull ghcr.io/pondersome/grunt:jazzy

# Pull dev image (Jazzy)
docker pull ghcr.io/pondersome/grunt:jazzy-dev

# Pull legacy Humble dev image
docker pull ghcr.io/pondersome/grunt:humble-dev
```

### List Local Images

```bash
# List all grunt images
docker images | grep grunt

# List all images with specific tag
docker images ghcr.io/pondersome/grunt

# Show image sizes
docker images --format "table {{.Repository}}:{{.Tag}}\t{{.Size}}"
```

### Remove Images

```bash
# Remove specific image
docker rmi grunt:jazzy-dev

# Remove all unused images
docker image prune -a

# Force remove image (even if containers exist)
docker rmi -f grunt:jazzy-dev
```

### Inspect Images

```bash
# Show image details (layers, env vars, etc.)
docker inspect ghcr.io/pondersome/grunt:jazzy-dev

# Show image history (layer sizes)
docker history ghcr.io/pondersome/grunt:jazzy-dev

# Show image architecture
docker inspect ghcr.io/pondersome/grunt:jazzy-dev | grep Architecture
```

---

## Docker Compose

### Basic Operations

```bash
# Start services (default: Jazzy)
docker compose -f compose/viz/rviz.yaml up

# Start services in background
docker compose -f compose/viz/rviz.yaml up -d

# Stop services
docker compose -f compose/viz/rviz.yaml down

# Stop and remove volumes
docker compose -f compose/viz/rviz.yaml down -v

# View logs
docker compose -f compose/viz/rviz.yaml logs

# Follow logs in real-time
docker compose -f compose/viz/rviz.yaml logs -f
```

### Multi-Distro Operations

```bash
# Use legacy Humble instead of Jazzy (default)
ROS_DISTRO=humble docker compose -f compose/viz/rviz.yaml up

# Set custom ROS_DOMAIN_ID
ROS_DOMAIN_ID=42 docker compose -f compose/viz/rviz.yaml up

# Combine environment variables
ROS_DISTRO=humble ROS_DOMAIN_ID=42 docker compose -f compose/viz/rviz.yaml up
```

### Visualization Services

```bash
# Launch RViz only
docker compose -f compose/viz/rviz.yaml up

# Launch RQT only
docker compose -f compose/viz/rqt.yaml up

# Launch both RViz and RQT (combined container - recommended)
docker compose -f compose/viz/viz-combined.yaml up

# Launch both RViz and RQT (separate containers)
docker compose -f compose/viz/rviz.yaml -f compose/viz/rqt.yaml up

# Interactive bash with multicast DDS (local network)
docker compose -f compose/viz/bash-multicast.yaml run --rm bash

# Interactive bash with unicast DDS (cross-VPN/NAT)
docker compose -f compose/viz/bash.yaml run --rm bash
```

### Web-Based Visualization

```bash
# Foxglove Bridge (web-based visualization)
docker compose -f compose/foxglove/bridge.yaml up

# Start in background (persistent monitoring)
docker compose -f compose/foxglove/bridge.yaml up -d

# View logs
docker compose -f compose/foxglove/bridge.yaml logs -f

# Stop bridge
docker compose -f compose/foxglove/bridge.yaml down

# Use custom port
FOXGLOVE_PORT=9000 docker compose -f compose/foxglove/bridge.yaml up
```

**Connect to bridge:**
- Web browser: http://localhost:8765
- Foxglove Studio: ws://localhost:8765
- Remote: ws://<admin-machine-ip>:8765

### Multiple Services

```bash
# Start multiple services simultaneously
docker compose -f compose/viz/bash.yaml -f compose/viz/rviz.yaml up

# Combined viz with bash (multicast)
docker compose -f compose/viz/bash-multicast.yaml -f compose/viz/viz-combined.yaml up

# RViz/RQT with Foxglove Bridge
docker compose -f compose/viz/viz-combined.yaml -f compose/foxglove/bridge.yaml up
```

### Service-Specific Operations

```bash
# Build specific service
docker compose -f compose/viz/rviz.yaml build

# Rebuild without cache
docker compose -f compose/viz/rviz.yaml build --no-cache

# Start specific service
docker compose -f compose/viz/rviz.yaml up rviz

# Restart service
docker compose -f compose/viz/rviz.yaml restart rviz

# Execute command in running service
docker compose -f compose/viz/bash.yaml exec bash ros2 topic list
```

---

## Vizanti

Vizanti normally runs as a **Docker container on hal**, the operator
workstation — not on the robot. Barney keeps a native
`vizanti.service` as a maintained fallback for when hal is down or off
the network, but it is **disabled** at boot (2026-09-22) so it can't
race the container.

**Only one instance at a time.** Both put identically named nodes on
the same graph. Check which is live before starting either:

```bash
# Which vizanti is on the graph?
ros2 node list | grep vizanti
```

| Node present | Which instance |
|---|---|
| `/vizanti_rosbridge` | hal's Docker container (rosbridge backend) |
| `/vizanti_rws_server` | Barney's native service (RWS backend) |

The other three nodes (`/vizanti_flask_node`,
`/vizanti_service_handler_node`, `/vizanti_tf_handler_node`) appear in
both, so the bridge node is the discriminator.

### Hal invocation

Vizanti is never baked into images — it builds from the bind-mounted
dev workspace (see [docs/vizanti-setup.md](vizanti-setup.md) for the
one-time setup and update loop).

```bash
# Start the server (from repo root)
docker compose -f compose/vizanti/server.yaml up -d

# Logs / restart / stop
docker compose -f compose/vizanti/server.yaml logs -f
docker compose -f compose/vizanti/server.yaml restart
docker compose -f compose/vizanti/server.yaml down

# Custom ports (defaults: 5000 web UI, 5001 rosbridge websocket)
VIZANTI_PORT=8000 VIZANTI_ROSBRIDGE_PORT=8001 \
  docker compose -f compose/vizanti/server.yaml up -d

# Health status
docker inspect vizanti_jazzy --format '{{json .State.Health}}'
```

**Operator URLs** (browser needs both ports reachable — UI on 5000,
websocket on 5001):
- Local / LAN: `http://localhost:5000`
- ZeroTier devices: `http://halbuntu.robodojo.net:5000` (works out of
  the box); `http://hal.robodojo.net:5000` after the one-time
  `tools\windows\vizanti-portproxy.ps1` (elevated PowerShell)

Robot-side fallback commands, operator URLs, and the rationale live in
the **ponderdocs** repo (separate checkout, sibling of this one under
`ros2_ws/`):

- `ponderdocs/grunt/docs/operators_cheatsheet.md` → "Vizanti web UI" —
  fallback service commands + URLs
- `ponderdocs/grunt/docs/operators_guide.md` → "Where vizanti runs" —
  why it's on hal, duplicate-node hazard

---

## Hal Audio

Runs byc's two audio hardware endpoints (mic capture + response
playback) on hal via WSLg PulseAudio, so the robot can use hal's
headset. Requires `audio_common` + `by_your_command` built in dev_ws —
see the header of `compose/audio/hal-audio.yaml`.

```bash
# Start (namespace must match byc on the robot)
AUDIO_NS=/grunt1/agent docker compose -f compose/audio/hal-audio.yaml up -d

# Logs / stop
docker compose -f compose/audio/hal-audio.yaml logs -f
docker compose -f compose/audio/hal-audio.yaml down

# Verify the mic topic is flowing (~31 Hz)
docker exec hal_audio_jazzy bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 topic hz /grunt1/agent/audio"
```

**Don't run this while the robot-local capturer is publishing in the
same namespace** — launch byc without its audio nodes when hal audio
is active. Note the capturer publishes best-effort (sensor-data) QoS;
custom subscribers need a matching QoS profile or they'll receive
nothing.

---

## Running Containers Directly

### Interactive Shells

```bash
# Interactive bash shell (Jazzy dev)
docker run -it --rm \
  --network=host \
  ghcr.io/pondersome/grunt:jazzy-dev \
  bash

# Interactive bash with workspace mounted
docker run -it --rm \
  --network=host \
  -v ~/ros2/jazzy/dev_ws:/home/dev/dev_ws:rw \
  --user dev \
  ghcr.io/pondersome/grunt:jazzy-dev \
  bash

# Interactive bash with WSLg GUI support
docker run -it --rm \
  --network=host \
  -v /mnt/wslg:/mnt/wslg:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -e DISPLAY=${DISPLAY} \
  -e WAYLAND_DISPLAY=${WAYLAND_DISPLAY} \
  -e XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR} \
  --user dev \
  ghcr.io/pondersome/grunt:jazzy-dev \
  bash
```

### Running Specific Tools

```bash
# Launch RViz2
docker run -it --rm \
  --network=host \
  -v /mnt/wslg:/mnt/wslg:rw \
  -e WAYLAND_DISPLAY=${WAYLAND_DISPLAY} \
  -e XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR} \
  --user dev \
  ghcr.io/pondersome/grunt:jazzy-dev \
  rviz2

# Launch RQT
docker run -it --rm \
  --network=host \
  -v /mnt/wslg:/mnt/wslg:rw \
  -e WAYLAND_DISPLAY=${WAYLAND_DISPLAY} \
  -e XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR} \
  --user dev \
  ghcr.io/pondersome/grunt:jazzy-dev \
  rqt

# Run ROS 2 CLI commands
docker run -it --rm \
  --network=host \
  --user dev \
  ghcr.io/pondersome/grunt:jazzy-dev \
  ros2 topic list

docker run -it --rm \
  --network=host \
  --user dev \
  ghcr.io/pondersome/grunt:jazzy-dev \
  ros2 node list
```

### With Custom DDS Configuration

```bash
# Run with unicast DDS profile
docker run -it --rm \
  --network=host \
  -v $(pwd)/config/dds:/dds_config:ro \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/dds_config/fastrtps_unicast.xml \
  --user dev \
  ghcr.io/pondersome/grunt:jazzy-dev \
  bash
```

---

## Multi-Arch Setup

### Initial Setup

```bash
# Install QEMU for cross-architecture emulation
sudo apt-get update
sudo apt-get install -y qemu-user-static

# Verify QEMU is registered
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Create buildx builder (this repo's builder is named grunt-builder)
docker buildx create --use --name grunt-builder --driver docker-container

# Or use existing builder
docker buildx use grunt-builder

# Inspect builder
docker buildx inspect --bootstrap

# List available builders
docker buildx ls
```

### Test Multi-Arch Build

```bash
# Test ARM64 emulation
docker run --rm --platform linux/arm64 ubuntu:24.04 uname -m
# Should output: aarch64

# Test multi-arch build without pushing
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=jazzy \
  --target base \
  -t test:multi \
  -f base/Dockerfile .
```

**Note**: Multi-arch builds without `--push` won't be loadable locally. Use `--platform linux/amd64 --load` to test locally.

---

## Registry Operations

### Authentication

```bash
# Login to GHCR (GitHub Container Registry)
echo $GITHUB_TOKEN | docker login ghcr.io -u USERNAME --password-stdin

# Verify login
docker login ghcr.io

# Logout
docker logout ghcr.io
```

### Push/Pull

```bash
# Tag local image for registry
docker tag grunt:jazzy-dev ghcr.io/pondersome/grunt:jazzy-dev

# Push to registry
docker push ghcr.io/pondersome/grunt:jazzy-dev

# Pull from registry
docker pull ghcr.io/pondersome/grunt:jazzy-dev

# Pull specific platform
docker pull --platform linux/arm64 ghcr.io/pondersome/grunt:jazzy-dev
```

### Inspect Registry Images

```bash
# Show manifest (architectures available)
docker buildx imagetools inspect ghcr.io/pondersome/grunt:jazzy

# Show manifest for specific tag
docker buildx imagetools inspect ghcr.io/pondersome/grunt:jazzy-dev
```

---

## Debugging & Inspection

### Container Management

```bash
# List running containers
docker ps

# List all containers (including stopped)
docker ps -a

# Show container logs
docker logs grunt_rviz_jazzy

# Follow container logs
docker logs -f grunt_rviz_jazzy

# Execute command in running container
docker exec -it grunt_bash_jazzy bash

# Inspect container details
docker inspect grunt_bash_jazzy

# Show container resource usage
docker stats grunt_bash_jazzy
```

### Container Cleanup

```bash
# Stop all running containers
docker stop $(docker ps -q)

# Remove all stopped containers
docker container prune

# Remove specific container
docker rm grunt_bash_jazzy

# Force remove running container
docker rm -f grunt_bash_jazzy
```

### Workspace Debugging

```bash
# Check workspace inside container
docker exec -it grunt_bash_jazzy ls -la /home/dev/dev_ws

# Check if workspace is sourced
docker exec -it grunt_bash_jazzy bash -c "source /home/dev/dev_ws/install/setup.bash && ros2 pkg list"

# Build workspace inside container
docker exec -it grunt_bash_jazzy bash -c "cd ~/dev_ws && colcon build --symlink-install"

# Check ROS environment
docker exec -it grunt_bash_jazzy printenv | grep ROS
```

### Network Debugging

```bash
# Check network interfaces inside container
docker exec -it grunt_bash_jazzy ip addr

# Check if ZeroTier interface is visible
docker exec -it grunt_bash_jazzy ifconfig | grep zt

# Test DDS discovery
docker exec -it grunt_bash_jazzy ros2 topic list

# Check DDS participants
docker exec -it grunt_bash_jazzy ros2 daemon status
docker exec -it grunt_bash_jazzy ros2 daemon stop
docker exec -it grunt_bash_jazzy ros2 daemon start
```

### GUI Debugging (WSLg/X11)

```bash
# Check WSLg environment variables
echo $WAYLAND_DISPLAY
echo $XDG_RUNTIME_DIR
echo $DISPLAY

# Check WSLg socket
ls -la /mnt/wslg/

# Test X11 connection
docker run --rm -it \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -e DISPLAY=${DISPLAY} \
  ubuntu:24.04 \
  bash -c "apt-get update && apt-get install -y x11-apps && xeyes"

# Test Wayland connection (WSLg)
docker run --rm -it \
  -v /mnt/wslg:/mnt/wslg:rw \
  -e WAYLAND_DISPLAY=${WAYLAND_DISPLAY} \
  -e XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR} \
  ubuntu:24.04 \
  bash -c "apt-get update && apt-get install -y weston && weston-info"
```

---

## Common Workflows

### Full Development Cycle

```bash
# 1. Pull latest dev image
docker pull ghcr.io/pondersome/grunt:jazzy-dev

# 2. Start interactive container
docker compose -f compose/viz/bash.yaml up

# 3. (In container) Build workspace
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash

# 4. (In another terminal) Launch visualization tools
# Option A: RViz and RQT (GUI)
docker compose -f compose/viz/viz-combined.yaml up

# Option B: Foxglove Bridge (web-based)
docker compose -f compose/foxglove/bridge.yaml up -d
# Then open browser to http://localhost:8765

# Option C: Both (GUI + web)
docker compose -f compose/viz/viz-combined.yaml -f compose/foxglove/bridge.yaml up

# 5. (In container) Test your nodes
ros2 launch my_package my_launch.py

# 6. When done, stop all containers
docker compose -f compose/viz/bash-multicast.yaml -f compose/viz/viz-combined.yaml down
```

### Build and Push New Image

```bash
# 1. Make changes to Dockerfile
vim base/Dockerfile

# 2. Build locally to test (single arch)
docker build \
  --build-arg ROS_DISTRO=jazzy \
  --target dev \
  -t grunt:jazzy-dev-test \
  -f base/Dockerfile .

# 3. Test the image
docker run -it --rm grunt:jazzy-dev-test bash

# 4. Build multi-arch and push
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=jazzy \
  --target dev \
  -t ghcr.io/pondersome/grunt:jazzy-dev \
  --push \
  -f base/Dockerfile .

# 5. Verify push
docker buildx imagetools inspect ghcr.io/pondersome/grunt:jazzy-dev
```

### Quick Testing Without Compose

```bash
# Test ROS 2 environment
docker run -it --rm --network=host \
  ghcr.io/pondersome/grunt:jazzy-dev \
  bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"

# Test package availability
docker run -it --rm \
  ghcr.io/pondersome/grunt:jazzy-dev \
  bash -c "source /opt/ros/jazzy/setup.bash && ros2 pkg list | grep moveit"

# Check installed dependencies
docker run -it --rm \
  ghcr.io/pondersome/grunt:jazzy-dev \
  dpkg -l | grep ros-jazzy
```

---

## Environment Variables Reference

Common environment variables used in compose files and docker run:

| Variable | Purpose | Example |
|----------|---------|---------|
| `ROS_DISTRO` | Select ROS distribution | `jazzy` (default), `humble` (legacy) |
| `ROS_DOMAIN_ID` | ROS 2 domain ID | `0` (default), `1`, `42` |
| `RMW_IMPLEMENTATION` | ROS middleware | `rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp` |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | DDS config file path | `/dds_config/fastrtps_unicast.xml` |
| `DISPLAY` | X11 display | `:0`, `:1` |
| `WAYLAND_DISPLAY` | Wayland display | `wayland-0`, `wayland-1` |
| `XDG_RUNTIME_DIR` | Runtime directory | `/run/user/1000` |
| `PULSE_SERVER` | PulseAudio server | `/mnt/wslg/PulseServer` |
| `VIZANTI_PORT` | Vizanti web UI port | `5000` (default) |
| `VIZANTI_ROSBRIDGE_PORT` | Vizanti websocket port | `5001` (default) |
| `AUDIO_NS` | Hal-audio namespace (match byc) | `/grunt1/agent` (default) |
| `DATE_TAG` | Date tag for bake release builds | `20260922` |

---

## Tips & Tricks

### Speed Up Builds

```bash
# Use BuildKit cache
export DOCKER_BUILDKIT=1

# Use inline cache for multi-arch builds
docker buildx build \
  --cache-from=type=registry,ref=ghcr.io/pondersome/grunt:jazzy-dev \
  --cache-to=type=inline \
  ...
```

### Clean Up Everything

```bash
# Nuclear option: remove everything
docker system prune -a --volumes

# More selective cleanup
docker container prune  # Remove stopped containers
docker image prune -a   # Remove unused images
docker volume prune     # Remove unused volumes
docker network prune    # Remove unused networks
```

### Quick Compose Reference

```bash
# All common compose operations
docker compose -f <file> up        # Start services
docker compose -f <file> up -d     # Start in background
docker compose -f <file> down      # Stop services
docker compose -f <file> logs      # View logs
docker compose -f <file> logs -f   # Follow logs
docker compose -f <file> ps        # List services
docker compose -f <file> restart   # Restart services
docker compose -f <file> exec <service> <cmd>  # Run command in service
```

---

## See Also

- [docs/getting-started-wsl2.md](getting-started-wsl2.md) - Fresh WSL2 setup guide
- [docs/ghcr-setup.md](ghcr-setup.md) - Detailed GHCR authentication and buildx setup
- [docs/native-docker-wsl2-setup.md](native-docker-wsl2-setup.md) - Docker CE installation
- [docs/dev-workflow.md](dev-workflow.md) - Development workflow patterns
- [docs/wsl2-visualization.md](wsl2-visualization.md) - WSLg and DDS troubleshooting
- [docs/foxglove-setup.md](foxglove-setup.md) - Foxglove Bridge setup and deployment options
- [docs/vizanti-setup.md](vizanti-setup.md) - Vizanti off-robot server setup, build loop, ZeroTier access
