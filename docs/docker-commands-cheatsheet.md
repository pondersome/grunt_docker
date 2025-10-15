# Docker Commands Cheat Sheet

Quick reference for common Docker, buildx, and compose commands used in this project.

---

## Table of Contents

- [Image Building](#image-building)
- [Image Management](#image-management)
- [Docker Compose](#docker-compose)
- [Running Containers Directly](#running-containers-directly)
- [Multi-Arch Setup](#multi-arch-setup)
- [Registry Operations](#registry-operations)
- [Debugging & Inspection](#debugging--inspection)

---

## Image Building

### Build Single Architecture (Local Testing)

```bash
# Build base stage for Humble (x86_64 only)
docker build \
  --build-arg ROS_DISTRO=humble \
  --target base \
  -t grunt:humble \
  -f base/Dockerfile .

# Build dev stage for Humble (x86_64 only)
docker build \
  --build-arg ROS_DISTRO=humble \
  --target dev \
  -t grunt:humble-dev \
  -f base/Dockerfile .

# Build Jazzy dev stage
docker build \
  --build-arg ROS_DISTRO=jazzy \
  --target dev \
  -t grunt:jazzy-dev \
  -f base/Dockerfile .
```

### Build Multi-Architecture (buildx)

```bash
# Build and push multi-arch base stage (Humble)
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=humble \
  --target base \
  -t ghcr.io/pondersome/grunt:humble \
  --push \
  -f base/Dockerfile .

# Build and push multi-arch dev stage (Humble)
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=humble \
  --target dev \
  -t ghcr.io/pondersome/grunt:humble-dev \
  --push \
  -f base/Dockerfile .

# Build multi-arch for Jazzy
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=jazzy \
  --target dev \
  -t ghcr.io/pondersome/grunt:jazzy-dev \
  --push \
  -f base/Dockerfile .

# Build locally without pushing (load to local Docker)
docker buildx build \
  --platform linux/amd64 \
  --build-arg ROS_DISTRO=humble \
  --target dev \
  -t grunt:humble-dev \
  --load \
  -f base/Dockerfile .
```

**Note**: `--push` requires registry authentication. Use `--load` for local testing (single architecture only).

---

## Image Management

### Pull Images

```bash
# Pull base image (Humble)
docker pull ghcr.io/pondersome/grunt:humble

# Pull dev image (Humble)
docker pull ghcr.io/pondersome/grunt:humble-dev

# Pull Jazzy dev image
docker pull ghcr.io/pondersome/grunt:jazzy-dev
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
docker rmi grunt:humble-dev

# Remove all unused images
docker image prune -a

# Force remove image (even if containers exist)
docker rmi -f grunt:humble-dev
```

### Inspect Images

```bash
# Show image details (layers, env vars, etc.)
docker inspect ghcr.io/pondersome/grunt:humble-dev

# Show image history (layer sizes)
docker history ghcr.io/pondersome/grunt:humble-dev

# Show image architecture
docker inspect ghcr.io/pondersome/grunt:humble-dev | grep Architecture
```

---

## Docker Compose

### Basic Operations

```bash
# Start services (default: Humble)
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
# Use Jazzy instead of Humble
ROS_DISTRO=jazzy docker compose -f compose/viz/rviz.yaml up

# Set custom ROS_DOMAIN_ID
ROS_DOMAIN_ID=42 docker compose -f compose/viz/rviz.yaml up

# Combine environment variables
ROS_DISTRO=jazzy ROS_DOMAIN_ID=42 docker compose -f compose/viz/rviz.yaml up
```

### Multiple Services

```bash
# Start multiple services simultaneously
docker compose -f compose/viz/bash.yaml -f compose/viz/rviz.yaml up

# Start all services in compose/viz/
docker compose -f compose/viz/bash.yaml -f compose/viz/rviz.yaml -f compose/viz/rqt.yaml up
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

## Running Containers Directly

### Interactive Shells

```bash
# Interactive bash shell (Humble dev)
docker run -it --rm \
  --network=host \
  ghcr.io/pondersome/grunt:humble-dev \
  bash

# Interactive bash with workspace mounted
docker run -it --rm \
  --network=host \
  -v ~/ros2/humble/dev_ws:/home/dev/dev_ws:rw \
  --user dev \
  ghcr.io/pondersome/grunt:humble-dev \
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
  ghcr.io/pondersome/grunt:humble-dev \
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
  ghcr.io/pondersome/grunt:humble-dev \
  rviz2

# Launch RQT
docker run -it --rm \
  --network=host \
  -v /mnt/wslg:/mnt/wslg:rw \
  -e WAYLAND_DISPLAY=${WAYLAND_DISPLAY} \
  -e XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR} \
  --user dev \
  ghcr.io/pondersome/grunt:humble-dev \
  rqt

# Run ROS 2 CLI commands
docker run -it --rm \
  --network=host \
  --user dev \
  ghcr.io/pondersome/grunt:humble-dev \
  ros2 topic list

docker run -it --rm \
  --network=host \
  --user dev \
  ghcr.io/pondersome/grunt:humble-dev \
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
  ghcr.io/pondersome/grunt:humble-dev \
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

# Create buildx builder
docker buildx create --use --name gruntx --driver docker-container

# Or use existing builder
docker buildx use gruntx

# Inspect builder
docker buildx inspect --bootstrap

# List available builders
docker buildx ls
```

### Test Multi-Arch Build

```bash
# Test ARM64 emulation
docker run --rm --platform linux/arm64 ubuntu:22.04 uname -m
# Should output: aarch64

# Test multi-arch build without pushing
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=humble \
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
docker tag grunt:humble-dev ghcr.io/pondersome/grunt:humble-dev

# Push to registry
docker push ghcr.io/pondersome/grunt:humble-dev

# Pull from registry
docker pull ghcr.io/pondersome/grunt:humble-dev

# Pull specific platform
docker pull --platform linux/arm64 ghcr.io/pondersome/grunt:humble-dev
```

### Inspect Registry Images

```bash
# Show manifest (architectures available)
docker buildx imagetools inspect ghcr.io/pondersome/grunt:humble

# Show manifest for specific tag
docker buildx imagetools inspect ghcr.io/pondersome/grunt:humble-dev
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
docker logs grunt_rviz_humble

# Follow container logs
docker logs -f grunt_rviz_humble

# Execute command in running container
docker exec -it grunt_bash_humble bash

# Inspect container details
docker inspect grunt_bash_humble

# Show container resource usage
docker stats grunt_bash_humble
```

### Container Cleanup

```bash
# Stop all running containers
docker stop $(docker ps -q)

# Remove all stopped containers
docker container prune

# Remove specific container
docker rm grunt_bash_humble

# Force remove running container
docker rm -f grunt_bash_humble
```

### Workspace Debugging

```bash
# Check workspace inside container
docker exec -it grunt_bash_humble ls -la /home/dev/dev_ws

# Check if workspace is sourced
docker exec -it grunt_bash_humble bash -c "source /home/dev/dev_ws/install/setup.bash && ros2 pkg list"

# Build workspace inside container
docker exec -it grunt_bash_humble bash -c "cd ~/dev_ws && colcon build --symlink-install"

# Check ROS environment
docker exec -it grunt_bash_humble printenv | grep ROS
```

### Network Debugging

```bash
# Check network interfaces inside container
docker exec -it grunt_bash_humble ip addr

# Check if ZeroTier interface is visible
docker exec -it grunt_bash_humble ifconfig | grep zt

# Test DDS discovery
docker exec -it grunt_bash_humble ros2 topic list

# Check DDS participants
docker exec -it grunt_bash_humble ros2 daemon status
docker exec -it grunt_bash_humble ros2 daemon stop
docker exec -it grunt_bash_humble ros2 daemon start
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
  ubuntu:22.04 \
  bash -c "apt-get update && apt-get install -y x11-apps && xeyes"

# Test Wayland connection (WSLg)
docker run --rm -it \
  -v /mnt/wslg:/mnt/wslg:rw \
  -e WAYLAND_DISPLAY=${WAYLAND_DISPLAY} \
  -e XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR} \
  ubuntu:22.04 \
  bash -c "apt-get update && apt-get install -y weston && weston-info"
```

---

## Common Workflows

### Full Development Cycle

```bash
# 1. Pull latest dev image
docker pull ghcr.io/pondersome/grunt:humble-dev

# 2. Start interactive container
docker compose -f compose/viz/bash.yaml up

# 3. (In container) Build workspace
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash

# 4. (In another terminal) Launch RViz
docker compose -f compose/viz/rviz.yaml up

# 5. (In another terminal) Launch RQT
docker compose -f compose/viz/rqt.yaml up

# 6. (In container) Test your nodes
ros2 launch my_package my_launch.py

# 7. When done, stop all containers
docker compose -f compose/viz/bash.yaml -f compose/viz/rviz.yaml -f compose/viz/rqt.yaml down
```

### Build and Push New Image

```bash
# 1. Make changes to Dockerfile
vim base/Dockerfile

# 2. Build locally to test (single arch)
docker build \
  --build-arg ROS_DISTRO=humble \
  --target dev \
  -t grunt:humble-dev-test \
  -f base/Dockerfile .

# 3. Test the image
docker run -it --rm grunt:humble-dev-test bash

# 4. Build multi-arch and push
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=humble \
  --target dev \
  -t ghcr.io/pondersome/grunt:humble-dev \
  --push \
  -f base/Dockerfile .

# 5. Verify push
docker buildx imagetools inspect ghcr.io/pondersome/grunt:humble-dev
```

### Quick Testing Without Compose

```bash
# Test ROS 2 environment
docker run -it --rm --network=host \
  ghcr.io/pondersome/grunt:humble-dev \
  bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# Test package availability
docker run -it --rm \
  ghcr.io/pondersome/grunt:humble-dev \
  bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep moveit"

# Check installed dependencies
docker run -it --rm \
  ghcr.io/pondersome/grunt:humble-dev \
  dpkg -l | grep ros-humble
```

---

## Environment Variables Reference

Common environment variables used in compose files and docker run:

| Variable | Purpose | Example |
|----------|---------|---------|
| `ROS_DISTRO` | Select ROS distribution | `humble`, `jazzy` |
| `ROS_DOMAIN_ID` | ROS 2 domain ID | `0` (default), `1`, `42` |
| `RMW_IMPLEMENTATION` | ROS middleware | `rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp` |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | DDS config file path | `/dds_config/fastrtps_unicast.xml` |
| `DISPLAY` | X11 display | `:0`, `:1` |
| `WAYLAND_DISPLAY` | Wayland display | `wayland-0`, `wayland-1` |
| `XDG_RUNTIME_DIR` | Runtime directory | `/run/user/1000` |
| `PULSE_SERVER` | PulseAudio server | `/mnt/wslg/PulseServer` |

---

## Tips & Tricks

### Speed Up Builds

```bash
# Use BuildKit cache
export DOCKER_BUILDKIT=1

# Use inline cache for multi-arch builds
docker buildx build \
  --cache-from=type=registry,ref=ghcr.io/pondersome/grunt:humble-dev \
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

- [docs/ghcr-setup.md](ghcr-setup.md) - Detailed GHCR authentication and buildx setup
- [docs/native-docker-wsl2-setup.md](native-docker-wsl2-setup.md) - Docker CE installation
- [docs/dev-workflow.md](dev-workflow.md) - Development workflow patterns
- [docs/wsl2-visualization.md](wsl2-visualization.md) - WSLg and DDS troubleshooting
