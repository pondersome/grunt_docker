# Development Workflow Guide

This guide explains the **dev layer pattern** and **workspace management** for containerized ROS 2 development on WSL2.

---

## Overview

The grunt_docker dev layer provides a **container-primary development environment** where:

- **Dependencies are baked into the image** (persistent across container restarts)
- **Source code is bind-mounted from the host** (editable on host, built in container)
- **Workspaces persist** across container lifecycles (no rebuild required)

This pattern balances the convenience of containers with the flexibility of native development.

---

## Dev Layer Architecture

### Multi-Stage Dockerfile

The `base/Dockerfile` uses a **two-stage build**:

```
┌─────────────────────────────────────────┐
│ Base Stage (grunt:humble)          │
│ - ROS 2 Humble/Jazzy Desktop            │
│ - Gazebo Harmonic                        │
│ - Core dependencies (Cyclone DDS, etc.) │
│ - /ros2_ws from dependencies.repos      │
│ - dev user (UID 1000)                    │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│ Dev Stage (grunt:humble-dev)       │
│ + MoveIt2 (manipulation)                 │
│ + Nav2 (navigation)                      │
│ + RealSense SDK (perception)            │
│ + ros2_control (controllers)            │
│ + Audio dependencies (voice control)    │
└─────────────────────────────────────────┘
```

### Why Two Stages?

**Base stage** is minimal and suitable for:
- Production deployments (robots, headless services)
- Lightweight visualization tools (RViz, RQT without development packages)
- CI/CD pipelines

**Dev stage** adds development-specific packages:
- Heavy manipulation/navigation stacks
- Hardware SDKs (RealSense, audio)
- Optional tools and examples

This keeps production images small while providing full development capabilities when needed.

---

## Dependency Strategy

### Ephemeral vs Persistent

In containerized development, there are two types of dependencies:

| Type | Location | Installed via | Persistence |
|------|----------|---------------|-------------|
| **System dependencies** | Image layers | apt-get in Dockerfile | Permanent (in image) |
| **ROS packages (binary)** | Image layers | apt-get in Dockerfile | Permanent (in image) |
| **ROS packages (source)** | Bind-mounted workspace | colcon build in container | Persistent (on host) |
| **Runtime rosdep installs** | Container filesystem | rosdep install at runtime | **EPHEMERAL** (lost on restart) |

**Key Insight**: Running `rosdep install` in a container is ephemeral unless done in the Dockerfile. Dependencies must be **baked into the image** to persist.

### What Goes Where?

**Baked into Dev Image (Dockerfile)**:
- Stable ROS packages from apt repos (MoveIt2, Nav2, ros2_control)
- Hardware SDKs (RealSense, PortAudio, ALSA)
- System libraries (GStreamer, Qt, OpenCV)
- Python packages needed system-wide (numpy, lark, vcstool)

**Built from Source (Workspaces)**:
- Your development packages (grunt, roarm_ws_em0, by_your_command)
- Forked/modified upstream packages (audio_common, realsense-ros)
- Experimental packages under active development
- Robot-specific configurations

---

## Workspace Layering

### Three-Tier Structure

```
/opt/ros/humble/                    ← Layer 1: ROS Base (immutable)
    └── setup.bash

/ros2_ws/                           ← Layer 2: External Dependencies (immutable)
    ├── src/
    │   ├── p2os2/                  (from dependencies.repos)
    │   └── grunt/                  (core packages)
    └── install/setup.bash

/home/dev/dev_ws/                   ← Layer 3: Development (bind-mounted)
    ├── src/
    │   ├── roarm_ws_em0/           (your development repos)
    │   ├── audio_common/
    │   ├── by_your_command/
    │   └── realsense-ros/
    └── install/setup.bash

/home/dev/sim_ws/                   ← Layer 4: Simulation (bind-mounted, optional)
    ├── src/
    │   ├── simulation_worlds/
    │   └── robot_models/
    └── install/setup.bash
```

### Sourcing Order

The `entrypoint.sh` automatically sources in order:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash       # ROS base
source /ros2_ws/install/setup.bash              # External deps
source ~/dev_ws/install/setup.bash              # Dev workspace (if exists)
source ~/sim_ws/install/setup.bash              # Sim workspace (if exists)
```

Later workspaces **overlay** earlier ones, allowing you to override packages.

---

## Multi-Distro Development

### Distro-Specific Workspaces

To support both Humble and Jazzy, use **distro-specific workspace directories** on the host:

```
~/ros2/
├── humble/
│   ├── dev_ws/
│   │   ├── src/          ← Clone repos here
│   │   ├── build/
│   │   └── install/
│   └── sim_ws/
│       └── src/
└── jazzy/
    ├── dev_ws/
    │   ├── src/          ← Same repos, different distro
    │   ├── build/
    │   └── install/
    └── sim_ws/
        └── src/
```

### Compose File Integration

Compose files use `${ROS_DISTRO:-humble}` to mount the correct workspace:

```yaml
volumes:
  # Development workspace (bind-mounted from WSL2)
  - ~/ros2/${ROS_DISTRO:-humble}/dev_ws:/home/dev/dev_ws:rw

  # Simulation workspace (bind-mounted from WSL2)
  - ~/ros2/${ROS_DISTRO:-humble}/sim_ws:/home/dev/sim_ws:rw
```

### Switching Distros

```bash
# Default: Humble
docker compose -f compose/viz/bash.yaml up

# Override to Jazzy
ROS_DISTRO=jazzy docker compose -f compose/viz/bash.yaml up
```

The same source code in `~/ros2/jazzy/dev_ws/src/` gets built against Jazzy dependencies.

---

## Development Workflow

### Initial Setup

#### 1. Create Workspace Structure

```bash
# On WSL2 host
mkdir -p ~/ros2/humble/{dev_ws,sim_ws}/src
mkdir -p ~/ros2/jazzy/{dev_ws,sim_ws}/src
```

#### 2. Clone Development Repos

```bash
cd ~/ros2/humble/dev_ws/src

# Core packages
git clone https://github.com/pondersome/grunt.git
git clone https://github.com/pondersome/roarm_ws_em0.git

# Perception
git clone -b ros2 https://github.com/IntelRealSense/realsense-ros.git

# Voice control
git clone https://github.com/pondersome/by_your_command.git
git clone https://github.com/pondersome/audio_common.git

# Additional packages as needed
```

#### 3. Pull Dev Image

```bash
docker pull ghcr.io/pondersome/grunt:humble-dev
```

### Daily Development Loop

#### 1. Start Interactive Container

```bash
docker compose -f compose/viz/bash.yaml up
```

This gives you a bash shell with:
- ROS environment sourced
- Workspaces mounted at `~/dev_ws` and `~/sim_ws`
- GUI support via WSLg

#### 2. Build Workspace (First Time)

```bash
# Inside container
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash
```

**Tip**: Use `--symlink-install` to avoid copying Python files. Changes to Python files are immediately visible without rebuild.

#### 3. Edit Code on Host

**On Windows**:
- Use VSCode, Sublime, or any editor
- Files are in `\\wsl$\Ubuntu\home\<username>\ros2\humble\dev_ws\src\`

**On WSL2**:
- Use vim, nano, or VSCode with WSL Remote extension
- Files are in `~/ros2/humble/dev_ws/src/`

**File permissions**: Because container runs as `dev` (UID 1000) and your WSL2 user is typically UID 1000, file ownership is correct on both sides.

#### 4. Rebuild After Changes

```bash
# Inside container
cd ~/dev_ws

# Rebuild only changed packages
colcon build --symlink-install

# Or rebuild specific package
colcon build --symlink-install --packages-select roarm_description

# Re-source workspace
source install/setup.bash
```

#### 5. Test/Run Nodes

```bash
# Inside container
ros2 launch roarm_description view_robot.launch.py

# Or run individual nodes
ros2 run by_your_command voice_control_node
```

#### 6. Launch GUI Tools

While bash container is running, launch RViz/RQT in separate terminals:

```bash
# Terminal 2
docker compose -f compose/viz/rviz.yaml up

# Terminal 3
docker compose -f compose/viz/rqt.yaml up
```

All containers share `network_mode: host`, so they see the same ROS graph.

---

## Adding New Dependencies

### Binary Packages (apt install)

If you need a new ROS package available in apt repos:

#### Option A: Add to Dockerfile (Recommended)

Edit `base/Dockerfile` dev stage:

```dockerfile
# Install new package
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-your-package-name \
    && rm -rf /var/lib/apt/lists/*
```

Rebuild and push image:

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=humble \
  --target dev \
  -t ghcr.io/pondersome/grunt:humble-dev \
  --push \
  -f base/Dockerfile .
```

#### Option B: Install at runtime (Ephemeral)

For quick testing only:

```bash
# Inside container
sudo apt-get update
sudo apt-get install ros-humble-your-package-name
source /opt/ros/humble/setup.bash
```

**Warning**: This will be lost when container stops. Use Option A for permanent changes.

### Source Packages (colcon build)

If you need a package not in apt repos:

```bash
# On host
cd ~/ros2/humble/dev_ws/src
git clone https://github.com/org/your_package.git

# Inside container
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

**Note**: If `rosdep install` finds dependencies, add them to Dockerfile for persistence.

---

## Troubleshooting

### Package Not Found After Build

**Symptom**: `Package 'foo' not found` after `colcon build` succeeds

**Solution**: Source the workspace

```bash
source ~/dev_ws/install/setup.bash
```

Or restart container (entrypoint sources automatically if workspace exists).

### File Permission Errors

**Symptom**: `Permission denied` when building or editing files

**Cause**: UID mismatch between host user and container user

**Solution**: Check UIDs

```bash
# On host
id -u    # Should be 1000

# Inside container
id -u    # Should be 1000
```

If different, rebuild image with correct UID or change host user UID.

### Dependency Changes Not Persisting

**Symptom**: After running `rosdep install` in container, dependencies are missing on next restart

**Cause**: rosdep installs to container filesystem, which is ephemeral

**Solution**: Add dependencies to Dockerfile dev stage (see "Adding New Dependencies" above).

### Colcon Build Very Slow

**Symptom**: `colcon build` takes many minutes, even for small changes

**Causes**:
1. Not using `--symlink-install` (copies files every build)
2. Building all packages instead of changed packages
3. WSL2 filesystem I/O overhead

**Solutions**:

```bash
# Always use symlink-install
colcon build --symlink-install

# Build only changed packages
colcon build --symlink-install --packages-select my_package

# Use ccache for C++ compilation
sudo apt-get install ccache
export CC="ccache gcc"
export CXX="ccache g++"
colcon build --symlink-install
```

### ROS Packages Not Visible in Workspace

**Symptom**: Packages in `~/dev_ws/src/` don't show up after `colcon build`

**Possible causes**:

1. **No package.xml**: Ensure each package has a valid `package.xml`
   ```bash
   cd ~/dev_ws/src/my_package
   ls package.xml    # Must exist
   ```

2. **Nested workspace**: Don't clone into a subdirectory with its own workspace
   ```bash
   # Bad
   ~/dev_ws/src/some_repo/workspace/package/  # Has its own src/

   # Good
   ~/dev_ws/src/package/  # Direct package or metapackage
   ```

3. **Build errors**: Check build output for errors
   ```bash
   colcon build --symlink-install --event-handlers console_cohesion+
   ```

---

## Advanced Patterns

### Developing Against Multiple Robots

If you have multiple robots (Barney, Betty, BamBam), you can use **separate workspaces** or **separate branches**:

#### Option A: Separate Workspaces

```
~/ros2/humble/
├── dev_ws/          ← General development
├── barney_ws/       ← Barney-specific packages
└── betty_ws/        ← Betty-specific packages
```

Update compose files to mount different workspaces.

#### Option B: Git Branches

```bash
# In ~/ros2/humble/dev_ws/src/grunt/
git checkout barney    # Barney config
git checkout betty     # Betty config
git checkout main      # Shared code
```

Rebuild workspace after switching branches.

### VSCode DevContainer Integration

You can use VSCode DevContainers with the dev image:

Create `.devcontainer/devcontainer.json` in your workspace:

```json
{
  "name": "ROS 2 Humble Dev",
  "image": "ghcr.io/pondersome/grunt:humble-dev",
  "workspaceMount": "source=${localWorkspaceFolder},target=/home/dev/dev_ws,type=bind",
  "workspaceFolder": "/home/dev/dev_ws",
  "remoteUser": "dev",
  "mounts": [
    "source=/mnt/wslg,target=/mnt/wslg,type=bind",
    "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,readonly"
  ],
  "containerEnv": {
    "DISPLAY": "${localEnv:DISPLAY}",
    "WAYLAND_DISPLAY": "${localEnv:WAYLAND_DISPLAY}",
    "XDG_RUNTIME_DIR": "${localEnv:XDG_RUNTIME_DIR}"
  },
  "runArgs": ["--network=host"],
  "postCreateCommand": "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash",
  "extensions": [
    "ms-vscode.cpptools",
    "ms-python.python",
    "ms-iot.vscode-ros"
  ]
}
```

Then open workspace in VSCode and click "Reopen in Container".

---

## Best Practices

### 1. Keep Source Code on Host

**Always** clone repos on the host filesystem (`~/ros2/humble/dev_ws/src/`), not inside the container. This ensures:
- Code persists across container restarts
- You can edit with host tools
- Version control works normally
- Backups include your code

### 2. Use Symlink Install

Always build with `--symlink-install`:

```bash
colcon build --symlink-install
```

This avoids copying Python files, making incremental builds much faster.

### 3. Add Persistent Dependencies to Dockerfile

If you run `rosdep install` or `apt-get install` in a container, those changes are lost on restart. **Always** add persistent dependencies to the Dockerfile dev stage.

### 4. One Distro Per Workspace

Don't mix Humble and Jazzy in the same workspace. Use separate directories:
- `~/ros2/humble/dev_ws/`
- `~/ros2/jazzy/dev_ws/`

### 5. Commit Workspace Changes Carefully

The `build/`, `install/`, and `log/` directories are build artifacts. Add to `.gitignore`:

```gitignore
build/
install/
log/
.colcon/
```

Only commit the `src/` directory (or use separate repos for each package).

### 6. Test Multi-Arch Locally Before Pushing

When modifying the Dockerfile, test both architectures:

```bash
# Build locally for native arch
docker build --build-arg ROS_DISTRO=humble --target dev -t grunt:humble-dev-test .

# Test it
docker run -it --rm grunt:humble-dev-test bash

# Only push after verifying
docker buildx build --platform linux/amd64,linux/arm64 --push ...
```

---

## Summary

The dev layer pattern provides:

- **Persistent dependencies** baked into image (MoveIt2, Nav2, RealSense, etc.)
- **Editable source code** bind-mounted from host
- **Multi-distro support** via workspace isolation
- **Multi-arch support** via Docker buildx
- **GUI support** via WSLg (Wayland)
- **Full ROS toolchain** ready to use

This balances the convenience of containers (dependency isolation, reproducibility) with the flexibility of native development (fast iteration, familiar tools).

---

## Next Steps

- Read [wsl2-visualization.md](wsl2-visualization.md) for WSLg and DDS troubleshooting
- Read [ghcr-setup.md](ghcr-setup.md) for multi-arch build and registry push
- See [ROADMAP.md](ROADMAP.md) for planned features and improvements
