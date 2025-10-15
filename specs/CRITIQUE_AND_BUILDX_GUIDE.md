# PRD Critique & Multi-Arch Build Guide

## Executive Summary Enhancement

### Current PRD Executive Summary (Section 1)
The current executive summary is **too technical and jumps straight into implementation details** without establishing the **why** behind the architecture.

### Proposed Enhanced Executive Summary

```markdown
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
```

---

## Multi-Architecture Builds with Docker Buildx

### What is Buildx?
Docker Buildx is an **extended build engine** that enables:
- **Multi-architecture images** (build for x86_64 + ARM64 from one Dockerfile)
- **Cross-compilation** using QEMU emulation
- **Build caching** to speed up repeated builds
- **Multi-stage builds** with better dependency management

### Why You Need Multi-Arch

Your fleet has **two CPU architectures**:
- **x86_64**: Barney (NUC), Windows workstations
- **ARM64**: Betty (Jetson Orin Nano), BamBam (RPi 5)

Without multi-arch builds, you'd need:
- Separate Dockerfiles for each architecture
- Manual builds on each target device
- Complex CI/CD to build on multiple runners

With buildx, **one Dockerfile → images for both architectures**.

### How Buildx Works (Simplified)

1. **QEMU emulation**: Your x86_64 laptop can emulate ARM64 instructions
2. **BuildKit backend**: Smarter layer caching and parallel builds
3. **Manifest lists**: Docker Hub stores one tag pointing to multiple arch-specific images

When you `docker pull pondersome/grunt_viz:humble` on:
- x86_64 machine → gets the `linux/amd64` variant
- ARM64 machine → gets the `linux/arm64` variant

### Practical Example: Building Your Base Image

#### Step 1: Enable Buildx (one-time setup)
```bash
# Create a new builder instance
docker buildx create --name gruntbuilder --use

# Verify it's active
docker buildx ls
# Should show gruntbuilder with linux/amd64, linux/arm64 platforms
```

#### Step 2: Write a Multi-Arch Dockerfile
Your current `base/Dockerfile` is **already multi-arch compatible**! You just need to build it correctly.

```dockerfile
# This works on both x86_64 and arm64 because:
FROM osrf/ros:${ROS_DISTRO}-desktop AS base

# The osrf/ros image is already multi-arch
# Docker automatically pulls the right variant for the build platform
```

#### Step 3: Build for Multiple Architectures

```bash
# Build for both x86_64 and ARM64
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --tag pondersome/grunt:jazzy \
  --file base/Dockerfile \
  --push \
  .

# Explanation:
# --platform: Comma-separated list of target architectures
# --push: Automatically push to registry (required for multi-arch)
# --file: Path to Dockerfile (if not in current directory)
# . : Build context (current directory)
```

**Important**: Multi-arch builds **must push to a registry**. You can't load them directly to local Docker because Docker doesn't support multi-arch locally.

#### Step 4: Using the Image

On any machine:
```bash
docker pull pondersome/grunt:jazzy
docker run -it pondersome/grunt:jazzy bash

# Docker automatically selected the right architecture!
```

### Local Testing Without Registry

If you want to test locally before pushing:

```bash
# Build for ONLY your current architecture and load locally
docker buildx build \
  --platform linux/amd64 \
  --tag grunt:jazzy-local \
  --file base/Dockerfile \
  --load \
  .

# Now you can test it locally
docker run -it grunt:jazzy-local bash
```

### Advanced: Platform-Specific Logic in Dockerfile

```dockerfile
# Get the target architecture as a variable
ARG TARGETARCH

# Install architecture-specific packages
RUN if [ "$TARGETARCH" = "arm64" ]; then \
      apt-get install -y jetson-specific-package; \
    elif [ "$TARGETARCH" = "amd64" ]; then \
      apt-get install -y intel-specific-package; \
    fi
```

### Jetson Caveat: NVIDIA Runtime Images

For **Betty (Jetson)**, you'll need Jetson-specific base images:

```dockerfile
# Separate Dockerfile for Jetson builds
FROM nvcr.io/nvidia/l4t-base:r36.2.0 AS jetson-base
# JetPack 6.2 = L4T r36.2

# Then install ROS 2 on top
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-gz-sim-vendor
```

**These can't be multi-arch** because they require native ARM64 + CUDA. Build them directly on Betty:

```bash
# On Betty (Jetson):
docker build -f base/Dockerfile.jetson -t grunt:humble-jetson .
```

---

## Aligning PRD with Current Dockerfile

### Current State Analysis

Your `base/Dockerfile` implements:
1. ✅ Multi-stage build (just `base` stage so far)
2. ✅ Python venv for build tools (`/opt/venv`)
3. ✅ Complete workspace build (`/ros2_ws`)
4. ✅ Graceful Gazebo fallback (vendor → meta → standalone)
5. ✅ Groot2 AppImage download
6. ✅ Entrypoint script with workspace sourcing

### PRD Misalignments (with Dockerfile as Ground Truth)

| PRD Section | Current State | Should Be |
|-------------|---------------|-----------|
| **5) Repository layout** | Shows separate `Dockerfile.humble`, `Dockerfile.jazzy` | Single `Dockerfile` with `ARG ROS_DISTRO` |
| **6) Image strategy** | Doesn't mention Python venv isolation | Document `/opt/venv` pattern |
| **7.1) WSLg mounts** | Shows X11 fallback in example | Primary: WSLg/Wayland, X11 optional |
| **Section 6** | No user account strategy | Add: root (build), dev (development), robot (production) |
| **Section 7** | Missing workspace layering | Add: `/ros2_ws`, `/overlay_ws`, `~/dev_ws` convention |

### Proposed PRD Updates (Priority Order)

#### 1. Update Section 5 (Repository Layout)
```markdown
├─ base/
│  ├─ Dockerfile              # Multi-stage: base → overlay → dev
│  ├─ entrypoint.sh           # Workspace sourcing script
│  └─ dependencies.repos      # External ROS packages to build
```

#### 2. Add Section 6.5 (Workspace Layering Convention)
```markdown
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

#### 3. Add Section 6.6 (User Account Strategy)
```markdown
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
```

#### 4. Add Section 6.7 (Python Dependency Isolation)
```markdown
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
```

#### 5. Update Section 7.1 (WSLg Primary, X11 Fallback)
```markdown
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
```

#### 6. Update Section 18A (Compose Example)
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

---

## Directory Restructuring Plan

### Current Structure
```
grunt_docker/
├── base/
│   ├── Dockerfile
│   └── entrypoint.sh
├── dependencies.repos
├── bashrc_custom
└── README.md
```

### Proposed Structure (Aligned with PRD + Your Dockerfile)
```
grunt_docker/
├── base/
│   ├── Dockerfile           # Multi-stage: base → overlay → dev
│   ├── entrypoint.sh
│   └── dependencies.repos   # Moved from root
├── compose/
│   ├── viz/
│   │   ├── rviz.yaml
│   │   ├── rqt.yaml
│   │   └── foxglove-bridge.yaml
│   ├── betty/
│   │   └── camera-oakd.yaml
│   └── dev/
│       └── dev-env.yaml     # Interactive development container
├── env/
│   ├── env.example          # Template for ROS_DOMAIN_ID, etc.
│   ├── env.wsl              # WSL2-specific settings
│   └── env.jetson           # Jetson-specific settings
├── docs/
│   ├── getting-started.md
│   ├── wsl2-visualization.md
│   ├── buildx-guide.md      # This document!
│   └── robots/
│       ├── barney.md
│       └── betty.md
├── specs/
│   ├── grunt_docker_prd_v_0.md
│   ├── CRITIQUE_AND_BUILDX_GUIDE.md  # This file
│   ├── research/
│   └── sessions/
├── bashrc_custom            # Kept at root for easy COPY in Dockerfile
├── README.md
└── .gitignore
```

### Migration Steps

1. **Keep current structure working** (don't break existing builds)
2. **Create new directories** (`compose/`, `env/`, `docs/`)
3. **Move `dependencies.repos`** into `base/` (update Dockerfile COPY path)
4. **Add compose files** one at a time (start with `viz/rviz.yaml`)
5. **Update README.md** with new structure and quick-start commands

---

## Immediate Next Steps (Recommended Order)

### Phase 1: Documentation (No Code Changes)
1. ✅ Create this critique document (done)
2. ⬜ Update PRD Section 1 (Executive Summary) with robot vs. workstation narrative
3. ⬜ Add Sections 6.5, 6.6, 6.7 to PRD (workspace/user/Python strategies)
4. ⬜ Update Section 7.1 (WSLg primary, X11 fallback)
5. ⬜ Create `docs/buildx-guide.md` (simplified version of this doc for end users)

### Phase 2: Directory Restructuring
6. ⬜ Create `compose/`, `env/`, `docs/` directories
7. ⬜ Move `dependencies.repos` → `base/dependencies.repos` (update Dockerfile)
8. ⬜ Create `env/env.example` with ROS_DOMAIN_ID, RMW_IMPLEMENTATION templates
9. ⬜ Update README.md with new structure

### Phase 3: First Compose Stack (WSL2 Viz)
10. ⬜ Create `compose/viz/rviz.yaml` based on Section 18A updates
11. ⬜ Test on WSL2: `docker compose -f compose/viz/rviz.yaml up`
12. ⬜ Document in `docs/wsl2-visualization.md`

### Phase 4: Multi-Arch Build Setup
13. ⬜ Set up GHCR: Create PAT, login (see Quick GHCR Setup above or `docs/ghcr-setup.md`)
14. ⬜ Set up buildx builder: `docker buildx create --name gruntbuilder --use`
15. ⬜ Test local build: `docker buildx build --platform linux/amd64 --load -t ghcr.io/pondersome/grunt:jazzy-test -f base/Dockerfile .`
16. ⬜ First multi-arch push: `docker buildx build --platform linux/amd64,linux/arm64 --push -t ghcr.io/pondersome/grunt:jazzy -f base/Dockerfile .`
17. ⬜ Make package public on GitHub
18. ⬜ Test pull from another machine: `docker pull ghcr.io/pondersome/grunt:jazzy`

---

## Registry Decision: GitHub Container Registry (GHCR)

**Decision: Use `ghcr.io/pondersome/*` for all images**

Reasons:
- ✅ Already have GitHub account (no new signup)
- ✅ Integrated with repository (packages show on profile)
- ✅ Better for CI/CD (GitHub Actions integration)
- ✅ Higher pull rate limits
- ✅ Free for public images

### Quick GHCR Setup (5 minutes)

1. **Create Personal Access Token**
   - Go to: https://github.com/settings/tokens
   - Click "Generate new token (classic)"
   - Name: `Docker BuildX for grunt_docker`
   - Scopes: `write:packages`, `read:packages`, `delete:packages` (optional)
   - Generate and copy token (starts with `ghp_`)

2. **Save token securely**
   ```bash
   # On WSL2 or Linux
   export GITHUB_PAT="ghp_xxxxxxxxxxxxxxxxxxxx"
   echo $GITHUB_PAT > ~/.github-docker-token
   chmod 600 ~/.github-docker-token

   # Add to .bashrc for persistence
   echo 'export GITHUB_PAT=$(cat ~/.github-docker-token 2>/dev/null)' >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Login to GHCR**
   ```bash
   echo $GITHUB_PAT | docker login ghcr.io -u pondersome --password-stdin
   # Should see: Login Succeeded
   ```

4. **After first push, make package public**
   - Go to: https://github.com/pondersome?tab=packages
   - Click your package (e.g., `grunt`)
   - Settings → Change visibility → Public

**See `docs/ghcr-setup.md` for detailed walkthrough with troubleshooting.**

### Image Naming Convention

**Decision: Start simple, add complexity as needed**

```
ghcr.io/pondersome/<component>:<distro>[-<variant>]

Examples:
  ghcr.io/pondersome/grunt:jazzy
  ghcr.io/pondersome/grunt:humble
  ghcr.io/pondersome/grunt_viz:jazzy
  ghcr.io/pondersome/grunt_betty:humble-jetson
```

Later additions (when needed):
- Dated tags: `grunt:jazzy-20250108`
- Semver: `grunt:v0.1.0`
- Latest pointer: `grunt:latest` → `jazzy`

---

## Implementation Decisions

### 1. When to add overlay/dev stages
   - Now (complete the multi-stage Dockerfile)
   - Later (once viz compose stack is working)
   - Recommendation: Later, one stage at a time

4. **Jetson image strategy**:
   - Separate `base/Dockerfile.jetson` (non-multi-arch)
   - Same Dockerfile with `ARG TARGETARCH` conditionals
   - Recommendation: Separate Dockerfile (clearer, less conditional logic)

---

## Appendix: Buildx Command Reference

### Setup
```bash
# Create and activate builder
docker buildx create --name gruntbuilder --use

# Inspect builder capabilities
docker buildx inspect gruntbuilder --bootstrap
```

### Build Commands (GHCR)
```bash
# Local test (single arch, load to docker)
docker buildx build \
  --platform linux/amd64 \
  --load \
  -t ghcr.io/pondersome/grunt:jazzy-test \
  -f base/Dockerfile .

# Multi-arch push to GHCR
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --push \
  -t ghcr.io/pondersome/grunt:jazzy \
  -f base/Dockerfile .

# Multi-arch with build args
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --build-arg ROS_DISTRO=humble \
  --build-arg GZ_VERSION=gz-garden \
  --push \
  -t ghcr.io/pondersome/grunt:humble \
  -f base/Dockerfile .

# Build specific stage from multi-stage Dockerfile
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --target dev \
  --push \
  -t ghcr.io/pondersome/grunt_dev:jazzy \
  -f base/Dockerfile .

# Push to multiple registries (GHCR + Docker Hub)
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --push \
  -t ghcr.io/pondersome/grunt:jazzy \
  -t pondersome/grunt:jazzy \
  -f base/Dockerfile .
```

### Cleanup
```bash
# Remove builder
docker buildx rm gruntbuilder

# Prune build cache
docker buildx prune -a
```

---

## Conclusion

Your current Dockerfile is **more correct** than the PRD. The path forward:

1. **Update PRD** to match Dockerfile patterns (workspace layering, Python venv, WSLg mounts)
2. **Test buildx** locally before multi-arch pushes
3. **Restructure directories** gradually (don't break current builds)
4. **Start with viz compose stack** (highest priority, easiest to test on WSL2)
5. **Document as you go** (future-you will thank you)

The multi-arch build capability is **critical** for your x86_64 + ARM64 fleet, and buildx makes it straightforward once you understand the push-to-registry requirement.
