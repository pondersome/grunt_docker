# grunt_docker Implementation Roadmap

**Status**: v0.1 - Phase 1 Complete (WSL2 Visualization + Dev Layer)
**Last Updated**: 2025-10-14

This roadmap tracks implementation of the [PRD](../specs/grunt_docker_prd_v_0.md) vision. Tasks are organized by priority and dependencies.

---

## ✅ Completed (Foundation & Phase 1)

### Infrastructure
- [x] Multi-arch Dockerfile (Humble + Jazzy, x86_64 + ARM64)
- [x] Multi-stage Dockerfile (base + dev stages)
- [x] Fix pip `--break-system-packages` for multi-distro builds
- [x] Pin empy version for Humble compatibility
- [x] GHCR registry setup and authentication
- [x] Multi-arch buildx builder configuration
- [x] QEMU emulation troubleshooting
- [x] Dev user (UID 1000) for WSL2 bind-mount permissions
- [x] Python virtual environment (`/opt/venv`) for build tools
- [x] Workspace auto-sourcing in entrypoint.sh

### Dev Stage Dependencies
- [x] MoveIt2 - manipulation and motion planning
- [x] Nav2 - autonomous navigation stack
- [x] Intel RealSense SDK - camera drivers (userspace)
- [x] ROS 2 Control - controller_manager, ros2_controllers
- [x] Audio Common - PortAudio, ALSA, GStreamer dependencies

### Compose Files
- [x] `compose/viz/rviz.yaml` - RViz2 with WSLg, multicast DDS
- [x] `compose/viz/rqt.yaml` - RQT with WSLg, multicast DDS
- [x] `compose/viz/bash.yaml` - Interactive shell with unicast DDS
- [x] Multi-distro support via `${ROS_DISTRO:-humble}` in all compose files
- [x] Workspace bind-mounts (`~/ros2/{humble,jazzy}/{dev_ws,sim_ws}`)

### Documentation
- [x] Create `docs/ghcr-setup.md` (GHCR auth, buildx workflow, troubleshooting)
- [x] Create `docs/native-docker-wsl2-setup.md` (Docker CE installation, buildx setup)
- [x] Create `docs/wsl2-visualization.md` (WSLg, DDS, ZeroTier troubleshooting)
- [x] Create `docs/dev-workflow.md` (dev layer pattern, workspace management)
- [x] Create `specs/CRITIQUE_AND_BUILDX_GUIDE.md` (architecture analysis)
- [x] Update PRD Section 1 (Executive Summary with system architecture)
- [x] Add PRD Section 6.5 (Workspace Layering Convention)
- [x] Add PRD Section 6.6 (User Account Strategy)
- [x] Add PRD Section 6.7 (Python Dependency Isolation)
- [x] Update PRD Section 7.1 (WSLg/Wayland Primary)
- [x] Update PRD Section 18A (Correct WSLg compose mounts)
- [x] Update PRD Section 10 (Confirm GHCR registry)
- [x] Complete README.md rewrite (current architecture, dev layer, multi-distro)

### DDS Configuration
- [x] Create `config/dds/fastrtps_unicast.xml` (cross-VPN unicast profile)
- [x] Document multicast vs unicast modes
- [x] Test with ZeroTier VPN across networks

### Images Published
- [x] `ghcr.io/pondersome/grunt:jazzy` (base stage, multi-arch)
- [x] `ghcr.io/pondersome/grunt:humble` (base stage, multi-arch)
- [ ] `ghcr.io/pondersome/grunt:humble-dev` (dev stage, built locally, pending push)
- [ ] `ghcr.io/pondersome/grunt:jazzy-dev` (dev stage, pending build/push)

---

## ✅ Phase 1: WSL2 Visualization Stack (COMPLETED)

**Goal**: One-liner to run RViz2/RQT on WSL2 with WSLg/Wayland

### Compose Files
- [x] Create `compose/viz/rviz.yaml`
  - WSLg/Wayland mounts (`/mnt/wslg`)
  - Environment variables (WAYLAND_DISPLAY, XDG_RUNTIME_DIR, PULSE_SERVER)
  - network_mode: host for DDS discovery
  - Uses `ghcr.io/pondersome/grunt:humble-dev`
  - Multi-distro support via `${ROS_DISTRO:-humble}`
  - Workspace bind-mounts (`~/ros2/{humble,jazzy}/{dev_ws,sim_ws}`)

- [x] Create `compose/viz/rqt.yaml`
  - Similar WSLg setup as RViz
  - Command: `rqt`
  - Multi-distro support

- [x] Create `compose/viz/bash.yaml`
  - Interactive debug container
  - Unicast DDS profile for cross-VPN discovery
  - Full development environment

- [ ] Create `compose/viz/plotjuggler.yaml`
  - Install PlotJuggler in base or separate image
  - WSLg GUI support

### Docker Images
- [x] Multi-stage Dockerfile (`base` + `dev`)
  - Base stage: Core ROS 2, Gazebo, essential tools
  - Dev stage: MoveIt2, Nav2, RealSense, ros2_control, audio dependencies
  - Dev user (UID 1000) for bind-mount permissions
  - Multi-distro support (Humble + Jazzy)
  - Multi-arch support (x86_64 + ARM64)

- [x] Published images
  - `ghcr.io/pondersome/grunt:humble` (base stage, multi-arch)
  - `ghcr.io/pondersome/grunt:jazzy` (base stage, multi-arch)
  - `ghcr.io/pondersome/grunt:humble-dev` (dev stage, local build pending push)

### Documentation
- [x] Create `docs/wsl2-visualization.md`
  - Quick start: `docker compose -f compose/viz/rviz.yaml up`
  - Troubleshooting WSLg socket issues
  - X11 fallback instructions
  - Environment variable setup
  - DDS configuration (multicast vs unicast)
  - ZeroTier integration

- [x] Create `docs/native-docker-wsl2-setup.md`
  - Docker CE installation (required for buildx)
  - Multi-arch build setup with QEMU
  - Comparison: Docker CE vs docker.io vs Docker Desktop

- [x] Create `docs/dev-workflow.md`
  - Dev layer pattern explanation
  - Workspace management (three-tier layering)
  - Multi-distro development workflow
  - Dependency strategy (baked vs bind-mounted)
  - Troubleshooting guide

- [x] Update `README.md`
  - Complete rewrite for current architecture
  - Multi-stage build documentation
  - Workspace strategy
  - DDS configuration modes
  - Dev layer inventory table

### DDS Configuration
- [x] Create `config/dds/fastrtps_unicast.xml`
  - Unicast profile for cross-VPN discovery
  - Used by bash.yaml for restrictive environments

- [x] Document DDS discovery modes
  - Multicast (default, for RViz/RQT)
  - Unicast (explicit peers, for bash/cross-NAT)

### Testing
- [x] Test RViz2 on WSL2 with live robot (Barney)
  - Wayland rendering works
  - URDF meshes display correctly (with dev workspace mounted)
  - Camera feed visible (requires RQT as workaround)

- [x] Test RQT with live robot
  - Image view plugin works
  - Camera feed displays correctly

- [x] Test workspace persistence
  - Bind-mounted workspaces persist across container restarts
  - File permissions correct (UID 1000 match)
  - Builds complete successfully (12 packages tested)

- [x] Test multi-distro workflow
  - Compose files work with both Humble and Jazzy
  - `${ROS_DISTRO:-humble}` override mechanism verified

### Known Issues
- [ ] RViz frame rate slow on WSLg (Wayland rendering overhead)
- [ ] Camera/image view in RViz doesn't work (use RQT as workaround)
- [ ] tf_static discrepancy between multicast and unicast modes (requires investigation)

---

## 🌐 Phase 2: Web-Based Visualization (Priority 1)

**Goal**: Headless containers for Foxglove and Vizanti

### Compose Files
- [ ] Create `compose/viz/foxglove-bridge.yaml`
  - Use official `ghcr.io/foxglove/foxglove-bridge:latest`
  - Expose port 8765
  - network_mode: host

- [ ] Create `compose/viz/vizanti.yaml`
  - Determine if needs custom image or can use existing
  - Expose web port (8080 or similar)
  - Configure for ROS 2 topic access

### Documentation
- [ ] Add Foxglove section to `docs/wsl2-visualization.md`
  - Connection URL
  - Basic usage

---

## 🔧 Phase 3: Environment & Configuration

**Goal**: Standardize environment variables and DDS tuning

### Environment Templates
- [ ] Create `env/env.example`
  - ROS_DOMAIN_ID
  - RMW_IMPLEMENTATION
  - WAYLAND_DISPLAY, XDG_RUNTIME_DIR (WSL2)
  - PULSE_SERVER

- [ ] Create `env/env.wsl`
  - WSL2-specific defaults
  - ZeroTier network settings placeholder

- [ ] Create `env/env.jetson`
  - Jetson-specific settings
  - NVIDIA runtime variables

### DDS Configuration
- [ ] Create `config/fastrtps_profiles.xml` (examples)
  - ZeroTier interface whitelisting
  - Multi-LAN discovery tweaks
  - Bandwidth optimization for constrained devices

### Documentation
- [ ] Create `docs/networking-zerotier.md`
  - ZeroTier setup per host
  - ROS_DOMAIN_ID strategy
  - Namespace conventions (`/<robot>/<subsystem>`)

- [ ] Create `docs/ros-graph-conventions.md`
  - TF frame naming
  - Topic namespacing
  - QoS profiles

- [ ] Create `docs/dds-fastdds-tuning.md`
  - Interface binding to ZeroTier
  - Discovery tuning
  - Link to FastDDS documentation

---

## 🤖 Phase 4: Betty (Jetson) Integration (Priority 2)

**Goal**: OAK-D-Lite camera pipeline on Jetson Orin Nano

### Prerequisites
- [ ] Test `ghcr.io/pondersome/grunt:humble` on Betty (ARM64 pull)
- [ ] Verify NVIDIA Container Toolkit on Betty
- [ ] Confirm JetPack 6.2 compatibility

### Jetson-Specific Images
- [ ] Decide: Separate Dockerfile vs ARG TARGETARCH conditionals
  - Recommendation: Separate `base/Dockerfile.jetson` for clarity

- [ ] Create `jetson/Dockerfile.oakd`
  - FROM `nvcr.io/nvidia/l4t-base:r36.2.0` (JetPack 6.2)
  - Install ROS 2 Humble
  - Install DepthAI dependencies
  - Configure OAK-D-Lite

### Compose Files
- [ ] Create `compose/betty/camera-oakd.yaml`
  - runtime: nvidia
  - devices: `/dev/bus/usb`
  - NVIDIA GPU access
  - Launch DepthAI ROS 2 driver

### Documentation
- [ ] Create `docs/robots/betty.md`
  - Hardware specs (Jetson Orin Nano, JetPack 6.2)
  - OAK-D-Lite setup
  - Container vs native decision matrix

- [ ] Create `docs/devices/oakd-lite.md`
  - DepthAI driver setup
  - ROS 2 integration
  - Calibration notes

---

## 📚 Phase 5: Documentation Expansion

### Robot Documentation
- [ ] Create `docs/robots/barney.md`
  - x86_64 NUC specs
  - Native ROS 2 Humble install
  - No containerization (current state)
  - Link to main grunt repo for details

- [ ] Create `docs/robots/bambam.md`
  - RPi 5 specs
  - Ubuntu 24.04 + Jazzy
  - Test multi-arch images
  - Optional isolated ROS graph notes

### Device Documentation
- [ ] Create `docs/devices/realsense-d455.md`
  - Intel RealSense SDK setup
  - ROS 2 wrapper
  - Barney configuration

- [ ] Create `docs/devices/webcam-c920.md`
  - v4l2 setup
  - usb_cam ROS 2 node
  - Simple testing

### General Docs
- [ ] Create `docs/getting-started.md`
  - Repo overview
  - Quick start for WSL2 users
  - Links to platform-specific guides

---

## 🎯 Phase 6: Advanced Features

### rosbag2 Integration
- [ ] Create `compose/common/bagging.yaml`
  - rosbag2 record/play services
  - QoS override file mounts
  - Log rotation (`--max-bag-size`, `--max-splits`)

- [ ] Create `tools/rosbag2-qos-profiles/`
  - QoS override examples
  - Sensor data profiles

### Development Environment
- [ ] Extend Dockerfile with `dev` stage
  - Create `dev` user (UID 1000)
  - sudo access
  - Bind-mount support for `~/dev_ws`
  - VSCode devcontainer integration

### Build Automation
- [ ] Create buildx bake file
  - Multi-distro builds (Humble, Jazzy)
  - Multi-arch targets
  - Tag management

---

## 🔬 Phase 7: Experimental (Lower Priority)

### Jazzy + Zenoh
- [ ] Create `docs/jazzy-zenoh-notes.md`
  - Zenoh router setup
  - rmw_zenoh configuration
  - Multi-site graph patterns
  - Test on BamBam (RPi 5)

### Gazebo Simulation
- [ ] Create `compose/sim/gazebo-garden.yaml`
  - Gazebo Harmonic (preferred) or Garden
  - ros_gz_bridge configuration
  - Document Humble caveats

### Additional Viz Tools
- [ ] Investigate additional tools
  - PlotJuggler (if not in Phase 1)
  - Custom RQT plugins
  - Web-based alternatives

---

## 📋 Acceptance Criteria (v0.1)

From [PRD Section 14](../specs/grunt_docker_prd_v_0.md#14-acceptance-criteria-v01):

- [x] `compose/viz/rviz.yaml` runs RViz on Windows 11 WSL2 (Wayland)
  - **Status**: Working, tested with live robot (Barney)
  - URDF meshes display correctly with dev workspace
  - Multicast DDS for full topic discovery

- [ ] `compose/viz/foxglove-bridge.yaml` exposes working WebSocket bridge
  - **Status**: Not implemented (Phase 2)

- [ ] `compose/betty/camera-oakd.yaml` runs on Jetson with OAK-D-Lite, publishes camera topics
  - **Status**: Not implemented (Phase 4)

- [x] Docs exist for ZeroTier join + ROS_DOMAIN_ID + namespace conventions
  - **Status**: Documented in `docs/wsl2-visualization.md`
  - DDS configuration modes explained
  - ZeroTier integration tested

- [x] Barney host-native instructions linked; no regression to current workflows
  - **Status**: Native install documented in README and PRD
  - Containers verified to interoperate with native Barney nodes
  - RViz/RQT containers successfully visualize Barney's topics

---

## 🗂️ Repository Restructuring (When Ready)

### Migration Tasks
- [ ] Move `dependencies.repos` → `base/dependencies.repos`
  - Update Dockerfile COPY path

- [ ] Create directory structure matching PRD Section 5
  - `compose/` (already exists in plan)
  - `env/` (already exists in plan)
  - `samples/`
  - `tools/colcon-mixin-profiles/`

- [ ] Update `README.md`
  - New structure overview
  - Quick start commands
  - Links to documentation

---

## 🏷️ Image Tagging Strategy

### Current (Published)
- `ghcr.io/pondersome/grunt:humble` (base stage, multi-arch: x86_64 + ARM64)
- `ghcr.io/pondersome/grunt:jazzy` (base stage, multi-arch: x86_64 + ARM64)

### Current (Local, Pending Push)
- `ghcr.io/pondersome/grunt:humble-dev` (dev stage, multi-arch)
- `ghcr.io/pondersome/grunt:jazzy-dev` (dev stage, pending build)

### Planned
- `ghcr.io/pondersome/grunt_viz:jazzy` (RViz/RQT-specific image, if needed)
- `ghcr.io/pondersome/grunt_viz:humble` (RViz/RQT-specific image, if needed)
- `ghcr.io/pondersome/grunt_betty:humble-jetson` (Jetson-specific, ARM64 only)

### Versioning (Future)
- Add dated tags: `grunt:humble-20251014`
- Add semantic versions: `grunt:v0.1.0`
- Add `latest` pointer
- Add stage-specific tags: `grunt:humble-base`, `grunt:humble-dev`

---

## 📝 Notes

- **Priority 1 (WSL2 Viz)**: Highest value, easiest to test locally
- **Priority 2 (Betty)**: Requires hardware access, can test ARM64 images
- **GHCR Auth**: Already configured, documented in `docs/ghcr-setup.md`
- **Multi-arch**: Base images proven working, extend pattern to other images
- **Native Installs**: Barney stays native; document in `docs/robots/barney.md`
- **Platform Support**: Documentation emphasizes WSL2 (primary platform), but images/compose files work on native Ubuntu Linux and cloud instances with standard Docker practices. See README Platform Support section.

---

## 🔗 Related Documents

- [PRD](../specs/grunt_docker_prd_v_0.md) - Product Requirements
- [CRITIQUE & Buildx Guide](../specs/CRITIQUE_AND_BUILDX_GUIDE.md) - Architecture Analysis
- [GHCR Setup](ghcr-setup.md) - Registry Authentication & Multi-Arch Builds
