# grunt_docker • Product Requirements Doc (PRD) v0.1

**Purpose**
Unify all ROS 2 + Docker configurations for the *Grunt* sentry platform into one repo (this repo), while documenting non‑containerized setups that participate in the same robot graph. Optimize for clarity for new contributors and repeatable, cross‑arch builds.

---
## 1) Executive summary
- **Single ROS graph** initially (Barney + Betty + laptops/desktops). BamBam can be segmented later via domain/namespace if needed.
- **Transport**: Fast DDS today; evaluate **zenoh** for Jazzy migrations.
- **Priorities**
  1. Desktop/laptop **visualization stacks** (RViz2, RQT, PlotJuggler, Foxglove/Vizanti) on Windows 11 + WSL2.
  2. **Betty (Jetson Orin Nano, Ubuntu 22.04, JetPack 6.2)** running Humble (host or containers) and OAK‑D‑Lite camera.
  3. Shared **networking patterns** over ZeroTier (same domain id, namespacing, TF frames).
- **Barney (x86_64, 22.04+Humble)** remains host‑native for now; repo documents but does not containerize core robot nodes yet.

---
## 2) Platforms & roles
| Host | CPU/GPU | OS | ROS | Role | Container stance |
|---|---|---|---|---|---|
| **Barney** | x86_64 iGPU | Ubuntu 22.04 | Humble | Primary dev robot | Native (host) for core nodes; optional per‑subsystem containers later |
| **Betty** | ARM64 + Jetson GPU | Ubuntu 22.04 (JetPack 6.2) | Humble initially | GPU accel; OAK‑D‑Lite | Mix of host + containers. Use NVIDIA Container Toolkit. |
| **BamBam** | RPi 5 (ARM64) | Ubuntu 24.04 | Jazzy | Migration testbed | Documented; optional isolated ROS graph if needed |
| **Windows 11 workstations (WSL2)** | x86_64 + some GPU | WSL2 Ubuntu 24.04 | Humble/Jazzy client tools | Visualization, dev tools, occasional compute | Prefer containers for GUI stacks; WSLg Wayland.

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
│  │  ├─ Dockerfile.humble    # x86_64 + arm64
│  │  └─ Dockerfile.jazzy
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
## 7) Compose stacks (first wave)
### 7.1 Visualization on Windows 11 + WSL2 (Wayland/WSLg)
- Goal: one‑liner `docker compose -f compose/viz/rviz.yaml up` runs RViz on WSL2 with Wayland output.
- Mounts:
  - Wayland socket: ensure `/run/user/UID/wayland-0` exists; fallback symlink from `/mnt/wslg/runtime-dir/wayland-0` if needed.
  - PulseAudio/ALSA (optional) for audio.
- Env: `WAYLAND_DISPLAY=wayland-0`, `XDG_RUNTIME_DIR=/run/user/UID`.
- Network: default bridge; ROS graph connectivity via ZeroTier from host or container (document both patterns).

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
- **Registry (TBD)**: start with Docker Hub under `pondersome/*`; later: GHCR or self‑hosted.
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
      - WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
      - XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/1000}
    volumes:
      - ${XDG_RUNTIME_DIR:-/run/user/1000}/${WAYLAND_DISPLAY:-wayland-0}:${XDG_RUNTIME_DIR:-/run/user/1000}/${WAYLAND_DISPLAY:-wayland-0}
      - /tmp/.X11-unix:/tmp/.X11-unix:ro   # fallback for X11 apps
      - ../..:/work:ro
    network_mode: host   # simplest for ROS graph on a dev box
    command: ["bash","-lc","source /opt/ros/humble/setup.bash && rviz2"]
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
- ROS 2 Humble/Jazzy docs; QoS primer; rosbag2 QoS overrides.
- Docker Compose v2 reference.
- WSLg GUI apps guide & troubleshooting notes for Wayland sockets.
- NVIDIA Container Toolkit + L4T images for Jetson; JetPack 6.2 docs.
- Gazebo Garden + ros_gz bridge (Humble caveats noted).
- DepthAI (OAK‑D) + ROS 2 driver.
- Foxglove Bridge; Vizanti.

---
*End of PRD v0.1*

