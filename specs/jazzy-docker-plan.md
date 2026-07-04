# Jazzy Docker Migration Plan

**Date:** 2026-07-04
**Goal:** Bring grunt_docker from Humble-first to Jazzy-first, matching barney's post-migration reality (Ubuntu 24.04 / ROS 2 Jazzy, migrated natively April 2026), add an off-robot Vizanti service, validate everything locally, and publish refreshed multi-arch images to GHCR.

## Context

- **The robot moved; the containers didn't.** Barney was migrated in place to Jazzy on 2026-04-05..09 (see `W:\projects-ai\jazzy-upgrade\jazzy-upgrade\progress.md`). The published `ghcr.io/pondersome/grunt:jazzy` image dates from **2025-10-14** — its baked `/ros2_ws` (p2os2 + grunt) is ~9 months stale, and `grunt:jazzy-dev` was never pushed.
- **Docker's role is unchanged (hybrid stance):** robot core nodes run native; containers serve operator workstations (RViz/RQT/Foxglove/dev shells) and off-robot headless services. This plan keeps that stance.
- **New requirement: Vizanti off-robot.** Barney currently runs the vizanti web server on its N100 (4 E-cores, load avg 11+, per `ponderdocs/grunt/docs/cpu_constraints.md`). Moving vizanti (and its rosbridge/rws websocket server) into a container on an admin machine removes that load. Vizanti is under **very active development**, so it must NOT be baked into published images — it lives in the bind-mounted `dev_ws` like `by_your_command`.
- **Good news from the audit:** `base/Dockerfile` is already distro-parameterized (`ARG ROS_DISTRO=jazzy` is its default), `GZ_VERSION=gz-harmonic` is the correct Jazzy pairing, PEP-668 `--break-system-packages` handling is already jazzy-gated, and multi-arch buildx + GHCR infrastructure is proven (existing `grunt:jazzy` manifest is amd64+arm64). The migration debt is concentrated in **defaults, the baked workspace contents, dev-layer deps, and docs**.
- **No GPU/CUDA needed:** nothing in barney's source tree uses CUDA (`by_your_command` explicitly installs CPU torch). The `cuda-keyring` deb at `W:\` is a leftover. GPU images remain a Betty/Jetson (JetPack 7.x) future item, out of scope here.

## Container boundary — what never runs in a container

Hardware-driver nodes that need physical port access run **on the robot, natively, always**: `p2os_driver` (`/dev/grunt_p3at`), `ublox_gps`/NTRIP (`/dev/grunt_f9p`), `bno055` (`/dev/grunt_imu`), `roarm_driver` (`/dev/grunt_arm`), realsense (USB), `l2lidar_node` (Ethernet/UDP + Qt 6.10). Containers consume their *topics*; images need only their *message/description packages*. Nothing in this plan containerizes a device driver.

**The exception is audio.** `audio_common` capture/playback is hardware-adjacent but relocatable: byc (running on-bot, as it does today and usually will) can wire its mic/speaker to **hal** instead of the robot's audio hardware, with audio flowing as ROS topics. **Verified feasible 2026-07-04:** from inside a grunt dev container on hal, WSLg's PulseAudio bridge (`/mnt/wslg/PulseServer`) enumerates both `RDPSink` (Windows playback) and `RDPSource` (Windows mic, 1ch/44.1kHz); `parecord` captured real mic data and `paplay` played back successfully. So Windows hardware → WSLg → container audio works in both directions with just the mounts/env the compose files already pass. **Human-confirmed same day:** the capture/playback loop test audibly reproduced live speech through hal's headset.

**Future, not in this plan:** running all of byc on a collaborator machine (hal). Feasible in principle over ZeroTier, but needs bandwidth-aware partitioning first — e.g. not streaming full-frame-rate RealSense video off-robot only to discard most frames on hal before forwarding to Gemini. Requires refinement in byc itself (frame decimation/on-demand capture at the source); park it.

## Key facts driving the design

| Fact | Source | Consequence |
|---|---|---|
| `grunt` main now depends on `l2lidar_node`, `grunt_behaviors`, `isr_wifi` | `W:/src/grunt/*/package.xml` | Baked-image `rosdep install` will fail on unresolvable keys → expand `--skip-keys` |
| `behaviors` and `isr` repos have **no git remote** | `W:/src/*/.git/config` | Can't be cloned into image builds; their message types won't decode off-robot until pushed |
| `l2lidar_node` needs Qt 6.10 (not in 24.04 apt) | `ponderdocs/.../l2_driver_analysis.md` | Never build it in containers; driver is robot-only, viz consumes standard PointCloud2 |
| Vizanti = Flask :5000 + rosbridge/rws :5001 | `W:/src/vizanti/vizanti_server/launch/*.launch.py` | Compose service shape; needs `python3-flask`, `python3-waitress`, `ros-jazzy-rosbridge-suite` in dev image; `rws` (v-kiniv) as source-built alternative |
| Robot DDS = FastDDS unicast, static peers by hostname (`barney/wilma/betty/hal/kvlapblack.robodojo.net`) | `W:/fastrtps_profiles.xml` | Repo's `config/dds/fastrtps_unicast.xml` (hardcoded 10.147.20.x IPs) must be reconciled; containers need those hostnames resolvable (`extra_hosts` or ZeroTier DNS) |
| realsense-ros fork obsolete — tf_prefix PR merged upstream; org moved to `realsenseai` | migration docs; `W:/src/realsense-ros/.git/config` | Update `tools/grunt_repos.yaml` to upstream `realsenseai/realsense-ros` |
| robot_localization on barney is source-built jazzy-devel HEAD (EKF divergence workaround lives in grunt supervisor, not r_l) | `grunt_localization_jazzy_followup.md` | No image change needed — r_l comes via rosdep for viz purposes only |
| Local WSL host: Docker 28.5.1, buildx+binfmt, WSLg, GTX 1070, but no `~/ros2/jazzy` workspace and no GHCR login | direct inspection | Test env is ready; need `setup-dev-workspace.sh jazzy` and a one-time `docker login ghcr.io` before push |
| Uncommitted Foxglove work (Phase 2) sits in the working tree | `git status` | Commit it first, separately from Jazzy changes |

## Phase 0 — Baseline and housekeeping

1. **Commit the pending Foxglove work as-is** (README, base/Dockerfile foxglove-bridge block, ROADMAP, cheatsheet, `compose/foxglove/`, `docs/foxglove-setup.md`) as its own commit so Jazzy changes diff cleanly.
2. **Cold rebuild of `grunt:jazzy` base (amd64, `--no-cache`)** against today's `pondersome/grunt` main to surface rot: expected failures are new rosdep keys (`l2lidar_node`, `grunt_behaviors`, `isr_wifi`) and any apt/repo drift since October. This tells us exactly what Phase 1 must fix.
3. Retire the stale `bashrc_custom` (references nonexistent `~/grunt_ws`); move `dependencies.repos` → `base/dependencies.repos` per the ROADMAP's planned layout, updating the Dockerfile COPY.

## Phase 1 — Jazzy-first configuration

1. **Flip defaults** `${ROS_DISTRO:-humble}` → `${ROS_DISTRO:-jazzy}` in all compose files (`compose/viz/rviz.yaml`, `rqt.yaml`, `bash.yaml`, `bash-multicast.yaml`, `viz-combined.yaml`, `compose/foxglove/bridge.yaml`) and `tools/setup-dev-workspace.sh:20`. Humble remains available via explicit `ROS_DISTRO=humble` override. Add a root `.env.example` (`ROS_DISTRO=jazzy`, `FOXGLOVE_PORT`, `VIZANTI_PORT`, …) as the single documented override point (starts ROADMAP Phase 3's env work).
2. **`base/Dockerfile`:**
   - Keep `FROM osrf/ros:${ROS_DISTRO}-desktop`, `GZ_VERSION=gz-harmonic`.
   - Expand rosdep skip: `--skip-keys='rviz l2lidar_node grunt_behaviors isr_wifi'` (grunt main's new deps that are robot-only or unpublishable).
   - Add `ros-${ROS_DISTRO}-foxglove-msgs` explicitly if rosdep doesn't already pull it (grunt_bringup depends on it).
   - **Dev stage additions for vizanti hosting:** `ros-${ROS_DISTRO}-rosbridge-suite`, `python3-flask`, `python3-waitress`. (foxglove-bridge already added in the pending diff.)
   - **Dev stage additions for hal audio:** `pulseaudio-utils`, `libasound2-plugins` (ALSA→Pulse bridge so audio_common's PortAudio path can reach WSLg's Pulse server), `python3-pyaudio` if not already pulled by the existing portaudio deps.
   - **Fix the librealsense apt repo:** the October image's `librealsense.intel.com` repo now fails GPG verification (Intel spun RealSense out to realsenseai, July 2025). Switch to the realsenseai vendor repo (barney uses librealsense 2.57.7 from it) — userspace libs only, no DKMS, as before.
3. **`tools/grunt_repos.yaml`:** add `vizanti` (pondersome fork, branch `ros2`) and `rws` (v-kiniv, for the higher-performance websocket variant barney already uses); switch `realsense-ros` from the pondersome fork to `realsenseai/realsense-ros` (`ros2-development`), matching barney.
4. **DDS reconciliation:** regenerate `config/dds/fastrtps_unicast.xml` from barney's live `W:/fastrtps_profiles.xml` — hostname-based peers (`*.robodojo.net`) covering barney, wilma, betty, hal, kvlapblack. Add `extra_hosts` guidance (or verify ZeroTier DNS inside containers) in the DDS docs. Document the coupling: an operator machine only discovers barney if it appears in the robot-side static peer list.

## Phase 2 — Vizanti off-robot service

1. **`compose/vizanti/server.yaml`** modeled directly on `compose/foxglove/bridge.yaml`: image `ghcr.io/pondersome/grunt:${ROS_DISTRO:-jazzy}-dev`, `user: 1000:1000`, `network_mode: host`, `restart: unless-stopped`, read-only `config/dds` mount, `~/ros2/${ROS_DISTRO}/dev_ws` mount (read-write — vizanti is built there), command sourcing dev_ws then `ros2 launch vizanti_server vizanti_server.launch.py flask_debug:=false`. HTTP healthcheck against `:5000` (a real HTTP endpoint, unlike the WebSocket-only Foxglove port). Variables `VIZANTI_PORT=5000`, `VIZANTI_ROSBRIDGE_PORT=5001`. Provide a commented `vizanti_rws.launch.py` alternative.
2. **Active-development workflow documented in `docs/vizanti-setup.md`:** vizanti never bakes into images; update loop is `git -C ~/ros2/jazzy/dev_ws/src/vizanti pull` → one-shot `docker compose run` colcon build → restart service. Mirror foxglove-setup.md's Option A/B/C structure and include the "stop the on-robot vizanti to reclaim N100 CPU" step (this is the point of the exercise).
3. Also fix the Foxglove healthcheck while in there (curl against a WebSocket port can false-fail; use a WS handshake or TCP check).

## Phase 2b — Hal audio in/out (near-term priority)

Let byc (on-bot) use hal's microphone and speakers via ROS audio topics. Plumbing already verified (see Container boundary section).

1. **`compose/audio/hal-audio.yaml`:** service on `grunt:jazzy-dev`, `network_mode: host`, mounts `/mnt/wslg` + `PULSE_SERVER=unix:/mnt/wslg/PulseServer` (same pattern as the viz files), mounts `dev_ws` (audio_common is in `grunt_repos.yaml` and builds there), runs audio_common capturer + player nodes namespaced so byc on barney can select hal audio vs. robot audio.
2. **Topic/namespace contract with byc:** decide how byc switches audio endpoints (e.g. `/hal/audio/*` vs robot-local topics, or remapping at launch). Small byc-side config change at most; coordinate rather than modify byc here.
3. **Format note:** WSLg capture is 1ch/44.1kHz via RDP; byc's VAD/ASR path wants 16kHz mono — resample at the capture node (audio_common supports rate config) to avoid shipping 44.1kHz over ZeroTier.
4. Human acceptance: speak into hal's mic → byc hears it; byc TTS → hal speakers. Latency sanity check (RDP audio adds some; fine for conversational use, measure anyway).

## Phase 3 — Local validation (no robot, no babysitting)

Everything here runs on this machine's WSL2 (Docker 28.5.1, WSLg, 8 cores/23 GB). Scripted where practical under `tools/test/` so it's repeatable.

1. **Builds:** `grunt:jazzy` and `grunt:jazzy-dev`, amd64, cold cache.
2. **Container smoke:** entrypoint sources cleanly; `ros2 doctor`; RMW default fastrtps and switchable to cyclonedds.
3. **Two-container DDS:** talker/listener over host network (multicast), then again with the unicast profile in loopback mode.
4. **GUI:** rviz2 and rqt launch under WSLg and stay alive (repo already documents WSLg quirks).
5. **Foxglove bridge:** service healthy, WebSocket handshake on 8765 succeeds.
6. **Vizanti end-to-end (fully self-testable):** `setup-dev-workspace.sh jazzy` → colcon build vizanti(+rws) in the dev container → service up → HTTP 200 on :5000, WS handshake on :5001, page serves.
7. **Bag replay integration test:** copy a bag from `W:/bags` locally; play it in a `bash` container; verify topics *and message types* decode in Foxglove/Vizanti/RViz. This exercises the baked message packages (p2os_msgs, ublox_msgs, …) with real robot data — the closest thing to a live-robot test that needs zero robot time. Note which topics fail to decode (expected: `isr_msgs` types, since `isr` has no public remote).
8. **Robot-workspace compile test (the big one):** copy `W:/src` into a scratch jazzy dev_ws, run rosdep + colcon build inside `grunt:jazzy-dev` with barney's known COLCON_IGNOREs (moveit_servo, roarm_web_app, ros2web) plus l2lidar_node excluded (Qt 6.10). Success proves the dev image is a complete build environment for the entire robot codebase.
9. **Audio loop test:** container on hal records from `RDPSource` and plays to `RDPSink` (plumbing already proven); then audio_common capturer→topic→player loop between two containers. Only the "is it audible / did the mic hear me" check needs a human.
10. **arm64 builds** via binfmt/QEMU (slow; run in background) to keep the multi-arch manifest honest.

**Deferred to a short live checklist with the user (KVM/robot required):** live DDS to barney over ZeroTier (operator host must be in barney's static peers), Foxglove/Vizanti against the live robot, stopping on-robot vizanti and confirming CPU recovery, joystick/audio passthrough if wanted.

## Phase 4 — Distribution

1. One-time `docker login ghcr.io` (no credentials currently stored in WSL) — needs a PAT with `write:packages`.
2. **Push multi-arch (amd64+arm64)** via buildx: `grunt:jazzy`, `grunt:jazzy-dev`, plus immutable date tags `jazzy-YYYYMMDD` / `jazzy-dev-YYYYMMDD` for rollback.
3. Add **`docker-bake.hcl`** encoding the distro × target × platform matrix (ROADMAP Phase 6 item) so future refreshes are one command. GH Actions automation stays future work.
4. Leave `humble` tags frozen as legacy; stop rebuilding them.
5. Post-push pull test from a clean image cache.

## Phase 5 — Docs sweep and wrap-up

- Jazzy-first pass over `README.md`, `docs/docker-commands-cheatsheet.md`, `docs/dev-workflow.md`, `docs/getting-started-wsl2.md`, `docs/wsl2-visualization.md`, `docs/foxglove-setup.md` (container names, `~/ros2/jazzy/...` paths, `grunt:jazzy-dev` pulls, Ubuntu 24.04 references). Keep a short "using Humble (legacy)" note rather than deleting.
- Update `docs/ROADMAP.md`: robot-state table (barney = 24.04/Jazzy), mark vizanti done, record the realsense/realsenseai change.
- New `docs/vizanti-setup.md` (Phase 2).
- Session log into `specs/sessions/` per repo convention; commits in logical units (foxglove → hygiene → jazzy flip → vizanti → docs).

## Risks / open items

- **`behaviors` and `isr` have no remotes** — until pushed, their message types can't ship in images and won't decode in off-robot viz. Recommend pushing at least `isr_msgs`/interface packages (or vendoring them into `grunt`).
- **PRD drift:** the PRD still describes the abandoned `grunt_<role>` tagging and 22.04 fleet; treat ROADMAP as truth, optionally annotate the PRD header.
- **Betty/Jetson Jazzy (JetPack 7.x) and rmw_zenoh** remain future phases, unblocked but untouched by this plan.
- QEMU arm64 builds of colcon workspaces are slow (potentially hours); acceptable, run overnight if needed.
- WSLg audio is RDP-bridged: mono 44.1kHz capture, unmeasured added latency. Plumbing is proven; conversational quality is probable but unconfirmed until the human loop test. If latency disappoints, fallback is running the capture node natively in WSL (same Pulse socket, one less hop) — the compose service makes that swap trivial.
- Full byc-on-hal partitioning is explicitly out of scope (needs bandwidth-aware video handling in byc first).
