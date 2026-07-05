# Vizanti Setup — Off-Robot Web Mission Planner

Vizanti is the web-based mission planner/visualizer used with the grunt fleet.
This guide runs it in a container on an admin machine (hal or any operator
workstation), **taking its flask server, websocket bridge, and tf_consolidator
load off the robot's CPU-constrained N100**.

## Why off-robot

Barney's N100 (4 E-cores, ~6W) runs ~55 nodes and sits at load average 11+.
The vizanti server stack is a pure consumer of ROS topics — nothing about it
needs to be on the robot. Browser clients connect to the admin machine instead
of the robot; topic traffic flows once over the VPN either way.

> After bringing this service up, **stop the on-robot vizanti** to actually
> reclaim the CPU.

## Vizanti is never baked into images

Vizanti is under active development. It builds from source in the bind-mounted
dev workspace, so updating it never requires an image rebuild or pull:

```bash
# one-time: clone the dev workspace repos (includes vizanti + rws)
./tools/setup-dev-workspace.sh jazzy

# build inside the container
docker compose -f compose/viz/bash-multicast.yaml run --rm bash
# inside:
cd ~/dev_ws
colcon build --symlink-install --packages-up-to vizanti_server vizanti_cpp vizanti_msgs
exit
```

The grunt dev image supplies the runtime dependencies (`rosbridge-suite`,
`python3-flask`, `python3-waitress`), so no extra installs are needed.

### Update loop (frequent, by design)

```bash
cd ~/ros2/jazzy/dev_ws/src/vizanti && git pull
docker compose -f compose/viz/bash-multicast.yaml run --rm bash \
  -c "cd ~/dev_ws && colcon build --symlink-install --packages-up-to vizanti_server vizanti_cpp vizanti_msgs"
docker compose -f compose/vizanti/server.yaml restart
```

## Quick start

```bash
docker compose -f compose/vizanti/server.yaml up -d
```

Open `http://<admin-machine>:5000` in a browser.

### Access from ZeroTier devices (phones, tablets, laptops)

Two names reach the same server from anywhere on the robodojo ZeroTier network:

- **`http://halbuntu.robodojo.net:5000`** — works out of the box. This is the
  WSL-side ZeroTier node, which is where the container actually listens.
- **`http://hal.robodojo.net:5000`** — the Windows-side ZeroTier node. Because
  this machine runs WSL2 in NAT mode, Windows must forward ports 5000/5001
  into WSL. One-time setup from an **elevated** PowerShell:

  ```powershell
  powershell -ExecutionPolicy Bypass -File tools\windows\vizanti-portproxy.ps1
  ```

  The script adds `netsh portproxy` rules (bound to the ZT address only,
  forwarding via the reboot-stable localhost relay) plus matching firewall
  rules. Both ports are required: the browser loads the UI from 5000, then
  opens its websocket directly to 5001.

| Port | Env var | Purpose |
|---|---|---|
| 5000 | `VIZANTI_PORT` | Web UI (HTTP, flask/waitress) |
| 5001 | `VIZANTI_ROSBRIDGE_PORT` | rosbridge websocket (the browser connects here too) |

## Deployment options

- **Option A — admin machine (this compose file, recommended):** server runs on
  hal/operator workstation, discovers robot topics via DDS over ZeroTier.
  Robot CPU untouched.
- **Option B — native on robot (legacy):** the original arrangement, vizanti in
  barney's ros2_ws launched with the bringup. Only appropriate for tethered
  field work with no admin machine reachable.

## DDS notes

Same-LAN (multicast) discovery works out of the box. For cross-VPN operation,
uncomment the `FASTRTPS_DEFAULT_PROFILES_FILE` line in
`compose/vizanti/server.yaml` to use `config/dds/fastrtps_unicast.xml`, and
make sure this machine is listed in the robot-side
`grunt_bringup/config/fastrtps_profiles.xml` `initialPeersList` — unicast
discovery is mutual. See `docs/dds-cross-nat-troubleshooting.md`.

## rws variant (higher performance)

`vizanti_rws.launch.py` replaces rosbridge+rosapi with
[rws](https://github.com/v-kiniv/rws), a faster rosbridge-protocol server
(cloned into dev_ws by `setup-dev-workspace.sh`, branch `jazzy`). Build it
alongside vizanti (`--packages-up-to ... rws`), then swap the launch file name
in the compose `command:`. Functionally equivalent from the browser's
perspective; useful when many topics or large messages make rosbridge the
bottleneck.

## Troubleshooting

- **`dev_ws not built` error on startup** — the compose command checks for
  `~/dev_ws/install/setup.bash`; run the build steps above.
- **Web UI loads but no topics** — DDS discovery problem, not vizanti. Verify
  with the bash container: `ros2 topic list` under the same DDS mode
  (multicast vs unicast profile).
- **Healthcheck failing** — `docker inspect vizanti_jazzy --format
  '{{json .State.Health}}'`; the check is a plain HTTP GET on the web UI port.
- **Port already in use** — the service uses host networking; set
  `VIZANTI_PORT`/`VIZANTI_ROSBRIDGE_PORT` to free ports.
