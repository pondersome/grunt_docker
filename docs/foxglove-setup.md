# Foxglove Bridge Setup

This guide covers setting up Foxglove Bridge for web-based ROS 2 visualization on Windows 11 WSL2 and native Ubuntu Linux.

**Foxglove Studio** is a modern, web-based robotics visualization and debugging platform that provides an alternative to RViz2 and RQT with better remote access capabilities.

---

## What is Foxglove?

**Foxglove Studio** is a visualization tool that runs in your web browser (or as a desktop app) and connects to ROS 2 via a WebSocket bridge.

**Key Advantages over RViz/RQT:**
- **Web-based**: Access from any device with a browser (laptop, tablet, phone)
- **Remote-friendly**: Works well over VPN and WAN connections
- **Modern UI**: Drag-and-drop panels, customizable layouts, dark mode
- **Performance**: Efficient binary protocol, selective topic subscription
- **Cross-platform**: Same interface on Windows, macOS, Linux, mobile
- **Shareable layouts**: Export/import panel configurations
- **Data recording**: Built-in MCAP recording (modern bag format)

**When to Use Foxglove vs RViz:**
- **Foxglove**: Remote monitoring, multi-user access, web-based dashboards, data analysis
- **RViz**: 3D visualization, robot modeling (URDF), marker displays, local development

Both tools are complementary - use RViz for intensive 3D work, Foxglove for remote monitoring and dashboards.

---

## Prerequisites

Before setting up Foxglove Bridge, you must have:

- **WSL2 with Ubuntu 22.04/24.04** (Windows 11) or **native Ubuntu Linux**
- **Docker CE** installed (native Docker, not Docker Desktop)
  - See [native-docker-wsl2-setup.md](native-docker-wsl2-setup.md) for installation
- **Working ROS 2 environment** (Jazzy, or legacy Humble)
  - Verify: `docker compose -f compose/viz/bash.yaml run --rm bash` should work
- **VPN connection** to robot network (ZeroTier or similar)
  - See [wsl2-visualization.md](wsl2-visualization.md#zerotier-setup-options)
- **ROS 2 topics available** from robots
  - Verify: `ros2 topic list` shows robot topics (e.g., `/barney/odom`)

**If you haven't completed initial setup**, see [getting-started-wsl2.md](getting-started-wsl2.md) first.

---

## Deployment Options Analysis

Foxglove Bridge can run in different locations. Understanding the tradeoffs helps choose the right deployment for your use case.

### Option A: Bridge on Admin Machine (Recommended for Development)

**Architecture:**
```
Robot (Barney) → DDS/ROS 2 → VPN → Admin Machine (Bridge) → WebSocket → Browser
```

**How it works:**
- Bridge runs on WSL2/operator workstation via Docker Compose
- Connects to robot topics via DDS over ZeroTier VPN
- Serves WebSocket to browser (localhost or remote)

**Advantages:**
- ✅ **No robot resource usage** - Barney's CPU/memory/network unaffected
- ✅ **Easy updates** - Update bridge without touching robot (docker pull)
- ✅ **Multi-robot support** - One bridge sees all robots (barney, betty, bambam)
- ✅ **Containerized** - Clean install/uninstall via compose
- ✅ **Development-friendly** - Stop/restart/reconfigure without affecting robot

**Disadvantages:**
- ❌ **Network bandwidth** - All message traffic traverses VPN:
  - Robot → VPN → Admin machine (DDS traffic)
  - Admin machine → Browser (WebSocket, typically local)
- ❌ **Latency** - Additional network hop adds ~10-50ms (VPN dependent)
- ❌ **DDS discovery** - Must ensure admin machine discovers all robot topics
- ❌ **Single point of failure** - Bridge down = no Foxglove access

**Performance impact:**
- **Low-bandwidth topics** (tf, odom, joint_states): ~1-5 Mbps total - **works great**
- **High-bandwidth topics** (cameras, point clouds):
  - 1080p camera @ 30fps: ~30-60 Mbps
  - RealSense depth @ 30fps: ~50-100 Mbps
  - VPN becomes bottleneck with multiple camera streams

**Best for:**
- Active development with operator workstation always available
- Monitoring multiple robots from single dashboard
- Quick iteration and configuration changes
- Scenarios where VPN bandwidth is sufficient (low-bandwidth topics or selective camera viewing)

---

### Option B: Bridge on Robot (Native Install)

**Architecture:**
```
Robot (Barney) → Bridge (localhost) → WebSocket → VPN → Browser on Admin Machine
```

**How it works:**
- Bridge installed natively on Barney (systemd service)
- Talks to ROS nodes via localhost (no network overhead)
- Serves WebSocket over VPN to remote browser

**Advantages:**
- ✅ **Minimal latency** - Bridge talks to ROS via localhost (no DDS network)
- ✅ **Efficient bandwidth** - WebSocket protocol more efficient than DDS multicast
- ✅ **No DDS discovery issues** - Bridge sees all local topics automatically
- ✅ **Works offline** - Accessible via direct connection (no VPN required)
- ✅ **Selective subscription** - Bridge only sends topics Foxglove client requests

**Disadvantages:**
- ❌ **Robot resource usage**:
  - CPU: ~5-15% (depending on topic selection and rates)
  - Memory: ~100-200 MB
  - Network: Outbound WebSocket traffic
- ❌ **Updates require robot access** - Can't update bridge without SSH to robot
- ❌ **Per-robot deployment** - Need bridge on each robot (not DRY)
- ❌ **Native install complexity** - Not containerized (manual systemd setup)

**Best for:**
- Remote field operations with poor/intermittent VPN
- Production deployments requiring persistent monitoring
- Local web UI on robot network (direct WiFi to robot AP)
- Autonomous robots operating far from home base

---

### Option C: Hybrid (Both Deployments)

**Best of both worlds:**
- Bridge on admin machine for **active development** (Option A)
- Bridge on robot for **field operations** (Option B)
- Toggle based on scenario

**Implementation:**
```yaml
# compose/foxglove/bridge.yaml (admin machine) - Use during development
# Native install on Barney (systemd service) - Disabled by default, enable for field ops
```

**When to use:**
- Development phase: Use admin machine bridge (easy iteration)
- Field deployment: Enable robot bridge (reliable remote access)

---

### Recommendation: Start with Option A

For the Grunt platform in **active development phase**, we recommend **Option A (Bridge on Admin Machine)**.

**Rationale:**
1. **Aligns with hybrid strategy** - Visualization on workstation, native on robot
2. **Keeps robot lean** - Barney's resources reserved for navigation/manipulation
3. **Easy iteration** - Update bridge without touching robot
4. **Multi-robot support** - Single bridge sees all robots
5. **VPN bandwidth sufficient** - For low-bandwidth topics (tf, odom, joint_states)
6. **Future flexibility** - Can add Option B later for field ops

**Mitigation for bandwidth concerns:**
- Use selective topic subscription in Foxglove UI (don't view all topics at once)
- Enable camera topics only when actively debugging vision
- Foxglove only subscribes to topics visible in active panels

---

## Quick Start

### Step 1: Launch Foxglove Bridge

From the `grunt_docker` repository root:

```bash
docker compose -f compose/foxglove/bridge.yaml up
```

**First time**: Docker will pull `ghcr.io/pondersome/grunt:jazzy-dev` if not already cached locally

The image includes `ros-jazzy-foxglove-bridge` installed via apt.

**Output should show:**
```
foxglove_bridge_jazzy | [INFO] Foxglove Bridge started on port 8765
foxglove_bridge_jazzy | [INFO] Listening for WebSocket connections...
```

### Step 2: Open Foxglove Studio

**Option A: Web Browser** (Recommended)

Open browser to: **https://app.foxglove.dev** (official hosted version)

**Option B: Desktop App** (Alternative)

Download from: https://foxglove.dev/download

### Step 3: Connect to Bridge

1. In Foxglove Studio, click **"Open connection"**
2. Select **"Foxglove WebSocket"**
3. Enter WebSocket URL: `ws://localhost:8765`
4. Click **"Open"**

**If connecting remotely** (bridge on different machine):
```
ws://<admin-machine-ip>:8765
# Example: ws://10.147.20.21:8765 (via ZeroTier)
```

### Step 4: Verify Topics

In Foxglove Studio:
1. **Topics panel** (left sidebar) should show robot topics:
   - `/barney/tf`
   - `/barney/odom`
   - `/barney/joint_states`
   - etc.

2. **Add visualization panels**:
   - Click **"+"** → **"3D Panel"** (for tf, robot model)
   - Click **"+"** → **"Plot"** (for odometry, sensor data)
   - Click **"+"** → **"Image"** (for camera feeds)

### Step 5: Create Custom Layout

Drag and arrange panels as desired, then:
1. Click **"Layouts"** (top right)
2. Click **"Save layout"**
3. Name it (e.g., "Barney Monitoring")
4. Export layout file for version control (optional)

---

## Configuration

### Environment Variables

All environment variables support customization:

| Variable | Default | Purpose |
|----------|---------|---------|
| `ROS_DOMAIN_ID` | `0` | ROS 2 DDS domain (must match robots) |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | DDS middleware |
| `FOXGLOVE_PORT` | `8765` | WebSocket port |
| `ROS_DISTRO` | `jazzy` | ROS 2 distro (jazzy, or humble for legacy) |

**Override example:**

```bash
# Use different port
FOXGLOVE_PORT=9000 docker compose -f compose/foxglove/bridge.yaml up

# Use legacy Humble instead of Jazzy (default)
ROS_DISTRO=humble docker compose -f compose/foxglove/bridge.yaml up

# Use different ROS domain
ROS_DOMAIN_ID=42 docker compose -f compose/foxglove/bridge.yaml up
```

### Custom Message Types

If you have custom ROS 2 messages in your workspace:

1. **Build workspace first** (bridge needs compiled message definitions):
   ```bash
   docker compose -f compose/viz/bash.yaml run --rm bash
   # Inside container:
   cd ~/dev_ws
   colcon build --symlink-install
   exit
   ```

2. **Bridge auto-sources workspace** via volume mount:
   - `~/ros2/jazzy/dev_ws` → `/home/dev/dev_ws` (read-only)
   - `~/ros2/jazzy/sim_ws` → `/home/dev/sim_ws` (read-only)

3. **Restart bridge** to pick up new message types:
   ```bash
   docker compose -f compose/foxglove/bridge.yaml restart
   ```

---

## Performance Tuning

### Reduce VPN Bandwidth Usage

**Symptom**: High VPN bandwidth, slow visualization, lag in Foxglove

**Solutions:**

#### 1. Selective Panel Usage (Easiest)

Foxglove only subscribes to topics visible in active panels:
- **Remove unused panels** - Close panels you're not viewing
- **Minimize camera panels** - Only open when debugging vision
- **Use Plot instead of Raw Messages** - More efficient for numeric data

#### 2. Topic Whitelisting (Advanced)

Uncomment and customize in `compose/foxglove/bridge.yaml`:

```yaml
environment:
  # Only subscribe to specific topics
  - ROS_TOPIC_WHITELIST=/barney/tf,/barney/tf_static,/barney/odom,/barney/joint_states
```

**Whitelist examples:**

```bash
# Navigation monitoring only
ROS_TOPIC_WHITELIST=/barney/tf,/barney/tf_static,/barney/odom,/barney/nav/plan,/barney/nav/local_costmap

# Add camera when needed
ROS_TOPIC_WHITELIST=/barney/tf,/barney/odom,/barney/camera/color/image_raw

# Multi-robot (specific topics from multiple robots)
ROS_TOPIC_WHITELIST=/barney/odom,/betty/odom,/bambam/odom
```

**After editing**, restart bridge:
```bash
docker compose -f compose/foxglove/bridge.yaml restart
```

#### 3. Image Compression (For Camera Topics)

Use compressed image topics instead of raw:
- Subscribe to `/barney/camera/color/image_raw/compressed` (JPEG)
- Instead of `/barney/camera/color/image_raw` (uncompressed)
- ~10x bandwidth reduction

Ensure robot publishes compressed topics:
```bash
# On robot or in launch file
ros2 run image_transport republish raw compressed --ros-args -r in:=/barney/camera/color/image_raw -r out/compressed:=/barney/camera/color/image_raw/compressed
```

---

## Multi-Robot Scenarios

### Viewing All Robots

Bridge discovers all topics from all robots by default:
- `/barney/odom`
- `/betty/odom`
- `/bambam/odom`

**In Foxglove Studio:**
- All robot topics visible in Topics panel
- Add panels for each robot's data
- Create multi-robot layouts

### Filtering by Robot Namespace

**Option 1: Filter in Foxglove UI** (Recommended)

1. In Topics panel, use search box: `/barney/`
2. Only shows Barney's topics
3. Add panels using filtered topics

**Option 2: Topic Whitelist** (Advanced)

Edit `compose/foxglove/bridge.yaml`:

```yaml
environment:
  # Only Barney topics
  - ROS_TOPIC_WHITELIST=/barney/*
```

**Note**: Wildcard support depends on Foxglove Bridge version. Explicit list may be required:
```yaml
  - ROS_TOPIC_WHITELIST=/barney/tf,/barney/tf_static,/barney/odom,/barney/joint_states
```

---

## Running in Background

For persistent monitoring, run bridge as background service:

```bash
# Start in detached mode
docker compose -f compose/foxglove/bridge.yaml up -d

# Check logs
docker compose -f compose/foxglove/bridge.yaml logs -f

# Stop bridge
docker compose -f compose/foxglove/bridge.yaml down
```

Bridge will auto-restart if it crashes (configured with `restart: unless-stopped`).

---

## Troubleshooting

### Bridge Starts But No Topics Visible

**Symptom**: Foxglove connects to WebSocket, but Topics panel is empty

**Solutions:**

1. **Check ROS_DOMAIN_ID matches robots**:
   ```bash
   # On WSL2 host
   echo $ROS_DOMAIN_ID

   # Inside bridge container
   docker exec -it foxglove_bridge_jazzy bash -c 'echo $ROS_DOMAIN_ID'

   # On robot (SSH to Barney)
   echo $ROS_DOMAIN_ID
   ```

   All must match (typically `0`).

2. **Check DDS discovery**:
   ```bash
   # Inside bridge container
   docker exec -it foxglove_bridge_jazzy bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic list
   # Should see robot topics
   ```

   If `ros2 topic list` is empty, DDS discovery is broken. See [wsl2-visualization.md - ROS 2 Topics Not Visible](wsl2-visualization.md#ros-2-topics-not-visible).

3. **Check VPN connectivity**:
   ```bash
   # From WSL2
   ping 10.147.20.1  # Barney's ZeroTier IP
   # Should get responses
   ```

4. **Check FastRTPS profile** (if using unicast):
   ```bash
   # Inside bridge container
   docker exec -it foxglove_bridge_jazzy bash -c 'echo $FASTRTPS_DEFAULT_PROFILES_FILE'
   # Should be empty (multicast) or /dds_config/fastrtps_unicast.xml
   ```

   **If using unicast**, verify peer list includes all robots in `config/dds/fastrtps_unicast.xml`.

### WebSocket Connection Refused

**Symptom**: Foxglove Studio shows "Connection refused" or "Failed to connect"

**Solutions:**

1. **Check bridge is running**:
   ```bash
   docker ps | grep foxglove_bridge
   # Should show container running
   ```

2. **Check logs for errors**:
   ```bash
   docker compose -f compose/foxglove/bridge.yaml logs
   ```

3. **Check port is listening**:
   ```bash
   # From WSL2
   ss -tlnp | grep 8765
   # Should show docker-proxy listening on 8765
   ```

4. **Check Windows Firewall** (if connecting from Windows browser):
   ```powershell
   # From PowerShell
   Test-NetConnection -ComputerName localhost -Port 8765
   # Should show TcpTestSucceeded: True
   ```

   If blocked, allow port 8765 inbound.

### Bridge Container Exits Immediately

**Symptom**: `docker compose up` exits after a few seconds

**Check logs**:
```bash
docker compose -f compose/foxglove/bridge.yaml logs
```

**Common issues:**

1. **ROS 2 distro mismatch**:
   - Bridge image expects Jazzy or legacy Humble
   - Check: `docker exec -it foxglove_bridge_jazzy bash -c 'echo $ROS_DISTRO'`

2. **Custom message build failure**:
   - If workspace has build errors, bridge may fail to source it
   - Fix: Rebuild workspace inside bash container first

3. **Port already in use**:
   ```bash
   # Check what's using port 8765
   ss -tlnp | grep 8765
   # Kill process or use different port
   FOXGLOVE_PORT=9000 docker compose -f compose/foxglove/bridge.yaml up
   ```

### Slow Performance / High Latency

**Symptom**: Foxglove UI is laggy, data updates slowly

**Solutions:**

1. **Check VPN latency**:
   ```bash
   ping 10.147.20.1  # Barney's ZeroTier IP
   # Latency should be <50ms for good performance
   ```

2. **Reduce topic subscription** - Close unused panels in Foxglove

3. **Use topic whitelist** - See [Performance Tuning](#performance-tuning)

4. **Check CPU usage**:
   ```bash
   docker stats foxglove_bridge_jazzy
   # CPU should be <20% for typical workload
   ```

   High CPU may indicate too many high-rate topics subscribed.

5. **Use compressed images** - See [Image Compression](#3-image-compression-for-camera-topics)

---

## Advanced: Unicast Mode for Cross-VPN

By default, bridge uses **multicast DDS** for maximum topic discovery. For cross-VPN scenarios with strict NAT/firewall, use **unicast mode**:

1. **Edit `compose/foxglove/bridge.yaml`**, uncomment:
   ```yaml
   environment:
     - FASTRTPS_DEFAULT_PROFILES_FILE=/dds_config/fastrtps_unicast.xml
   ```

2. **Update peer list** in `config/dds/fastrtps_unicast.xml`:
   ```xml
   <simple>
       <metatrafficUnicastLocatorList>
           <locator>
               <udpv4>
                   <address>10.147.20.1</address>  <!-- Barney -->
               </udpv4>
           </locator>
           <locator>
               <udpv4>
                   <address>10.147.20.2</address>  <!-- Betty -->
               </udpv4>
           </locator>
           <!-- Add admin machine IP if needed -->
       </metatrafficUnicastLocatorList>
   </simple>
   ```

3. **Restart bridge**:
   ```bash
   docker compose -f compose/foxglove/bridge.yaml restart
   ```

See [wsl2-visualization.md](wsl2-visualization.md) for detailed DDS configuration.

---

## Security Considerations

### WebSocket Authentication

**Default**: Foxglove Bridge has **no authentication** - anyone with network access can connect.

**For production deployments**:
1. **Firewall port 8765** - Only allow trusted IPs
2. **Use VPN-only access** - Don't expose to public internet
3. **Consider reverse proxy** - Add authentication layer (nginx with basic auth)

**Example: nginx reverse proxy with authentication:**

```nginx
# /etc/nginx/sites-available/foxglove
server {
    listen 8766 ssl;
    server_name admin-machine.robodojo.net;

    ssl_certificate /etc/letsencrypt/live/admin-machine.robodojo.net/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/admin-machine.robodojo.net/privkey.pem;

    auth_basic "Foxglove Bridge";
    auth_basic_user_file /etc/nginx/.htpasswd;

    location / {
        proxy_pass http://localhost:8765;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
}
```

Then connect to `wss://admin-machine.robodojo.net:8766` (requires password).

---

## Alternative: Foxglove Bridge on Robot (Option B)

If you decide to deploy bridge on Barney (native install):

### Installation on Robot

```bash
# SSH to Barney
ssh barney.robodojo.net

# Install Foxglove Bridge via apt (if available for ROS 2 Jazzy)
sudo apt update
sudo apt install ros-jazzy-foxglove-bridge

# Or install via pip
pip3 install foxglove-bridge
```

### Systemd Service

Create `/etc/systemd/system/foxglove-bridge.service`:

```ini
[Unit]
Description=Foxglove Bridge
After=network.target

[Service]
Type=simple
User=barney
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && foxglove-bridge --port 8765"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Enable and start**:
```bash
sudo systemctl daemon-reload
sudo systemctl enable foxglove-bridge
sudo systemctl start foxglove-bridge
```

**Connect from admin machine**:
```
ws://barney.robodojo.net:8765
# Or via ZeroTier IP: ws://10.147.20.1:8765
```

---

## Related Documentation

- **[getting-started-wsl2.md](getting-started-wsl2.md)** - Initial WSL2 and Docker setup
- **[wsl2-visualization.md](wsl2-visualization.md)** - RViz/RQT setup, DDS troubleshooting
- **[dds-cross-nat-troubleshooting.md](dds-cross-nat-troubleshooting.md)** - Detailed DDS configuration
- **[native-docker-wsl2-setup.md](native-docker-wsl2-setup.md)** - Docker CE installation
- **[ROADMAP.md](ROADMAP.md)** - Implementation roadmap

---

## External Resources

- **Foxglove Documentation**: https://docs.foxglove.dev/
- **Foxglove Bridge (ROS 2)**: https://github.com/foxglove/ros-foxglove-bridge
- **Foxglove Studio**: https://app.foxglove.dev (web version)
- **Download Desktop App**: https://foxglove.dev/download

---

## Next Steps

- **Create custom layouts** for different monitoring scenarios (navigation, manipulation, telemetry)
- **Export layouts** to version control for team sharing
- **Integrate with data logging** (MCAP format for post-mission analysis)
- **Explore Foxglove extensions** for custom visualizations
- **Phase 3**: Headless web tools (Vizanti, PlotJuggler Web) - see [ROADMAP.md](ROADMAP.md)
