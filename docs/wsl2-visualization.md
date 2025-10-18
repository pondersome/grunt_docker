# WSL2 Visualization with WSLg

This guide covers running ROS 2 GUI tools (RViz2, RQT) in Docker containers on Windows 11 WSL2 using WSLg (Wayland/X11 passthrough).

---

## Prerequisites

### Windows Requirements
- **Windows 11** (WSLg is built-in on recent builds)
- **WSL2** installed and updated (`wsl --update`)
- **WSLg enabled** (default on Windows 11)
- **Native Docker in WSL2** (NOT Docker Desktop - see [setup guide](native-docker-wsl2-setup.md))
- **ZeroTier VPN** (or similar) for robot network access - see [ZeroTier Setup Options](#zerotier-setup-options)

### Important Setup Steps

**Before using these compose files**, you MUST:

1. **Enable WSL2 mirrored networking** - See [native-docker-wsl2-setup.md](native-docker-wsl2-setup.md#1-enable-wsl2-mirrored-networking)
2. **Install native Docker** - See [native-docker-wsl2-setup.md](native-docker-wsl2-setup.md#3-install-native-docker-in-wsl2)
3. **Configure Windows Firewall** - See [native-docker-wsl2-setup.md](native-docker-wsl2-setup.md#5-configure-windows-firewall)

**Why native Docker?** Docker Desktop's `network_mode: host` only refers to its internal VM, not the actual WSL2/Windows network. This breaks DDS discovery across VPN networks.

### Verify WSLg is Working

From WSL2 Ubuntu terminal:

```bash
# Check Wayland socket exists
ls -la /mnt/wslg/
# Should show: wayland-0, runtime-dir/, PulseServer, etc.

# Check environment variables
echo $WAYLAND_DISPLAY   # Should show: wayland-0
echo $XDG_RUNTIME_DIR   # Should show: /mnt/wslg/runtime-dir or /run/user/1000
echo $PULSE_SERVER      # Should show: /mnt/wslg/PulseServer
```

If these are empty or `/mnt/wslg/` doesn't exist, see [Troubleshooting](#troubleshooting-wslg-not-available).

---

## ZeroTier Setup Options

For cross-network robot communication, ZeroTier (or similar VPN) is required. There are **two valid approaches**:

### Option A: Mirrored Networking (Recommended if it works)

**How it works:**
- ZeroTier installed on **Windows host**
- WSL2 **mirrored networking** enabled in `.wslconfig`
- WSL2 VM sees Windows network interfaces directly
- Containers inherit network via `network_mode: host`

**Setup:**
1. Install ZeroTier on Windows
2. Join your ZeroTier network
3. Enable mirrored networking - see [native-docker-wsl2-setup.md](native-docker-wsl2-setup.md#1-enable-wsl2-mirrored-networking)
4. Configure Windows Firewall for DDS ports - see [native-docker-wsl2-setup.md](native-docker-wsl2-setup.md#5-configure-windows-firewall)

**Verification:**
```bash
# From WSL2
ip addr | grep zt
# Should show ZeroTier interface from Windows
```

### Option B: ZeroTier in WSL2 (Alternative if Option A fails)

**How it works:**
- ZeroTier installed **directly in WSL2 VM**
- WSL2 gets its own ZeroTier IP (separate from Windows)
- No mirrored networking required
- Containers inherit network via `network_mode: host`

**Setup:**
1. Install ZeroTier in WSL2:
   ```bash
   curl -s https://install.zerotier.com | sudo bash
   sudo zerotier-cli join <your-network-id>
   ```

2. Authorize the WSL2 node on ZeroTier Central

3. Verify:
   ```bash
   sudo zerotier-cli listnetworks
   ifconfig | grep zt
   ```

4. **Optional**: Give your WSL2 node a DNS name (e.g., `halbuntu.robodojo.net`)

**Advantages:**
- Works when mirrored networking doesn't route traffic
- WSL2 VM has independent VPN identity
- No Windows Firewall configuration needed (VPN traffic bypasses it)

**Disadvantages:**
- ZeroTier needs to be managed in each WSL2 distribution
- Separate IP from Windows host

### Which Option to Choose?

**Try Option A first** (mirrored networking) as it's simpler and uses a single VPN connection.

**Fall back to Option B** if:
- Mirrored networking fails to route traffic to internet
- You want independent VPN identity for WSL2
- You prefer managing network at WSL2 level

Both options work with the same compose files - containers use `network_mode: host` in both cases.

---

## Quick Start

### Run RViz2

From the `grunt_docker` repository root:

```bash
docker compose -f compose/viz/rviz.yaml up
```

RViz2 should launch in a native Windows window via WSLg.

**First time?** Docker will pull `ghcr.io/pondersome/grunt:humble` (multi-arch, ~1.2GB compressed).

### Run RQT

```bash
docker compose -f compose/viz/rqt.yaml up
```

RQT GUI should launch with plugin selection dialog.

### Stop Services

```bash
# Ctrl+C in the terminal, then:
docker compose -f compose/viz/rviz.yaml down
docker compose -f compose/viz/rqt.yaml down
```

---

## Configuration

### Environment Variables

All compose files use these environment variables (with defaults):

| Variable | Default | Purpose |
|----------|---------|---------|
| `ROS_DOMAIN_ID` | `0` | ROS 2 DDS domain (change for multi-robot isolation) |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | DDS middleware (can use `rmw_cyclonedds_cpp`) |
| `WAYLAND_DISPLAY` | `wayland-0` | Wayland socket name (WSLg default) |
| `XDG_RUNTIME_DIR` | `/mnt/wslg/runtime-dir` | Runtime directory for sockets |
| `PULSE_SERVER` | `/mnt/wslg/PulseServer` | PulseAudio socket for audio |

**Override example**:

```bash
# Use different ROS domain
ROS_DOMAIN_ID=42 docker compose -f compose/viz/rviz.yaml up

# Use CycloneDDS instead of FastRTPS
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp docker compose -f compose/viz/rviz.yaml up
```

### Custom RViz Configuration

Mount a custom `.rviz` file:

```yaml
# Edit compose/viz/rviz.yaml, uncomment:
volumes:
  - ./config/rviz/default.rviz:/root/.rviz2/default.rviz:ro
```

Create `config/rviz/default.rviz` in the repo, then restart.

### Custom RQT Perspective

Mount a custom perspective:

```yaml
# Edit compose/viz/rqt.yaml, uncomment:
volumes:
  - ./config/rqt/default.perspective:/root/.config/ros.org/rqt_gui.ini:ro
```

---

## How It Works

### WSLg Architecture

WSLg (Windows Subsystem for Linux GUI) enables Linux GUI apps in WSL2:

1. **Wayland compositor** runs in a WSL2 system VM
2. **Socket at `/mnt/wslg/wayland-0`** connects containers to compositor
3. **Windows renders** the GUI natively via Weston/RDP bridge
4. **PulseAudio** provides audio passthrough

### Docker Configuration

Key parts of `compose/viz/rviz.yaml`:

```yaml
environment:
  - WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
  - XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}

volumes:
  - /mnt/wslg:/mnt/wslg:rw

network_mode: host  # Required for ROS 2 DDS discovery
```

- **`/mnt/wslg` mount**: Exposes Wayland/X11 sockets to container
- **`network_mode: host`**: Container shares host network stack for ROS 2 topic discovery
- **Environment passthrough**: Container uses same display as WSL2 host

---

## Testing with ROS 2 Topics

### Publish Test Data for RViz

In a separate terminal, publish a test transform:

```bash
# From WSL2 (host or another container)
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

In RViz2:
1. Set **Fixed Frame** to `map`
2. **Add** → **TF** → Should see `map` → `base_link` transform

### Publish Test Data for RQT

```bash
# Publish a test topic
ros2 topic pub /test_topic std_msgs/msg/String "data: 'Hello from WSL2'" --rate 1
```

In RQT:
1. **Plugins** → **Topics** → **Topic Monitor**
2. Should see `/test_topic` with string messages

---

## Troubleshooting

### WSLg Not Available

**Symptom**: `/mnt/wslg/` directory doesn't exist or is empty

**Solutions**:

1. **Update WSL**:
   ```powershell
   # In PowerShell (Windows host)
   wsl --update
   wsl --shutdown
   # Restart WSL2
   ```

2. **Check Windows version**:
   - WSLg requires Windows 11 or Windows 10 Build 19044+ (21H2)
   - Check: `Settings → System → About`

3. **Reinstall WSLg**:
   ```powershell
   # PowerShell as Administrator
   wsl --update --web-download
   ```

### GUI Apps Slow to Launch or Hang

**Symptom**: GUI applications take 20+ seconds to open window, generate errors, or hang

**Solution**: Update WSL2 to latest version

```powershell
# In PowerShell (Windows host)
wsl --update
wsl --shutdown
```

Then restart your WSL2 distribution. This fixes known WSLg rendering issues in older WSL2 versions.

**Verification**:
```bash
# From WSL2
wsl.exe --version
# Should show recent version (0.51.0+)
```

### X11 Fallback (If Wayland Fails)

If Wayland doesn't work, use X11:

1. **Edit compose file** (e.g., `compose/viz/rviz.yaml`):
   ```yaml
   environment:
     - DISPLAY=${DISPLAY}  # Uncomment this
     # - WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}  # Comment out Wayland

   volumes:
     - /tmp/.X11-unix:/tmp/.X11-unix:ro  # Uncomment this
   ```

2. **Restart container**:
   ```bash
   docker compose -f compose/viz/rviz.yaml down
   docker compose -f compose/viz/rviz.yaml up
   ```

### Container Exits Immediately

**Check logs**:

```bash
docker compose -f compose/viz/rviz.yaml logs
```

**Common issues**:

- **Missing ROS topics**: RViz/RQT may exit if no ROS 2 system is running
  - Solution: Set `restart: unless-stopped` (already in compose files)

- **Display errors**: Check `$WAYLAND_DISPLAY` is set in WSL2 host

### GUI Doesn't Appear

1. **Check container is running**:
   ```bash
   docker ps | grep grunt_rviz
   ```

2. **Check Windows Firewall** isn't blocking WSLg

3. **Test native WSL GUI** first:
   ```bash
   # From WSL2, test a simple GUI app
   sudo apt install x11-apps
   xcalc  # Should open calculator in Windows
   ```

   If `xcalc` doesn't work, WSLg itself is broken (not a Docker issue).

### ROS 2 Topics Not Visible

**Symptom**: RViz/RQT launches but doesn't see topics from host or other containers/robots

**Common causes**:
1. Docker Desktop being used instead of native Docker
2. WSL2 mirrored networking not enabled
3. Windows Firewall blocking DDS ports
4. ROS 2 daemon not started in container

**Solutions**:

1. **Verify native Docker** (NOT Docker Desktop):
   ```bash
   docker info | grep "Operating System"
   # Should show: "Ubuntu" or similar Linux OS
   # Docker Desktop shows: "Docker Desktop"
   ```

   If using Docker Desktop, follow [native-docker-wsl2-setup.md](native-docker-wsl2-setup.md) to switch.

2. **Check VPN visibility**:
   ```bash
   # Inside container
   docker exec -it grunt_rviz bash -c 'ip addr show | grep 10.147.20'
   # Should show your VPN IP (e.g., 10.147.20.21)
   ```

   If not visible, WSL2 mirrored networking is not enabled.

3. **Check ROS 2 daemon**:
   ```bash
   # ROS 2 daemon may not auto-start
   docker exec -it grunt_rviz bash
   ros2 daemon start
   ros2 topic list
   ```

4. **Check ROS_DOMAIN_ID matches**:
   ```bash
   # In WSL2 host
   echo $ROS_DOMAIN_ID

   # In container
   docker exec -it grunt_rviz bash -c 'echo $ROS_DOMAIN_ID'
   ```

5. **Test DDS connectivity**:
   ```bash
   # In container
   docker exec -it grunt_rviz bash
   source /ros2_ws/install/setup.bash
   ros2 daemon start
   ros2 topic list  # Should see topics from remote robots
   ```

6. **Check Windows Firewall**:
   ```powershell
   # From PowerShell
   Get-NetFirewallRule -DisplayName "ROS 2 DDS Discovery"
   ```

   If rule doesn't exist, see [native-docker-wsl2-setup.md](native-docker-wsl2-setup.md#5-configure-windows-firewall).

7. **Debug with bash.yaml**:
   ```bash
   docker compose -f compose/viz/bash.yaml run --rm bash
   # Inside container:
   ros2 daemon start
   bash /dds_config/diagnose_dds.sh
   ```

   See [dds-cross-nat-troubleshooting.md](dds-cross-nat-troubleshooting.md) for detailed debugging.

### Permission Denied on /mnt/wslg

**Symptom**: Container can't access `/mnt/wslg` sockets

**Solution**: Run with user permissions fix (advanced):

```yaml
# Edit compose file
user: "${UID}:${GID}"
volumes:
  - /mnt/wslg:/mnt/wslg:rw
  - /run/user/1000:/run/user/1000:rw  # Add host user runtime dir
```

---

## Advanced: Running Both RViz and RQT Together

Create `compose/viz/viz-stack.yaml`:

```yaml
services:
  rviz:
    extends:
      file: rviz.yaml
      service: rviz

  rqt:
    extends:
      file: rqt.yaml
      service: rqt
```

Run together:

```bash
docker compose -f compose/viz/viz-stack.yaml up
```

---

## Performance Tips

### Reduce Startup Time

**Pre-pull images**:

```bash
docker pull ghcr.io/pondersome/grunt:humble
```

### Monitor Resource Usage

```bash
# Check memory/CPU usage
docker stats grunt_rviz grunt_rqt

# Check image size
docker images ghcr.io/pondersome/grunt
```

---

## Alternative: Native ROS 2 Install on WSL2

If Docker overhead is too high, consider native ROS 2 Humble install on WSL2:

**Pros**:
- No containerization overhead
- Direct hardware access
- Simpler GUI setup

**Cons**:
- Pollutes WSL2 environment
- Harder to manage dependencies
- Not portable to other robots

See [ROS 2 Humble Ubuntu install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

---

## Related Documentation

- [GHCR Setup](ghcr-setup.md) - Pulling multi-arch images
- [ROADMAP](ROADMAP.md) - Implementation progress
- [PRD Section 7.1](../specs/grunt_docker_prd_v_0.md#71-wslgwayland-primary) - WSLg strategy
- [PRD Section 18A](../specs/grunt_docker_prd_v_0.md#18a-wsl2-gui-visualization) - Example compose files

---

## Next Steps

- **Phase 2**: Web-based visualization (Foxglove, Vizanti) - see [ROADMAP](ROADMAP.md)
- **Phase 3**: Environment configuration and DDS tuning
- **Test on robot hardware**: Betty (Jetson), BamBam (RPi 5)
