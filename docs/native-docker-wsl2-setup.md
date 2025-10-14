# Native Docker in WSL2 Setup Guide

This guide explains how to set up native Docker in WSL2 for ROS 2 development, including WSL2 mirrored networking for ZeroTier VPN access.

## Why Native Docker Instead of Docker Desktop?

### Docker Desktop Limitations for ROS 2

Docker Desktop on Windows uses an internal VM, which causes issues for ROS 2 DDS discovery:

| Feature | Docker Desktop | Native Docker in WSL2 |
|---------|---------------|----------------------|
| `network_mode: host` | VM network only | True host network |
| ZeroTier/VPN visibility | ❌ No | ✅ Yes (with mirrored networking) |
| NAT layers | Windows → Docker VM → WSL2 → Container | Windows → WSL2 → Container |
| DDS discovery | Complicated | Straightforward |
| Performance | Extra VM overhead | Direct |

**For ROS 2 development**: Native Docker + WSL2 mirrored networking eliminates NAT complexity and provides direct access to VPN networks.

## Prerequisites

- Windows 11 (for mirrored networking support)
- WSL2 installed and updated
- ZeroTier (or other VPN) installed on Windows

## Installation Steps

### 1. Enable WSL2 Mirrored Networking

Mirrored networking allows WSL2 to share Windows' network stack directly, including VPN interfaces.

**Create/edit `.wslconfig`**:

```powershell
# From Windows PowerShell
notepad $env:USERPROFILE\.wslconfig
```

Add this content:

```ini
[wsl2]
networkingMode=mirrored
```

**Restart WSL2**:

```powershell
wsl --shutdown
```

Then reopen your WSL2 terminal.

**Verify mirrored networking**:

```bash
# In WSL2 - should show Windows' network interfaces including VPN
ip addr show

# Look for your VPN IP (e.g., 10.147.20.21 for ZeroTier)
ip addr show | grep "10.147.20"
```

If you see the VPN IP, mirrored networking is working!

### 2. Shut Down Docker Desktop (if installed)

**Option A: Temporary shutdown**

Right-click Docker Desktop icon in system tray → "Quit Docker Desktop"

**Option B: Permanent removal**

Uninstall Docker Desktop from Windows Settings → Apps

**Note**: Your Docker images stored in Docker Desktop will not be accessible to native Docker. You'll need to pull/build images again.

### 3. Install Native Docker in WSL2

```bash
# Update package list
sudo apt-get update

# Install Docker
sudo apt-get install -y docker.io docker-compose

# Add your user to docker group (avoid needing sudo)
sudo usermod -aG docker $USER

# Apply group membership (or log out/in)
newgrp docker

# Start Docker service
sudo service docker start

# Enable Docker to start on boot
sudo systemctl enable docker 2>/dev/null || echo "systemd not available, start manually"

# Verify installation
docker --version
docker ps
```

**For automatic Docker startup** (add to `.bashrc` or `.zshrc`):

```bash
# Auto-start Docker if not running
if ! pgrep -x dockerd > /dev/null; then
    sudo service docker start
fi
```

### 4. Pull/Build Your Images

Since native Docker has a separate image cache:

```bash
# Pull pre-built images
docker pull ghcr.io/pondersome/grunt_base:humble

# Or build from scratch
cd ~/grunt_docker/base
docker build -t ghcr.io/pondersome/grunt_base:humble .
```

### 5. Configure Windows Firewall

**Critical step**: Windows Firewall blocks inbound UDP by default, which prevents DDS discovery packets from reaching WSL2/containers.

**Option A: Allow ROS 2 DDS ports only** (recommended)

From PowerShell as Administrator:

```powershell
New-NetFirewallRule -DisplayName "ROS 2 DDS Discovery" `
    -Direction Inbound `
    -Protocol UDP `
    -LocalPort 7400-7500 `
    -Action Allow `
    -Profile Any
```

**Option B: Allow all traffic through WSL2** (less secure but simpler)

```powershell
New-NetFirewallRule -DisplayName "WSL2 Full Access" `
    -Direction Inbound `
    -Action Allow `
    -Program "C:\Windows\System32\wsl.exe" `
    -Profile Any
```

**Verify firewall rule**:

```powershell
Get-NetFirewallRule -DisplayName "ROS 2 DDS Discovery"
```

### 6. Test the Setup

**Verify network visibility**:

```bash
# Start a container with host networking
docker-compose -f compose/viz/bash.yaml run --rm bash

# Inside container - verify VPN is visible
ip addr show | grep "10.147.20"

# Should show something like:
#   inet 10.147.20.21/24 brd 10.147.20.255 scope global noprefixroute eth0
```

**Test DDS discovery**:

```bash
# Inside container
export FASTRTPS_DEFAULT_PROFILES_FILE=/dds_config/fastrtps_unicast.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# Start ROS 2 daemon
ros2 daemon start

# Test discovery
ros2 topic list

# Should see topics from other robots on the VPN
ros2 topic echo /chatter
```

## Configuration Updates

### Update FastRTPS Config

With native Docker + mirrored networking, container interfaces change. Update `config/dds/fastrtps_unicast.xml`:

```xml
<interfaceWhiteList>
  <!-- Include only interfaces that exist in the container -->
  <address>127.0.0.1</address>
  <address>10.147.20.21</address>     <!-- Your VPN IP -->
  <address>192.168.X.X</address>      <!-- Your WiFi/Ethernet IP -->
  <address>172.17.0.1</address>       <!-- docker0 bridge -->
</interfaceWhiteList>
```

**How to find current interfaces**:

```bash
# Inside container - use the detection script
bash /dds_config/detect_interfaces.sh

# Or manually check:
ip addr show

# Look for inet addresses, e.g.:
# inet 127.0.0.1/8 (localhost)
# inet 10.147.20.21/24 (VPN)
# inet 172.17.0.1/16 (docker bridge)
```

**Important notes**:
- Only include interfaces that actually exist on the container
- Don't include temporary network IPs (mobile hotspots, guest WiFi) in the committed config
- The VPN IP (10.147.20.x) and docker bridge (172.17.0.1) are typically stable
- See `config/dds/detect_interfaces.sh` for an automated detection helper

### Update Compose Files

No changes needed! Compose files already use `network_mode: host`, which now provides true host networking with native Docker.

## Troubleshooting

### WSL2 doesn't show VPN interface

**Problem**: `ip addr show` doesn't show your VPN IP (e.g., 10.147.20.x)

**Solutions**:

1. Verify `.wslconfig` has `networkingMode=mirrored`
2. Run `wsl --shutdown` and restart WSL2
3. Check VPN is connected on Windows: `ipconfig` in PowerShell should show VPN adapter
4. Some VPNs don't support mirrored mode - check VPN documentation

### Container doesn't see VPN interface

**Problem**: Inside container, `ip addr show` doesn't show VPN IP

**Solutions**:

1. Verify `network_mode: host` in compose file
2. Check you're using native Docker, not Docker Desktop: `docker info | grep "Operating System"`
   - Should show "Ubuntu" or similar Linux OS
   - Docker Desktop shows "Docker Desktop"
3. Try running without compose: `docker run --network host --rm -it ghcr.io/pondersome/grunt_base:humble bash`

### DDS discovery doesn't work

**Problem**: `ros2 topic list` doesn't show remote topics

**Debug steps**:

1. **Check if packets are being sent**:
   ```bash
   # Inside container
   tcpdump -i any 'udp and dst host 10.147.20.30 and port 7410' -n -c 5
   # Should see outbound packets
   ```

2. **Check if packets are being received**:
   ```bash
   # Inside container
   tcpdump -i any 'udp and src host 10.147.20.30 and port 7410' -n -c 5
   # Should see inbound packets
   ```

3. **If packets sent but not received**: Windows Firewall is likely blocking
   - Verify firewall rule exists: `Get-NetFirewallRule -DisplayName "ROS 2 DDS Discovery"` (PowerShell)
   - Temporarily disable firewall to test: `Set-NetFirewallProfile -Profile Domain,Public,Private -Enabled False`
   - Re-enable after test: `Set-NetFirewallProfile -Profile Domain,Public,Private -Enabled True`

4. **Check FastRTPS config**:
   ```bash
   # Verify profile is loaded
   echo $FASTRTPS_DEFAULT_PROFILES_FILE
   cat $FASTRTPS_DEFAULT_PROFILES_FILE

   # Verify interfaces in whitelist match actual interfaces
   ip addr show
   ```

5. **Check ROS 2 daemon**:
   ```bash
   # Stop and restart daemon
   ros2 daemon stop
   ros2 daemon start

   # Test without daemon
   ros2 topic list --no-daemon
   ```

### Docker service won't start

**Problem**: `sudo service docker start` fails

**Solutions**:

1. Check if another Docker daemon is running:
   ```bash
   ps aux | grep docker
   # Kill Docker Desktop's daemon if present
   ```

2. Check Docker logs:
   ```bash
   sudo journalctl -u docker.service -n 50
   ```

3. Try manual start:
   ```bash
   sudo dockerd
   # Look for error messages
   ```

### Permission denied errors

**Problem**: `docker ps` shows "permission denied"

**Solutions**:

1. Verify you're in docker group:
   ```bash
   groups | grep docker
   ```

2. If not, add yourself:
   ```bash
   sudo usermod -aG docker $USER
   newgrp docker
   ```

3. Or restart WSL2:
   ```bash
   exit  # Exit WSL2
   # From PowerShell:
   wsl --shutdown
   # Restart WSL2
   ```

### Images need to be re-pulled

**Problem**: `docker images` is empty after switching from Docker Desktop

**Explanation**: Native Docker has a separate image cache from Docker Desktop.

**Solution**: Pull or build images again:

```bash
# Pull from registry
docker pull ghcr.io/pondersome/grunt_base:humble

# Or build locally
cd ~/grunt_docker/base
docker build -t ghcr.io/pondersome/grunt_base:humble .
```

## Switching Back to Docker Desktop

If you need to switch back:

1. Stop native Docker:
   ```bash
   sudo service docker stop
   ```

2. Start Docker Desktop from Windows

3. Note: You'll lose access to native Docker's image cache and WSL2 mirrored networking benefits

## Performance Tips

### Auto-start Docker on WSL2 launch

Add to `~/.bashrc`:

```bash
# Auto-start Docker service if not running
if ! pgrep -x dockerd > /dev/null; then
    sudo service docker start > /dev/null 2>&1
fi
```

### Prevent systemd warnings

WSL2 doesn't use systemd by default, which causes warnings with `systemctl enable docker`. Ignore these warnings or configure WSL2 to use systemd:

Edit `/etc/wsl.conf`:

```ini
[boot]
systemd=true
```

Then restart WSL2: `wsl --shutdown`

### Optimize Docker Build Caching

Set Docker to use WSL2's native filesystem for better build performance:

```bash
# Store Docker data in WSL2 filesystem (not Windows mount)
# Default is already optimal: /var/lib/docker
sudo docker info | grep "Docker Root Dir"
```

## Comparison with Docker Desktop

### Advantages of Native Docker

- ✅ True host networking with `network_mode: host`
- ✅ Direct access to VPN interfaces (with mirrored networking)
- ✅ No extra VM overhead
- ✅ Better integration with WSL2 filesystem
- ✅ Free and open source

### Disadvantages of Native Docker

- ❌ Manual installation (no GUI installer)
- ❌ No graphical dashboard
- ❌ Need to manually start service
- ❌ Separate image cache (can't share with Docker Desktop)
- ❌ WSL2-only (Docker Desktop works on Windows directly)

### When to Use Each

**Use Native Docker for**:
- ROS 2 development with DDS discovery across networks
- VPN/ZeroTier access from containers
- Maximum performance
- Linux development workflows

**Use Docker Desktop for**:
- Cross-platform development (Windows/Mac/Linux)
- Need GUI dashboard
- Simple installation
- Non-networking-intensive workloads

## References

- [WSL2 Networking Modes](https://learn.microsoft.com/en-us/windows/wsl/networking)
- [Docker Installation on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
- [Windows Firewall Configuration](https://learn.microsoft.com/en-us/powershell/module/netsecurity/new-netfirewallrule)
- [ROS 2 DDS Configuration](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)

## Additional Resources

- See `docs/dds-cross-nat-troubleshooting.md` for detailed DDS debugging
- See `docs/wsl2-visualization.md` for WSLg GUI setup
- See `config/dds/fastrtps_unicast.xml` for FastRTPS configuration example
