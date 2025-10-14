# DDS Cross-NAT Discovery - RESOLVED

## Problem Statement
Successfully enabled ROS 2 DDS topic discovery between:
- **Barney** (robot at 10.147.20.30 on ZeroTier VPN)
- **Container** (Native Docker on Windows 11 WSL2 with mirrored networking)

## Solution Summary

The issue required **three components** to work together:

1. ✅ **WSL2 Mirrored Networking** - Allows WSL2 to see ZeroTier network directly
2. ✅ **Native Docker in WSL2** - Provides true `network_mode: host` (Docker Desktop's host mode is limited)
3. ✅ **Windows Firewall Rule** - Allows inbound UDP on DDS discovery ports (7400-7500)

## Network Topology (Final Working Configuration)

```
Barney (10.147.20.30 - ZeroTier VPN)
  ↕ VPN (UDP discovery packets on ports 7400/7410/7412)
Windows Host "kvlapblack" (10.147.20.21 - ZeroTier VPN interface)
  ↕ Mirrored Networking (WSL2 shares Windows network stack)
WSL2 VM (sees 10.147.20.21 directly via eth0)
  ↕ Native Docker with network_mode: host
Container (shares WSL2's network stack, sees 10.147.20.21)
```

**Key difference from before**: With mirrored networking + native Docker, there is NO NAT between container and ZeroTier network.

## Step-by-Step Solution

### 1. Enable WSL2 Mirrored Networking

Create/edit `C:\Users\<username>\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

Restart WSL2:
```powershell
# From Windows PowerShell
wsl --shutdown
# Then restart WSL2 terminal
```

Verify ZeroTier network is visible in WSL2:
```bash
# In WSL2
ip addr show
# Should show eth0 with 10.147.20.21
```

### 2. Switch from Docker Desktop to Native Docker

**Uninstall Docker Desktop** (or shut it down):
```powershell
# From Windows
taskkill /IM "Docker Desktop.exe" /F
```

**Install native Docker in WSL2**:
```bash
# In WSL2
sudo apt-get update
sudo apt-get install -y docker.io docker-compose

# Add user to docker group
sudo usermod -aG docker $USER

# Start Docker service
sudo service docker start

# Verify
docker ps
```

**Note**: Docker Desktop's `network_mode: host` only refers to its internal VM, not the actual Windows/WSL2 network. Native Docker provides true host networking.

### 3. Update FastRTPS Configuration

Edit `config/dds/fastrtps_unicast.xml` to match actual container interfaces:

```xml
<interfaceWhiteList>
  <!-- Only include interfaces that actually exist with native Docker + mirrored networking -->
  <address>127.0.0.1</address>
  <address>10.147.20.21</address>     <!-- ZeroTier VPN (kvlapblack) -->
  <address>192.168.43.91</address>    <!-- Mobile hotspot or your network -->
  <address>172.17.0.1</address>       <!-- docker0 bridge -->
</interfaceWhiteList>

<initialPeersList>
  <locator>
    <udpv4>
      <address>127.0.0.1</address>
    </udpv4>
  </locator>
  <locator>
    <udpv4>
      <address>10.147.20.21</address>  <!-- kvlapblack -->
    </udpv4>
  </locator>
  <locator>
    <udpv4>
      <address>10.147.20.30</address>  <!-- barney -->
    </udpv4>
  </locator>
  <!-- Remove unreachable peers to avoid timeout lag -->
</initialPeersList>
```

**Key changes**:
- `interfaceWhiteList` now includes `10.147.20.21` (ZeroTier IP visible in container)
- Removed Docker Desktop IPs (`192.168.65.x`) that no longer exist
- Commented out unreachable peers (wilma) to avoid 3-5 second lag

### 4. Configure Windows Firewall

**The critical step**: Windows Firewall was blocking inbound UDP packets from barney.

**Create firewall rule** (from PowerShell as Administrator):

```powershell
# Allow inbound UDP for ROS 2 DDS discovery ports
New-NetFirewallRule -DisplayName "ROS 2 DDS Discovery" -Direction Inbound -Protocol UDP -LocalPort 7400-7500 -Action Allow -Profile Any
```

**Alternative - Allow all traffic to WSL2** (less secure but simpler):
```powershell
# Allow all inbound traffic through wsl.exe
New-NetFirewallRule -DisplayName "WSL2 Full Access" -Direction Inbound -Action Allow -Program "C:\Windows\System32\wsl.exe" -Profile Any
```

**Verify firewall rule**:
```powershell
Get-NetFirewallRule -DisplayName "ROS 2 DDS Discovery"
```

### 5. Test Discovery

```bash
# Start container with native Docker
docker-compose -f compose/viz/bash.yaml run --rm bash

# In container, verify ZeroTier network is visible
ip addr show | grep 10.147.20.21

# Start ROS 2 daemon (important!)
ros2 daemon start

# Test discovery
ros2 topic list

# Should see barney's topics, e.g., /chatter
ros2 topic echo /chatter
```

## Debugging Process Summary

### Issue 1: Docker Desktop's Limited Host Networking
**Symptom**: Container with `network_mode: host` showed `192.168.65.x` IPs instead of `10.147.20.21`

**Root cause**: Docker Desktop uses its own VM, and "host" mode refers to that VM's network, not WSL2's.

**Solution**: Switch to native Docker in WSL2 for true host networking.

### Issue 2: WSL2 Couldn't See ZeroTier Network
**Symptom**: `ip addr show` in WSL2 showed `172.24.26.173` but not `10.147.20.21`

**Root cause**: WSL2 default NAT mode creates isolated network namespace.

**Solution**: Enable `networkingMode=mirrored` in `.wslconfig`.

### Issue 3: Invalid Interface IPs in FastRTPS Config
**Symptom**: `ss -ulpn | grep 74` showed no listening ports

**Root cause**: `interfaceWhiteList` contained non-existent Docker Desktop IPs (`192.168.65.x`), causing transport initialization to fail.

**Solution**: Update whitelist with actual container interfaces from `ip addr show`.

### Issue 4: Windows Firewall Blocking Return Path
**Symptom**:
- ✅ Container sending discovery packets to barney (confirmed with tcpdump)
- ✅ Barney receiving packets (confirmed with tcpdump on barney)
- ✅ Barney sending replies (confirmed with tcpdump on barney)
- ❌ Container not receiving replies

**Root cause**: Windows Firewall was blocking inbound UDP on ports 7400-7500.

**Solution**: Create firewall rule to allow inbound UDP on DDS ports.

**Debugging commands used**:
```bash
# On container - verify outbound packets
tcpdump -i any 'udp and dst host 10.147.20.30 and (port 7400 or port 7410 or port 7412)' -n -c 10

# On barney - verify packets arriving
sudo tcpdump -i any 'udp and src host 10.147.20.21 and (port 7400 or port 7410 or port 7412)' -n -c 10

# On barney - verify replies being sent
sudo tcpdump -i any 'udp and dst host 10.147.20.21 and (port 7400 or port 7410 or port 7412)' -n -c 10

# On container - verify replies NOT arriving (before firewall fix)
tcpdump -i any 'udp and src host 10.147.20.30' -n -c 10

# On WSL2 - verify replies not even reaching WSL2 (before firewall fix)
sudo tcpdump -i any 'udp and src host 10.147.20.30' -n -c 10
```

This methodical packet tracing isolated the firewall as the blocker.

### Issue 5: ROS 2 Daemon Connection Timeout
**Symptom**: `ros2 topic list` hung with `TimeoutError: [Errno 110] Connection timed out`

**Root cause**: ROS 2 daemon socket wasn't created (`/tmp/ros2_daemon_*` missing).

**Solution**: Run `ros2 daemon start` explicitly. With daemon running, all `ros2` CLI commands work normally.

## Performance Issue: Unreachable Peers

**Problem**: Including unreachable peers in `initialPeersList` causes severe lag:
- 3-5 second delays between messages
- 5+ minute delay to handle Ctrl+C signals
- Extremely slow startup times

**Root cause**: FastDDS waits for timeouts on unreachable peers in the discovery list.

**Solution**: Only include active/reachable peers. Comment out offline robots:

```xml
<!-- wilma - commented out to avoid timeout lag from unreachable peer
<locator>
  <udpv4>
    <address>10.147.20.33</address>
  </udpv4>
</locator>
-->
```

**Long-term implications**: Current FastDDS unicast discovery is fragile for dynamic networks. Consider:
- FastDDS Discovery Server (centralized discovery)
- CycloneDDS (better NAT handling)
- Timeout configuration tuning

## Verification

### Both Directions Working
- ✅ Container can see barney's `/chatter` topic
- ✅ Barney can see container's `/chatter` topic
- ✅ `ros2 topic echo /chatter` shows interleaved messages from both sources

### Commands to Verify
```bash
# In container
ros2 topic list
# Should show /chatter, /parameter_events, /rosout

ros2 topic echo /chatter
# Should show messages from barney's talker

ros2 run demo_nodes_cpp talker
# Barney should now see two streams of messages
```

## Configuration Files

### Container's FastRTPS Profile
**Location**: `config/dds/fastrtps_unicast.xml`

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <!-- Only include interfaces that actually exist with native Docker + mirrored networking -->
        <address>127.0.0.1</address>
        <address>10.147.20.21</address>     <!-- ZeroTier VPN (kvlapblack) -->
        <address>192.168.43.91</address>    <!-- Mobile hotspot -->
        <address>172.17.0.1</address>       <!-- docker0 bridge -->
      </interfaceWhiteList>
      <maxInitialPeersRange>100</maxInitialPeersRange>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="default_part_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>127.0.0.1</address>
            </udpv4>
          </locator>
          <locator>
            <udpv4>
              <address>10.147.20.21</address>  <!-- kvlapblack -->
            </udpv4>
          </locator>
          <locator>
            <udpv4>
              <address>10.147.20.30</address>  <!-- barney -->
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports>
        <transport_id>udp_transport</transport_id>
      </userTransports>
    </rtps>
  </participant>
</profiles>
```

### Barney's FastRTPS Profile
**Location**: `/ros2_ws/src/grunt/grunt_bringup/config/fastrtps_profiles.xml` (on barney)

```xml
<interfaceWhiteList>
  <address>127.0.0.1</address>
  <address>10.147.20.21</address>  <!-- kvlapblack -->
  <address>10.147.20.30</address>  <!-- barney -->
</interfaceWhiteList>

<initialPeersList>
  <locator>
    <udpv4>
      <address>127.0.0.1</address>
    </udpv4>
  </locator>
  <locator>
    <udpv4>
      <address>10.147.20.21</address>  <!-- kvlapblack -->
    </udpv4>
  </locator>
  <locator>
    <udpv4>
      <address>10.147.20.30</address>  <!-- barney -->
    </udpv4>
  </locator>
</initialPeersList>
```

## Key Learnings

### interfaceWhiteList Behavior
**Answer**: `interfaceWhiteList` controls **local interface binding only**, not remote peer filtering.

**Evidence**:
- Including non-existent IPs (like `10.147.20.30` in container that only has `10.147.20.21`) causes transport to fail
- Once only actual local interfaces were listed, discovery worked immediately
- Remote peers are specified separately in `initialPeersList`

**Best practice**: Only include interfaces that exist on the local machine according to `ip addr show`.

### Docker Desktop vs Native Docker

| Feature | Docker Desktop | Native Docker in WSL2 |
|---------|---------------|----------------------|
| `network_mode: host` | VM network only | True host network |
| ZeroTier visibility | ❌ No | ✅ Yes (with mirrored networking) |
| Performance | Extra VM overhead | Direct |
| Setup complexity | Easy installer | Manual apt install |
| Cross-platform | Windows/Mac/Linux | Linux only |

**For ROS 2 on WSL2**: Native Docker is superior for network transparency.

### Windows Firewall Impact
- By default, Windows Firewall blocks inbound UDP on non-standard ports
- This affects WSL2 even with mirrored networking
- Must create explicit rules for DDS ports (7400-7500) or allow all WSL2 traffic
- Disabling firewall entirely is not recommended for security

## Troubleshooting Commands

### Verify Setup
```bash
# Check ZeroTier visible in WSL2
ip addr show | grep 10.147.20

# Check ZeroTier visible in container (with native Docker + host networking)
docker-compose -f compose/viz/bash.yaml run --rm bash
ip addr show | grep 10.147.20

# Check DDS ports listening
ss -ulpn | grep 74

# Check firewall rule (PowerShell as Admin)
Get-NetFirewallRule -DisplayName "ROS 2 DDS Discovery"
```

### Debug Discovery
```bash
# Outbound packets from container to barney
tcpdump -i any 'udp and dst host 10.147.20.30 and (port 7400 or port 7410 or port 7412)' -n -c 10

# Inbound packets from barney to container
tcpdump -i any 'udp and src host 10.147.20.30 and (port 7400 or port 7410 or port 7412)' -n -c 10

# Test ROS 2 discovery
ros2 daemon start
ros2 topic list
ros2 topic echo /chatter
```

## References

- WSL2 Mirrored Networking: https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking
- FastDDS Interface Whitelist: https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/whitelist.html
- ROS 2 DDS Tuning: https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html
- Windows Firewall Rules: https://learn.microsoft.com/en-us/powershell/module/netsecurity/new-netfirewallrule

## Status: ✅ RESOLVED

Cross-NAT DDS discovery is now working reliably with:
- Native Docker in WSL2
- WSL2 mirrored networking
- Windows Firewall rules for DDS ports
- FastRTPS unicast discovery with properly configured interface whitelists
