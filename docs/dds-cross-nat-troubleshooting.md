# DDS Cross-NAT Discovery Troubleshooting

## Problem Statement
Attempting to enable ROS 2 DDS topic discovery between:
- **Barney** (robot at 10.147.20.30 on ZeroTier VPN)
- **Container** (Docker on Windows 11 WSL2, NAT'd through kvlapblack at 10.147.20.21)

## Network Topology
```
Barney (10.147.20.30 - ZeroTier VPN)
  ↕ VPN
Windows Host "kvlapblack" (10.147.20.21 - ZeroTier VPN interface)
  ↕ Hyper-V NAT
WSL2 VM (172.24.26.173)
  ↕ Docker NAT
Container (192.168.65.3, 192.168.65.6, 172.17.0.1)
```

**Key issue**: Container has NO 10.147.20.x interface - all VPN traffic is NAT'd through multiple layers.

## Current Status

### ✅ Working
1. **IP Connectivity**: Container can ping barney (10.147.20.30) - routing works
2. **Barney local discovery**: Works after reducing peer list (see below)
3. **Container DDS ports**: Listening on 7400, 7410, 7412 across all interfaces
4. **GUI display**: RViz/RQT windows render correctly via WSLg

### ❌ Not Working
1. **Cross-network discovery**: Container cannot see barney's `/chatter` topic
2. **Verbose logging**: `FASTDDS_VERBOSITY=info` produces no output (unclear why)

### ⚠️ Performance Issue Identified
**Problem**: Including unreachable peers in `initialPeersList` causes severe lag:
- 3-5 second delays between messages
- 5+ minute delay to handle Ctrl+C signals
- Extremely slow startup times

**Root cause**: FastDDS waits for timeouts on unreachable peers in the discovery list.

**Solution**: Only include active/reachable peers in `initialPeersList`.

## Configuration Files

### Barney's Config
**Location**: `/ros2_ws/src/grunt/grunt_bringup/config/fastrtps_profiles.xml`

**Current working config** (after pruning peer list):
```xml
<transport_descriptors>
  <transport_descriptor>
    <transport_id>udp_transport</transport_id>
    <type>UDPv4</type>
    <interfaceWhiteList>
      <address>127.0.0.1</address>
      <address>10.147.20.21</address>  <!-- kvlapblack -->
      <address>10.147.20.30</address>  <!-- barney -->
      <!-- Removed: hal, betty, bambam - were causing timeout lag -->
    </interfaceWhiteList>
    <maxInitialPeersRange>100</maxInitialPeersRange>
  </transport_descriptor>
</transport_descriptors>

<participant profile_name="default_part_profile" is_default_profile="true">
  <rtps>
    <builtin>
      <initialPeersList>
        <!-- localhost -->
        <locator>
          <udpv4>
            <address>127.0.0.1</address>
          </udpv4>
        </locator>
        <!-- kvlapblack -->
        <locator>
          <udpv4>
            <address>10.147.20.21</address>
          </udpv4>
        </locator>
        <!-- barney itself -->
        <locator>
          <udpv4>
            <address>10.147.20.30</address>
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
```

### Container's Config
**Location**: `/home/karim/grunt_docker/config/dds/fastrtps_unicast.xml`

**Current config**:
```xml
<transport_descriptors>
  <transport_descriptor>
    <transport_id>udp_transport</transport_id>
    <type>UDPv4</type>
    <interfaceWhiteList>
      <address>127.0.0.1</address>
      <address>10.147.20.21</address>  <!-- kvlapblack VPN IP -->
      <address>10.147.20.30</address>  <!-- barney VPN IP -->
      <address>10.147.20.33</address>  <!-- wilma VPN IP -->
      <!-- Container's actual network interfaces (from diagnostic) -->
      <address>192.168.65.3</address>
      <address>192.168.65.6</address>
      <address>172.17.0.1</address>
    </interfaceWhiteList>
    <maxInitialPeersRange>100</maxInitialPeersRange>
  </transport_descriptor>
</transport_descriptors>

<participant profile_name="default_part_profile" is_default_profile="true">
  <rtps>
    <builtin>
      <initialPeersList>
        <!-- localhost -->
        <locator>
          <udpv4>
            <address>127.0.0.1</address>
          </udpv4>
        </locator>
        <!-- kvlapblack -->
        <locator>
          <udpv4>
            <address>10.147.20.21</address>
          </udpv4>
        </locator>
        <!-- barney -->
        <locator>
          <udpv4>
            <address>10.147.20.30</address>
          </udpv4>
        </locator>
        <!-- wilma -->
        <locator>
          <udpv4>
            <address>10.147.20.33</address>
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
```

**Mounted via**: `compose/viz/bash.yaml`, `compose/viz/rviz.yaml`, `compose/viz/rqt.yaml`
- Volume: `../../config/dds:/dds_config:ro`
- Env: `FASTRTPS_DEFAULT_PROFILES_FILE=/dds_config/fastrtps_unicast.xml`

## Unresolved Questions

### 1. interfaceWhiteList Behavior

**Question**: Does `interfaceWhiteList` control:
- A) Only LOCAL interface binding (which NICs to use for sending/receiving)?
- B) Both local binding AND remote peer filtering (firewall-like)?

**Official Documentation** (https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/whitelist.html):
> "Communication interfaces used by the DomainParticipants whose TransportDescriptorInterface defines an interfaceWhiteList is limited to the interfaces' addresses defined in that list"

**The quote is ambiguous** - it doesn't clearly state whether non-existent local IPs should be included.

**Current uncertainty**:
- Container's whitelist includes `10.147.20.21`, `10.147.20.30`, `10.147.20.33` which **don't exist as local interfaces**
- Docs say: "If none of the values in the transport descriptor's whitelist match the interfaces on the host, then all the interfaces in the whitelist are filtered out and therefore no communication will be established"
- **BUT**: Does this mean each individual non-matching entry is ignored, or the whole list fails?

**Arguments for "local binding only"**:
- Would be very fragile if configured-but-offline peers break the entire network
- FastDDS shouldn't continuously retry binding to non-existent interfaces

**Arguments for "also filters remote"**:
- Named "whitelist" suggests security/filtering function
- Documentation mentions "limiting communication"

**Empirical test needed**: Remove VPN IPs from container's whitelist, keep only actual container IPs, test if discovery still works.

### 2. Why No Cross-NAT Discovery?

**Theories**:
1. **interfaceWhiteList mismatch**: Non-existent IPs causing transport to fail
2. **Windows firewall**: Blocking inbound DDS ports from WSL2/Docker
3. **NAT port mapping**: DDS discovery ports not properly forwarded through NAT layers
4. **Initial peers not working**: Unicast discovery packets not routing correctly
5. **Transport type mismatch**: Some subtle incompatibility in transport config

**What we know**:
- ✅ ICMP works (ping succeeds)
- ✅ TCP likely works (normal internet traffic routes fine)
- ❓ UDP ports 7400/7410/7412 - unknown if they can traverse the NAT
- ❓ Barney receiving discovery packets from container - no visibility

## Next Debugging Steps

### Step 1: Test interfaceWhiteList Theory
Edit container's `config/dds/fastrtps_unicast.xml`:

```xml
<interfaceWhiteList>
  <!-- Only actual container interfaces -->
  <address>127.0.0.1</address>
  <address>192.168.65.3</address>
  <address>192.168.65.6</address>
  <address>172.17.0.1</address>
  <!-- REMOVED: 10.147.20.21, 10.147.20.30, 10.147.20.33 -->
</interfaceWhiteList>
```

**Keep** `initialPeersList` unchanged (still targeting VPN IPs for discovery).

Test: `docker compose -f compose/viz/bash.yaml run --rm bash` → `ros2 topic list`

**Expected outcomes**:
- If it works: interfaceWhiteList is local-only, remote IPs were breaking it
- If it fails: interfaceWhiteList wasn't the problem

### Step 2: Install Network Tools in Container
Add to base Dockerfile or test container:
```bash
apt-get update && apt-get install -y netcat-openbsd tcpdump
```

Then test UDP connectivity:
```bash
# Listen on barney
nc -ul 9999

# Send from container
echo "test" | nc -u 10.147.20.30 9999
```

### Step 3: Packet Capture on Barney
```bash
# On barney, capture DDS discovery traffic
sudo tcpdump -i any 'udp and (port 7400 or port 7410 or port 7412)' -n

# In container, try to discover
ros2 topic list
```

**Look for**: Packets from 10.147.20.21 (NAT'd container traffic) arriving at barney.

### Step 4: Try CycloneDDS Instead
FastRTPS might have NAT traversal issues. Test with CycloneDDS which has better NAT handling:

```bash
# In container
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list
```

(Requires CycloneDDS to be installed in base image)

### Step 5: Simplify to Minimal Config
Create a completely minimal FastRTPS profile:
```xml
<profiles xmlns="http://www.eprosima.com">
  <participant profile_name="default_part_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>10.147.20.30</address>
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

No custom transports, no whitelists, just "please talk to barney via unicast".

## Diagnostic Commands

### In Container
```bash
# Check environment
echo $FASTRTPS_DEFAULT_PROFILES_FILE
cat $FASTRTPS_DEFAULT_PROFILES_FILE

# Check network interfaces
ip addr show

# Check DDS ports
ss -ulpn | grep 74

# Verbose DDS logging (broken - doesn't produce output)
export FASTDDS_VERBOSITY=info
ros2 topic list

# Test ping
ping -c 3 10.147.20.30

# Test DNS
nslookup barney.robodojo.net
```

### On Barney
```bash
# Check what's listening
ss -ulpn | grep 74

# Check config
echo $FASTRTPS_DEFAULT_PROFILES_FILE
cat $FASTRTPS_DEFAULT_PROFILES_FILE

# Packet capture
sudo tcpdump -i any 'udp and (port 7400 or port 7410)' -n

# Test local discovery
ros2 topic list
ros2 run demo_nodes_cpp talker
```

## Files Created

### Compose Files
- `compose/viz/rviz.yaml` - RViz2 with WSLg support
- `compose/viz/rqt.yaml` - RQT with WSLg support
- `compose/viz/bash.yaml` - Interactive debugging shell

### Configuration
- `config/rviz/default.rviz` - RViz window size (1200x800)
- `config/dds/fastrtps_unicast.xml` - Custom DDS profile for container
- `config/dds/diagnose_dds.sh` - DDS diagnostic script
- `config/dds/test_discovery.sh` - Discovery test script

### Documentation
- `docs/wsl2-visualization.md` - WSLg setup guide
- `docs/dds-cross-nat-troubleshooting.md` - This file

## References

- FastDDS Interface Whitelist: https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/whitelist.html
- FastDDS Transport Descriptors: https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/transports.html
- ROS 2 DDS Tuning: https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html

## Open Issues for Long-Term

1. **Peer list fragility**: Current FastDDS behavior of severe lag when peers are unreachable is unacceptable for production
   - Need discovery timeout configuration
   - Or switch to FastDDS Discovery Server mode
   - Or use CycloneDDS with better NAT handling

2. **Config management**: Need to update grunt_bringup profile in grunt repo when:
   - New robots added to network
   - Testing from new development machines
   - Network topology changes

3. **NAT traversal**: May need to investigate:
   - FastDDS Discovery Server (centralized discovery)
   - ROS 2 bridge nodes
   - VPN directly in WSL2 (if ZeroTier supports it)
   - Port forwarding rules in Windows firewall
