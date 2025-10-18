# Getting Started: Fresh WSL2 Setup

This guide walks through setting up the Grunt Docker development environment on a **fresh Windows 11 WSL2 machine** from scratch. Follow the steps in order - no jumping around required.

**Time estimate**: 30-45 minutes
**Tested on**: Windows 11, WSL2 Ubuntu 22.04/24.04

---

## What You'll Get

By the end of this guide, you'll have:
- WSL2 with mirrored networking (or ZeroTier alternative)
- Native Docker Engine in WSL2
- Grunt Docker development environment
- ROS 2 workspace with standard repositories
- RViz2 and RQT GUI tools working via WSLg
- Cross-network robot communication via VPN

---

## Prerequisites

- **Windows 11** (recent build with WSLg support)
- **Administrator access** to Windows
- **GitHub account** with access to pondersome repositories
- **ZeroTier account** (or similar VPN for robot network access)

---

## Step 1: Install and Update WSL2

### 1.1 Install WSL2 (if not already installed)

Open PowerShell as Administrator:

```powershell
wsl --install -d Ubuntu-24.04
```

Reboot when prompted.

### 1.2 Update WSL2 to Latest Version

**IMPORTANT**: Older WSL2 versions have GUI performance issues (20+ second window launch times).

```powershell
# In PowerShell as Administrator
wsl --update
wsl --shutdown
```

### 1.3 Verify WSL2 Version

```powershell
wsl.exe --version
# Should show version 0.51.0 or newer
```

### 1.4 Launch WSL2 Ubuntu

From PowerShell or Windows Terminal:

```powershell
wsl
```

You should now be at a bash prompt in Ubuntu.

---

## Step 2: Enable WSL2 Mirrored Networking

Mirrored networking lets WSL2 see Windows network interfaces directly (including VPN interfaces).

### 2.1 Create WSL Configuration File

**From Windows** (not WSL2), create `C:\Users\<YourUsername>\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
dnsTunneling=true
firewall=true
autoProxy=true
```

**Alternative**: Use Notepad from Windows:
```powershell
notepad $env:USERPROFILE\.wslconfig
```

### 2.2 Restart WSL2

```powershell
# In PowerShell
wsl --shutdown
wsl
```

### 2.3 Verify Mirrored Networking

```bash
# From WSL2
ip addr
# Should show same interfaces as Windows (ethernet, WiFi, etc.)
```

**If interfaces don't match Windows**: Mirrored networking may not be routing traffic correctly. See [Step 3B: ZeroTier in WSL2 (Alternative)](#step-3b-zerotier-in-wsl2-alternative) below.

---

## Step 3A: Install ZeroTier on Windows (Recommended)

If mirrored networking is working, install ZeroTier on Windows host.

### 3A.1 Install ZeroTier for Windows

1. Download from https://www.zerotier.com/download/
2. Run installer
3. Join your ZeroTier network

### 3A.2 Authorize Node on ZeroTier Central

1. Go to https://my.zerotier.com
2. Find your new node (Windows machine)
3. Check "Authorized"
4. Note the assigned IP (e.g., 10.147.20.21)

### 3A.3 Verify WSL2 Sees ZeroTier Interface

```bash
# From WSL2
ip addr | grep zt
# Should show ZeroTier interface with IP from Windows
```

**If you see the ZeroTier interface**: Continue to [Step 4: Configure Windows Firewall](#step-4-configure-windows-firewall)

**If you DON'T see it**: Fall back to [Step 3B: ZeroTier in WSL2](#step-3b-zerotier-in-wsl2-alternative)

---

## Step 3B: ZeroTier in WSL2 (Alternative)

Use this if mirrored networking doesn't work or doesn't route traffic correctly.

### 3B.1 Install ZeroTier in WSL2

```bash
# From WSL2
curl -s https://install.zerotier.com | sudo bash
sudo zerotier-cli join <your-network-id>
```

### 3B.2 Authorize Node on ZeroTier Central

1. Go to https://my.zerotier.com
2. Find your new node (WSL2 machine - different from Windows)
3. Check "Authorized"
4. Note the assigned IP (e.g., 10.147.20.42)
5. *Optional*: Give it a DNS name (e.g., `halbuntu.robodojo.net`)

### 3B.3 Verify ZeroTier is Running

```bash
# From WSL2
sudo zerotier-cli listnetworks
# Should show your network with "OK PRIVATE" status

ip addr | grep zt
# Should show ZeroTier interface with assigned IP
```

**Note**: With this approach, WSL2 has its own ZeroTier IP separate from Windows. Docker containers using `network_mode: host` will use the WSL2 ZeroTier IP.

**Skip Step 4** (Windows Firewall) if using this approach - VPN traffic bypasses Windows Firewall.

---

## Step 4: Configure Windows Firewall

**Only needed if using Step 3A (ZeroTier on Windows).**

ROS 2 DDS uses specific UDP ports for discovery and communication.

### 4.1 Create Firewall Rule

From PowerShell as Administrator:

```powershell
# Allow ROS 2 DDS traffic (UDP ports 7400-7500)
New-NetFirewallRule -DisplayName "ROS 2 DDS Discovery" -Direction Inbound -Protocol UDP -LocalPort 7400-7500 -Action Allow -Profile Any
```

### 4.2 Verify Rule

```powershell
Get-NetFirewallRule -DisplayName "ROS 2 DDS Discovery"
# Should show the rule as enabled
```

---

## Step 5: Install Native Docker in WSL2

**IMPORTANT**: Use native Docker Engine, NOT Docker Desktop. Docker Desktop's `network_mode: host` only refers to its internal VM, breaking DDS discovery.

### 5.1 Remove Docker Desktop (if installed)

If you previously installed Docker Desktop:
1. Uninstall from Windows Settings → Apps
2. Restart WSL2: `wsl --shutdown` then `wsl`

### 5.2 Install Docker Engine

```bash
# From WSL2
# Update package index
sudo apt-get update

# Install prerequisites
sudo apt-get install -y ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Add Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### 5.3 Enable Docker Service

```bash
# Start Docker daemon
sudo systemctl enable docker
sudo systemctl start docker

# Add your user to docker group (avoid needing sudo)
sudo usermod -aG docker $USER

# Log out and back in for group changes to take effect
exit
# Then: wsl
```

### 5.4 Verify Docker Installation

```bash
# Should work without sudo after re-login
docker --version
docker compose version

# Verify native Docker (NOT Docker Desktop)
docker info | grep "Operating System"
# Should show: "Ubuntu 22.04" or "Ubuntu 24.04"
# Docker Desktop would show: "Docker Desktop"
```

---

## Step 6: Verify WSLg (GUI Support)

WSLg enables Linux GUI applications (RViz, RQT) to display as native Windows windows.

### 6.1 Check WSLg Mounts

```bash
# From WSL2
ls -la /mnt/wslg/
# Should show: wayland-0, runtime-dir/, PulseServer, etc.
```

### 6.2 Check Environment Variables

```bash
echo $WAYLAND_DISPLAY   # Should show: wayland-0
echo $XDG_RUNTIME_DIR   # Should show: /mnt/wslg/runtime-dir or /run/user/1000
echo $PULSE_SERVER      # Should show: /mnt/wslg/PulseServer
```

### 6.3 Test with Simple GUI App

```bash
# Install test app
sudo apt-get install -y x11-apps

# Launch calculator - should open in Windows
xcalc
```

If `xcalc` opens a calculator window in Windows, WSLg is working correctly.

**If WSLg doesn't work**: See [wsl2-visualization.md - Troubleshooting](wsl2-visualization.md#troubleshooting-wslg-not-available)

---

## Step 7: Clone Grunt Docker Repository

### 7.1 Install Git (if needed)

```bash
# From WSL2
sudo apt-get install -y git
```

### 7.2 Configure Git

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### 7.3 Clone Repository

```bash
# Clone to home directory
cd ~
git clone https://github.com/pondersome/grunt_docker.git
cd grunt_docker
```

---

## Step 8: Pull Grunt Docker Images

### 8.1 Authenticate to GitHub Container Registry

```bash
# Create GitHub Personal Access Token (PAT) if you don't have one:
# 1. Go to https://github.com/settings/tokens
# 2. Generate new token (classic)
# 3. Select scope: read:packages
# 4. Copy the token

# Login to GHCR
echo "YOUR_GITHUB_PAT" | docker login ghcr.io -u YOUR_GITHUB_USERNAME --password-stdin
```

### 8.2 Pull Images

```bash
# Pull the development image (includes ROS 2, RViz, RQT, Nav2)
docker pull ghcr.io/pondersome/grunt:humble-dev

# Verify image
docker images | grep grunt
# Should show: ghcr.io/pondersome/grunt with tag humble-dev
```

**Image size**: ~1.2GB compressed, ~3.5GB uncompressed

---

## Step 9: Setup ROS 2 Workspace

The workspace setup script automates cloning the standard repositories.

### 9.1 Ensure Correct Permissions

**IMPORTANT**: Docker Compose can create directories as root. Ensure workspace parent directory has correct ownership:

```bash
# If ~/ros2 doesn't exist yet, create it now
mkdir -p ~/ros2/humble

# Verify ownership
ls -ld ~/ros2
# Should show: drwxr-xr-x ... <your-username> <your-username> ...

# If owned by root, fix it:
# sudo chown -R $USER:$USER ~/ros2
```

### 9.2 Run Workspace Setup Script

```bash
# From grunt_docker repository root
cd ~/grunt_docker
./tools/setup-dev-workspace.sh humble
```

This script will:
1. Create `~/ros2/humble/dev_ws/src/`
2. Install `vcstool` (if not already installed)
3. Clone 5 standard repositories:
   - `grunt` - Core platform packages
   - `roarm_ws_em0` - Arm manipulation
   - `audio_common` - Audio processing
   - `by_your_command` - Voice control
   - `realsense-ros` - RealSense camera wrapper

### 9.3 Verify Workspace Structure

```bash
ls ~/ros2/humble/dev_ws/src/
# Should show: grunt, roarm_ws_em0, audio_common, by_your_command, realsense-ros
```

---

## Step 10: Build Workspace

### 10.1 Launch Interactive Container

```bash
# From grunt_docker repository root
cd ~/grunt_docker
docker compose -f compose/viz/bash.yaml run --rm bash
```

This launches an interactive bash shell in the development container with:
- Workspace mounted at `~/dev_ws`
- ROS 2 Humble environment sourced
- GUI support via WSLg
- Host networking for ROS 2 discovery

### 10.2 Build Workspace

```bash
# Inside container
cd ~/dev_ws
colcon build --symlink-install
```

**Build time**: 2-5 minutes depending on machine

### 10.3 Source Workspace

```bash
# Inside container
source ~/dev_ws/install/setup.bash

# Verify packages are available
ros2 pkg list | grep grunt
# Should show: grunt_util, grunt_bringup, etc.
```

### 10.4 Exit Container

```bash
exit
```

---

## Step 11: Test RViz2

### 11.1 Launch RViz

```bash
# From grunt_docker repository root
cd ~/grunt_docker
docker compose -f compose/viz/rviz.yaml up
```

RViz2 should launch in a native Windows window with the `barney.rviz` configuration.

**First launch**: May take 10-20 seconds to appear (subsequent launches are faster)

### 11.2 Verify ROS 2 Topics

In a new WSL2 terminal:

```bash
# Publish a test transform
docker exec -it grunt_rviz_humble bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link"
```

In RViz window:
1. Set **Fixed Frame** to `map`
2. **Add** → **TF**
3. Should see `map` → `base_link` transform

### 11.3 Stop RViz

```bash
# Ctrl+C in the terminal, then:
docker compose -f compose/viz/rviz.yaml down
```

---

## Step 12: Test RQT

### 12.1 Launch RQT

```bash
# From grunt_docker repository root
cd ~/grunt_docker
docker compose -f compose/viz/rqt.yaml up
```

RQT should launch with the `barney.perspective` layout.

### 12.2 Verify ROS 2 Topics

In RQT window:
1. **Plugins** → **Topics** → **Topic Monitor**
2. Should see available topics

In a new WSL2 terminal:

```bash
# Publish a test topic
docker exec -it grunt_rqt_humble bash -c "ros2 topic pub /test_topic std_msgs/msg/String \"data: 'Hello from WSL2'\" --rate 1"
```

Should see `/test_topic` with messages in Topic Monitor.

### 12.3 Stop RQT

```bash
# Ctrl+C in the terminal, then:
docker compose -f compose/viz/rqt.yaml down
```

---

## Step 13: Test Cross-Network Communication

Verify ROS 2 topic discovery across VPN.

### 13.1 Verify VPN Connectivity

```bash
# From WSL2
ip addr | grep zt
# Should show your ZeroTier IP (e.g., 10.147.20.X)

# Ping another robot on the network
ping 10.147.20.1
# Should get responses
```

### 13.2 Launch RViz in Background

```bash
cd ~/grunt_docker
docker compose -f compose/viz/rviz.yaml up -d
```

### 13.3 Check for Remote Topics

```bash
# From WSL2 host
docker exec -it grunt_rviz_humble bash -c "ros2 topic list"
# Should see topics from remote robots (e.g., /betty/camera/image_raw)
```

**If you don't see remote topics**: See [wsl2-visualization.md - Troubleshooting](wsl2-visualization.md#ros-2-topics-not-visible)

### 13.4 Stop Background Container

```bash
docker compose -f compose/viz/rviz.yaml down
```

---

## Alternative: Combined RViz + RQT

For normal use when you want both tools, use the combined container (more efficient):

```bash
# From grunt_docker repository root
docker compose -f compose/viz/viz-combined.yaml up
```

Launches both RViz2 and RQT in a single container:
- ~50% less memory overhead
- Single startup command
- Both tools share same ROS 2 daemon

---

## Next Steps

### Development Workflow

- **Interactive development**: Use `compose/viz/bash.yaml` for shell access
- **Multicast debugging**: Use `compose/viz/bash-multicast.yaml` for local network testing
- **Build code**: Build inside container at `~/dev_ws`
- **GUI tools**: Use dedicated compose files or combined container

### Add More Repositories

Edit `tools/grunt_repos.yaml` to add repositories, then re-run:

```bash
./tools/setup-dev-workspace.sh humble
```

### Advanced Topics

- **DDS Tuning**: See [dds-cross-nat-troubleshooting.md](dds-cross-nat-troubleshooting.md)
- **Visualization**: See [wsl2-visualization.md](wsl2-visualization.md)
- **Multi-Robot**: See [PRD Section 11](../specs/grunt_docker_prd_v_0.md#11-multi-robot-scenarios)

---

## Troubleshooting Quick Reference

| Issue | Solution |
|-------|----------|
| GUI apps slow to launch | `wsl --update` then `wsl --shutdown` (see [Step 1.2](#12-update-wsl2-to-latest-version)) |
| No remote ROS topics | Check VPN, firewall, ROS_DOMAIN_ID (see [wsl2-visualization.md](wsl2-visualization.md#ros-2-topics-not-visible)) |
| Permission denied on workspace | `sudo chown -R $USER:$USER ~/ros2` (see [Step 9.1](#91-ensure-correct-permissions)) |
| Docker requires sudo | Re-login after `usermod -aG docker $USER` (see [Step 5.3](#53-enable-docker-service)) |
| WSLg not available | Check Windows version, run `wsl --update` (see [wsl2-visualization.md](wsl2-visualization.md#troubleshooting-wslg-not-available)) |

---

## Summary

You now have a complete Grunt Docker development environment with:
- ✅ WSL2 with mirrored networking or ZeroTier
- ✅ Native Docker Engine
- ✅ ROS 2 Humble workspace with standard packages
- ✅ RViz2 and RQT visualization tools
- ✅ Cross-network robot communication

**Total setup time**: 30-45 minutes (excluding build time)

For detailed documentation on specific topics, see:
- [WSL2 Visualization Guide](wsl2-visualization.md)
- [Native Docker Setup](native-docker-wsl2-setup.md)
- [DDS Troubleshooting](dds-cross-nat-troubleshooting.md)
- [Docker Commands Cheat Sheet](docker-commands-cheatsheet.md)
