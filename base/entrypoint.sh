#!/bin/bash
# ROS 2 Container Entrypoint Script
#
# This script sources ROS 2 environments and passes control to the container command.
#
# Note on ROS 2 Daemon:
# The ROS 2 daemon is supposed to start automatically on first use, but there's a known
# issue in Docker where it can hang (ros2/ros2#1531). If you see timeouts with ros2 CLI
# commands, manually start the daemon:
#   ros2 daemon start
#
# The daemon is NOT auto-started here because:
# 1. It should start on-demand when ros2 commands are first used
# 2. Auto-starting in entrypoint can cause issues with non-interactive containers
# 3. The workaround (manual start) is simple and documented

set -e

# Source ROS 2 base environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace overlays
source /ros2_ws/install/setup.bash
# source /sim_ws/install/setup.bash  # Uncomment if using simulation workspace

# Optional: Build and source development workspace
# Uncomment these lines if you have a bind-mounted dev workspace
#if [ -d ~/dev_ws ]; then
#  cd ~/dev_ws
#  colcon build --symlink-install
#  source install/setup.bash
#fi

# Execute the command passed to the container
exec "$@"