# ROS 2 Daemon Issues in Docker Containers

## Problem

The ROS 2 daemon may not start automatically in Docker containers, causing `ros2` CLI commands to hang with timeout errors:

```
TimeoutError: [Errno 110] Connection timed out
```

## Root Cause

This is a **known issue** tracked in [ros2/ros2#1531](https://github.com/ros2/ros2/issues/1531). The ROS 2 daemon is supposed to start automatically on-demand when you first use `ros2` CLI tools, but this can fail in Docker containers on certain host systems.

## Symptoms

- `ros2 topic list` hangs indefinitely
- `ros2 node list` times out
- `ros2 topic echo /topic` doesn't receive data
- No `/tmp/ros2_daemon_*` socket file created

## Workarounds

### Option 1: Manual Daemon Start (Recommended)

Simply start the daemon manually once per container session:

```bash
# In container
ros2 daemon start

# Now ros2 commands work
ros2 topic list
ros2 node list
```

**This is the approach used in grunt_docker containers.**

### Option 2: Use --no-daemon Flag

All `ros2` commands support the `--no-daemon` flag to bypass the daemon entirely:

```bash
ros2 topic list --no-daemon
ros2 node list --no-daemon
ros2 topic echo /chatter --no-daemon
```

**Pros**: No daemon needed
**Cons**: Slightly slower, must add flag to every command

### Option 3: Auto-start in Entrypoint (Not Recommended)

You could add `ros2 daemon start` to the entrypoint script, but this has drawbacks:

- Can cause issues with non-interactive containers
- May fail silently
- The daemon should start on-demand (auto-starting works around the bug incorrectly)

## Why We Don't Auto-Start

The grunt_docker containers **do not** auto-start the daemon in `entrypoint.sh` because:

1. **Principle**: The daemon should start on-demand (as designed)
2. **Reliability**: Manual start is more reliable than auto-start in entrypoint
3. **Simplicity**: Easy to document "run `ros2 daemon start` once"
4. **Interactive tools**: GUI tools (RViz, RQT) don't always need the daemon

## Docker Exec Sessions

When you attach to a running container with `docker exec`, the ROS environment is automatically sourced via `/etc/bash.bashrc`, but you still need to start the daemon manually:

```bash
# Attach to container
docker exec -it <container-name> bash

# Start daemon (once per session)
ros2 daemon start

# Now ros2 commands work
ros2 topic list
```

## Testing if Daemon is Running

```bash
# Check for daemon socket
ls /tmp/ros2_daemon_*

# If exists, daemon is running
# If not found, run: ros2 daemon start
```

## Impact on Different Use Cases

### Interactive Debugging (bash.yaml)
**Impact**: Manual start required
**Workaround**: Document in compose file header (already done)

### GUI Tools (RViz, RQT)
**Impact**: May work without daemon for simple use cases
**Workaround**: Start daemon if topic discovery fails

### Automated Scripts
**Impact**: Scripts that use `ros2` commands need daemon
**Workaround**: Add `ros2 daemon start` to script, or use `--no-daemon` flag

### ROS 2 Nodes (talker/listener)
**Impact**: None - nodes don't use the daemon
**Workaround**: Not needed

## Alternative: CycloneDDS

If the FastDDS daemon issues are persistent, consider switching to CycloneDDS:

```yaml
environment:
  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

CycloneDDS has different discovery mechanisms and may not have the same daemon issues.

## Related Issues

- [ros2/ros2#1531](https://github.com/ros2/ros2/issues/1531) - ROS 2 daemon hangs on Arch with Docker
- [Stack Overflow: docker exec ROS 2 commands](https://stackoverflow.com/questions/74114225/using-docker-exec-to-run-ros2-commands)

## Summary

**The daemon issue is a known Docker + ROS 2 incompatibility.**

**Our solution**: Manually start daemon once per container session. This is simple, documented, and reliable.

**For automated workflows**: Use `--no-daemon` flag or ensure daemon starts before running ros2 commands.
