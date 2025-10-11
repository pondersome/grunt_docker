#!/bin/bash
# DDS Diagnostic Script
# Run this inside the container to diagnose FastRTPS/DDS connectivity issues

echo "=========================================="
echo "DDS/FastRTPS Diagnostic Tool"
echo "=========================================="
echo ""

echo "1. Environment Variables"
echo "----------------------------------------"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "FASTRTPS_DEFAULT_PROFILES_FILE: $FASTRTPS_DEFAULT_PROFILES_FILE"
echo ""

echo "2. FastRTPS Profile Status"
echo "----------------------------------------"
if [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "✓ Profile file exists: $FASTRTPS_DEFAULT_PROFILES_FILE"
    echo "Profile contents:"
    cat "$FASTRTPS_DEFAULT_PROFILES_FILE"
else
    echo "✗ Profile file not found: $FASTRTPS_DEFAULT_PROFILES_FILE"
fi
echo ""

echo "3. Network Interfaces"
echo "----------------------------------------"
ip addr show | grep -E "^[0-9]+:|inet "
echo ""

echo "4. Network Connectivity Tests"
echo "----------------------------------------"
echo "Testing localhost:"
ping -c 2 127.0.0.1 2>&1 | head -n 5

echo ""
echo "Testing barney.robodojo.net (10.147.20.30):"
ping -c 2 10.147.20.30 2>&1 | head -n 5

echo ""
echo "Testing kvlapblack.robodojo.net (10.147.20.21):"
ping -c 2 10.147.20.21 2>&1 | head -n 5
echo ""

echo "5. UDP Port Check (FastRTPS discovery ports)"
echo "----------------------------------------"
echo "Checking if discovery ports are listening:"
ss -ulpn | grep -E ":(7400|7401|7402|7410|7411|7412)" || echo "No FastRTPS ports found listening"
echo ""

echo "6. ROS 2 Discovery Test"
echo "----------------------------------------"
echo "Running: ros2 daemon stop && ros2 daemon start"
ros2 daemon stop 2>&1
sleep 2
ros2 daemon start 2>&1
sleep 3
echo ""

echo "Attempting to list ROS 2 topics (30 second timeout):"
timeout 30 ros2 topic list 2>&1 || echo "✗ Topic list timed out or failed"
echo ""

echo "7. FastRTPS Discovery Tool (if available)"
echo "----------------------------------------"
if command -v fastrtps_discovery_server >/dev/null 2>&1; then
    echo "FastRTPS discovery server available"
else
    echo "FastRTPS discovery server not found (this is normal)"
fi
echo ""

echo "8. Multicast Test"
echo "----------------------------------------"
echo "Checking multicast routes:"
ip route show | grep -i multicast || echo "No multicast routes found"
echo ""

echo "9. Firewall/iptables"
echo "----------------------------------------"
echo "Checking iptables rules (requires root):"
iptables -L -n 2>&1 | head -n 20 || echo "Cannot read iptables (permission denied)"
echo ""

echo "=========================================="
echo "Diagnostic Complete"
echo "=========================================="
