#!/bin/bash
# Helper script to detect network interfaces for FastRTPS interfaceWhiteList
# Run this inside a container to see which IPs to add to fastrtps_unicast.xml

echo "=========================================="
echo "FastRTPS Interface Detection"
echo "=========================================="
echo ""
echo "Current network interfaces:"
echo ""

# Show all interfaces with IP addresses
ip -4 addr show | grep -E "^[0-9]+:|inet " | sed 's/^[0-9]*: /  /'

echo ""
echo "=========================================="
echo "Recommended interfaceWhiteList entries:"
echo "=========================================="
echo ""

# Extract just the IP addresses
echo "<!-- Add these to interfaceWhiteList in fastrtps_unicast.xml -->"
ip -4 addr show | grep "inet " | awk '{print $2}' | cut -d'/' -f1 | while read ip; do
    # Determine what this interface likely is
    case $ip in
        127.0.0.1)
            echo "<address>$ip</address>        <!-- localhost (always required) -->"
            ;;
        10.147.20.*)
            echo "<address>$ip</address>     <!-- ZeroTier VPN -->"
            ;;
        172.17.*)
            echo "<address>$ip</address>       <!-- docker0 bridge -->"
            ;;
        192.168.*)
            echo "<address>$ip</address>    <!-- local network -->"
            ;;
        10.*)
            echo "<address>$ip</address>     <!-- private network -->"
            ;;
        *)
            echo "<address>$ip</address>     <!-- unknown -->"
            ;;
    esac
done

echo ""
echo "=========================================="
echo "Notes:"
echo "=========================================="
echo "- Always include 127.0.0.1 (localhost)"
echo "- Always include your ZeroTier VPN IP (10.147.20.x)"
echo "- Include docker0 bridge (172.17.0.1) if present"
echo "- Only include other IPs if needed for local discovery"
echo "- DO NOT include IPs that don't exist on this machine"
echo ""
echo "See: config/dds/fastrtps_unicast.xml"
echo ""
