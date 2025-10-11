#!/bin/bash
# Test DDS discovery between container and barney

echo "Testing DDS Discovery"
echo "===================="
echo ""

echo "1. Check if we can reach barney's DDS ports:"
nc -zvu 10.147.20.30 7400 2>&1 | head -3
nc -zvu 10.147.20.30 7410 2>&1 | head -3
echo ""

echo "2. Try verbose FastDDS logging:"
export FASTDDS_VERBOSITY=info
echo "Starting ros2 topic list with verbose logging..."
timeout 10 ros2 topic list 2>&1 | grep -i "participant\|discovery\|endpoint\|chatter" | head -20
echo ""

echo "3. Check what IPs FastDDS is binding to:"
ss -ulpn | grep python3 | grep 74
echo ""

echo "4. Try listening on specific topic:"
echo "Attempting to echo /chatter for 10 seconds..."
timeout 10 ros2 topic echo /chatter std_msgs/msg/String 2>&1 || echo "Topic not found or timeout"
