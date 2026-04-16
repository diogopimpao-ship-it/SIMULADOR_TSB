#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS_DIR="$SCRIPT_DIR"

cd "$ROS2_WS_DIR" || exit 1

source /opt/ros/humble/setup.bash
source "$ROS2_WS_DIR/install/setup.bash"

echo "A arrancar udp_receiver..."
ros2 run boat_bridge udp_receiver &
UDP_PID=$!

sleep 2

echo "A arrancar RViz..."
rviz2 &
RVIZ_PID=$!

echo ""
echo "ROS 2 automático arrancado."
echo "Workspace: $ROS2_WS_DIR"
echo "udp_receiver PID: $UDP_PID"
echo "rviz2 PID: $RVIZ_PID"

wait