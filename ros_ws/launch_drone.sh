#!/bin/bash

# Quick Launch Script for Autonomous Drone
# This script sources the ROS2 workspace and launches all nodes

echo "=========================================="
echo "  Autonomous Drone - Launch All Nodes"
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace
if [ -f ~/Desktop/Autonomous-Drone/ros_ws/install/setup.bash ]; then
    echo "Sourcing workspace..."
    source ~/Desktop/Autonomous-Drone/ros_ws/install/setup.bash
    
    echo "Launching all nodes..."
    echo ""
    ros2 launch autonomous_drone autonomous_drone.launch.py
else
    echo "Error: Workspace not built yet!"
    echo ""
    echo "Please run first:"
    echo "  cd ~/Desktop/Autonomous-Drone/ros_ws"
    echo "  ./build_package.sh"
    exit 1
fi
