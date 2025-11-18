#!/bin/bash

# Quick Launch Script for Autonomous Drone
# This script sources the ROS2 workspace and launches all nodes
# Usage: ./launch_drone.sh [sim|hw]

echo "=========================================="
echo "  Autonomous Drone - Launch All Nodes"
echo "=========================================="
echo ""

# Determine launch mode (default to sim)
MODE="${1:-sim}"

if [ "$MODE" != "sim" ] && [ "$MODE" != "hw" ]; then
    echo "Usage: $0 [sim|hw]"
    echo "  sim - Launch simulation nodes (default)"
    echo "  hw  - Launch hardware nodes"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    echo "Sourcing workspace..."
    source "$SCRIPT_DIR/install/setup.bash"
    
    if [ "$MODE" == "sim" ]; then
        echo "Launching SIMULATION nodes..."
        echo ""
        ros2 launch autonomous_drone autonomous_drone_sim.launch.py
    else
        echo "Launching HARDWARE nodes..."
        echo ""
        ros2 launch autonomous_drone autonomous_drone_hw.launch.py
    fi
else
    echo "Error: Workspace not built yet!"
    echo ""
    echo "Please run first:"
    echo "  cd $SCRIPT_DIR"
    echo "  ./build_package.sh"
    exit 1
fi
