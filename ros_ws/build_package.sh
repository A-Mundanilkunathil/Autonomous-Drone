#!/bin/bash

# ROS2 Package Build and Install Script
# This script builds the autonomous_drone ROS2 package

echo "=========================================="
echo "  Building Autonomous Drone ROS2 Package"
echo "=========================================="
echo ""

# Navigate to workspace root
cd ~/Desktop/Autonomous-Drone/ros_ws

# Check if src exists
if [ ! -d "src" ]; then
    echo "Error: src directory not found!"
    exit 1
fi

# Source ROS2
echo "Sourcing ROS2..."
source /opt/ros/jazzy/setup.bash

# Build the package
echo ""
echo "Building package from workspace root..."
colcon build --packages-select autonomous_drone --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "  Build Successful!"
    echo "=========================================="
    echo ""
    echo "To use the package, run:"
    echo "  source ~/Desktop/Autonomous-Drone/ros_ws/install/setup.bash"
    echo ""
    echo "Then launch with:"
    echo "  ros2 launch autonomous_drone autonomous_drone.launch.py"
    echo ""
else
    echo ""
    echo "Build failed! Check the errors above."
    exit 1
fi
