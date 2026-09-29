#!/bin/bash

# ROS2 Package Build and Install Script
# This script builds the autonomous_drone ROS2 package

echo "=========================================="
echo "  Building Autonomous Drone ROS2 Package"
echo "=========================================="
echo ""

# Navigate to workspace root regardless of where the repository is cloned.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

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
    echo "  source $SCRIPT_DIR/install/setup.bash"
    echo ""
    echo "Then launch with:"
    echo "  Simulation: ros2 launch autonomous_drone autonomous_drone_sim.launch.py"
    echo "  Hardware:   ros2 launch autonomous_drone autonomous_drone_hw.launch.py"
    echo ""
else
    echo ""
    echo "Build failed! Check the errors above."
    exit 1
fi
