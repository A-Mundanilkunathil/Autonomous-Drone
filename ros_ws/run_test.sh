#!/bin/bash
# Simple script to run test files from the ROS2 workspace

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source the ROS2 workspace
source "$SCRIPT_DIR/install/setup.bash"

# Check if a test file was provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <test_name>"
    echo ""
    echo "Available tests:"
    echo "  - follow      : Test object following behavior"
    echo "  - avoidance   : Test obstacle avoidance during mission"
    echo "  - gps         : Test GPS navigation and waypoint following"
    echo ""
    echo "Example: $0 follow"
    exit 1
fi

# Map test names to files
case "$1" in
    follow)
        TEST_FILE="$SCRIPT_DIR/src/autonomous_drone/tests/test_follow.py"
        ;;
    avoidance)
        TEST_FILE="$SCRIPT_DIR/src/autonomous_drone/tests/test_avoidance.py"
        ;;
    gps)
        TEST_FILE="$SCRIPT_DIR/src/autonomous_drone/tests/test_gps.py"
        ;;
    *)
        echo "Unknown test: $1"
        echo "Available tests: follow, avoidance, gps"
        exit 1
        ;;
esac

# Run the test
echo "Running test: $1"
echo "Test file: $TEST_FILE"
echo ""
python3 "$TEST_FILE"
