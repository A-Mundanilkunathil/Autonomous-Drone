# Autonomous Drone ROS2 Workspace

## Build Package
```bash
cd ~/Desktop/Autonomous-Drone/ros_ws
colcon build --packages-select autonomous_drone --symlink-install
source install/setup.bash
```

## Connect Flight Controller to ROS2

**For simulation:**
```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://0.0.0.0:14550@ \
  -p tgt_system:=1 \
  -p tgt_component:=1
```

**For real hardware:**
```bash
# Check the USB port
ls /dev/ttyUSB* /dev/ttyACM*

# Connect to the flight controller
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=/dev/ttyUSB0:57600 \
  -p tgt_system:=1 \
  -p tgt_component:=1
```

## Run Nodes

### Individual Nodes

**Run camera stream (simulation):**
```bash
ros2 run autonomous_drone sim_bridge
```

**Run camera stream (real hardware via UDP):**
```bash
ros2 run autonomous_drone udp_custom_receiver
```

**Run object detector:**
```bash
ros2 run autonomous_drone object_detector
```

**Run object avoidance:**
```bash
ros2 run autonomous_drone object_avoidance
```

**Run object following:**
```bash
ros2 run autonomous_drone object_following
```

**Run autonomous drone interface:**
```bash
ros2 run autonomous_drone node_interface
```

### Launch All Nodes

**For simulation (using launch script):**
```bash
cd ~/Desktop/Autonomous-Drone/ros_ws
./launch_drone.sh sim
```

**For real hardware (using launch script):**
```bash
cd ~/Desktop/Autonomous-Drone/ros_ws
./launch_drone.sh hw
```

**Or use ros2 launch directly:**
```bash
# For simulation
ros2 launch autonomous_drone autonomous_drone_sim.launch.py

# For real hardware
ros2 launch autonomous_drone autonomous_drone_hw.launch.py
```
## Run Tests
```bash
cd ~/Desktop/Autonomous-Drone/ros_ws
./run_test.sh follow       # Object following test
./run_test.sh avoidance    # Obstacle avoidance test
./run_test.sh gps          # GPS navigation test
```