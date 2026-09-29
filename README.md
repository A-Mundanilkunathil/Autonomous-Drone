# Autonomous-Drone

A comprehensive ROS2-based framework for autonomous drone control, featuring GPS navigation, obstacle avoidance, and object detection/following capabilities. Designed for use with ArduPilot-based flight controllers.

## Features

- **Autonomous Navigation**: GPS waypoint navigation with dynamic obstacle avoidance.
- **Perception**: Real-time object detection using YOLO and monocular depth estimation.
- **Control Modes**:
  - **GPS Mission**: Navigate to global coordinates while avoiding obstacles.
  - **Object Following**: Track and follow detected objects.
  - **Object Avoidance**: Reactive avoidance using depth data or simulated sensors.
- **Simulation**: Full SITL support with ArduPilot and Gazebo/Webots.
- **Hardware Support**: Compatible with ESP32 bridges, RealSense cameras, and standard MAVLink flight controllers.

## Project Structure

- **`ros_ws/`**: The ROS2 workspace containing the core logic.
  - `src/autonomous_drone/`: Main ROS2 package.
  - `launch/`: Launch files for simulation and hardware.
- **`src/`**: Standalone Python libraries for control and perception.
- **`sim/`**: Simulation environment setup, including ArduPilot SITL configurations.
- **`platforms/`**: Firmware and setup for embedded platforms like ESP32.
- **`config/`**: Vehicle parameter files.

## Installation

### Prerequisites
- **ROS2** 
- **Python 3.8+**
- **ArduPilot** (for simulation)

### Setup

1. **Clone simulation dependencies**:
    ```bash
    git submodule update --init --recursive
    ```

2. **Install Python Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

3. **Build the ROS2 Workspace**:
    ```bash
    cd ros_ws
    ./build_package.sh
    ```

## Usage

### 1. Simulation
To run the full autonomous stack in simulation:

```bash
cd ros_ws
./launch_drone.sh sim
```

Use the `namespace` launch argument when running more than one vehicle stack:

```bash
ros2 launch autonomous_drone autonomous_drone_sim.launch.py namespace:=uav1
```



### 2. Real Hardware
To run on the physical drone:

```bash
cd ros_ws
./launch_drone.sh hw
```



### 3. Running Tests

The default automated suite contains ROS-independent control, safety, and vision
geometry tests and
does not communicate with flight hardware:

```bash
PYTHONPATH=ros_ws/src/autonomous_drone \
  python -m pytest -q ros_ws/src/autonomous_drone/test
```

Install `requirements-dev.txt` to run the same suite through pytest. Hardware and
SITL scenarios remain explicit manual tests:

```bash
cd ros_ws
./run_test.sh follow       # Test object following
./run_test.sh avoidance    # Test obstacle avoidance
./run_test.sh gps          # Test GPS navigation with avoidance
./run_test.sh gps_with_avoidance # Test GPS navigation
```

## Architecture

The production ROS2 package under `ros_ws/src/autonomous_drone` is the canonical
runtime. Its `core` package contains ROS-independent navigation and safety rules;
ROS nodes adapt MAVROS and perception messages around that core. Only the flight
manager publishes final MAVROS setpoints. The top-level `src/` tree contains
standalone hardware and perception utilities and should not be imported by the
ROS runtime.

All application topics are relative so a complete stack can run under a ROS
namespace. Simulator-owned topics, such as the Gazebo camera input, are supplied
as parameters or remappings.

The `vslam_node` executable currently provides experimental RGB-D visual
odometry and a sparse map. RGB and depth are paired by their source timestamp,
and metric motion is estimated with PnP. Experimental loop correction and
virtual-GPS output are disabled by default; neither should be enabled for flight
control until validated against recorded ground truth.

## Hardware Configuration
[Connect ESCs and Motors](https://ardupilot.org/copter/docs/connect-escs-and-motors.html)

**ArduPilot Connections**
-   **Radio Receiver**:
    - Mission Planner: 115200, RCIN, Port 2
    - iBus FS-I6 transmitter -> FS-IA6B receiver
    - Flight Controller: R2, 4V5, G
-   **Mavlink Receiver**:
    - Mission Planner: 57600, Mavlink2, Port 6
    - Flight Controller: R6, T6, 4V5, G
-   **GPS + Compass**:
    - Mission Planner: 115200, GPS
    - Flight Controller: R4, I2C, 4V5, G
