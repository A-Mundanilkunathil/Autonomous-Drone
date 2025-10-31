**Launch Gazebo**
```
gz sim -v4 ~/Desktop/Autonomous-Drone/sim/ardupilot_gazebo/worlds/iris_runway.sdf
```

**Run ArduPilot SITL**
```bash
cd sim/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON
```

**Connect to ROS2**
- For simulation
  ```
  ros2 run mavros mavros_node --ros-args \
    -p fcu_url:=udp://0.0.0.0:14550@ \
    -p tgt_system:=1 \
    -p tgt_component:=1
  ```
  
- For real hardware
  ```
   ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=/dev/ttyUSB0:57600 \
  -p tgt_system:=1 \
  -p tgt_component:=1
  ```