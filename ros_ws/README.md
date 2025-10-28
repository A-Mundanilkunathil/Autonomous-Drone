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

  **Run**
  ```
  python node_interface.py
  ```