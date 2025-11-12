**Connect flight controller to ROS2**
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

**Run camera stream**
```
cd ~/Desktop/Autonomous-Drone/ros_ws/bridges
python3 udp_custom_receiver.py --ros-args \
  -p calib_npz:=/home/hp/Desktop/Autonomous-Drone/ros_ws/bridges/camera_calib.npz \
  -p port:=5005 \
  -p expected_width:=640 \
  -p expected_height:=480
```

**Run object detector**
```
cd ~/Desktop/Autonomous-Drone/ros_ws
python3 "perception/object_detector.py"
```

**Run object avoidance**
```
cd ~/Desktop/Autonomous-Drone/ros_ws
python3 perception/object_avoidance.py --ros-args \
  -p midas_calib_npz:=/home/hp/Desktop/Autonomous-Drone/ros_ws/perception/esp32_midas_calibration.npz
```

**Run autonomous drone**
```
cd ~/Desktop/Autonomous-Drone/ros_ws
python3 node_interface.py
```