**Forward port from local**
```
# Run if not auto forwarded
python ros_ws/bridges/udp_forwarder.py
```

**Forward stream from simulation**
```
ros2 run ros_gz_bridge parameter_bridge /world/iris_warehouse/model/camera/link/link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image

python3 ros_ws/bridges/sim_bridge.py
```

**Stream**
```
cd ~/Desktop/Autonomous-Drone/ros_ws/bridges
python3 udp_custom_receiver.py --ros-args -p port:=5005
```

**View the stream**
```
# Install image_view 
sudo apt install ros-jazzy-image-view

# View images
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw

# View detection images
cd ~/Desktop/Autonomous-Drone/ros_ws
python3 " perception/object_detector.py"

python ros_ws/bridges/detection_viewer.py
```