**Forward port from local**
```
# Run if not auto forwarded
python ros_ws/bridges/udp_forwarder.py
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