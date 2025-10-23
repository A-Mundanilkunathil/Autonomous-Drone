# Setup ROS 2

**Ubuntu 24.04 (Noble)**
- Download: https://cdimage.ubuntu.com/noble/daily-live/current/

**Install ROS 2 Jazzy**
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install curl gnupg2 lsb-release software-properties-common -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt install ros-jazzy-desktop -y
```

**Install rosdep**
```bash
sudo apt update
sudo apt install python3-rosdep2 -y

sudo rosdep init
rosdep update
```

**Environment Setup**
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Test ROS 2**
```bash
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2
ros2 run demo_nodes_cpp listener
```

**ROS 2 + Gazebo Integration**
```bash
sudo apt install ros-jazzy-ros-gz -y
```

**MAVROS**
```
sudo apt update
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras

wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

ros2 run mavros mavros_node --ros-args -p fcu_url:=tcp://localhost:5760
```