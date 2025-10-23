# Setup Gazebo Simulator

**Install Gazebo**
```bash
sudo apt update
sudo apt install curl lsb-release gnupg -y
sudo apt install mesa-utils

sudo curl -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

sudo apt update

sudo apt install gz-jetty -y
```

**ArduPilot Gazebo Plugin**
```bash
cd sim
git clone https://github.com/ArduPilot/ardupilot_gazebo

sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt install python3-pyqt5

cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
sudo make install
```

**Environment Setup**
```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=~/Desktop/Autonomous-Drone/sim/ardupilot_gazebo/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

**Set Up ArduPilot Firmware**
```
git clone https://github.com/ArduPilot/ardupilot.git

cd ardupilot
git submodule update --init --recursive

./waf configure --board sitl
./waf build
```

**Test Gazebo**
```bash
gz sim
```

**Run with ArduPilot SITL**
```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
```

**Launch Gazebo**
```
gz sim ~/Desktop/Autonomous-Drone/sim/ardupilot_gazebo/worlds/iris_runway.sdf
```