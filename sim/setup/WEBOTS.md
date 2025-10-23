# Setup Webots Simulator

**Webots**
```
brew install --cask webots
```

**ArduPilot**
```
cd sim && git clone https://github.com/ArduPilot/ardupilot.git

cd ardupilot && git submodule update --init --recursive
```

**SITL**
```
./waf configure --board sitl
./waf copter
```

**PROTO (optional)**
```
cd /Users/hp/Desktop/Projects/Autonomous-Drone/sim/ardupilot/libraries/SITL/examples/Webots_Python/worlds

for f in *.wbt; do
  sed -i '' 's#https://raw.githubusercontent.com/cyberbotics/webots/.*/projects/#webots://projects/#g' "$f"
done
```
-  Revert if needed
```
cd /Users/hp/Desktop/Projects/Autonomous-Drone/sim/ardupilot/libraries/SITL/examples/Webots_Python/worlds

# Restore original URLs
for f in *.wbt; do
  sed -i '' 's#webots://projects/#https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/#g' "$f"
done
```

**Controller**
```
/Users/hp/Desktop/Projects/Autonomous-Drone/sim/ardupilot/libraries/SITL/examples/Webots_Python/controllers/ardupilot_vehicle_controller/ardupilot_vehicle_controller.py
```
-  Replace ```#!/usr/bin/env python3``` with correct python env ```#!/Users/hp/Desktop/Projects/Autonomous-Drone/venv/bin/python3```

**Run**
```
Tools/autotest/sim_vehicle.py -v ArduCopter -w \
  --model webots-python \
  --add-param-file=/Users/hp/Desktop/Projects/Autonomous-Drone/sim/ardupilot/libraries/SITL/examples/Webots_Python/params/iris.parm
```

**MAVProxy (optional)**
```
mavproxy.py --master=tcp:127.0.0.1:5760 --out 127.0.0.1:14550
```

