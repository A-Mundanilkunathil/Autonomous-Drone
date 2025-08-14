# Autonomous-Drone


QBRAIN ESC 4 in 1 20A 
-   ESCs Firmware Minimums: 
    - 700(min) - 770(lowest start) - 760 : lowest speed : 750 : True Zero. 

1. parallel power check
2.   X - nano - solo power - connect with esp32
3.   iterate - IMU gyro scope with Nano..
4.   Camera calibration after everythihng passes. 

192.168.1.128


ArduPilot:
-   Mission Planner: 115200, RCIN, Port 2
    - iBus FS-I6 transmitter -> FS-IA6B receiver
    - SpeedybeeF405v4: R2, 4V5, G
-   Mission Planner: 57600, Mavlink2, Port 6
    - Telemetry radio: R6, T6, 4V5, G

Drones:
-   kakute: Betaflight
-   loki: OpenPilot
-   hunter_pence: OpenPilot
-   max_max: OpenPilot