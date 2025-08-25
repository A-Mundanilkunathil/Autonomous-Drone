# Autonomous-Drone

QBRAIN ESC 4 in 1 20A 

192.168.1.128

Wiring:
-   Red: Battery Voltage
-   White: GND
-   Brown: M1 -> Front Right
-   Orange: M2 -> Front Left
-   Green: M3 -> Rear Left
-   Purple: M4 -> Rear Right

ArduPilot:
-   Radio Receiver
    - Mission Planner: 115200, RCIN, Port 2
    - iBus FS-I6 transmitter -> FS-IA6B receiver
    - SpeedybeeF405v4: R2, 4V5, G
-   Mavlink Receiver  
    - Mission Planner: 57600, Mavlink2, Port 6
    - SpeedybeeF405v4: R6, T6, 4V5, G

Drones:
-   kakute: Betaflight
-   loki: OpenPilot
-   hunter_pence: OpenPilot
-   max_max: OpenPilot
-   thor: OpenPilot