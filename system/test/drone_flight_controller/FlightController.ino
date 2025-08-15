#include "config.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"

bool emergencyStop = false; // Flag for emergency stop

PID pidPitch(KP, KI, KD); // PID controller for pitch
PID pidRoll(KP, KI, KD); // PID controller for roll
PID pidYaw(KP, KI, KD); // PID controller for yaw

float pitch = 0, roll = 0, yaw = 0; // Current angles
float gx, gy, gz; // Gyroscope readings

float targetPitch = 0;
float targetRoll = 0;
float targetYaw = 0; 
int throttle = BASE_THROTTLE; // Base throttle value

unsigned long lastLoop = 0; // Last loop time

void setup() {
    Serial.begin(115200); // Initialize serial communication
    imuInit(); // Initialize IMU
    motorsInit(); // Initialize motors
    lastLoop = millis(); // Set last loop time
}

void loop() {
    unsigned long now = millis(); // Get current time
    float dt = (now - lastLoop) / 1000.0; // Calculate time delta

    if (dt < LOOP_DT) {
        return; // Skip loop if not enough time has passed
    }
    lastLoop = now; // Update last loop time

    if (Serial.available()) {
        char c = Serial.read();

         if (c == 'w') targetPitch = -5; // Forward
        if (c == 's') targetPitch = 5;  // Backward
        if (c == 'a') targetRoll = -5;  // Left
        if (c == 'd') targetRoll = 5;   // Right
        if (c == 'q') targetYaw = -10;  // Rotate left
        if (c == 'e') targetYaw = 10;   // Rotate right
        if (c == 'r') throttle += 50;   // Increase throttle
        if (c == 'f') throttle -= 50;   // Decrease throttle
        if (c == 'x') emergencyStop = true; // Emergency stop
    }

    readIMU(pitch, roll, yaw, gx, gy, gz); // Read IMU data

    float pitchOut = pidPitch.compute(targetPitch, pitch, dt); // Compute PID output for pitch
    float rollOut = pidRoll.compute(targetRoll, roll, dt); // Compute PID output for roll
    float yawOut = pidYaw.compute(targetYaw, yaw, dt); // Compute PID output for yaw

    // Calculate motor speeds
    int speed1 = throttle + pitchOut - rollOut + yawOut;  
    int speed2 = throttle + pitchOut + rollOut - yawOut;
    int speed3 = throttle - pitchOut + rollOut - yawOut;    
    int speed4 = throttle - pitchOut - rollOut + yawOut;

    if (emergencyStop) {
        setMotorsSpeed(700, 700, 700, 700); // Set all motors to minimum throttle
        Serial.println("Emergency stop activated!"); // Print emergency stop message
        return; // Exit loop
    }
    
    // Set motor speeds
    setMotorsSpeed(speed1, speed2, speed3, speed4); // Set motor speeds

    // Print debug information
    Serial.print("Throttle: "); Serial.print(throttle);
    Serial.print("Pitch: "); Serial.print(pitch);
    Serial.print(" Roll: "); Serial.print(roll);    
    Serial.print(" Yaw: "); Serial.print(yaw);
}

