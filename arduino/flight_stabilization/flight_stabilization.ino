#include <Arduino_LSM9DS1.h>
#include <Servo.h>

// Create motor ESC objects
Servo esc1, esc2, esc3, esc4;

// Pin definitions
const int esc1Pin = 6;
const int esc2Pin = 9;
const int esc3Pin = 10;
const int esc4Pin = 11;

// Configuration and State Variables
float pitch = 0.0; // Current pitch angle
float roll = 0.0; // Current roll angle
float yaw = 0.0; // Current yaw angle

float gx, gy, gz; // Gyroscope readings
float ax, ay, az; // Accelerometer readings

unsigned long lastTime = 0; // Last time the loop ran
float dt = 0.01; // Time step for calculations (10 ms)

// PID control variables
float kp = 1.0; // Proportional gain
float ki = 0.0; // Integral gain
float kd = 0.0; // Derivative gain

int baseSpeed = 800; // Base speed for motors

// PID error variables
float pitchError = 0.0; // Pitch error
float rollError = 0.0; // Roll error
float yawError = 0.0; // Yaw error

float pitchIntegral = 0.0; // Integral of pitch error
float rollIntegral = 0.0; // Integral of roll error
float yawIntegral = 0.0; // Integral of yaw error

float lastPitchError = 0.0; // Last pitch error
float lastRollError = 0.0; // Last roll error
float lastYawError = 0.0; // Last yaw error

void setup() {
    Serial.begin(115200); // Initialize serial communication

    while (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        delay(1000);
    }

    esc1.attach(esc1Pin);  
    esc2.attach(esc2Pin);
    esc3.attach(esc3Pin);
    esc4.attach(esc4Pin);

    delay(2000); // Wait for ESCs to initialize
    lastTime = millis(); // Initialize last time
}

void loop() {
    unsigned long currentTime = millis();
    dt = (currentTime - lastTime) / 1000.0; // Calculate time step
    lastTime = currentTime;

    // Read IMU data
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        // Calculate pitch, roll, and yaw angles
        pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
        roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
        yaw = gz; // Assuming yaw is directly from gyroscope
    }

    // Target angles (for level flight)
    float targetPitch = 0.0; // Target pitch angle
    float targetRoll = 0.0; // Target roll angle
    float targetYaw = 0.0; // Target yaw angle

    // Pitch PID
    pitchError = targetPitch - pitch; // Calculate pitch error
    pitchIntegral += pitchError * dt; // Calculate integral of pitch error
    float pitchDerivative = (pitchError - lastPitchError) / dt; // Calculate derivative of pitch error
    float pitchOutput = kp * pitchError + ki * pitchIntegral + kd * pitchDerivative; // PID output for pitch
    lastPitchError = pitchError; // Update last pitch error

    // Roll PID
    rollError = targetRoll - roll; // Calculate roll error
    rollIntegral += rollError * dt; // Calculate integral of roll error
    float rollDerivative = (rollError - lastRollError) / dt; // Calculate derivative of roll error
    float rollOutput = kp * rollError + ki * rollIntegral + kd * rollDerivative; // PID output for roll
    lastRollError = rollError; // Update last roll error

    // Yaw PID
    yawError = targetYaw - yaw; // Calculate yaw error
    yawIntegral += yawError * dt; // Calculate integral of yaw error
    float yawDerivative = (yawError - lastYawError) / dt; // Calculate derivative of yaw error
    float yawOutput = kp * yawError + ki * yawIntegral + kd * yawDerivative; // PID output for yaw
    lastYawError = yawError; // Update last yaw error

    // Calculate motor speeds based on PID outputs
    int speed1 = baseSpeed + pitchOutput - rollOutput + yawOutput; // Motor 1 speed
    int speed2 = baseSpeed + pitchOutput + rollOutput - yawOutput; // Motor 2 speed
    int speed3 = baseSpeed - pitchOutput + rollOutput + yawOutput; // Motor 3 speed
    int speed4 = baseSpeed - pitchOutput - rollOutput - yawOutput; // Motor 4 speed

    // Constrain motor speeds to valid range
    speed1 = constrain(speed1, 700, 2000);
    speed2 = constrain(speed2, 700, 2000);
    speed3 = constrain(speed3, 700, 2000);
    speed4 = constrain(speed4, 700, 2000);

    // Set motor speeds
    esc1.writeMicroseconds(speed1);
    esc2.writeMicroseconds(speed2);
    esc3.writeMicroseconds(speed3);
    esc4.writeMicroseconds(speed4);

    // Print debug information
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" Roll: ");
    Serial.print(roll);
    Serial.print(" Yaw: ");
    Serial.print(yaw);

    delay(10); // Delay for stability
}