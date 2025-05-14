#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

// Configuration and State Variables
int currentSpeed = 0; // Current speed of the ESCs
int targetSpeed = 0; // Target speed to be set
int stepSize = 100; // Step size for gradual speed change
int delayBetweenSteps = 20; // Delay between each step
int maxSpeed = 2000; // Maximum speed for the ESCs
int minSpeed = 700; // Minimum speed for the ESCs

// --- Maneuver State Flags ---
bool applyForwardPitch = false; // Flag to apply forward pitch


// --- Adhyustment Magnitudes ---
const int PITCH_ADJUSTMENT = 50; // Adjustment for forward pitch


void setup() {
  Serial.begin(9600);         // USB debug
  Serial1.begin(9600);        // UART from ESP32 (TX to pin 0)

  // Attach ESCs to pins with min/max pulse widths
  esc1.attach(9, 700, 2000);  
  esc2.attach(10, 700, 2000); 
  esc3.attach(11, 700, 2000);
  esc4.attach(12, 700, 2000);

  // Initialize ESCs to min speed
  setESCs(minSpeed, minSpeed, minSpeed, minSpeed);
  currentSpeed = minSpeed;
  targetSpeed = minSpeed;

  Serial.println("Nano 33 BLE listening via Serial1 on pin 0...");

  Serial.println("Nano 33 BLE Flight Controller Initialized.");
  Serial.println("Listening via Serial1 on pin D0 for ESP32 commands.");
}

void loop() {
    if (Serial1.available() > 0) {
        char command = Serial1.read(); // Read the command from Serial1

        switch (expression){
            case 'W': // Command to go forward
                go_forward();
                break;
            case 'Q': // Command to level flight
                level_flight_pitch();
                break;
            case 'T': // Command to set target speed
                if (Serial1.available() >= 2) {
                    uint16_t newSpeed = Serial1.read();
                    newSpeed |= (Serial1.read() << 8); // Read the next byte and combine

                    if (newSpeed >= minSpeed && newSpeed <= maxSpeed) {
                        targetSpeed = newSpeed; // Set the target speed
                        Serial.print("Target speed set to: ");
                        Serial.println(targetSpeed);
                    } else {
                        Serial.print("Invalid speed: ");
                        Serial.println(newSpeed);
                    }
                } else {
                    Serial.println("Not enough data to set target speed.");
                    while (Serial1.available()) {
                        Serial1.read(); // Clear the buffer
                    }
                }
                break;
            default:
                Serial.println("Unknown command received.");
                Serial.println(command);
                break;
        }

        if (currentSpeed < targetSpeed) { // Increase speed
            currentSpeed += stepSize;

            // Check if current speed exceeds target speed
            if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
        } else if (currentSpeed > targetSpeed) { // Decrease speed
            currentSpeed -= stepSize;

            // Check if current speed is below target speed
            if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
        }
        currentSpeed = constrain(currentSpeed, minSpeed, maxSpeed); // Constrain speed to valid range

        int speedFL = currentSpeed;
        int speedFR = currentSpeed;
        int speedRL = currentSpeed;
        int speedRR = currentSpeed;

        if (applyForwardPitch) {
            speedFL -= PITCH_ADJUSTMENT; // Adjust speed for forward pitch
            speedFR -= PITCH_ADJUSTMENT; // Adjust speed for forward pitch
            speedRL += PITCH_ADJUSTMENT; // Adjust speed for forward pitch
            speedRR += PITCH_ADJUSTMENT; // Adjust speed for forward pitch
        }

        speedFL = constrain(speedFL, minSpeed, maxSpeed); // Constrain speed to valid range
        speedFR = constrain(speedFR, minSpeed, maxSpeed); // Constrain speed to valid range
        speedRL = constrain(speedRL, minSpeed, maxSpeed); // Constrain speed to valid range
        speedRR = constrain(speedRR, minSpeed, maxSpeed); // Constrain speed to valid range

        setESCs(speedFL, speedFR, speedRL, speedRR); // Set the ESCs to the calculated speeds

        delay(delayBetweenSteps); // Delay between each step
    }
}

/**
 * @brief Set the speed of the ESCs for each motor.
 * @details Given four speed values, set the speed of each motor to the given
 *          value. The values are in microseconds, so the range is from 700 to
 *          2000.
 * @param speed1 Speed for motor 1.
 * @param speed2 Speed for motor 2.
 * @param speed3 Speed for motor 3.
 * @param speed4 Speed for motor 4.
 */
void setESCs(int speed1, int speed2, int speed3, int speed4) {
  esc1.writeMicroseconds(speed1);
  esc2.writeMicroseconds(speed2);
  esc3.writeMicroseconds(speed3);
  esc4.writeMicroseconds(speed4);
}

/**
 * @brief COMMAND: Sets the state to apply forward pitch.
 */
void go_forward() {
    applyForwardPitch = true; // Set the flag to apply forward pitch
    Serial.println("CMD: Go Forward");
}

/**
 * @brief COMMAND: Resets pitch-related maneuver flags.
 */
void level_flight_pitch() {
    applyForwardPitch = false; // Clear the flag to stop applying forward pitch
    Serial.println("CMD: Level Flight");
}