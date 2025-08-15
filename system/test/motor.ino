#include <Servo.h>

Servo esc1, esc2, esc3, esc4; // create servo objects to control the ESCs
int speed = 1000;

void setup() {
    Serial.begin(9600); // Initialize serial communication for debugging
    esc1.attach(6);
    esc2.attach(7);
    esc3.attach(8);
    esc4.attach(9);

    // Initialize ESCs 
    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed);

    Serial.println("Enter speed (1000-2000): ");
    delay(3000); // Wait for ESCs to arm
}

void loop() {
   if (Serial.available() > 0) {
        int inputSpeed = Serial.parseInt(); // Read the speed from serial input

        if (inputSpeed >= 1000 && inputSpeed <= 2000) {
            speed = inputSpeed; // Update speed if within range

            // Set the speed for all ESCs
            esc1.writeMicroseconds(speed);
            esc2.writeMicroseconds(speed);
            esc3.writeMicroseconds(speed);
            esc4.writeMicroseconds(speed);
            
            Serial.print("Speed set to: ");
            Serial.println(speed);
        } else {
            Serial.println("Invalid speed. Please enter a value between 1000 and 2000.");
        }
   }
}