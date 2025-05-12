#include <Servo.h>
#include <NeoSWSerial.h>  // Include NeoSWSerial library

// Create a NeoSWSerial object for communication with ESP32
NeoSWSerial espSerial(8, 9);  // RX on pin 8, TX on pin 9

Servo esc1, esc2, esc3, esc4;

int currentSpeed = 0;
int targetSpeed = 0;
int stepSize = 100;
int delayBetweenSteps = 20;
int maxSpeed = 2000;
int minSpeed = 700;

void setup() {
  Serial.begin(9600);         // USB debug
  espSerial.begin(9600);      // UART from ESP32 (TX to pin 9, RX to pin 8)

  esc1.attach(3);  // Pin 3 for Servo 1
  esc2.attach(5);  // Pin 5 for Servo 2 (avoid pins conflicting with NeoSWSerial)
  esc3.attach(6);  // Pin 6 for Servo 3
  esc4.attach(10); // Pin 10 for Servo 4

  setESCs(currentSpeed);
  Serial.println("Arduino Uno listening via NeoSWSerial...");
}

void loop() {
  // Check if data is available on NeoSWSerial
  while (espSerial.available() >= 1) {
    uint16_t inputSpeed = espSerial.read();
    inputSpeed |= (espSerial.read() << 8);

    if (inputSpeed >= minSpeed && inputSpeed <= maxSpeed) {
      targetSpeed = inputSpeed;
      Serial.print("Target speed: ");
      Serial.println(targetSpeed);
    } else {
      Serial.print("Invalid speed: ");
      Serial.println(inputSpeed);
    }
  }

  if (currentSpeed < targetSpeed) {
    currentSpeed += stepSize;
    if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
  } else if (currentSpeed > targetSpeed) {
    currentSpeed -= stepSize;
    if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
  }

  setESCs(currentSpeed);
  delay(delayBetweenSteps);
}

void setESCs(int speed) {
  esc1.writeMicroseconds(speed);
  esc2.writeMicroseconds(speed);
  esc3.writeMicroseconds(speed);
  esc4.writeMicroseconds(speed);
}
