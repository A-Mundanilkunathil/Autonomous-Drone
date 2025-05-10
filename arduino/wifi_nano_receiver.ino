#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

int currentSpeed = 0;
int targetSpeed = 0;
int stepSize = 100;
int delayBetweenSteps = 20;
int maxSpeed = 2000;
int minSpeed = 700;

void setup() {
  Serial.begin(9600);         // USB debug
  Serial1.begin(9600);        // UART from ESP32 (TX to pin 0)

  esc1.attach(6);
  esc2.attach(9);
  esc3.attach(10);
  esc4.attach(11);

  setESCs(currentSpeed);
  Serial.println("Nano 33 BLE listening via Serial1 on pin 0...");
}

void loop() {
  while (Serial1.available() >= 2) {
    uint16_t inputSpeed = Serial1.read();
    inputSpeed |= (Serial1.read() << 8);

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
