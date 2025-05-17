// filepath: c:\Users\minhh\OneDrive\Desktop\CS\Raspberry-pi\arduino\wifi_test\uno_wifi_test\uno_hardware_serial_test.ino
// No SoftwareSerial needed for this version

const int ledPin = 13; // Built-in LED

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Start with LED off
  Serial.begin(4800); // Keep baud rate at 4800 (matching ESP32's Serial2)
  
  // Blink LED once at startup to show it's running
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    byte incomingByte = Serial.read(); // Read the byte, don't care about its value for now

    // If ANY byte is received, do a short, distinct blink
    // This is different from the 3-blink pattern for the specific sequence
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(200); // A small pause to distinguish from rapid sequence blinks
  }
}