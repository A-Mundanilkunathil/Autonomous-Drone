void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 2000); // optional wait
  Serial.println("Arduino Uno - Serial Loopback Test");
  Serial.println("Connect TX (Pin 1) to RX (Pin 0)");
}

void loop() {
  Serial.println("Sending test character 'A'");
  Serial.write('A');
  delay(100);

  if (Serial.available()) {
    char c = Serial.read();
    Serial.print("Received: ");
    Serial.println(c);
  } else {
    Serial.println("No data received");
  }

  delay(2000);
}
