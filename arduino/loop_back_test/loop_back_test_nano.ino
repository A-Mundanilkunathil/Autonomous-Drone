const int ledPin = LED_BUILTIN; // Or use 13 if LED_BUILTIN isn't defined for your specific Nano 33 BLE core version
                                // For Nano 33 BLE, LED_BUILTIN is usually Pin 13.
char testChar = 'A';
int testCounter = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  Serial.begin(9600); // For USB debugging output
  while (!Serial && millis() < 2000); // Wait for Serial Monitor to connect (optional)

  Serial.println("Nano 33 BLE - Serial1 Loopback Test");
  Serial.println("-------------------------------------");
  Serial.println("Connect Nano Pin D0 (RX1) to Nano Pin D1 (TX1) with a jumper wire.");
  Serial.println("Starting test...");

  // Initialize Serial1 (RX on D0, TX on D1 for Nano 33 BLE)
  Serial1.begin(9600);
}

void loop() {
  // Prepare the character to send (let's make it change over time)
  char charToSend = testChar + (testCounter % 26); // Cycle through A-Z

  Serial.print("Test #");
  Serial.print(testCounter + 1);
  Serial.print(": Sending '");
  Serial.print(charToSend);
  Serial.print("' out on Serial1 TX (Pin D1)... ");

  // Clear any old data in Serial1 RX buffer before sending
  while(Serial1.available()) {
    Serial1.read();
  }

  // Send the character
  size_t bytesSent = Serial1.write(charToSend);

  if (bytesSent == 1) {
    Serial.print("Sent. ");
  } else {
    Serial.print("Send FAILED. ");
  }

  delay(100); // Short delay to allow the character to loop back through the wire

  if (Serial1.available() > 0) {
    char receivedChar = Serial1.read();
    Serial.print("Received '");
    Serial.print(receivedChar);
    Serial.print("' on Serial1 RX (Pin D0). ");

    if (receivedChar == charToSend) {
      Serial.println("LOOPBACK SUCCESS!");
      // Quick blink for success
      digitalWrite(ledPin, HIGH);
      delay(50);
      digitalWrite(ledPin, LOW);
    } else {
      Serial.println("LOOPBACK FAILED - Character mismatch!");
      // Longer blink for error
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
    }
  } else {
    Serial.println("LOOPBACK FAILED - Nothing received on Serial1 RX (Pin D0).");
    // Solid ON for a moment for this specific failure
    digitalWrite(ledPin, HIGH);
    delay(300);
    digitalWrite(ledPin, LOW);
  }

  Serial.println("-------------------------------------");
  testCounter++;
  delay(2000); // Wait 2 seconds before the next test
}