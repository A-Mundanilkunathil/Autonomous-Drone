#define UART_TX_PIN 16 // ESP32 TX pin for Serial2
#define UART_RX_PIN -1 // ESP32 RX pin for Serial2 (not used in this sender-only example)

#define ESP32_LED_PIN 2 // Built-in LED on many ESP32 boards, or connect your own

void setup() {
  pinMode(ESP32_LED_PIN, OUTPUT);
  digitalWrite(ESP32_LED_PIN, LOW); // Start with LED off

  Serial.begin(115200); // For ESP32's own debug messages
  Serial2.begin(4800, SERIAL_8N1, UART_TX_PIN, UART_RX_PIN); // CHANGED BAUD RATE
  Serial.println("ESP32 Basic Serial Test Sender. Sending 0xAA 0xBB every second to Uno.");
  Serial.printf("ESP32 TX pin: %d, LED pin: %d. Baud: 4800\n", UART_TX_PIN, ESP32_LED_PIN);
}

void loop() {
  uint8_t byte1 = 0xAA;
  uint8_t byte2 = 0xBB;

  digitalWrite(ESP32_LED_PIN, HIGH); // Turn LED ON before sending
  Serial2.write(byte1);
  Serial2.write(byte2);
  digitalWrite(ESP32_LED_PIN, LOW);  // Turn LED OFF after sending

  Serial.printf("ESP32 Sent: 0x%02X 0x%02X\n", byte1, byte2);
  delay(1000);
}