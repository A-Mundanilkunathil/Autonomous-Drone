#include "esp_camera.h" // Keep for compilation, but init will be commented out
#include <WiFi.h>
#include <WebServer.h>
// #include "esp_http_server.h" // Not strictly needed if only using WebServer for /set

// CAMERA MODEL: AI-THINKER (Definitions kept for completeness, but camera init is disabled for this test)
#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define UART_TX_PIN 14 // ESP32 TX pin for Serial2
#define UART_RX_PIN 15 // ESP32 RX pin for Serial2 LOOPBACK - CHOOSE A FREE GPIO

#define LED_GPIO 33  // LED pin

uint16_t testSpeed = 1500; // Default test speed

const char* ssid = "HP"; // Your WiFi SSID
const char* password = "4085948901"; // Your WiFi Password
WebServer server(80);

// Forward declaration for camera server (though it won't be started in this test)
// httpd_handle_t stream_httpd = NULL;
// void startCameraServer();
// esp_err_t stream_handler(httpd_req_t *req);


void sendDataAndTestLoopback(uint16_t speed) {
  uint8_t lowByte = speed & 0xFF;
  uint8_t highByte = (speed >> 8) & 0xFF;

  Serial.printf("Attempting to send via Serial2 (TX Pin %d): Low=0x%02X, High=0x%02X\n", UART_TX_PIN, lowByte, highByte);
  
  // Clear any old data in Serial2 RX buffer before sending
  while(Serial2.available()) {
    Serial2.read();
  }

  Serial2.write(lowByte);
  Serial2.write(highByte);
  Serial.printf("Data sent. Speed: %d (0x%02X, 0x%02X)\n", speed, lowByte, highByte);

  // Attempt to read back (give a small delay for data to arrive)
  delay(20); // Small delay for data to loop back through the wire

  String receivedMsg = "";
  int bytesRead = 0;
  if (Serial2.available() >= 2) { // Expecting 2 bytes back
    uint8_t recLowByte = Serial2.read();
    uint8_t recHighByte = Serial2.read();
    bytesRead = 2;

    uint16_t receivedSpeed = (recHighByte << 8) | recLowByte;

    Serial.printf("Serial2 Loopback Received (RX Pin %d): Byte1=0x%02X, Byte2=0x%02X. Reconstructed Speed: %d\n", 
                  UART_RX_PIN, recLowByte, recHighByte, receivedSpeed);
    
    if (recLowByte == lowByte && recHighByte == highByte) {
      Serial.println("LOOPBACK TEST: SUCCESS - Bytes match!");
    } else {
      Serial.println("LOOPBACK TEST: FAILED - Bytes do not match.");
    }
  } else if (Serial2.available() > 0) {
    Serial.print("Serial2 Loopback Received (RX Pin ");
    Serial.print(UART_RX_PIN);
    Serial.print("): Incomplete data: ");
     while (Serial2.available() > 0) {
        char c = Serial2.read();
        Serial.print("0x");
        if (c < 0x10) Serial.print("0");
        Serial.print(c, HEX);
        Serial.print(" ");
    }
    Serial.println();
    Serial.println("LOOPBACK TEST: FAILED - Incomplete data.");
  }
  else {
    Serial.printf("Serial2 Loopback (RX Pin %d): Nothing received.\n", UART_RX_PIN);
    Serial.println("LOOPBACK TEST: FAILED - Nothing received.");
  }
  Serial.println("------------------------------------");
}

void handleRoot() {
  String ipAddress = "ESP32_IP_NOT_SET";
  if (WiFi.status() == WL_CONNECTED) {
    ipAddress = WiFi.localIP().toString();
  }
  String html = "<html><body><h2>ESP32 Serial2 Loopback Test</h2>"
                "<h3>Connect GPIO " + String(UART_TX_PIN) + " to GPIO " + String(UART_RX_PIN) + " on the ESP32.</h3>"
                "<form action='/set' method='get'>"
                "Speed (0-65535): <input type='number' name='v' min='0' max='65535' value='" + String(testSpeed) + "'><br><br>"
                "<input type='submit' value='Send & Test Loopback'></form>"
                "<br><h3>Live Stream (Disabled for this test)</h3>"
                // "<img src='http://" + ipAddress + ":81/stream' width='320'>" // Camera stream disabled
                "</body></html>";
  server.send(200, "text/html", html);
}

void handleSpeed() {
  if (server.hasArg("v")) {
    int val = server.arg("v").toInt();
    // For loopback test, allow any uint16_t value
    // if (val >= 700 && val <= 2000) { // Original speed validation
    if (val >= 0 && val <= 65535) { // Allow full uint16_t range for testing
      testSpeed = val;
      sendDataAndTestLoopback(testSpeed); // Call the test function
      
      digitalWrite(LED_GPIO, HIGH);
      delay(100); // Shorter blink for test
      digitalWrite(LED_GPIO, LOW);
      
      server.send(200, "text/plain", "Loopback test triggered for speed: " + String(testSpeed));
      return;
    }
  }
  server.send(400, "text/plain", "Invalid speed. Must be 0-65535 for this test.");
}

/*
// Camera stream handler - Not used in this simplified test
esp_err_t stream_handler(httpd_req_t *req) {
  // ... camera stream code ...
  return ESP_OK; 
}

void startCameraServer() {
  // ... camera server start code ...
}
*/

void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 Serial2 Loopback Test Initializing...");
  Serial.printf("Serial2 TX Pin: %d, Serial2 RX Pin: %d. Wire them together!\n", UART_TX_PIN, UART_RX_PIN);

  // Initialize Serial2 with both TX and RX pins for loopback
  Serial2.begin(9600, SERIAL_8N1, UART_TX_PIN, UART_RX_PIN); 

  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, LOW);

  /*
  // CAMERA INITIALIZATION - COMMENTED OUT FOR SERIAL LOOPBACK TEST
  camera_config_t config;
  // ... (all camera config lines) ...
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed (but ignored for loopback test)");
    // return; // Don't return, continue with WiFi for testing
  } else {
    Serial.println("Camera init success (but not used for loopback test)");
  }
  */
  Serial.println("Camera initialization skipped for Serial2 loopback test.");


  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false); // Optional: disable modem sleep for more responsive web server

  unsigned long startConnect = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startConnect < 15000) { // 15 second timeout
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    // startCameraServer(); // Camera server not started for this test
    
    server.on("/", HTTP_GET, handleRoot);
    server.on("/set", HTTP_GET, handleSpeed);
    server.begin();
    Serial.println("Web interface for loopback test ready.");
  } else {
    Serial.println("\nWiFi connection failed. Please check credentials or network.");
    Serial.println("Restarting in 10 seconds...");
    delay(10000);
    ESP.restart();
  }
  Serial.println("------------------------------------");
}

void loop() {
  server.handleClient();
  // No other tasks in loop for this focused test
}