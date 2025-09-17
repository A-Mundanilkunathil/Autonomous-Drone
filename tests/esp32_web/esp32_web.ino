#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// Camera pin config for ESP32-CAM AI-Thinker
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
#define LED_GPIO 33  // LED pin

const char* ssid = "";     // Replace with your WiFi SSID
const char* password = ""; // Replace with your WiFi password

WebServer server(80);

String htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Drone Cam Control</title>
  <style>
    body {
      font-family: 'Segoe UI', sans-serif;
      background-color: #f2f2f2;
      text-align: center;
      margin: 0;
      padding: 20px;
    }

    h2 {
      margin-bottom: 10px;
    }

    .video-feed {
      margin: 0 auto 20px;
      border: 4px solid #444;
      border-radius: 10px;
      overflow: hidden;
      max-width: 360px;
    }

    img {
      width: 100%;
      height: auto;
    }

    .controls {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      max-width: 360px;
      margin: 0 auto;
    }

    .controls button {
      padding: 12px;
      font-size: 18px;
      border: none;
      border-radius: 10px;
      background-color: #2196f3;
      color: white;
      cursor: pointer;
      transition: background-color 0.2s ease;
    }

    .controls button:hover {
      background-color: #1976d2;
    }

    .controls .emergency {
      grid-column: span 3;
      background-color: #f44336;
    }

    .controls .emergency:hover {
      background-color: #d32f2f;
    }
  </style>
</head>
<body>

  <h2>🚁 Drone Controller</h2>

  <div class="video-feed">
    <img src="/stream" alt="Drone Camera Feed">
  </div>

  <div class="controls">
    <button onclick="send('w')">⬆️</button>
    <button onclick="send('r')">➕ Speed</button>
    <button onclick="send('s')">⬇️</button>

    <button onclick="send('a')">⬅️</button>
    <button onclick="send('f')">➖ Speed</button>
    <button onclick="send('d')">➡️</button>

    <button class="emergency" onclick="send('x')">🛑 EMERGENCY STOP</button>
  </div>

  <!-- Joystick Container -->
  <div id="joystick-zone" style="width:200px;height:200px;margin:20px auto;"></div>

  <!-- Include nipple.js -->
  <script src="https://cdn.jsdelivr.net/npm/nipplejs@0.9.0/dist/nipplejs.min.js"></script>

  <script>
    function send(cmd) {
      fetch(`/cmd?val=${cmd}`);
    }
    
    // Initialize the joystick
    const joystick = nipplejs.create({
        zone: document.getElementById('joystick-zone'),
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: 'blue',
        size: 150
    });

    let lastDir = ''; // Variable to store the last direction

    // Event listeners for joystick movements
    joystick.on('dir', function (evt, data) {
        // Check if the joystick is moved in a direction
        if (data.direction && data.direction.angle !== lastDir) {
            lastDir = data.direction.angle; // Update the last direction
            let cmd = '';

            switch (data.direction.angle) {
                case 'up': cmd = 'w'; break;
                case 'down': cmd = 's'; break;
                case 'left': cmd = 'a'; break;
                case 'right': cmd = 'd'; break;
            }
            if (cmd) send(cmd);
        }
    });

    // Event listener for joystick release
    joystick.on('end', function () {
        lastDir = '';
    });

  </script>

</body>
</html>
)rawliteral";

// Camera stream handler
void handleStream() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (1) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) break;

    response = "--frame\r\nContent-Type: image/jpeg\r\n\r\n";
    server.sendContent(response);
    server.sendContent((const char*)fb->buf, fb->len);
    server.sendContent("\r\n");
    esp_camera_fb_return(fb);

    if (!client.connected()) break;
  }
}

// Command handler
void handleCommand() {
  String cmd = server.arg("val");
  Serial.write(cmd[0]); // Send to Arduino Nano via UART

  // Blink LED
  digitalWrite(LED_GPIO, HIGH);
  delay(100);
  digitalWrite(LED_GPIO, LOW);
  
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(9600); // UART to Nano

  pinMode(LED_GPIO, OUTPUT); // LED pin
  digitalWrite(LED_GPIO, LOW); // Make sure LED is off at start

  // Camera init
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  if (!esp_camera_init(&config)) {
    Serial.println("Camera init success");
  } else {
    Serial.println("Camera init failed");
    return;
  }

  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  // Add LED blink while connecting
  int connectionAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && connectionAttempts < 20) { // 20 * 500ms = 10 second timeout
    digitalWrite(LED_GPIO, !digitalRead(LED_GPIO)); // Toggle LED
    delay(500);
    Serial.print(".");
    connectionAttempts++;
  }
  digitalWrite(LED_GPIO, LOW); // LED off
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Optional: Flash LED to indicate successful connection
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_GPIO, HIGH);
      delay(100);
      digitalWrite(LED_GPIO, LOW);
      delay(100);
    }
  } else {
    Serial.println();
    Serial.println("WiFi connection failed. Check credentials or network availability.");
  }

  // Web server routes
  server.on("/", []() {
    server.send(200, "text/html", htmlPage);
  });

  server.on("/cmd", handleCommand);
  server.on("/stream", handleStream);

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();
  
  // Optional: Check WiFi connection and reconnect if needed
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting to reconnect...");
    WiFi.reconnect();
  }
}
