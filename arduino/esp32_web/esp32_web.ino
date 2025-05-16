#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// Camera pin config for ESP32-CAM AI-Thinker
#define PWDN_GPIO_NUM     -1
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

// Wi-Fi credentials
const char* ssid = "DroneAP";
const char* password = "12345678";

WebServer server(80);

String htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Drone Cam Control</title>
  <style>
    body { font-family: sans-serif; text-align: center; }
    button { width: 100px; height: 40px; font-size: 16px; margin: 5px; }
    img { width: 100%; max-width: 320px; }
  </style>
</head>
<body>
  <h2>Drone Controller</h2>
  <img src="/stream" /><br>
  <button onclick="send('w')">↑</button>
  <button onclick="send('s')">↓</button><br>
  <button onclick="send('a')">←</button>
  <button onclick="send('d')">→</button><br>
  <button onclick="send('r')">+ Speed</button>
  <button onclick="send('f')">- Speed</button><br>
  <button onclick="send('x')">🛑 EMERGENCY</button>

  <script>
    function send(cmd) {
      fetch(`/cmd?val=${cmd}`);
    }
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

  // Start AP
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("AP IP: " + IP.toString());

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
}
