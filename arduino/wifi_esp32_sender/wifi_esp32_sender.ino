#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "esp_http_server.h"

// CAMERA MODEL: AI-THINKER
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

#define UART_TX_PIN 14
#define UART_RX_PIN -1

#define LED_GPIO 33  // ✅ LED pin

uint16_t testSpeed = 1500;

const char* ssid = "HP";
const char* password = "4085948901";
WebServer server(80);

void sendSpeedToUno(uint16_t speed) {
  Serial2.write(speed & 0xFF);
  Serial2.write((speed >> 8) & 0xFF);
  Serial.printf("Sent speed %d to Uno\n", speed);
}

void handleRoot() {
  String html = "<html><body><h2>Set Motor Speed</h2>"
                "<form action='/set' method='get'>"
                "Speed (700-2000): <input type='number' name='v' min='700' max='2000'><br><br>"
                "<input type='submit' value='Send'></form>"
                "<br><h3>Live Stream</h3>"
                "<img src='http://" + WiFi.localIP().toString() + ":81/stream' width='320'></body></html>";
  server.send(200, "text/html", html);
}

void handleSpeed() {
  if (server.hasArg("v")) {
    int val = server.arg("v").toInt();
    if (val >= 700 && val <= 2000) {
      testSpeed = val;
      sendSpeedToUno(testSpeed);
      digitalWrite(LED_GPIO, HIGH);  // 🔴 LED ON
      delay(200);                    // 🔴 Short blink
      digitalWrite(LED_GPIO, LOW);   // 🔴 LED OFF
      server.send(200, "text/plain", "Speed set to: " + String(testSpeed));
      return;
    }
  }
  server.send(400, "text/plain", "Invalid speed. Must be 700–2000.");
}

httpd_handle_t stream_httpd = NULL;

esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb;
  esp_err_t res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;

    res = httpd_resp_send_chunk(req, "--frame\r\n", strlen("--frame\r\n"));
    if (res == ESP_OK) {
      char buf[64];
      size_t hlen = snprintf(buf, sizeof(buf),
        "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
      res = httpd_resp_send_chunk(req, buf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, UART_TX_PIN, UART_RX_PIN);

  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, LOW); // Make sure LED is off at start

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 15;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    Serial.println(WiFi.localIP());
    startCameraServer();
    server.on("/", handleRoot);
    server.on("/set", handleSpeed);
    server.begin();
    Serial.println("Web interface ready.");
  } else {
    Serial.println("WiFi failed. Restarting...");
    ESP.restart();
  }
}

void loop() {
  server.handleClient();
}
