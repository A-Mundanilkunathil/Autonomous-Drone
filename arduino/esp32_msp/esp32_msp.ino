#include "esp_camera.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ESP32-CAM AI-Thinker pin configuration
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
#define LED_GPIO          33

// Hardware serial for MSP communication with flight controller
HardwareSerial FCSerial(1);

// WiFi credentials
const char* ssid = "HP";
const char* password = "4085948901";

AsyncWebServer server(80); // Create web server on port 80

// MSP protocol constants
#define MSP_SET_RAW_RC 200

// Auto-reset timing for directional controls
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 200;
bool needsReset = false;

// Emergency landing system variables
bool emergencyMode = false;
unsigned long emergencyStartTime = 0;
uint16_t emergencyStartThrottle = 1000;

// Computer vision mode variables
bool cvModeActive = false;
String lastCVCommand = "none";
unsigned long lastCVTime = 0;

// Shared variables with mutex protection
SemaphoreHandle_t rcMutex;
uint16_t rc[8] = {1000, 1500, 1500, 1500, 1000, 1000, 1000, 1000};

// Camera status
bool cameraWorking = false;

// Task handle for camera task
TaskHandle_t cameraTaskHandle = NULL;
camera_fb_t* volatile currentFrame = NULL;
SemaphoreHandle_t frameMutex = NULL;


/**
 * Generates an HTML page as a string for the drone camera controller interface.
 * The page includes sections for video feed, control buttons, and emergency controls.
 * It dynamically adjusts the camera status section based on the camera's working state.
 * 
 * @return A string containing the complete HTML for the drone control interface.
 */

String getHtmlPage() {
  String cameraSection = "";
  
  if (cameraWorking) {
    cameraSection = R"(
  <div class="video-feed">
    <img src="/stream" alt="Drone Camera Feed">
  </div>)";
  } else {
    cameraSection = R"(
  <div class="video-feed" style="background-color: #333; color: white; display: flex; align-items: center; justify-content: center; height: 300px;">
    <div>
      <h3>📹 Camera Unavailable</h3>
      <p>I2C/SCCB Communication Error</p>
      <p>Drone control still functional</p>
    </div>
  </div>)";
  }

  return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Drone Camera Control</title>
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
      color: #333;
    }

    .video-feed {
      margin: 0 auto 20px;
      border: 4px solid #444;
      border-radius: 10px;
      overflow: hidden;
      max-width: 400px;
    }

    img {
      width: 100%;
      height: auto;
    }

    .controls {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 15px;
      max-width: 360px;
      margin: 0 auto 20px;
    }

    .emergency-controls {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 10px;
      max-width: 360px;
      margin: 0 auto;
    }

    .controls button, .emergency-controls button {
      padding: 15px;
      font-size: 18px;
      border: none;
      border-radius: 10px;
      color: white;
      cursor: pointer;
      transition: background-color 0.2s ease;
    }

    .controls button {
      background-color: #2196f3;
    }

    .controls button:hover {
      background-color: #1976d2;
    }

    .controls .arm {
      background-color: #4caf50;
    }

    .controls .arm:hover {
      background-color: #45a049;
    }

    .controls .disarm {
      background-color: #f44336;
      grid-column: span 3;
    }

    .controls .disarm:hover {
      background-color: #d32f2f;
    }

    .controls .throttle {
      background-color: #ff9800;
    }

    .controls .throttle:hover {
      background-color: #f57c00;
    }

    .emergency-controls .emergency {
      background-color: #ff5722;
      font-weight: bold;
    }

    .emergency-controls .emergency:hover {
      background-color: #d84315;
    }

    .emergency-controls .force {
      background-color: #b71c1c;
      font-weight: bold;
    }

    .emergency-controls .force:hover {
      background-color: #8d1515;
    }

    .warning {
      background-color: #ffeb3b;
      color: #333;
      padding: 10px;
      border-radius: 5px;
      margin: 10px 0;
      font-weight: bold;
    }

    .status {
      background-color: )rawliteral" + String(cameraWorking ? "#4caf50" : "#ff5722") + R"rawliteral(
      color: white;
      padding: 10px;
      border-radius: 5px;
      margin: 10px 0;
      font-weight: bold;
    }
  </style>
</head>
<body>

  <h2>🚁 Drone Camera Controller</h2>
  
  <div class="status">
    📡 System Status: )rawliteral" + String(cameraWorking ? "Camera + Control Active" : "Control Only (Camera Failed)") + R"rawliteral(
  </div>

)rawliteral" + cameraSection + R"rawliteral(

  <div class="controls">
    <button class="arm" onclick="send('arm')">ARM ⚡</button>
    <button class="throttle" onclick="send('up')">UP ⬆️</button>
    <button class="throttle" onclick="send('down')">DOWN ⬇️</button>

    <button onclick="send('left')">LEFT ⬅️</button>
    <button onclick="send('fwd')">FWD ⬆️</button>
    <button onclick="send('right')">RIGHT ➡️</button>

    <div></div>
    <button onclick="send('back')">BACK ⬇️</button>
    <div></div>

    <button class="disarm" onclick="send('disarm')">🛑 SMART DISARM</button>
  </div>

  <div class="warning">
    ⚠️ Emergency Controls - Use Only When Necessary
  </div>

  <div class="emergency-controls">
    <button class="emergency" onclick="confirmAction('emergency_land', '🚨 Emergency Land?')">
      🚨 EMERGENCY LAND
    </button>
    <button class="force" onclick="confirmAction('force_disarm', '⚠️ FORCE DISARM? (Dangerous!)')">
      ⚡ FORCE DISARM
    </button>
  </div>

  <script>
    function send(cmd) {
      fetch(`/${cmd}`)
        .then(response => response.text())
        .then(data => {
          console.log(data);
          if (data.includes('EMERGENCY')) {
            alert('🚨 ' + data);
          }
        })
        .catch(error => console.error('Error:', error));
    }

    function confirmAction(cmd, message) {
      if (confirm(message)) {
        send(cmd);
      }
    }
  </script>

</body>
</html>
)rawliteral";
}

/**
 * @brief Schedules a reset of the directional controls.
 * 
 * Sets the needsReset flag to true and records the current time
 * to lastCommandTime, which is used to determine when the reset should
 * occur based on a timeout. This function ensures that the directional
 * controls are returned to a neutral state after a specified period
 * without receiving commands.
 */

void scheduleReset() {
  needsReset = true;
  lastCommandTime = millis();
}


/**
 * @brief Sends a MultiWii Serial Protocol (MSP) command to set raw RC values.
 * 
 * @details This function constructs a payload containing the RC channel values
 * and sends it using the MSP protocol to the flight controller. It ensures 
 * synchronized access to the RC values using a semaphore. The function first 
 * packs the RC values into a payload, computes a checksum, and then sends the 
 * command along with the payload and checksum over the serial interface. 
 * Semaphore access is managed to ensure thread-safe operations.
 */

void sendMSP() {
  // Ensure thread-safe access to the rc array
  if (xSemaphoreTake(rcMutex, portMAX_DELAY)) {
    uint8_t payload[16]; // MSP payload for 8 RC channels (2 bytes each)
    
    // Pack RC values into payload
    for (int i = 0; i < 8; i++) {
      payload[2 * i] = rc[i] & 0xFF; // Lower byte
      payload[2 * i + 1] = rc[i] >> 8; // Upper byte
    }

    // Send MSP command
    FCSerial.write('$'); // Start of MSP command
    FCSerial.write('M'); // MSP identifier
    FCSerial.write('<'); // Start of MSP command
    FCSerial.write(16); // Length of payload (16 bytes for 8 channels)
    FCSerial.write(MSP_SET_RAW_RC); 

    // Calculate checksum
    uint8_t checksum = 16 ^ MSP_SET_RAW_RC; 
    for (int i = 0; i < 16; i++) {
      FCSerial.write(payload[i]);
      checksum ^= payload[i];
    }
    FCSerial.write(checksum);
    FCSerial.flush();
    
    xSemaphoreGive(rcMutex); // Release the mutex after sending
  }
}


/**
 * @brief Initializes the camera module.
 *
 * @details This function initializes the camera module. It deinitializes the
 * camera first if it was previously initialized. It then sets the camera
 * configuration optimized for the ESP32-CAM module. The function also powers
 * cycles the camera if the PWDN pin is available. Finally, it initializes the
 * camera, sets the camera sensor adjustments, and tests the capture. If the
 * test capture fails, it deinitializes the camera and returns false. Otherwise,
 * it returns true.
 *
 * @return true if the camera was successfully initialized, false otherwise.
 */
bool initializeCamera() {
  Serial.println("🔧 Initializing camera...");
  
  // Deinitialize camera first if it was previously initialized
  esp_camera_deinit();
  delay(100);

  // Maximum CPU speed for better processing
  setCpuFrequencyMhz(240);
  
  // Camera configuration for ESP32-CAM AI-Thinker
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
  
  // Adjust camera settings for better stability
  config.xclk_freq_hz = 16000000;  // Reduced frequency for stability
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST; // Use latest frame grab mode
  config.frame_size = FRAMESIZE_VGA;  // Start with small frame
  config.jpeg_quality = 15;  // Lower number = better quality
  config.fb_count = 2;  // Use single frame buffer for lower memory usage
  config.fb_location = CAMERA_FB_IN_PSRAM;

  // Power cycle the camera if PWDN pin is defined
  if (PWDN_GPIO_NUM != -1) {
    pinMode(PWDN_GPIO_NUM, OUTPUT); // Set PWDN pin as output
    digitalWrite(PWDN_GPIO_NUM, HIGH); // Power on the camera
    delay(10);
    digitalWrite(PWDN_GPIO_NUM, LOW);
    delay(10);
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed with error 0x%x\n", err);
    return false;
  }

  if (err == ESP_OK) {
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
      // Brightness and exposure settings
      s->set_brightness(s, 2); // Set brightness to 2
      s->set_contrast(s, 2); // Set contrast to 2
      s->set_saturation(s, 1); // Set saturation to 1

      // White balance and exposure settings
      s->set_whitebal(s, 1); // Enable white balance
      s->set_exposure_ctrl(s, 1); // Enable exposure control
      s->set_aec2(s, 1); // Enable AEC2 for better exposure
      s->set_gain_ctrl(s, 1); // Enable gain control
      s->set_agc_gain(s, 5); // Enable AGC gain

      s->set_ae_level(s, 2); // Set AE level to 2

      s->set_lenc(s, 1); // Enable lens correction
      s->set_raw_gma(s, 1); // Enable raw gain

      Serial.println("✅ Camera initialized successfully");
    }
  }

  // Test capture
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    Serial.printf("✅ Camera initialized! Frame size: %dx%d, length: %d\n", 
                  fb->width, fb->height, fb->len);
    esp_camera_fb_return(fb);
    return true;
  } else {
    Serial.println("❌ Test capture failed");
    esp_camera_deinit();
    return false;
  }
}

/**
 * @brief Handle a HTTP request for the camera stream.
 *
 *        This function is registered as a handler for the /stream URL. It
 *        returns a chunked response with the camera stream.
 *
 *        The response format is multipart/x-mixed-replace, which is a
 *        special content type that allows the server to send multiple
 *        images in a single response. Each image is separated by a
 *        boundary string, which is defined as "--frame". The server
 *        sends the boundary string followed by a Content-Type header
 *        and a Content-Length header with the length of the image, and
 *        then the image data.
 *
 * @param request The HTTP request to handle.
 *
 * @return void
 */
void handleStream(AsyncWebServerRequest *request) {
  if (!cameraWorking) {
    request->send(503, "text/plain", "Camera not available");
    return;
  }

  setCpuFrequencyMhz(240); // Max CPU speed during streaming

  // Start chunked response for streaming
  AsyncWebServerResponse *response = request->beginChunkedResponse(
    "multipart/x-mixed-replace; boundary=frame",
    [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      static camera_fb_t *fb = nullptr; // Static pointer to hold the current frame buffer
      static size_t fb_index = 0; // Index within the current frame buffer
      static bool boundary_sent = false; // Flag to track if boundary has been sent

      // Get new frame if needed
      if (fb == nullptr) {
        fb = esp_camera_fb_get(); // Get a new frame buffer

        if (!fb) {
          Serial.println("❌ Camera capture failed");
          return 0; // No data available
        }
        fb_index = 0; // Reset index for new frame
        boundary_sent = false; // Reset boundary sent flag
      }

      size_t bytes_written = 0; // Number of bytes written to the buffer

      // Send boundary first
      if (!boundary_sent) {
        String header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + 
                       String(fb->len) + "\r\n\r\n";
        
        // Check if header fits in the buffer
        if (header.length() <= maxLen) {

          // Copy header to buffer
          memcpy(buffer, header.c_str(), header.length()); 
          bytes_written = header.length(); // Update bytes written
          boundary_sent = true; // Mark boundary as sent
        } else {
          // Buffer too small for header
          esp_camera_fb_return(fb);
          fb = nullptr;
          return 0;
        }
      }
      // Send image data
      else { 
        size_t remaining = fb->len - fb_index; // Remaining bytes in the frame buffer
        size_t to_send = min(remaining, maxLen - bytes_written); // Bytes to send in this iteration
        
        memcpy(buffer + bytes_written, fb->buf + fb_index, to_send);
        fb_index += to_send;
        bytes_written += to_send;

        // Check if frame is complete
        if (fb_index >= fb->len) {
          // Add trailing boundary
          String trailing = "\r\n";
          if (bytes_written + trailing.length() <= maxLen) {
            memcpy(buffer + bytes_written, trailing.c_str(), trailing.length());
            bytes_written += trailing.length();
          }
          
          esp_camera_fb_return(fb);
          fb = nullptr;
        }
      }

      return bytes_written;
    }
  );

  response->addHeader("Access-Control-Allow-Origin", "*");
  response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  response->addHeader("Pragma", "no-cache");
  response->addHeader("Expires", "0");
  response->addHeader("Buffer-Size", "131072");  // Increase from 65536 to 128KB
  
  request->send(response);
  Serial.println("📹 Stream started");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, LOW);
  FCSerial.begin(115200, SERIAL_8N1, 14, 15);

  // Create mutex for thread-safe RC array access
  rcMutex = xSemaphoreCreateMutex();
  
  Serial.println("🔧 ESP32-CAM Drone Controller Starting...");
  
  // Initialize camera with multiple attempts
  cameraWorking = initializeCamera();
  if (!cameraWorking) {
    Serial.println("❌ Camera failed - continuing with control-only mode");
  }

  // WiFi setup
  WiFi.setSleep(false); // Disable WiFi sleep mode
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Set max TX power for better range
  WiFi.begin(ssid, password);
  Serial.print("🔌 Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected: " + WiFi.localIP().toString());

  // ASYNC WEB SERVER SETUP
  
  // Main page - dynamic based on camera status
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", getHtmlPage());
  });

  // Stream endpoint
  server.on("/stream", HTTP_GET, handleStream);

  // Control endpoints - ALL ASYNC!
  server.on("/arm", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Check if mutex is available to modify RC values
    if (xSemaphoreTake(rcMutex, 100 / portTICK_PERIOD_MS)) {
      rc[4] = 2000;
      xSemaphoreGive(rcMutex); // Release mutex after modifying RC values
      sendMSP();
      request->send(200, "text/plain", "Armed");
      Serial.println("✅ Armed");
    } else {
      request->send(503, "text/plain", "System busy");
    }
  });

  server.on("/disarm", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (xSemaphoreTake(rcMutex, 100 / portTICK_PERIOD_MS)) {
      rc[4] = 1000;
      rc[0] = 1000;
      xSemaphoreGive(rcMutex);
      sendMSP();
      request->send(200, "text/plain", "Disarmed");
      Serial.println("✅ Disarmed");
    } else {
      request->send(503, "text/plain", "System busy");
    }
  });

  server.on("/up", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (xSemaphoreTake(rcMutex, 100 / portTICK_PERIOD_MS)) {
      rc[0] = constrain(rc[0] + 50, 1000, 2000);
      xSemaphoreGive(rcMutex);
      sendMSP();
      request->send(200, "text/plain", "Up: " + String(rc[0]));
      Serial.println("✅ Up: " + String(rc[0]));
    } else {
      request->send(503, "text/plain", "System busy");
    }
  });

  server.on("/down", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (xSemaphoreTake(rcMutex, 100 / portTICK_PERIOD_MS)) {
      rc[0] = constrain(rc[0] - 50, 1000, 2000);
      xSemaphoreGive(rcMutex);
      sendMSP();
      request->send(200, "text/plain", "Down: " + String(rc[0]));
      Serial.println("✅ Down: " + String(rc[0]));
    } else {
      request->send(503, "text/plain", "System busy");
    }
  });

  server.on("/fwd", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (xSemaphoreTake(rcMutex, 100 / portTICK_PERIOD_MS)) {
      rc[2] = 1650;
      xSemaphoreGive(rcMutex);
      scheduleReset();
      sendMSP();
      request->send(200, "text/plain", "Forward");
      Serial.println("✅ Forward");
    } else {
      request->send(503, "text/plain", "System busy");
    }
  });

  server.on("/back", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (xSemaphoreTake(rcMutex, 100 / portTICK_PERIOD_MS)) {
      rc[2] = 1350;
      xSemaphoreGive(rcMutex);
      scheduleReset();
      sendMSP();
      request->send(200, "text/plain", "Backward");
      Serial.println("✅ Backward");
    } else {
      request->send(503, "text/plain", "System busy");
    }
  });

  server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (xSemaphoreTake(rcMutex, 100 / portTICK_PERIOD_MS)) {
      rc[1] = 1350;
      xSemaphoreGive(rcMutex);
      scheduleReset();
      sendMSP();
      request->send(200, "text/plain", "Left");
      Serial.println("✅ Left");
    } else {
      request->send(503, "text/plain", "System busy");
    }
  });

  server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (xSemaphoreTake(rcMutex, 100 / portTICK_PERIOD_MS)) {
      rc[1] = 1650;
      xSemaphoreGive(rcMutex);
      scheduleReset();
      sendMSP();
      request->send(200, "text/plain", "Right");
      Serial.println("✅ Right");
    } else {
      request->send(503, "text/plain", "System busy");
    }
  });

  server.on("/emergency_land", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (xSemaphoreTake(rcMutex, 100 / portTICK_PERIOD_MS)) {
      emergencyStartThrottle = rc[0];
      emergencyMode = true;
      emergencyStartTime = millis();
      xSemaphoreGive(rcMutex);
      request->send(200, "text/plain", "EMERGENCY LANDING");
      Serial.println("🚨 Emergency landing started");
    } else {
      request->send(503, "text/plain", "System busy");
    }
  });

  server.on("/force_disarm", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (xSemaphoreTake(rcMutex, 100 / portTICK_PERIOD_MS)) {
      rc[4] = 1000;
      rc[0] = 1000;
      emergencyMode = false;
      xSemaphoreGive(rcMutex);
      sendMSP();
      request->send(200, "text/plain", "FORCE DISARMED");
      Serial.println("⚡ Force disarmed");
    } else {
      request->send(503, "text/plain", "System busy");
    }
  });

  // CV Action endpoint - ASYNC POST
  server.on("/cv/action", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String body = String((char*)data).substring(0, len);
      Serial.println("🎯 CV Action: " + body);
      
      if (xSemaphoreTake(rcMutex, 100 / portTICK_PERIOD_MS)) {
        
        if (body.indexOf("\"action\":\"forward\"") > 0) {
          rc[2] = 1650;
          scheduleReset();
          lastCVCommand = "forward";
        }
        else if (body.indexOf("\"action\":\"backward\"") > 0) {
          rc[2] = 1350;
          scheduleReset();
          lastCVCommand = "backward";
        }
        else if (body.indexOf("\"action\":\"left\"") > 0) {
          rc[1] = 1350;
          scheduleReset();
          lastCVCommand = "left";
        }
        else if (body.indexOf("\"action\":\"right\"") > 0) {
          rc[1] = 1650;
          scheduleReset();
          lastCVCommand = "right";
        }
        else if (body.indexOf("\"action\":\"up\"") > 0) {
          rc[0] = constrain(rc[0] + 50, 1000, 2000);
          lastCVCommand = "up";
        }
        else if (body.indexOf("\"action\":\"down\"") > 0) {
          rc[0] = constrain(rc[0] - 50, 1000, 2000);
          lastCVCommand = "down";
        }
        else if (body.indexOf("\"action\":\"hover\"") > 0) {
          rc[1] = 1500;
          rc[2] = 1500;
          lastCVCommand = "hover";
        }
        
        xSemaphoreGive(rcMutex);
        sendMSP();
        lastCVTime = millis();
        
        Serial.println("✅ CV processed: " + lastCVCommand);
        request->send(200, "application/json", "{\"status\":\"ok\",\"command\":\"" + lastCVCommand + "\"}");
      } else {
        request->send(503, "application/json", "{\"status\":\"error\",\"message\":\"System busy\"}");
      }
    });

  // Camera retry endpoint
  server.on("/camera/retry", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("🔄 Camera retry requested");
    cameraWorking = initializeCamera();
    String status = cameraWorking ? "Camera initialized successfully" : "Camera initialization failed";
    request->send(200, "text/plain", status);
  });

  // Status endpoint
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String status = "{";
    status += "\"camera\":" + String(cameraWorking ? "true" : "false") + ",";
    status += "\"wifi\":\"connected\",";
    status += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    status += "\"emergency\":" + String(emergencyMode ? "true" : "false");
    status += "}";
    request->send(200, "application/json", status);
  });

  // Start async server
  server.begin();
  Serial.println("🚀 AsyncWebServer started!");
  if (cameraWorking) {
    Serial.println("📹 Stream: http://" + WiFi.localIP().toString() + "/stream");
  }
  Serial.println("🎮 Control: http://" + WiFi.localIP().toString());
  Serial.println("🔄 Camera retry: http://" + WiFi.localIP().toString() + "/camera/retry");
  Serial.println("📊 Status: http://" + WiFi.localIP().toString() + "/status");
  
  sendMSP();
  
  // Create camera task
  frameMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    cameraTask,
    "CameraTask",
    4096,
    NULL,
    2,
    &cameraTaskHandle,
    0  // Run on Core 0 (WiFi/BT runs on Core 0, Arduino runs on Core 1)
  );
}

void loop() {
  // Handle reset logic
  if (needsReset && (millis() - lastCommandTime >= commandTimeout)) {
    if (xSemaphoreTake(rcMutex, 10 / portTICK_PERIOD_MS)) {
      rc[1] = 1500;
      rc[2] = 1500;
      rc[3] = 1500;
      xSemaphoreGive(rcMutex);
      sendMSP();
      needsReset = false;
    }
  }

  // Handle emergency landing
  if (emergencyMode) {
    unsigned long emergencyTime = millis() - emergencyStartTime;
    
    // Emergency landing logic
    if (emergencyTime <= 1000) {
      if (xSemaphoreTake(rcMutex, 10 / portTICK_PERIOD_MS)) {
        rc[0] = emergencyStartThrottle;
        rc[1] = 1500;
        rc[2] = 1500;
        rc[3] = 1500;
        xSemaphoreGive(rcMutex);
        sendMSP();
      }
    }

    // Gradual descent logic
    else if (emergencyTime > 1000 && emergencyTime <= 6000) {
      int timeInDescent = emergencyTime - 1000;
      int throttleRange = emergencyStartThrottle - 1000;
      int reduction = (timeInDescent * throttleRange) / 5000;
      int targetThrottle = emergencyStartThrottle - reduction;
      
      if (xSemaphoreTake(rcMutex, 10 / portTICK_PERIOD_MS)) {
        rc[0] = constrain(targetThrottle, 1000, 2000);
        rc[1] = 1500;
        rc[2] = 1500;
        rc[3] = 1500;
        xSemaphoreGive(rcMutex);
        sendMSP();
      }
      
      // Blink LED to indicate emergency descent
      if ((emergencyTime / 500) % 2 == 0) {
        digitalWrite(LED_GPIO, HIGH);
      } else {
        digitalWrite(LED_GPIO, LOW);
      }
      
      // Print target throttle for debugging
      if (emergencyTime % 1000 < 50) {
        Serial.println("Emergency descent: " + String(targetThrottle));
      }
    }

    // Emergency landing complete
    else if (emergencyTime > 6000) {
      if (xSemaphoreTake(rcMutex, 10 / portTICK_PERIOD_MS)) {
        rc[4] = 1000;
        rc[0] = 1000;
        emergencyMode = false;
        xSemaphoreGive(rcMutex);
        sendMSP();
      }
      digitalWrite(LED_GPIO, LOW);
      Serial.println("Emergency landing complete - auto disarmed");
    }
  }
  
  delay(10);
}

/**
 * @brief Task function for continuous camera capture.
 * 
 * This function runs in a loop and captures frames from the camera at a
 * target rate of ~33 frames per second. It uses a mutex to protect access
 * to the shared currentFrame variable, ensuring that the latest frame is
 * always available for streaming. Older frames are returned to the camera
 * driver for reuse. The task runs on Core 0 of the ESP32, separate from
 * the main Arduino loop.
 * 
 * @param parameter Task parameter (not used).
 */
void cameraTask(void *parameter) {
  for(;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      if (xSemaphoreTake(frameMutex, portMAX_DELAY)) {
        if (currentFrame) {
          esp_camera_fb_return(currentFrame);
        }
        currentFrame = fb;
        xSemaphoreGive(frameMutex);
      } else {
        esp_camera_fb_return(fb);
      }
    }
    delay(30);  // ~33 fps target rate
  }
}