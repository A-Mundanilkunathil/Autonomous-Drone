#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ===== Direct UDP target settings =====
WiFiUDP udp;
IPAddress target_ip(192,168,4,2);   // client IP after connecting to ESP32 AP
int target_port = 5005;             // UDP port to send to

QueueHandle_t frame_queue;
TaskHandle_t udp_task;

volatile uint32_t frame_id = 0; // Shared between task

struct FrameData {
  uint8_t* data;
  size_t length;
  uint32_t id;
  camera_fb_t* fb_ptr;
};

struct PacketHeader {
  uint32_t frame_id;
  uint16_t packet_num;
  uint16_t total_packets;
  uint32_t frame_size;
  uint16_t data_size;
} __attribute__((packed));

// Camera task for capturing frames
void cameraTask(void* parameter) {
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(1);
      continue;
    }

    // Check if queue has space
    if (uxQueueSpacesAvailable(frame_queue) > 0) {
      FrameData frame_data;
      frame_data.length = fb->len;
      frame_data.id = ++frame_id;
      frame_data.data = fb->buf;
      frame_data.fb_ptr = fb;

      xQueueSend(frame_queue, &frame_data, 0);
    } else {
      // If queue full, drop frame
      esp_camera_fb_return(fb);
    }
  }
}

// UDP task handle transmission
void udpTransmissionTask(void* parameter) {
  FrameData frame_data;

  while (true) {
    if (xQueueReceive(frame_queue, &frame_data, portMAX_DELAY)) {
      sendFrameUDP(frame_data.data, frame_data.length, frame_data.id);
      esp_camera_fb_return(frame_data.fb_ptr);
    }
  }
}

// ===== Camera pins =====
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

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ===== Start ESP32 in AP mode =====
  const char* ssid = "ESP32-CAM-AP";
  const char* password = "12345678";   // at least 8 chars
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("ESP32-CAM AP IP address: ");
  Serial.println(myIP);

  // ===== Camera configuration =====
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 15;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while(true);
  }

  udp.begin(target_port);
  Serial.println("Camera initialized, ready to stream!");

  frame_queue = xQueueCreate(2, sizeof(FrameData)); // Queue for 2 frames
  xTaskCreatePinnedToCore(cameraTask, "Camera", 4096, NULL, 3, NULL, 1); // Camera task on core 1
  xTaskCreatePinnedToCore(udpTransmissionTask, "UDP", 8192, NULL, 1, &udp_task, 0); // UDP task on core 0
}

void loop() {    
    static unsigned long lastReport = 0;
    if (millis() - lastReport > 3000) {
      Serial.printf("Frame ID: %lu, Queue spaces: %d\n", 
                     frame_id, uxQueueSpacesAvailable(frame_queue));
      lastReport = millis();
    }
}

void sendFrameUDP(uint8_t* data, size_t length, uint32_t id) {
  const int max_data_per_packet = 1446;
  uint16_t total_packets = (length + max_data_per_packet - 1) / max_data_per_packet;

  for (uint16_t packet_num = 0; packet_num < total_packets; packet_num++) {
      PacketHeader header;
      header.frame_id = id;
      header.packet_num = packet_num;
      header.total_packets = total_packets;
      header.frame_size = length;

      int offset = packet_num * max_data_per_packet;
      int bytes_left = length - offset;
      header.data_size = (bytes_left > max_data_per_packet) ? max_data_per_packet : bytes_left;

      udp.beginPacket(target_ip, target_port);
      udp.write((uint8_t*)&header, sizeof(header));
      udp.write(data + offset, header.data_size);
      udp.endPacket();
  }
}
