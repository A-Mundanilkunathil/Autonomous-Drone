#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>

WiFiUDP udp;
uint32_t frame_id = 0;

// ===== Default UDP settings =====
char custom_ip[16] = "192.168.1.89";
char custom_port[6] = "5005";
WiFiManagerParameter ip_param("target_ip", "Target IP", custom_ip, 16);
WiFiManagerParameter port_param("target_port", "Target Port", custom_port, 6);

IPAddress target_ip;
int target_port;

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

struct PacketHeader {
  uint32_t frame_id;
  uint16_t packet_num;
  uint16_t total_packets;
  uint32_t frame_size;
  uint16_t data_size;
} __attribute__((packed));

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ===== WiFiManager setup =====
  WiFiManager wm;
  wm.addParameter(&ip_param);
  wm.addParameter(&port_param);
  if (!wm.autoConnect("ESP32-CAM-Setup")) {
    Serial.println("Failed to connect & timeout");
    ESP.restart();
  }

  target_ip.fromString(String(ip_param.getValue()));
  target_port = atoi(port_param.getValue());

  Serial.print("Target IP: "); Serial.println(target_ip);
  Serial.print("Target port: "); Serial.println(target_port);

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
  config.jpeg_quality = 25;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while(true);
  }

  udp.begin(target_port);
  Serial.println("Camera initialized!");
}

void loop() {
  
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    return;
  }

  frame_id++;
 
  const int max_data_per_packet = 1446;  
  uint16_t total_packets = (fb->len + max_data_per_packet - 1) / max_data_per_packet;
  
  for (uint16_t packet_num = 0; packet_num < total_packets; packet_num++) {
    PacketHeader header;
    header.frame_id = frame_id;
    header.packet_num = packet_num;
    header.total_packets = total_packets;
    header.frame_size = fb->len;
    
    int offset = packet_num * max_data_per_packet;
    int bytes_left = fb->len - offset;
    header.data_size = (bytes_left > max_data_per_packet) ? max_data_per_packet : bytes_left;
    
    udp.beginPacket(target_ip, target_port);
    udp.write((uint8_t*)&header, sizeof(header));
    udp.write(fb->buf + offset, header.data_size);
    udp.endPacket();
  }

  esp_camera_fb_return(fb);
}

