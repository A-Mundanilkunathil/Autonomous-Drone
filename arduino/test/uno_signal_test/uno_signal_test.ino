#include <WiFi.h>

#define WIFI_SSID "YOUR_SSID"         // ✅ Replace with your Wi-Fi SSID
#define WIFI_PASSWORD "YOUR_PASSWORD" // ✅ Replace with your Wi-Fi Password

#define LED_PIN 33 // Adjust if your ESP32 uses a different onboard LED GPIO

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[BOOT] Starting ESP32 Wi-Fi Diagnostic...");

  // Step 1: Scan for available networks
  Serial.println("[SCAN] Scanning for Wi-Fi networks...");
  int networkCount = WiFi.scanNetworks();

  if (networkCount == 0) {
    Serial.println("[FAIL] No Wi-Fi networks found. Check antenna or hardware.");
    return;
  } else {
    Serial.printf("[INFO] Found %d Wi-Fi networks.\n", networkCount);
    for (int i = 0; i < networkCount; ++i) {
      Serial.printf("  %d: %s (RSSI: %d, Encryption: %s)\n",
        i + 1,
        WiFi.SSID(i).c_str(),
        WiFi.RSSI(i),
        WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "Open" : "Secured");
    }
  }

  // Step 2: Set station mode
  if (!WiFi.mode(WIFI_STA)) {
    Serial.println("[FAIL] Could not set WiFi mode to STA.");
    return;
  }

  // Step 3: Connect to Wi-Fi
  Serial.printf("[CONNECT] Connecting to \"%s\"...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  delay(2000); // Let Wi-Fi stack do its thing

  wl_status_t status = WiFi.status();

  if (status != WL_CONNECTED) {
    Serial.printf("[FAIL] Connection failed with status: %d\n", status);
    switch (status) {
      case WL_NO_SSID_AVAIL: Serial.println("[REASON] SSID not found."); break;
      case WL_CONNECT_FAILED: Serial.println("[REASON] Wrong password or no response."); break;
      case WL_DISCONNECTED: Serial.println("[REASON] Module is disconnected."); break;
      default: Serial.println("[REASON] Unknown failure."); break;
    }
    return;
  }

  Serial.println("[SUCCESS] Connected to Wi-Fi.");
  Serial.print("[INFO] IP Address: ");
  Serial.println(WiFi.localIP());

  // Step 4: Peripheral setup - LED test
  Serial.println("[SETUP] Running LED blink test...");
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 5; ++i) {
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(300);
  }

  Serial.println("[DONE] Setup complete. Entering loop.");
}

void loop() {
  // You can add additional diagnostics or background tasks here.
  delay(2000);
  Serial.println("[LOOP] Still running...");
}
