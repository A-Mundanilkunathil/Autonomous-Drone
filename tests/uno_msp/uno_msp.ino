#define MIN_THROTTLE 1200  // Higher value to ensure motors spin

// MSP Protocol with Flight Mode Control
#define MSP_SET_RAW_RC 200
#define MSP_SET_RAW_RC_RATE 30 // Send heartbeat even more frequently (about 33Hz)

// RC channels: [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
uint16_t channels[8] = {1500, 1500, 1000, 1500, 1000, 2000, 1000, 1000};

// Flight mode flags
bool armed = false;
bool angleMode = false;
bool posHoldMode = false;
bool rthMode = false;
bool altHoldMode = false;

unsigned long lastDebugTime = 0; // Add after your variables

void setup() {
  Serial.begin(115200);
  delay(5000); // Wait for FC boot
}

void loop() {
  // Debug output every second
  if (millis() - lastDebugTime > 1000) {
    Serial.print("Armed: ");
    Serial.print(armed ? "YES" : "NO");
    Serial.print(" | Throttle: ");
    Serial.print(channels[2]);
    Serial.print(" | Angle mode: ");
    Serial.println(angleMode ? "ON" : "OFF");
    lastDebugTime = millis();
  }

  // Process serial commands if available
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case 'a': // Toggle arm
        armed = !armed;
        
        if (armed) {
          // First set angle mode
          channels[5] = 2000;
          sendMSP_SET_RAW_RC();
          delay(200);
          
          // Now arm with min throttle
          channels[4] = 2000; // AUX1
          channels[2] = 1000; // Minimum throttle
          sendMSP_SET_RAW_RC();
          delay(800); // Wait for arming to complete
          
          // Only then increase throttle
          channels[2] = MIN_THROTTLE;
          Serial.println("ARMED with throttle " + String(MIN_THROTTLE));
        } else {
          channels[4] = 1000;
          channels[2] = 1000;
          Serial.println("DISARMED");
        }
        break;
      case 'h': // Toggle RTH
        rthMode = !rthMode;
        channels[7] = rthMode ? 2000 : 1000; // AUX4
        break;
      case 'm': // Toggle angle mode
        angleMode = !angleMode;
        channels[5] = angleMode ? 2000 : 1000; // AUX2
        break;
      // Basic control commands
      case 'u': channels[2] += 50; break; // Throttle up
      case 'd': channels[2] -= 50; break; // Throttle down
      case 'f': channels[1] = 1650; break; // Forward/ AUX3
      case 'b': channels[1] = 1350; break; // Back
      case 'l': channels[0] = 1350; break; // Left
      case 'r': channels[0] = 1650; break; // Right
      case 'c': // Center sticks
        channels[0] = 1500;
        channels[1] = 1500;
        channels[3] = 1500;
        break;
      case 'x': // Execute full test sequence
        Serial.println("\n==== MOTOR TEST SEQUENCE ====");
        
        // Step 1: Disarm and reset
        armed = false;
        channels[4] = 1000;
        channels[2] = 1000;
        sendMSP_SET_RAW_RC();
        delay(1000);
        Serial.println("1. Reset & Disarmed");
        
        // Step 2: Enable angle mode
        angleMode = true;
        channels[5] = 2000;
        sendMSP_SET_RAW_RC();
        delay(1000);
        Serial.println("2. Angle mode ON");
        
        // Step 3: Arm with higher throttle
        armed = true;
        channels[4] = 2000; 
        channels[2] = 1100; // CRITICAL: Start with higher throttle
        sendMSP_SET_RAW_RC();
        delay(2000);
        Serial.println("3. ARMED with throttle at 1100");
        
        // Step 4: Gradually increase to verify
        for (int t = 1100; t <= 1300; t += 20) {
          channels[2] = t;
          sendMSP_SET_RAW_RC();
          Serial.println("Throttle: " + String(t));
          delay(500);
        }
        
        // Step 5: Hold for 3 seconds
        Serial.println("Holding at 1300...");
        delay(3000);
        
        // Step 6: Return to safe state
        channels[2] = 1000;
        sendMSP_SET_RAW_RC();
        delay(1000);
        channels[4] = 1000; // Disarm
        armed = false;
        sendMSP_SET_RAW_RC();
        Serial.println("Test complete - DISARMED");
        break;
      case 't': // Throttle test
        // Ensure armed state first
        if (!armed) {
          Serial.println("Arming first...");
          armed = true;
          channels[4] = 2000; // Arm
          channels[5] = 2000; // Angle mode
          channels[2] = 1000; // Min throttle
          sendMSP_SET_RAW_RC();
          delay(1000);
        }
        
        // Ramp up throttle slowly
        Serial.println("Testing throttle...");
        for (int i = 1000; i <= 1300; i += 10) {
          channels[2] = i;
          sendMSP_SET_RAW_RC();
          Serial.print("Throttle: ");
          Serial.println(i);
          delay(100);
        }
        
        // Hold for 2 seconds
        delay(2000);
        
        // Return to min
        channels[2] = 1000;
        sendMSP_SET_RAW_RC();
        Serial.println("Test complete");
        break;
      case 'g': // GO - start motors after arming
        if (armed) {
          Serial.println("Starting motors...");
          // Ensure angle mode is on
          channels[5] = 2000;
          // Set throttle to spin-up value
          channels[2] = MIN_THROTTLE;
          sendMSP_SET_RAW_RC();
        } else {
          Serial.println("Not armed - arm first with 'a'");
        }
        break;
      case 'v': // Diagnostic test - verify communication
        Serial.println("\n--- MSP DIAGNOSTIC ---");
        // 1. Send an MSP request for FC version
        Serial.write('$');
        Serial.write('M');
        Serial.write('<');
        Serial.write(0); // No payload
        Serial.write(102); // MSP_VERSION
        Serial.write(102); // Checksum
        Serial.flush();
        
        // 2. Show any response
        delay(100);
        Serial.print("FC response: ");
        int bytesAvailable = 0;
        unsigned long startTime = millis();
        while (millis() - startTime < 500) {
          if (Serial.available()) {
            bytesAvailable++;
            Serial.print(Serial.read(), HEX);
            Serial.print(" ");
          }
        }
        if (bytesAvailable == 0) {
          Serial.println("NO RESPONSE - Check wiring!");
        } else {
          Serial.println("\nFound " + String(bytesAvailable) + " bytes");
        }
        break;
      case 'z': // Zero to full test sequence
        Serial.println("ADVANCED TEST SEQUENCE");
        // Disarm first
        armed = false;
        channels[4] = 1000;
        channels[2] = 1000;
        sendMSP_SET_RAW_RC();
        delay(1000);
        
        // Enable angle mode first!
        channels[5] = 2000; 
        sendMSP_SET_RAW_RC();
        delay(500);
        
        // Arm with MIN throttle
        armed = true;
        channels[4] = 2000;
        channels[2] = 1000; // Start with ZERO throttle
        sendMSP_SET_RAW_RC();
        delay(1000);
        
        // CRITICAL: Now gradually ramp up throttle in small steps
        for (int t = 1000; t <= 1500; t += 10) {
          channels[2] = t;
          sendMSP_SET_RAW_RC();
          
          // Display progress
          if (t % 50 == 0) {
            Serial.println("Throttle: " + String(t));
          }
          delay(50);  // Slower ramp = better for ESC initialization
        }
        
        // Hold at 1500 (50% throttle) briefly
        Serial.println("Holding at 1500...");
        delay(2000);
        
        // Gradually decrease throttle
        for (int t = 1500; t >= 1000; t -= 20) {
          channels[2] = t;
          sendMSP_SET_RAW_RC();
          if (t % 100 == 0) {
            Serial.println("Throttle: " + String(t));
          }
          delay(50);
        }
        
        // Disarm
        channels[4] = 1000;
        channels[2] = 1000;
        armed = false;
        sendMSP_SET_RAW_RC();
        Serial.println("Test complete - disarmed");
        break;
    } 
    
    // Constrain throttle
    channels[2] = constrain(channels[2], 1000, 2000);
  }    
  
  // Always send MSP commands at consistent rate
  sendMSP_SET_RAW_RC();
  delay(20); // 50Hz update rate
}

void sendMSP_SET_RAW_RC() {
  // Clear any pending input data
  while (Serial.available()) {
    Serial.read();
  }
  
  // MSP header
  Serial.write('$');
  Serial.write('M');
  Serial.write('<');
  
  // Payload size (16 bytes for 8 channels)
  Serial.write(16);
  
  // Command type
  Serial.write(MSP_SET_RAW_RC);
  
  // Calculate checksum
  uint8_t checksum = 16 ^ MSP_SET_RAW_RC;
  
  // Send channel data (8 channels)
  for (int i = 0; i < 8; i++) {
    // Send LOW byte first (VERY important for INAV)
    Serial.write(channels[i] & 0xFF);
    checksum ^= (channels[i] & 0xFF);
    
    // Then send HIGH byte
    Serial.write((channels[i] >> 8) & 0xFF);
    checksum ^= ((channels[i] >> 8) & 0xFF);
  }
  
  // Send checksum
  Serial.write(checksum);
  
  // Ensure data is sent immediately
  Serial.flush();
}