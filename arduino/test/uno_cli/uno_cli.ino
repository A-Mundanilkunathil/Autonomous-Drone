// Arduino UNO - Flight Controller CLI Motor Control
// Controls drone motors by sending CLI commands to flight controller

#include <SoftwareSerial.h>

// Software serial port for flight controller communication
// Connect Arduino D2 -> FC TX line
// Connect Arduino D3 -> FC RX line
SoftwareSerial fcSerial(10, 11); // RX, TX

#define MSP_SET_RAW_RC 200
#define MSP_ARM 210          // Custom command for arming

String inputBuffer = "";
bool cliMode = false;

unsigned long lastRcUpdate = 0;
const unsigned long RC_UPDATE_INTERVAL = 20; // 50Hz update rate

// Global variables for MSP v2 override
bool overrideModeActive = false;
uint16_t lastRoll = 1500;
uint16_t lastPitch = 1500;
uint16_t lastThrottle = 1000;
uint16_t lastYaw = 1500;
uint16_t lastAux1 = 2000;
uint16_t lastAux2 = 2000;
uint16_t lastAux3 = 2000;
uint16_t lastAux4 = 1000;

void setup() {
  // Initialize serial communication with computer
  Serial.begin(115200);
  Serial.println(F("Arduino Flight Controller CLI Motor Test"));
  Serial.println(F("Commands:"));
  Serial.println(F("  cli       - Enter CLI mode on flight controller"));
  Serial.println(F("  exit      - Exit CLI mode"));
  Serial.println(F("  motor X Y - Run motor X (0-3) at speed Y (1000-2000)"));
  Serial.println(F("  test      - Run test sequence on all motors"));
  
  // Initialize serial connection to flight controller
  fcSerial.begin(115200);
  delay(1000);
}

void loop() {
  // Read from flight controller and print to Serial Monitor
  while (fcSerial.available()) {
    char c = fcSerial.read();
    Serial.write(c);
  }

  // Read commands from Serial Monitor
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
  
  // Add RC update at fixed interval to prevent failsafe
  unsigned long currentTime = millis();
  if (currentTime - lastRcUpdate >= RC_UPDATE_INTERVAL) {
    // Only when override mode is active
    if (overrideModeActive) {
      // Send last known RC values using MSPv2
      sendRcValuesV2(lastRoll, lastPitch, lastThrottle, lastYaw, 
                    lastAux1, lastAux2, lastAux3, lastAux4);
    }
    lastRcUpdate = currentTime;
  }
}

void processCommand(String command) {
  command.trim();
  
  // Process commands
  if (command == "cli") {
    Serial.println(F("Entering CLI mode..."));
    fcSerial.write('#');
    fcSerial.write('\r');
    fcSerial.write('\n');
    cliMode = true;
  }
  else if (command == "exit") {
    Serial.println(F("Exiting CLI mode..."));
    fcSerial.println("exit");
    cliMode = false;
  }
  else if (command == "test") {
    runMotorTest();
  }
  else if (command.startsWith("motor ")) {
    // Forward command directly to FC
    fcSerial.println(command);
    Serial.println("Sent: " + command);
  }
  else if (command.startsWith("allmotors ")) {
    // Example: "allmotors 1200"
    int speed = command.substring(10).toInt();
    runAllMotors(speed);
  }
  else if (command == "hover") {
    hover(1200); // Base hover at 20% power
  }
  else if (command == "forward") {
    pitchForward(1200, 500);
  }
  else if (command == "backward") {
    pitchBackward(1200, 500);
  }
  else if (command == "left") {
    rollLeft(1200, 500);
  }
  else if (command == "right") {
    rollRight(1200, 500);
  }
  else if (command == "cw") {
    yawClockwise(1200, 500);
  }
  else if (command == "ccw") {
    yawCounterClockwise(1200, 500);
  }
  else if (command == "stop") {
    runAllMotors(1000); // Minimum throttle/stop
    fcSerial.println("disarm");
  }
  else if (command == "msptest") {
    testMspControl();
  }
  else if (command == "mspramp") {
    testMspRamp();
  }
  else if (command == "mspdebug") {
    debugMsp();
  }
  else if (command == "bypass") {
    bypassArming();
  }
  else if (command == "altarm") {
    altArming();
  }
  else if (command == "inavarm") {
    inavSpecificArm();
  }
  else if (command == "armstatus") {
    checkArmingStatus();
  }
  else if (command == "mspsetup") {
    setupMspRcOverride();
  }
  else if (command == "testoverride") {
    testMspRcOverride();
  }
  else if (command == "msptestarm") {
    fixMspSetup();
    delay(1000);
    armWithMspOverride();
  }
  else if (command == "mspv2test") {
    testMspV2Override();
  }
  else {
    // Forward all other commands directly to FC
    fcSerial.println(command);
  }
}

void runMotorTest() {
  if (!cliMode) {
    Serial.println(F("Enter CLI mode first with 'cli' command"));
    return;
  }
  
  // First ensure we're in the right mode and armed
  fcSerial.println("mixer load quad_x");
  delay(500);
  
  // Try to arm (essential step!)
  fcSerial.println("arm");
  Serial.println(F("Attempting to arm..."));
  delay(2000);
  
  Serial.println(F("Running motor test sequence..."));
  
  // Test each motor
  for (int motor = 0; motor < 4; motor++) {
    // Start motor at 10% power
    Serial.print(F("Testing motor ")); 
    Serial.print(motor);
    Serial.println(F(" at 10% power"));
    
    String motorCmd = "motor " + String(motor) + " 1100";
    fcSerial.println(motorCmd);
    delay(2000);  // Run for 2 seconds
    
    // Stop motor
    fcSerial.println("motor " + String(motor) + " 1000");
    delay(1000);  // Wait 1 second between motors
  }
  
  // Disarm when done
  fcSerial.println("disarm");
  Serial.println(F("Motor test complete, disarmed"));
}

void runAllMotors(int speed) {
  if (!cliMode) {
    Serial.println(F("Enter CLI mode first with 'cli' command"));
    return;
  }
  
  // Set all motors to the same speed
  fcSerial.println("motor 0 " + String(speed));
  fcSerial.println("motor 1 " + String(speed));
  fcSerial.println("motor 2 " + String(speed));
  fcSerial.println("motor 3 " + String(speed));
  Serial.print(F("All motors set to "));
  Serial.println(speed);
}

void hover(int power) {
  if (!cliMode) {
    Serial.println(F("Enter CLI mode first with 'cli' command"));
    return;
  }
  
  // Make sure we're armed
  fcSerial.println("arm");
  delay(500);
  
  // All motors at same power for hovering
  runAllMotors(power);
}

void pitchForward(int basePower, int difference) {
  if (!cliMode) {
    Serial.println(F("Enter CLI mode first"));
    return;
  }
  
  // Front motors lower, rear motors higher = pitch forward
  fcSerial.println("motor 0 " + String(basePower - difference)); // Front left
  fcSerial.println("motor 1 " + String(basePower - difference)); // Front right
  fcSerial.println("motor 2 " + String(basePower + difference)); // Rear left
  fcSerial.println("motor 3 " + String(basePower + difference)); // Rear right
  
  Serial.println(F("Pitching forward"));
}

void pitchBackward(int basePower, int difference) {
  if (!cliMode) {
    Serial.println(F("Enter CLI mode first"));
    return;
  }
  
  // Front motors higher, rear motors lower = pitch backward
  fcSerial.println("motor 0 " + String(basePower + difference)); // Front left
  fcSerial.println("motor 1 " + String(basePower + difference)); // Front right
  fcSerial.println("motor 2 " + String(basePower - difference)); // Rear left
  fcSerial.println("motor 3 " + String(basePower - difference)); // Rear right
  
  Serial.println(F("Pitching backward"));
}

void rollLeft(int basePower, int difference) {
  if (!cliMode) {
    Serial.println(F("Enter CLI mode first"));
    return;
  }
  
  // Right motors higher, left motors lower = roll left
  fcSerial.println("motor 0 " + String(basePower - difference)); // Front left
  fcSerial.println("motor 1 " + String(basePower + difference)); // Front right
  fcSerial.println("motor 2 " + String(basePower - difference)); // Rear left
  fcSerial.println("motor 3 " + String(basePower + difference)); // Rear right
  
  Serial.println(F("Rolling left"));
}

void rollRight(int basePower, int difference) {
  if (!cliMode) {
    Serial.println(F("Enter CLI mode first"));
    return;
  }
  
  // Left motors higher, right motors lower = roll right
  fcSerial.println("motor 0 " + String(basePower + difference)); // Front left
  fcSerial.println("motor 1 " + String(basePower - difference)); // Front right
  fcSerial.println("motor 2 " + String(basePower + difference)); // Rear left
  fcSerial.println("motor 3 " + String(basePower - difference)); // Rear right
  
  Serial.println(F("Rolling right"));
}

void yawClockwise(int basePower, int difference) {
  if (!cliMode) {
    Serial.println(F("Enter CLI mode first"));
    return;
  }
  
  // For quad-x configuration:
  // Increase diagonal motors (0,3) and decrease the other diagonal (1,2)
  fcSerial.println("motor 0 " + String(basePower + difference)); // Front left
  fcSerial.println("motor 1 " + String(basePower - difference)); // Front right
  fcSerial.println("motor 2 " + String(basePower - difference)); // Rear left
  fcSerial.println("motor 3 " + String(basePower + difference)); // Rear right
  
  Serial.println(F("Yawing clockwise"));
}

void yawCounterClockwise(int basePower, int difference) {
  if (!cliMode) {
    Serial.println(F("Enter CLI mode first"));
    return;
  }
  
  // Reverse diagonal motor speeds compared to clockwise yaw
  fcSerial.println("motor 0 " + String(basePower - difference)); // Front left
  fcSerial.println("motor 1 " + String(basePower + difference)); // Front right
  fcSerial.println("motor 2 " + String(basePower + difference)); // Rear left
  fcSerial.println("motor 3 " + String(basePower - difference)); // Rear right
  
  Serial.println(F("Yawing counter-clockwise"));
}

void setupMspControl() {
  // Ensure CLI mode is exited first (important!)
  if (cliMode) {
    fcSerial.println("exit");
    cliMode = false;
    delay(500);
  }
  
  Serial.println(F("Switching to MSP control..."));
  delay(1000);
}

void armViaMsp() {
  // First send mid-sticks except low throttle - all modes off
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000);
  delay(1000);
  
  // Now arm AND enable ANGLE mode (both AUX1 and AUX2 high)
  sendRcValues(1500, 1500, 1000, 1500, 1975, 1975, 1000, 1000);
  Serial.println(F("MSP: Arming with ANGLE mode enabled"));
}

// Update your testMspControl() function to use higher throttle
void testMspControl() {
  setupMspControl();
  
  // Arm with ANGLE mode
  armViaMsp();
  delay(3000);
  
  // Higher throttle with ANGLE mode still enabled
  Serial.println(F("MSP: Testing throttle"));
  sendRcValues(1500, 1500, 1300, 1500, 1975, 1975, 1000, 1000); 
  delay(3000);
  
  // Back to idle
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000);
  Serial.println(F("MSP: Test complete"));
}

// Update ramp test too:
void testMspRamp() {
  setupMspControl();
  
  // Arm first with ANGLE mode
  armViaMsp();
  delay(2000);
  Serial.println(F("Armed, ramping up throttle..."));
  
  // Gradually increase throttle (with ANGLE mode enabled)
  for (int throttle = 1050; throttle <= 1400; throttle += 25) { // Try higher max throttle
    Serial.print(F("Throttle: "));
    Serial.println(throttle);
    sendRcValues(1500, 1500, throttle, 1500, 1975, 1975, 1000, 1000);
    delay(1000);
  }
  
  // Disarm at the end
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000);
  Serial.println(F("Test complete, disarmed"));
}

void sendMspCommand(uint8_t command, uint8_t* data, uint8_t dataSize) {
  // Debug output
  inspectMspPacket(command, data, dataSize);
  
  // MSP header
  fcSerial.write('$'); // $
  fcSerial.write('M'); // M
  fcSerial.write('<'); // <
  fcSerial.write(dataSize); // Data length
  fcSerial.write(command); // Command type
  
  // Calculate checksum (XOR of size, command and all data bytes)
  uint8_t checksum = dataSize ^ command;
  
  // Send data if any
  for (uint8_t i = 0; i < dataSize; i++) {
    fcSerial.write(data[i]);
    checksum ^= data[i];
  }
  
  // Send checksum
  fcSerial.write(checksum);
}

// Add MSP v2 support

void sendMspV2Command(uint16_t cmd, uint8_t* data, uint16_t dataSize) {
  // MSP v2 header format: $X<flags><cmd><size><data><crc>
  uint8_t flags = 0;
  
  // Header
  fcSerial.write('$');
  fcSerial.write('X');
  fcSerial.write('<');
  fcSerial.write(flags);
  
  // Command (16-bit in v2)
  fcSerial.write(cmd & 0xFF);  // low byte
  fcSerial.write((cmd >> 8) & 0xFF);  // high byte
  
  // Size (16-bit in v2)
  fcSerial.write(dataSize & 0xFF);  // low byte
  fcSerial.write((dataSize >> 8) & 0xFF);  // high byte
  
  // Calculate CRC
  uint8_t crc = 0;
  crc ^= flags;
  crc ^= (cmd & 0xFF);
  crc ^= ((cmd >> 8) & 0xFF);
  crc ^= (dataSize & 0xFF);
  crc ^= ((dataSize >> 8) & 0xFF);
  
  // Data + update CRC
  for (uint16_t i = 0; i < dataSize; i++) {
    fcSerial.write(data[i]);
    crc ^= data[i];
  }
  
  // Send CRC
  fcSerial.write(crc);
  
  // Debug
  Serial.print(F("MSPv2 Command: "));
  Serial.print(cmd);
  Serial.print(F(", Size: "));
  Serial.println(dataSize);
}

// Updated function for sending RC channels via MSPv2
void sendRcValuesV2(uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw,
                   uint16_t aux1, uint16_t aux2, uint16_t aux3, uint16_t aux4) {
  uint8_t data[16];
  
  // Fill data same as before (little endian)
  data[0] = roll & 0xFF;
  data[1] = (roll >> 8) & 0xFF;
  data[2] = pitch & 0xFF;
  data[3] = (pitch >> 8) & 0xFF;
  data[4] = throttle & 0xFF;
  data[5] = (throttle >> 8) & 0xFF;
  data[6] = yaw & 0xFF;
  data[7] = (yaw >> 8) & 0xFF;
  data[8] = aux1 & 0xFF;
  data[9] = (aux1 >> 8) & 0xFF;
  data[10] = aux2 & 0xFF;
  data[11] = (aux2 >> 8) & 0xFF;
  data[12] = aux3 & 0xFF;
  data[13] = (aux3 >> 8) & 0xFF;
  data[14] = aux4 & 0xFF;
  data[15] = (aux4 >> 8) & 0xFF;
  
  // Send using MSPv2 (MSP_SET_RAW_RC = 200)
  sendMspV2Command(200, data, 16);
}

// Function to send RC values using MSPv1 protocol
void sendRcValues(uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw,
                 uint16_t aux1, uint16_t aux2, uint16_t aux3, uint16_t aux4) {
  uint8_t data[16];
  
  // Fill data (little endian format)
  data[0] = roll & 0xFF;
  data[1] = (roll >> 8) & 0xFF;
  data[2] = pitch & 0xFF;
  data[3] = (pitch >> 8) & 0xFF;
  data[4] = throttle & 0xFF;
  data[5] = (throttle >> 8) & 0xFF;
  data[6] = yaw & 0xFF;
  data[7] = (yaw >> 8) & 0xFF;
  data[8] = aux1 & 0xFF;
  data[9] = (aux1 >> 8) & 0xFF;
  data[10] = aux2 & 0xFF;
  data[11] = (aux2 >> 8) & 0xFF;
  data[12] = aux3 & 0xFF;
  data[13] = (aux3 >> 8) & 0xFF;
  data[14] = aux4 & 0xFF;
  data[15] = (aux4 >> 8) & 0xFF;
  
  // Send using MSPv1 (MSP_SET_RAW_RC = 200)
  sendMspCommand(MSP_SET_RAW_RC, data, 16);
}

void debugMsp() {
  Serial.println(F("MSP Debug - Sending test packets..."));
  
  // First just a simple idle packet with no modes
  Serial.println(F("1. Sending idle packet (no arm, no modes)"));
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000);
  delay(1000);
  
  // Try arming only
  Serial.println(F("2. Sending ARM only"));
  sendRcValues(1500, 1500, 1000, 1500, 1975, 1000, 1000, 1000);
  delay(3000);
  
  // Try with ARM and ANGLE mode
  Serial.println(F("3. Sending ARM + ANGLE mode"));
  sendRcValues(1500, 1500, 1000, 1500, 1975, 1975, 1000, 1000);
  delay(3000);
  
  // Try with throttle
  Serial.println(F("4. Sending ARM + ANGLE + throttle 1400"));
  sendRcValues(1500, 1500, 1400, 1500, 1975, 1975, 1000, 1000);
  delay(3000);
  
  // Back to idle
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000);
  Serial.println(F("Debug complete"));
}

// Add this new MSP debugging function

void bypassArming() {
  setupMspControl();
  
  // First try to go to idle position with throttle down
  Serial.println(F("Setting idle position..."));
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000);
  delay(2000);
  
  // Then try arming by moving throttle down+left (standard stick arm)
  // while also setting AUX1 high
  Serial.println(F("Attempting stick arming + AUX1..."));
  sendRcValues(1000, 1500, 1000, 1000, 1975, 1975, 1000, 1000);
  delay(4000);
  
  // Try throttle
  Serial.println(F("Testing throttle..."));
  sendRcValues(1500, 1500, 1100, 1500, 1975, 1975, 1000, 1000);
  delay(3000);
  
  // Higher throttle
  Serial.println(F("Testing higher throttle..."));
  sendRcValues(1500, 1500, 1200, 1500, 1975, 1975, 1000, 1000);
  delay(3000);
  
  // Back to idle
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000);
  Serial.println(F("Test complete"));
}

// Add this function for an alternative arming method

void altArming() {
  setupMspControl();
  
  // First set all channels to middle, throttle low
  Serial.println(F("Sending neutral signals..."));
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000);
  delay(2000);
  
  // Set throttle to lowest and yaw left for arming
  Serial.println(F("Sending arm command (yaw left, throttle down)..."));
  sendRcValues(1500, 1500, 1000, 1000, 1975, 1975, 1000, 1000);
  delay(4000);
  
  // Return yaw to center, keep throttle low
  Serial.println(F("Maintaining arm with neutral sticks..."));
  sendRcValues(1500, 1500, 1000, 1500, 1975, 1975, 1000, 1000);
  delay(2000);
  
  // Try throttle up
  for (int throttle = 1100; throttle <= 1500; throttle += 50) {
    Serial.print(F("Throttle: "));
    Serial.println(throttle);
    sendRcValues(1500, 1500, throttle, 1500, 1975, 1975, 1000, 1000);
    delay(1500);
  }
  
  // Return to neutral
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000);
  Serial.println(F("Test complete"));
}

// Add this function to properly arm INAV with correct AUX channel values

void inavSpecificArm() {
  setupMspControl();
  
  // Step 1: Start with safe values, AUX channels OFF
  Serial.println(F("Setting safe baseline..."));
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000);
  delay(1000);
  
  // Step 2: Send precise ARM value (AUX1 = 2000) - MUST be over 1975
  Serial.println(F("Sending ARM command (AUX1=2000)..."));
  sendRcValues(1500, 1500, 1000, 1500, 2000, 2000, 1000, 1000);
  delay(3000);
  
  // Step 3: Now add throttle (with ARM still active)
  Serial.println(F("Adding throttle with ARM active..."));
  sendRcValues(1500, 1500, 1200, 1500, 2000, 2000, 1000, 1000);
  delay(2000);
  
  // Step 4: More throttle
  Serial.println(F("Increasing throttle..."));
  sendRcValues(1500, 1500, 1400, 1500, 2000, 2000, 1000, 1000);
  delay(2000);
  
  // Back to safe values
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000);
  Serial.println(F("Test complete"));
}

// Add diagnostic function to show arming issues

void checkArmingStatus() {
  if (!cliMode) {
    Serial.println(F("Entering CLI mode..."));
    fcSerial.write('#');
    fcSerial.write('\r');
    fcSerial.write('\n');
    cliMode = true;
    delay(500);
  }
  
  Serial.println(F("Checking arming status:"));
  fcSerial.println("status");
  delay(1000);
  
  Serial.println(F("Checking all modes:"));
  fcSerial.println("modes");
  delay(1000);
  
  Serial.println(F("Checking motors:"));
  fcSerial.println("get min_throttle");
  fcSerial.println("get motor_pwm_protocol");
  delay(1000);
}

// Add this function for MSP RC OVERRIDE setup

void setupMspRcOverride() {
  if (!cliMode) {
    Serial.println(F("Entering CLI mode..."));
    fcSerial.write('#');
    fcSerial.write('\r');
    fcSerial.write('\n');
    cliMode = true;
    delay(500);
  }
  
  // Show current configuration
  Serial.println(F("Current MSP RC Override settings:"));
  fcSerial.println("get msp_override_channels");
  delay(500);
  fcSerial.println("aux");
  delay(1000);
  
  // Configure MSP RC OVERRIDE on AUX7
  Serial.println(F("Setting MSP RC OVERRIDE on AUX7 (1700-2100):"));
  fcSerial.println("aux 0 50 6 1700 2100"); // Mode ID 50 for MSP_RC_OVERRIDE
  delay(500);
  
  // Configure channels to override (binary 00001111 = first 4 channels)
  Serial.println(F("Setting channels to override (Roll, Pitch, Throttle, Yaw):"));
  fcSerial.println("set msp_override_channels=15");
  delay(500);
  
  // Save configuration
  Serial.println(F("Saving configuration:"));
  fcSerial.println("save");
  delay(1000);
  
  Serial.println(F("MSP RC OVERRIDE configured successfully!"));
}

// Add this function to test MSP RC Override

void testMspRcOverride() {
  setupMspControl();
  
  // First activate MSP RC OVERRIDE mode (AUX7 high)
  Serial.println(F("Activating MSP RC OVERRIDE mode (AUX7=2000)"));
  sendRcValues(1500, 1500, 1000, 1500, 1000, 1000, 2000, 1000);
  delay(1000);
  
  // Now simulate autonomous control
  Serial.println(F("MSP Override: Testing forward movement..."));
  sendRcValues(1500, 1600, 1300, 1500, 2000, 2000, 2000, 1000);
  delay(3000);
  
  Serial.println(F("MSP Override: Testing left turn..."));
  sendRcValues(1400, 1500, 1300, 1500, 2000, 2000, 2000, 1000);
  delay(3000);
  
  // Back to safe values but keep override active
  sendRcValues(1500, 1500, 1000, 1500, 2000, 2000, 2000, 1000);
  Serial.println(F("MSP RC Override test complete"));
}

// Add this function before sendMspCommand()

void inspectMspPacket(uint8_t command, uint8_t* data, uint8_t dataSize) {
  // Print MSP command details for debugging
  Serial.print(F("MSP Command: "));
  Serial.print(command);
  Serial.print(F(", Data size: "));
  Serial.println(dataSize);
  
  // Print data bytes in hex format
  if (dataSize > 0) {
    Serial.print(F("Data: "));
    for (uint8_t i = 0; i < dataSize; i++) {
      if (data[i] < 16) Serial.print(F("0")); // Add leading zero for values < 16
      Serial.print(data[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println();
  }
  
  // For MSP_SET_RAW_RC (200), decode and show channel values
  if (command == MSP_SET_RAW_RC && dataSize >= 16) {
    Serial.println(F("RC Channel values:"));
    for (int i = 0; i < 8; i++) {
      uint16_t value = data[i*2] | (data[i*2+1] << 8);
      Serial.print(F("CH"));
      Serial.print(i+1);
      Serial.print(F(": "));
      Serial.println(value);
    }
  }
}

void fixMspSetup() {
  // Exit CLI mode first if needed
  if (cliMode) {
    fcSerial.println("exit");
    delay(500);
    cliMode = false;
  }

  // Enter CLI to configure
  fcSerial.write('#');
  fcSerial.write('\r');
  fcSerial.write('\n');
  delay(500);
  cliMode = true;

  // Critical settings for MSP RC OVERRIDE
  fcSerial.println("set msp_override_channels=15"); // First 4 channels
  delay(200);
  fcSerial.println("aux 0 50 6 1700 2100"); // MSP_RC_OVERRIDE on AUX7
  delay(200);
  
  // Fix throttle validation settings
  fcSerial.println("set rx_min_usec=885"); // More permissive min value
  delay(200);
  fcSerial.println("set rx_max_usec=2115"); // More permissive max value
  delay(200);
  fcSerial.println("set min_throttle=1000"); // Ensure min throttle is standard
  delay(200);
  
  // Save and exit CLI
  fcSerial.println("save");
  delay(1000);
  fcSerial.println("exit");
  delay(500);
  cliMode = false;
}

void armWithMspOverride() {
  // Start with all channels at safe values
  sendRcValues(1500, 1500, 1000, 1500, 1000, 2000, 2000, 1000);
  delay(1000);
  
  // Ensure throttle is EXACTLY min_throttle (1000)
  Serial.println("Step 1: Setting throttle to minimum");
  
  // Enable ANGLE mode first
  Serial.println("Step 2: Enabling ANGLE mode");
  
  // Now arm with throttle at absolute minimum
  Serial.println("Step 3: Arming with throttle at minimum");
  sendRcValues(1500, 1500, 1000, 1500, 2000, 2000, 2000, 1000);
  delay(2000);
  
  // Now safely increase throttle
  Serial.println("Step 4: Slowly increasing throttle");
  for (int i = 1000; i <= 1200; i += 10) {
    sendRcValues(1500, 1500, i, 1500, 2000, 2000, 2000, 1000);
    delay(100);
  }
  
  Serial.println("Hovering at 1200 throttle");
  delay(3000);
  
  // Return to safe values but keep armed
  Serial.println("Returning to idle throttle");
  sendRcValues(1500, 1500, 1000, 1500, 2000, 2000, 2000, 1000);
  delay(1000);
  
  // Disarm
  Serial.println("Disarming");
  sendRcValues(1500, 1500, 1000, 1500, 1000, 2000, 2000, 1000);
}

// Add this function to test MSP v2 RC Override

void testMspV2Override() {
  setupMspControl();
  
  // Store these values globally and update them in the loop
  lastRoll = 1500;
  lastPitch = 1500;  
  lastThrottle = 1000;
  lastYaw = 1500;
  lastAux1 = 2000;  // ARM
  lastAux2 = 2000;  // ANGLE mode
  lastAux3 = 2000;  // MSP_RC_OVERRIDE
  lastAux4 = 1000;
  
  Serial.println(F("MSPv2: Starting continuous override..."));
  overrideModeActive = true;
  
  // The loop() function will now continuously send updates
}