#include <MAVLink.h>
#include <WiFi.h>

const char* ssid = "HP";
const char* password = "4085948901";

WiFiServer server(80);
String debugLog = "";

// Serial connection to flight controller
#define FC_SERIAL Serial2
#define FC_BAUD 115200

// MAVLink system configuration
#define SYSTEM_ID 255
#define COMPONENT_ID MAV_COMP_ID_MISSIONPLANNER
#define TARGET_SYSTEM 1
#define TARGET_COMPONENT 1

// Timing variables
unsigned long lastHeartbeat = 0;
unsigned long lastRequest = 0;
unsigned long lastDebug = 0;
int testStep = 0;
bool connectionEstablished = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Connect to WiFi for debugging
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  server.begin();
  
  addToLog("MAVLink 2.0 ESP32-CAM Test Starting...");
  addToLog("WiFi IP: " + WiFi.localIP().toString());
  addToLog("Open browser to view debug info");
  
  // Setup MAVLink serial
  FC_SERIAL.begin(FC_BAUD, SERIAL_8N1, 14, 15);
  
  addToLog("Connecting to Flight Controller...");
}

void loop() {
  // Handle web clients for debugging
  handleWebClients();
  
  // Send heartbeat every 1 second
  if (millis() - lastHeartbeat > 1000) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
  
  // Run test sequence every 10 seconds
  if (millis() - lastRequest > 10000) {
    if (connectionEstablished) {
      runTestSequence();
    }
    lastRequest = millis();
  }
  
  // Debug info every 5 seconds
  if (millis() - lastDebug > 5000) {
    String debugInfo = "Available bytes: " + String(FC_SERIAL.available()) + 
                      ", Connection: " + (connectionEstablished ? "YES" : "NO");
    addToLog(debugInfo);
    lastDebug = millis();
  }
  
  // Read MAVLink messages
  receiveMessages();
  
  delay(50);
}

void addToLog(String message) {
  debugLog += "[" + String(millis()) + "] " + message + "<br>";
  // Keep only last 50 lines
  int lineCount = 0;
  for (int i = 0; i < debugLog.length(); i++) {
    if (debugLog.substring(i, i+4) == "<br>") lineCount++;
  }
  if (lineCount > 50) {
    int firstBr = debugLog.indexOf("<br>");
    debugLog = debugLog.substring(firstBr + 4);
  }
}

void handleWebClients() {
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();
    
    // Send HTML response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();
    
    client.println("<!DOCTYPE html><html>");
    client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
    client.println("<meta http-equiv=\"refresh\" content=\"2\">"); // Auto refresh every 2 seconds
    client.println("<title>ESP32-CAM MAVLink Debug</title></head>");
    client.println("<body><h1>MAVLink Debug Log</h1>");
    client.println("<div style=\"font-family: monospace; font-size: 12px;\">");
    client.println(debugLog);
    client.println("</div></body></html>");
    
    client.stop();
  }
}

void sendHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_heartbeat_pack(
    SYSTEM_ID, COMPONENT_ID, &msg,
    MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
    MAV_MODE_MANUAL_DISARMED, 0, MAV_STATE_ACTIVE
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  FC_SERIAL.write(buf, len);
  
  addToLog("Heartbeat sent (" + String(len) + " bytes) - Magic: 0x" + String(buf[0], HEX));
}

void runTestSequence() {
  addToLog("Test Step: " + String(testStep));
  
  switch(testStep) {
    case 0: requestSystemInfo(); break;
    case 1: requestParameter("SYSID_THISMAV"); break;
    case 2: sendSimpleCommand(); break;
    default: testStep = -1; break;
  }
  testStep++;
}

void requestSystemInfo() {
  addToLog("Requesting system status...");
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_command_long_pack(
    SYSTEM_ID, COMPONENT_ID, &msg,
    TARGET_SYSTEM, TARGET_COMPONENT,
    MAV_CMD_REQUEST_MESSAGE, 0,
    MAVLINK_MSG_ID_SYS_STATUS, 0, 0, 0, 0, 0, 0
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  FC_SERIAL.write(buf, len);
}

void requestParameter(const char* param_id) {
  addToLog("Requesting parameter: " + String(param_id));
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_param_request_read_pack(
    SYSTEM_ID, COMPONENT_ID, &msg,
    TARGET_SYSTEM, TARGET_COMPONENT, param_id, -1
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  FC_SERIAL.write(buf, len);
}

void sendSimpleCommand() {
  addToLog("Sending simple command...");
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_command_long_pack(
    SYSTEM_ID, COMPONENT_ID, &msg,
    TARGET_SYSTEM, TARGET_COMPONENT,
    MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0,
    1, 0, 0, 0, 0, 0, 0
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  FC_SERIAL.write(buf, len);
}

void receiveMessages() {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while(FC_SERIAL.available() > 0) {
    uint8_t c = FC_SERIAL.read();
    
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      connectionEstablished = true;
      handleMessage(&msg);
    }
  }
}

void handleMessage(mavlink_message_t* msg) {
  String logMsg = "*** MAVLink MSG " + String(msg->msgid) + 
                  " from Sys:" + String(msg->sysid) + 
                  " Comp:" + String(msg->compid) + " ***";
  addToLog(logMsg);
  
  switch(msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      mavlink_heartbeat_t heartbeat;
      mavlink_msg_heartbeat_decode(msg, &heartbeat);
      addToLog("HEARTBEAT - Type:" + String(heartbeat.type) + 
               " AP:" + String(heartbeat.autopilot) + 
               " Mode:" + String(heartbeat.base_mode));
      break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
      mavlink_sys_status_t sys_status;
      mavlink_msg_sys_status_decode(msg, &sys_status);
      addToLog("SYS_STATUS - Battery:" + String(sys_status.voltage_battery) + "mV");
      break;
    }
    case MAVLINK_MSG_ID_TIMESYNC: {
      addToLog("TIMESYNC (MAVLink 2)");
      break;
    }
    default:
      addToLog("Other message ID: " + String(msg->msgid));
      break;
  }
}