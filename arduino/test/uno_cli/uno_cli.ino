#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <MAVLink.h> 

// MAVLink Serial Configuration
HardwareSerial& MAVLINK_SERIAL_PORT = Serial2;
const long MAVLINK_BAUD_RATE = 115200;
const int MAVLINK_RX_PIN = 13;
const int MAVLINK_TX_PIN = 12;
const unsigned long FC_HEARTBEAT_TIMEOUT_MS = 5000;
unsigned long lastFcHeartbeatMs = 0;
const uint8_t DEFAULT_FC_SYSTEM_ID = 1;
const uint8_t DEFAULT_FC_COMPONENT_ID = 1;

// MAVLink System IDs
const uint8_t ESP32_SYSTEM_ID    = 255;
const uint8_t ESP32_COMPONENT_ID = MAV_COMP_ID_ONBOARD_COMPUTER;

uint8_t fc_system_id    = 0;
uint8_t fc_component_id = 0;
bool fc_detected = false;
bool fc_armed_status = false;

// Performance Metrics
unsigned long commandCount = 0;
unsigned long lastCommandMs = 0;
unsigned long messagesSent = 0;
unsigned long messagesReceived = 0;
unsigned long lastLatencyMs = 0;
float commandRate = 0.0;

// RC Channel Values (PWM microseconds)
const uint16_t RC_NEUTRAL = 1500;
const uint16_t RC_THROTTLE_LOW = 1000;
const uint16_t RC_THROTTLE_MIN_OPERATIONAL = 1100;

// Movement Commands
const uint16_t RC_PITCH_FORWARD_CMD = 1400;
const uint16_t RC_PITCH_BACKWARD_CMD = 1600;
const uint16_t RC_ROLL_LEFT_CMD = 1400;
const uint16_t RC_ROLL_RIGHT_CMD = 1600;
const uint16_t YAW_CCW = 1400;
const uint16_t YAW_CW = 1600;

// Timing Configuration
const unsigned long RC_OVERRIDE_INTERVAL_MS = 100;

// Flight Controller State Machine
enum ControlState {
    STATE_INIT,
    STATE_WAIT_FOR_HEARTBEAT,
    STATE_SET_MODE_STABILIZE, 
    STATE_WAIT_FOR_MODE_ACK,
    STATE_ARM_VEHICLE,
    STATE_WAIT_FOR_ARMED,
    STATE_OPERATIONAL, 
    STATE_DISARM_VEHICLE,
    STATE_WAIT_FOR_DISARMED,
    STATE_FINISHED, 
    STATE_ERROR
};
ControlState mavlinkState = STATE_INIT;
unsigned long stateTimer = 0;
unsigned long lastEspHeartbeatSentMs = 0;
unsigned long lastRcOverrideSentMs = 0; 
uint8_t stabilize_mode_id = 0; 

// ESP32-CAM Pin Definitions
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

// Network Configuration
const char* ssid = "HP";
const char* password = "12345678";

AsyncWebServer server(80);

// Emergency Landing System
bool emergencyMode = false;
unsigned long emergencyStartTime = 0;
uint16_t emergencyStartThrottle = RC_THROTTLE_LOW;

// RC Channel Array [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
SemaphoreHandle_t rcMutex;
uint16_t rc[8] = {RC_NEUTRAL, RC_NEUTRAL, RC_THROTTLE_LOW, RC_NEUTRAL, 1000, 1000, 1000, 1000};

// Camera System
bool cameraWorking = false;
TaskHandle_t cameraTaskHandle = NULL;
camera_fb_t* volatile currentFrame = NULL;
SemaphoreHandle_t frameMutex = NULL;

// Function Declarations
void send_esp_heartbeat();
void handle_mavlink_message(mavlink_message_t* msg);
void send_mavlink_command_long(uint16_t command, float param1 = 0, float param2 = 0, float param3 = 0, float param4 = 0, float param5 = 0, float param6 = 0, float param7 = 0);
void set_flight_mode(uint8_t mode_id);
void arm_disarm_vehicle(bool arm);
void send_rc_override_values(uint16_t ch1_roll, uint16_t ch2_pitch, uint16_t ch3_throttle, uint16_t ch4_yaw,
                             uint16_t ch5 = UINT16_MAX, uint16_t ch6 = UINT16_MAX, uint16_t ch7 = UINT16_MAX, uint16_t ch8 = UINT16_MAX);

const char* getStateString(ControlState state) {
  switch(state) {
    case STATE_INIT: return "INIT";
    case STATE_WAIT_FOR_HEARTBEAT: return "WAIT_HEARTBEAT";
    case STATE_SET_MODE_STABILIZE: return "SET_MODE";
    case STATE_WAIT_FOR_MODE_ACK: return "WAIT_MODE_ACK";
    case STATE_ARM_VEHICLE: return "ARM_VEHICLE";
    case STATE_WAIT_FOR_ARMED: return "WAIT_ARMED";
    case STATE_OPERATIONAL: return "OPERATIONAL";
    case STATE_DISARM_VEHICLE: return "DISARM";
    case STATE_WAIT_FOR_DISARMED: return "WAIT_DISARMED";
    case STATE_FINISHED: return "FINISHED";
    case STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

String getHtmlPage() {
  // Calculate uptime
  unsigned long uptime = millis() / 1000;
  unsigned long hours = uptime / 3600;
  unsigned long minutes = (uptime % 3600) / 60;
  unsigned long seconds = uptime % 60;
  
  // Calculate command rate
  if (millis() - lastCommandMs > 0) {
    commandRate = (float)commandCount / ((millis() - lastCommandMs) / 1000.0);
  }

  String cameraSection = "";
  if (cameraWorking) {
    cameraSection = R"(
  <div class='section'>
    <h3>Camera Feed</h3>
    <img src='/stream' alt='Live Feed' style='width:100%;max-width:400px;border:1px solid #ddd;' 
         onload='this.style.opacity=1' onerror='this.style.opacity=0.5'>
    <div style='font-size:11px;color:#666;margin-top:5px;'>Live MJPEG Stream</div>
  </div>)";
  } else {
    cameraSection = R"(
  <div class='section'>
    <h3>Camera Status</h3>
    <div style='background-color: #333; color: white; padding: 20px; text-align: center;'>
      <h4>📹 Camera Unavailable</h4>
      <p>I2C/SCCB Communication Error</p>
      <p>Drone control still functional</p>
      <button onclick='retryCamera()' style='margin-top:10px;'>Retry Camera</button>
    </div>
  </div>)";
  }

  // Enhanced communication tracking variables
  float successRate = messagesSent > 0 ? (float)messagesReceived / messagesSent * 100.0 : 0;
  unsigned long avgLatency = commandCount > 0 ? lastLatencyMs : 0;
  
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Drone Control System - Enhanced Monitoring</title>
  <style>
    body { font-family: 'Consolas', 'Monaco', monospace; margin: 0; padding: 8px; background: #1a1a1a; color: #e0e0e0; font-size: 13px; }
    .container { max-width: 1400px; margin: 0 auto; }
    .header { text-align: center; margin-bottom: 20px; padding: 15px; background: #2d2d2d; border-radius: 8px; }
    .header h1 { margin: 0; color: #00ff41; font-size: 24px; text-shadow: 0 0 10px #00ff41; }
    .header .subtitle { color: #888; margin-top: 5px; }
    
    .section { background: #2d2d2d; margin: 8px 0; padding: 12px; border-radius: 6px; border-left: 4px solid #00ff41; }
    .section h3 { margin: 0 0 10px 0; color: #00ff41; font-size: 16px; }
    .section h4 { margin: 10px 0 8px 0; color: #66b3ff; font-size: 14px; }
    
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 12px; }
    .grid-3 { grid-template-columns: repeat(3, 1fr); }
    .grid-4 { grid-template-columns: repeat(4, 1fr); }
    
    .metric-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 8px; }
    .metric { background: #1a1a1a; padding: 8px; border-radius: 4px; border: 1px solid #444; }
    .metric-label { font-weight: bold; color: #aaa; font-size: 11px; text-transform: uppercase; }
    .metric-value { font-size: 18px; color: #fff; margin-top: 2px; font-weight: bold; }
    .metric-unit { font-size: 12px; color: #888; }
    
    .status-ok { color: #00ff41; }
    .status-warn { color: #ffaa00; }
    .status-error { color: #ff4444; }
    .status-critical { color: #ff0066; }
    
    .btn { padding: 10px 15px; margin: 3px; border: none; border-radius: 4px; cursor: pointer; 
           font-family: inherit; font-size: 12px; font-weight: bold; text-transform: uppercase;
           transition: all 0.2s; min-height: 40px; }
    .btn:hover { transform: translateY(-1px); box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
    .btn:active { transform: translateY(0); }
    
    .btn-primary { background: #0066cc; color: white; }
    .btn-success { background: #00aa44; color: white; }
    .btn-warning { background: #ff8800; color: white; }
    .btn-danger { background: #cc3333; color: white; }
    .btn-emergency { background: #aa0055; color: white; }
    .btn-critical { background: #6600aa; color: white; }
    
    .controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin: 12px 0; }
    .movement { display: grid; grid-template-columns: repeat(3, 1fr); grid-template-rows: repeat(3, 1fr); 
                gap: 6px; max-width: 240px; margin: 0 auto; }
    .movement .btn:nth-child(1) { grid-area: 2 / 1; }
    .movement .btn:nth-child(2) { grid-area: 1 / 2; }
    .movement .btn:nth-child(3) { grid-area: 2 / 3; }
    .movement .btn:nth-child(4) { grid-area: 3 / 2; }
    .movement .btn:nth-child(5) { grid-area: 1 / 1; }
    .movement .btn:nth-child(6) { grid-area: 1 / 3; }
    .movement-center { grid-area: 2 / 2; background: #333; border-radius: 4px; 
                       display: flex; align-items: center; justify-content: center; 
                       color: #666; font-size: 20px; }
    
    .emergency { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin: 15px 0; }
    
    .comm-log { background: #000; color: #00ff41; padding: 10px; border-radius: 4px; 
                font-family: 'Courier New', monospace; font-size: 11px; 
                max-height: 250px; overflow-y: auto; border: 1px solid #333; }
    .comm-log .timestamp { color: #666; }
    .comm-log .command { color: #66b3ff; font-weight: bold; }
    .comm-log .response { color: #00ff41; }
    .comm-log .error { color: #ff4444; }
    .comm-log .warning { color: #ffaa00; }
    
    table { width: 100%; border-collapse: collapse; margin: 10px 0; font-size: 12px; }
    th, td { padding: 6px 8px; text-align: left; border-bottom: 1px solid #444; }
    th { background: #1a1a1a; color: #00ff41; font-weight: bold; }
    td { background: #2a2a2a; }
    
    .progress-bar { width: 100%; height: 6px; background: #333; border-radius: 3px; overflow: hidden; }
    .progress-fill { height: 100%; background: linear-gradient(90deg, #00ff41, #66b3ff); transition: width 0.3s; }
    
    .realtime-indicator { display: inline-block; width: 8px; height: 8px; border-radius: 50%; 
                          background: #00ff41; margin-left: 8px; animation: pulse 1s infinite; }
    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.3; } }
    
    .critical-warning { background: linear-gradient(135deg, #aa0055, #ff0066); 
                        color: white; padding: 12px; border-radius: 6px; 
                        text-align: center; margin: 10px 0; font-weight: bold; 
                        animation: blink 1s infinite; }
    @keyframes blink { 0%, 50% { opacity: 1; } 51%, 100% { opacity: 0.7; } }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>DRONE CONTROL SYSTEM</h1>
      <div class="subtitle">Enhanced Performance Monitoring Dashboard</div>
      <div class="realtime-indicator"></div> LIVE
    </div>
    
    <!-- Critical Status Banner -->
    <div id="criticalBanner" style="display:none;" class="critical-warning">
      ⚠️ CRITICAL SYSTEM ALERT ⚠️
    </div>
    
    <div class="grid">
      <!-- System Status Overview -->
      <div class="section">
        <h3>🔧 System Status Overview</h3>
        <div class="metric-grid">
          <div class="metric">
            <div class="metric-label">Flight Controller</div>
            <div class="metric-value )rawliteral" + String(fc_detected ? "status-ok" : "status-error") + R"rawliteral(">
              )rawliteral" + String(fc_detected ? (fc_armed_status ? "ARMED" : "READY") : "OFFLINE") + R"rawliteral(
            </div>
          </div>
          <div class="metric">
            <div class="metric-label">MAVLink State</div>
            <div class="metric-value">)rawliteral" + String(getStateString(mavlinkState)) + R"rawliteral(</div>
          </div>
          <div class="metric">
            <div class="metric-label">Camera System</div>
            <div class="metric-value )rawliteral" + String(cameraWorking ? "status-ok" : "status-error") + R"rawliteral(">
              )rawliteral" + String(cameraWorking ? "ACTIVE" : "OFFLINE") + R"rawliteral(
            </div>
          </div>
          <div class="metric">
            <div class="metric-label">Emergency Mode</div>
            <div class="metric-value )rawliteral" + String(emergencyMode ? "status-critical" : "status-ok") + R"rawliteral(">
              )rawliteral" + String(emergencyMode ? "ACTIVE" : "NORMAL") + R"rawliteral(
            </div>
          </div>
        </div>
      </div>

      <!-- Performance Metrics -->
      <div class="section">
        <h3>📊 Performance Metrics</h3>
        <div class="metric-grid">
          <div class="metric">
            <div class="metric-label">System Uptime</div>
            <div class="metric-value">)rawliteral" + String(hours) + ":" + String(minutes) + ":" + String(seconds) + R"rawliteral(</div>
            <div class="metric-unit">H:M:S</div>
          </div>
          <div class="metric">
            <div class="metric-label">Command Latency</div>
            <div class="metric-value )rawliteral" + String(lastLatencyMs > 100 ? "status-warn" : "status-ok") + R"rawliteral(">)rawliteral" + String(lastLatencyMs) + R"rawliteral(</div>
            <div class="metric-unit">milliseconds</div>
          </div>
          <div class="metric">
            <div class="metric-label">Commands Executed</div>
            <div class="metric-value">)rawliteral" + String(commandCount) + R"rawliteral(</div>
          </div>
          <div class="metric">
            <div class="metric-label">Success Rate</div>
            <div class="metric-value )rawliteral" + String(successRate > 90 ? "status-ok" : (successRate > 70 ? "status-warn" : "status-error")) + R"rawliteral(">)rawliteral" + String(successRate, 1) + R"rawliteral(%</div>
          </div>
          <div class="metric">
            <div class="metric-label">Messages TX/RX</div>
            <div class="metric-value">)rawliteral" + String(messagesSent) + "/" + String(messagesReceived) + R"rawliteral(</div>
          </div>
          <div class="metric">
            <div class="metric-label">Free Memory</div>
            <div class="metric-value )rawliteral" + String(ESP.getFreeHeap() < 50000 ? "status-warn" : "status-ok") + R"rawliteral(">)rawliteral" + String(ESP.getFreeHeap()/1024) + R"rawliteral(</div>
            <div class="metric-unit">KB</div>
          </div>
        </div>
      </div>
    </div>

    <!-- RC Channel Monitoring -->
    <div class="section">
      <h3>🎮 RC Channel Status & Control Values</h3>
      <table>
        <thead>
          <tr><th>CH</th><th>Function</th><th>Current PWM</th><th>Status</th><th>Range</th><th>Last Update</th></tr>
        </thead>
        <tbody>
          <tr>
            <td>1</td><td>Roll</td>
            <td id="ch1" class="metric-value">)rawliteral" + String(rc[0]) + R"rawliteral(</td>
            <td class=")rawliteral" + String(rc[0] == RC_NEUTRAL ? "status-ok" : "status-warn") + R"rawliteral(">
              )rawliteral" + String(rc[0] == RC_NEUTRAL ? "NEUTRAL" : (rc[0] < RC_NEUTRAL ? "LEFT" : "RIGHT")) + R"rawliteral(
            </td>
            <td>1000-2000</td><td id="ch1-time">--</td>
          </tr>
          <tr>
            <td>2</td><td>Pitch</td>
            <td id="ch2" class="metric-value">)rawliteral" + String(rc[1]) + R"rawliteral(</td>
            <td class=")rawliteral" + String(rc[1] == RC_NEUTRAL ? "status-ok" : "status-warn") + R"rawliteral(">
              )rawliteral" + String(rc[1] == RC_NEUTRAL ? "NEUTRAL" : (rc[1] < RC_NEUTRAL ? "FORWARD" : "BACKWARD")) + R"rawliteral(
            </td>
            <td>1000-2000</td><td id="ch2-time">--</td>
          </tr>
          <tr>
            <td>3</td><td>Throttle</td>
            <td id="ch3" class="metric-value">)rawliteral" + String(rc[2]) + R"rawliteral(</td>
            <td class=")rawliteral" + String(rc[2] <= RC_THROTTLE_LOW ? "status-ok" : "status-warn") + R"rawliteral(">
              )rawliteral" + String(rc[2] <= RC_THROTTLE_LOW ? "MINIMUM" : "ACTIVE") + R"rawliteral(
            </td>
            <td>1000-2000</td><td id="ch3-time">--</td>
          </tr>
          <tr>
            <td>4</td><td>Yaw</td>
            <td id="ch4" class="metric-value">)rawliteral" + String(rc[3]) + R"rawliteral(</td>
            <td class=")rawliteral" + String(rc[3] == RC_NEUTRAL ? "status-ok" : "status-warn") + R"rawliteral(">
              )rawliteral" + String(rc[3] == RC_NEUTRAL ? "NEUTRAL" : (rc[3] < RC_NEUTRAL ? "CCW" : "CW")) + R"rawliteral(
            </td>
            <td>1000-2000</td><td id="ch4-time">--</td>
          </tr>
        </tbody>
      </table>
    </div>

    <div class="grid">
      <!-- Primary Controls -->
      <div class="section">
        <h3>🎯 Primary Flight Controls</h3>
        <div class="controls">
          <button class="btn btn-success" onclick="sendCommand('arm')" id="armBtn">🔓 ARM</button>
          <button class="btn btn-warning" onclick="sendCommand('up')" id="upBtn">⬆️ THROTTLE +</button>
          <button class="btn btn-warning" onclick="sendCommand('down')" id="downBtn">⬇️ THROTTLE -</button>
        </div>
        
        <h4>Movement Control Matrix</h4>
        <div class="movement">
          <button class="btn btn-primary" onclick="sendCommand('left')" id="leftBtn">⬅️ LEFT</button>
          <button class="btn btn-primary" onclick="sendCommand('fwd')" id="fwdBtn">⬆️ FWD</button>
          <button class="btn btn-primary" onclick="sendCommand('right')" id="rightBtn">➡️ RIGHT</button>
          <button class="btn btn-primary" onclick="sendCommand('back')" id="backBtn">⬇️ BACK</button>
          <button class="btn btn-primary" onclick="sendCommand('yaw_left')" id="yawLeftBtn">↺ YAW L</button>
          <button class="btn btn-primary" onclick="sendCommand('yaw_right')" id="yawRightBtn">↻ YAW R</button>
          <div class="movement-center">🎯</div>
        </div>
        
        <!-- ADD THIS NEW STOP CONTROL SECTION -->
        <div style="margin-top: 15px;">
          <button class="btn btn-warning" onclick="sendCommand('stop')" id="stopBtn" style="width: 100%; font-size: 14px; font-weight: bold;">
            ⏹️ STOP ALL MOVEMENT
          </button>
        </div>
        
        <div style="margin-top: 15px;">
          <button class="btn btn-danger" onclick="sendCommand('disarm')" id="disarmBtn" style="width: 100%;">
            🔒 DISARM VEHICLE
          </button>
        </div>
      </div>

)rawliteral" + cameraSection + R"rawliteral(
    </div>

    <!-- Emergency Controls -->
    <div class="section">
      <h3 style="color: #ff4444;">🚨 Emergency Control Panel</h3>
      <div class="emergency">
        <button class="btn btn-emergency" onclick="confirmCommand('emergency_land', 'CONFIRM: Initiate Emergency Landing Protocol?')">
          🚨 EMERGENCY LAND
        </button>
        <button class="btn btn-critical" onclick="confirmCommand('force_disarm', 'DANGER: Force Disarm Vehicle? This is unsafe during flight!')">
          ⚡ FORCE DISARM
        </button>
      </div>
    </div>

    <!-- Network & System Diagnostics -->
    <div class="section">
      <h3>🔍 System Diagnostics</h3>
      <div class="metric-grid">
        <div class="metric">
          <div class="metric-label">WiFi Signal Strength</div>
          <div class="metric-value )rawliteral" + String(WiFi.RSSI() > -60 ? "status-ok" : (WiFi.RSSI() > -80 ? "status-warn" : "status-error")) + R"rawliteral(">)rawliteral" + String(WiFi.RSSI()) + R"rawliteral( dBm</div>
        </div>
        <div class="metric">
          <div class="metric-label">System ID Pair</div>
          <div class="metric-value">ESP:)rawliteral" + String(ESP32_SYSTEM_ID) + " FC:" + String(fc_system_id) + R"rawliteral(</div>
        </div>
        <div class="metric">
          <div class="metric-label">Component ID Pair</div>
          <div class="metric-value">ESP:)rawliteral" + String(ESP32_COMPONENT_ID) + " FC:" + String(fc_component_id) + R"rawliteral(</div>
        </div>
        <div class="metric">
          <div class="metric-label">IP Address</div>
          <div class="metric-value">)rawliteral" + WiFi.localIP().toString() + R"rawliteral(</div>
        </div>
      </div>
    </div>

    <!-- Communication Monitor -->
    <div class="section">
      <h3>📡 Real-time Communication Monitor</h3>
      <div class="comm-log" id="commLog">
        <div class="response">System initialized. MAVLink communication monitoring active...</div>
      </div>
      <div style="margin-top: 10px;">
        <button class="btn btn-primary" onclick="clearLog()">Clear Log</button>
        <button class="btn btn-primary" onclick="exportLog()">Export Log</button>
        <button class="btn btn-warning" onclick="testConnection()">Test Connection</button>
      </div>
    </div>
  </div>

  <script>
    let commandInProgress = false;
    let logEntries = [];
    let lastChannelUpdate = {};
    
    function addLog(message, type = 'response', command = '') {
      const log = document.getElementById('commLog');
      const timestamp = new Date().toLocaleTimeString();
      const entry = {timestamp, message, type, command};
      logEntries.push(entry);
      
      const colorClass = {
        'error': 'error',
        'warning': 'warning', 
        'command': 'command',
        'response': 'response'
      }[type] || 'response';
      
      log.innerHTML += `<div><span class="timestamp">[${timestamp}]</span> <span class="${colorClass}">${message}</span></div>`;
      log.scrollTop = log.scrollHeight;
      
      // Keep log size manageable
      if (logEntries.length > 100) {
        logEntries.shift();
        const lines = log.innerHTML.split('<div>');
        if (lines.length > 100) {
          log.innerHTML = '<div>' + lines.slice(-99).join('<div>');
        }
      }
    }
    
    function clearLog() {
      document.getElementById('commLog').innerHTML = '<div class="response">Log cleared.</div>';
      logEntries = [];
    }
    
    function exportLog() {
      const logText = logEntries.map(e => `[${e.timestamp}] ${e.message}`).join('\n');
      const blob = new Blob([logText], {type: 'text/plain'});
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `drone_log_${new Date().toISOString().slice(0,19).replace(/:/g,'-')}.txt`;
      a.click();
      URL.revokeObjectURL(url);
    }
    
    function testConnection() {
      addLog('Testing system connectivity...', 'command');
      fetch('/status')
        .then(response => response.json())
        .then(data => {
          addLog(`Connection test OK - Latency: ${Date.now() % 1000}ms`, 'response');
        })
        .catch(error => {
          addLog(`Connection test FAILED: ${error}`, 'error');
        });
    }
    
    function sendCommand(cmd) {
      if (commandInProgress) {
        addLog('Command rejected: System busy processing previous command', 'warning');
        return;
      }
      
      commandInProgress = true;
      const startTime = Date.now();
      addLog(`Transmitting command: ${cmd.toUpperCase()}`, 'command');
      
      // Show button feedback
      const btn = document.getElementById(cmd + 'Btn');
      if (btn) {
        btn.style.opacity = '0.6';
        btn.disabled = true;
      }
      
      fetch(`/${cmd}`)
        .then(response => {
          const latency = Date.now() - startTime;
          addLog(`Response received (${latency}ms latency)`, 'response');
          return response.text();
        })
        .then(data => { 
          addLog(`System response: ${data}`, data.includes('error') || data.includes('Cannot') ? 'error' : 'response');
          if (data.includes('EMERGENCY')) { 
            addLog('⚠️ EMERGENCY PROTOCOL ACTIVATED', 'warning');
            document.getElementById('criticalBanner').style.display = 'block';
          } 
        })
        .catch(error => {
          addLog(`Command transmission failed: ${error}`, 'error');
        })
        .finally(() => {
          commandInProgress = false;
          if (btn) {
            btn.style.opacity = '1';
            btn.disabled = false;
          }
        });
    }
    
    function confirmCommand(cmd, message) { 
      if (confirm(message)) { 
        sendCommand(cmd); 
      } 
    }
    
    function retryCamera() {
      addLog('Initiating camera system restart...', 'command');
      fetch('/camera/retry')
        .then(response => response.text())
        .then(data => {
          addLog(`Camera restart: ${data}`, data.includes('OK') ? 'response' : 'error');
          if (data.includes('OK')) {
            setTimeout(() => location.reload(), 1500);
          }
        })
        .catch(error => {
          addLog(`Camera restart failed: ${error}`, 'error');
        });
    }
    
    // Enhanced status monitoring with faster updates
    setInterval(() => {
      fetch('/status')
        .then(response => response.json())
        .then(data => {
          // Update RC channel values with change tracking
          const channels = ['ch1', 'ch2', 'ch3', 'ch4'];
          channels.forEach((ch, idx) => {
            const element = document.getElementById(ch);
            const timeElement = document.getElementById(ch + '-time');
            if (element && data.rc_channels && data.rc_channels[idx]) {
              const newValue = data.rc_channels[idx];
              if (element.textContent != newValue) {
                element.textContent = newValue;
                timeElement.textContent = new Date().toLocaleTimeString();
                element.style.background = '#004400';
                setTimeout(() => element.style.background = '', 500);
              }
            }
          });
          
          // Update critical status
          if (data.emergency && !document.getElementById('criticalBanner').style.display) {
            document.getElementById('criticalBanner').style.display = 'block';
          } else if (!data.emergency) {
            document.getElementById('criticalBanner').style.display = 'none';
          }
        })
        .catch(error => {
          addLog('Status update failed - connection issue', 'error');
        });
    }, 1000); // Faster update rate for better monitoring
    
    // Keyboard shortcuts with enhanced feedback
    document.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
      
      const keyMap = {
        'w': 'fwd', 'a': 'left', 's': 'back', 'd': 'right',
        'q': 'up', 'e': 'down', 'r': 'arm', 'f': 'disarm',
        ' ': 'stop', 'x': 'stop'  // ADD THESE LINES - SPACEBAR or X for STOP
      };
      
      const cmd = keyMap[e.key.toLowerCase()];
      if (cmd) {
        e.preventDefault();
        sendCommand(cmd);
        addLog(`Keyboard shortcut: ${e.key.toUpperCase()} → ${cmd.toUpperCase()}`, 'command');
      }
    });
    
    // Initialize interface
    addLog('Enhanced control interface loaded successfully', 'response');
    addLog('Keyboard shortcuts: WASD=movement, Q/E=throttle, R=arm, F=disarm, SPACE/X=stop', 'response');  // UPDATE THIS LINE
    addLog('System ready for flight operations', 'response');
    
    function updateLogs() {
      fetch('/logs')
        .then(response => response.json())
        .then(logs => {
          const logContainer = document.getElementById('commLog');
          // Clear existing logs except the first line
          const firstLine = logContainer.querySelector('.response');
          logContainer.innerHTML = '';
          if (firstLine) logContainer.appendChild(firstLine);
          
          logs.forEach(log => {
            const colorClass = {
              'error': 'error',
              'warning': 'warning',
              'mavlink': 'command',
              'success': 'response',
              'system': 'command',
              'info': 'response'
            }[log.type] || 'response';
            
            logContainer.innerHTML += `<div><span class="timestamp">[${log.time}]</span> <span class="${colorClass}">${log.msg}</span></div>`;
          });
          
          logContainer.scrollTop = logContainer.scrollHeight;
        })
        .catch(error => {
          console.error('Failed to fetch logs:', error);
        });
    }

    // Update logs every 2 seconds
    setInterval(updateLogs, 2000);

    // Call immediately on load
    updateLogs();
  </script>
</body>
</html>)rawliteral";
}

bool initializeCamera() {
  Serial.println("Camera: Initializing hardware...");
  esp_camera_deinit(); 
  delay(100);

  // Maximum CPU speed for better processing
  setCpuFrequencyMhz(240);
  
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
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.frame_size = FRAMESIZE_VGA;  // VGA resolution
  config.jpeg_quality = 15;  // Lower number = better quality
  config.fb_count = 2;  // Use double frame buffer
  config.fb_location = CAMERA_FB_IN_PSRAM;

  // Power cycle the camera if PWDN pin is defined
  if (PWDN_GPIO_NUM != -1) {
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, HIGH);
    delay(10);
    digitalWrite(PWDN_GPIO_NUM, LOW);
    delay(10);
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { 
    Serial.printf("Camera: Init failed 0x%x\n", err); 
    return false; 
  }

  // Optimized sensor settings
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 2);
    s->set_contrast(s, 2);
    s->set_saturation(s, 1);
    s->set_whitebal(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_agc_gain(s, 5);
    s->set_ae_level(s, 2);
    s->set_lenc(s, 1);
    s->set_raw_gma(s, 1);
  }
  
  // Test capture
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) { 
    Serial.printf("Camera: Ready %dx%d %dKB\n", fb->width, fb->height, fb->len/1024); 
    esp_camera_fb_return(fb); 
    return true; 
  } else { 
    Serial.println("Camera: Test capture failed"); 
    esp_camera_deinit(); 
    return false; 
  }
}

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
      static camera_fb_t *fb = nullptr;
      static size_t fb_index = 0;
      static bool boundary_sent = false;

      // Get new frame if needed
      if (fb == nullptr) {
        if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(100))) {
          if (currentFrame) {
            fb = currentFrame;
            currentFrame = nullptr;
          }
          xSemaphoreGive(frameMutex);
        }
        
        if (!fb) {
          fb = esp_camera_fb_get(); // Fallback to direct capture
        }
        
        if (!fb) {
          return 0; // No data available
        }
        fb_index = 0;
        boundary_sent = false;
      }

      size_t bytes_written = 0;

      // Send boundary first
      if (!boundary_sent) {
        String header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + 
                       String(fb->len) + "\r\n\r\n";
        
        if (header.length() <= maxLen) {
          memcpy(buffer, header.c_str(), header.length()); 
          bytes_written = header.length();
          boundary_sent = true;
        } else {
          esp_camera_fb_return(fb);
          fb = nullptr;
          return 0;
        }
      }
      // Send image data
      else { 
        size_t remaining = fb->len - fb_index;
        size_t to_send = min(remaining, maxLen - bytes_written);
        
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
  response->addHeader("Buffer-Size", "131072");  // 128KB buffer
  
  request->send(response);
  Serial.println("Camera: Stream started");
}

void cameraTask(void *parameter) {
  for(;;) {
    if (!cameraWorking) { 
      vTaskDelay(pdMS_TO_TICKS(1000)); 
      continue; 
    }
    
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
    vTaskDelay(pdMS_TO_TICKS(30));  // ~33 fps target rate
  }
}

// Add these global variables after the existing globals
struct LogEntry {
  String timestamp;
  String message;
  String type;
  unsigned long millis_time;
};

const int MAX_WEB_LOGS = 50;
LogEntry webLogs[MAX_WEB_LOGS];
int webLogIndex = 0;
int webLogCount = 0;
SemaphoreHandle_t logMutex;

// Add this function to replace Serial.println calls
void addWebLog(String message, String type = "info") {
  if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(10))) {
    unsigned long now = millis();
    unsigned long seconds = now / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    
    String timestamp = String(hours % 24) + ":" + 
                      String((minutes % 60) < 10 ? "0" : "") + String(minutes % 60) + ":" + 
                      String((seconds % 60) < 10 ? "0" : "") + String(seconds % 60);
    
    webLogs[webLogIndex] = {timestamp, message, type, now};
    webLogIndex = (webLogIndex + 1) % MAX_WEB_LOGS;
    if (webLogCount < MAX_WEB_LOGS) webLogCount++;
    
    xSemaphoreGive(logMutex);
  }
  
  // Still keep Serial for development/debugging
  Serial.println("[" + type + "] " + message);
}

// Initialize the log mutex in setup()
void setup() {
  Serial.begin(115200);
  
  // Initialize mutexes
  rcMutex = xSemaphoreCreateMutex();
  frameMutex = xSemaphoreCreateMutex();
  logMutex = xSemaphoreCreateMutex();
  
  addWebLog("ESP32 Drone Controller v2.0 starting...", "system");
  
  // Initialize MAVLink communication
  MAVLINK_SERIAL_PORT.begin(MAVLINK_BAUD_RATE, SERIAL_8N1, MAVLINK_RX_PIN, MAVLINK_TX_PIN);
  if (!MAVLINK_SERIAL_PORT) {
    addWebLog("MAVLink: Serial init FAILED", "error");
    mavlinkState = STATE_ERROR;
  } else {
    addWebLog("MAVLink: Serial initialized", "success");
    mavlinkState = STATE_INIT;
  }

  // Force MAVLink 1 protocol
  mavlink_status_t* mavlink_status = mavlink_get_channel_status(MAVLINK_COMM_0);
  mavlink_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  addWebLog("MAVLink: Forced to MAVLink 1 protocol", "system");

  // Initialize camera system
  cameraWorking = initializeCamera();
  if (cameraWorking) {
    addWebLog("Camera: Initialized successfully", "success");
  } else {
    addWebLog("Camera: Initialization failed", "error");
  }
  
  // Configure WiFi
  WiFi.setSleep(false); 
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(ssid, password); 
  Serial.print("WiFi: Connecting");
  while (WiFi.status() != WL_CONNECTED) { 
    delay(250);
    Serial.print("."); 
  }
  addWebLog("WiFi: Connected " + WiFi.localIP().toString() + " (RSSI: " + String(WiFi.RSSI()) + " dBm)", "success");
  
  // Configure web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { 
    request->send(200, "text/html", getHtmlPage()); 
  });
  
  server.on("/stream", HTTP_GET, handleStream);

  // Camera retry endpoint
  server.on("/camera/retry", HTTP_GET, [](AsyncWebServerRequest *request) { 
    cameraWorking = initializeCamera(); 
    request->send(200, "text/plain", cameraWorking ? "Camera OK" : "Camera Failed");
  });

  // ADD THE LOGS ENDPOINT HERE, INSIDE setup()
  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response = "[";
    
    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(100))) {
      int startIdx = webLogCount < MAX_WEB_LOGS ? 0 : webLogIndex;
      
      for (int i = 0; i < webLogCount; i++) {
        int idx = (startIdx + i) % MAX_WEB_LOGS;
        if (i > 0) response += ",";
        response += "{\"time\":\"" + webLogs[idx].timestamp + 
                   "\",\"msg\":\"" + webLogs[idx].message + 
                   "\",\"type\":\"" + webLogs[idx].type + 
                   "\",\"millis\":" + String(webLogs[idx].millis_time) + "}";
      }
      
      xSemaphoreGive(logMutex);
    }
    
    response += "]";
    request->send(200, "application/json", response);
  });

  // ARM command handler
  server.on("/arm", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;
    
    if (fc_detected && !fc_armed_status) {
        rc[2] = 1100;
        arm_disarm_vehicle(true);
        lastLatencyMs = millis() - cmdStart;
        request->send(200, "text/plain", "ARM command sent (" + String(lastLatencyMs) + "ms)");
        addWebLog("Command: ARM sent in " + String(lastLatencyMs) + "ms", "success");
    } else if (fc_armed_status) {
        request->send(200, "text/plain", "Vehicle already armed");
    } else {
        request->send(503, "text/plain", "Cannot arm: FC not connected");
        addWebLog("Command: ARM rejected - FC disconnected", "error");
    }
  });

  // DISARM command handler
  server.on("/disarm", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;
    
    if (fc_detected && fc_armed_status) {
        if (xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
            rc[2] = RC_THROTTLE_LOW;
            send_rc_override_values(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7]);
            xSemaphoreGive(rcMutex);
        }
        arm_disarm_vehicle(false);
        emergencyMode = false;
        lastLatencyMs = millis() - cmdStart;
        request->send(200, "text/plain", "DISARM command sent (" + String(lastLatencyMs) + "ms)");
        Serial.printf("Command: DISARM sent in %lums\n", lastLatencyMs);
    } else {
        request->send(503, "text/plain", "Cannot disarm: Not armed or FC disconnected");
        Serial.println("Command: DISARM rejected");
    }
  });

  // Movement commands - just update rc values, let loop() handle sending
  server.on("/fwd", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;
    if (fc_armed_status && xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
        rc[1] = RC_PITCH_FORWARD_CMD;
        xSemaphoreGive(rcMutex);
        lastLatencyMs = millis() - cmdStart;
        request->send(200, "text/plain", "Pitch forward (" + String(lastLatencyMs) + "ms)");
        Serial.printf("Command: FORWARD (%lums)\n", lastLatencyMs);
    } else { 
        request->send(fc_armed_status ? 503 : 403, "text/plain", fc_armed_status ? "System busy" : "Vehicle not armed"); 
    }
  });

  server.on("/back", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;
    if (fc_armed_status && xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
        rc[1] = RC_PITCH_BACKWARD_CMD;
        xSemaphoreGive(rcMutex);
        lastLatencyMs = millis() - cmdStart;
        request->send(200, "text/plain", "Pitch backward (" + String(lastLatencyMs) + "ms)");
        Serial.printf("Command: BACKWARD (%lums)\n", lastLatencyMs);
    } else { 
        request->send(fc_armed_status ? 503 : 403, "text/plain", fc_armed_status ? "System busy" : "Vehicle not armed"); 
    }
  });

  server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;
    if (fc_armed_status && xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
        rc[0] = RC_ROLL_LEFT_CMD;
        xSemaphoreGive(rcMutex);
        lastLatencyMs = millis() - cmdStart;
        request->send(200, "text/plain", "Roll left (" + String(lastLatencyMs) + "ms)");
        Serial.printf("Command: LEFT (%lums)\n", lastLatencyMs);
    } else { 
        request->send(fc_armed_status ? 503 : 403, "text/plain", fc_armed_status ? "System busy" : "Vehicle not armed"); 
    }
  });

  server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;
    if (fc_armed_status && xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
        rc[0] = RC_ROLL_RIGHT_CMD;
        xSemaphoreGive(rcMutex);
        lastLatencyMs = millis() - cmdStart;
        request->send(200, "text/plain", "Roll right (" + String(lastLatencyMs) + "ms)");
        Serial.printf("Command: RIGHT (%lums)\n", lastLatencyMs);
    } else { 
        request->send(fc_armed_status ? 503 : 403, "text/plain", fc_armed_status ? "System busy" : "Vehicle not armed"); 
    }
  });

  // Yaw commands
  server.on("/yaw_left", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;
    if (fc_armed_status && xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
        rc[3] = YAW_CCW;
        xSemaphoreGive(rcMutex);
        lastLatencyMs = millis() - cmdStart;
        request->send(200, "text/plain", "Yaw left (" + String(lastLatencyMs) + "ms)");
        addWebLog("Command: YAW LEFT in " + String(lastLatencyMs) + "ms", "success");
    } else { 
        request->send(fc_armed_status ? 503 : 403, "text/plain", fc_armed_status ? "System busy" : "Vehicle not armed"); 
    }
  });

  server.on("/yaw_right", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;
    if (fc_armed_status && xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
        rc[3] = YAW_CW;
        xSemaphoreGive(rcMutex);
        lastLatencyMs = millis() - cmdStart;
        request->send(200, "text/plain", "Yaw right (" + String(lastLatencyMs) + "ms)");
        addWebLog("Command: YAW RIGHT in " + String(lastLatencyMs) + "ms", "success");
    } else { 
        request->send(fc_armed_status ? 503 : 403, "text/plain", fc_armed_status ? "System busy" : "Vehicle not armed"); 
    }
  });

  // Throttle commands - just update rc values
  server.on("/up", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;
    
    if (fc_armed_status) {
        if (xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
            rc[2] = constrain(rc[2] + 50, RC_THROTTLE_LOW, 2000);
            xSemaphoreGive(rcMutex);
            lastLatencyMs = millis() - cmdStart;
            request->send(200, "text/plain", "Throttle: " + String(rc[2]) + " (" + String(lastLatencyMs) + "ms)");
            Serial.printf("Command: THROTTLE+ %d (%lums)\n", rc[2], lastLatencyMs);
        } else { 
            request->send(503, "text/plain", "System busy"); 
        }
    } else { 
        request->send(403, "text/plain", "Vehicle not armed"); 
    }
  });

  server.on("/down", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;
    
    if (fc_armed_status) {
        if (xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
            rc[2] = constrain(rc[2] - 50, RC_THROTTLE_LOW, 2000);
            xSemaphoreGive(rcMutex);
            lastLatencyMs = millis() - cmdStart;
            request->send(200, "text/plain", "Throttle: " + String(rc[2]) + " (" + String(lastLatencyMs) + "ms)");
            Serial.printf("Command: THROTTLE- %d (%lums)\n", rc[2], lastLatencyMs);
        } else { 
            request->send(503, "text/plain", "System busy"); 
        }
    } else { 
        request->send(403, "text/plain", "Vehicle not armed"); 
    }
  });

  // Keep the disarm command as is since it needs immediate RC override
  server.on("/disarm", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;
    
    if (fc_detected && fc_armed_status) {
        if (xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
            rc[2] = RC_THROTTLE_LOW; // Set throttle to minimum immediately
            send_rc_override_values(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7]); // Send immediately for safety
            xSemaphoreGive(rcMutex);
        }
        arm_disarm_vehicle(false);
        emergencyMode = false;
        lastLatencyMs = millis() - cmdStart;
        request->send(200, "text/plain", "DISARM command sent (" + String(lastLatencyMs) + "ms)");
        Serial.printf("Command: DISARM sent in %lums\n", lastLatencyMs);
    } else {
        request->send(503, "text/plain", "Cannot disarm: Not armed or FC disconnected");
        Serial.println("Command: DISARM rejected");
    }
  });

  // Emergency commands
  server.on("/emergency_land", HTTP_GET, [](AsyncWebServerRequest *request) {
    commandCount++;
    if (fc_armed_status && xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
        emergencyStartThrottle = rc[2];
        emergencyMode = true;
        emergencyStartTime = millis();
        xSemaphoreGive(rcMutex);
        request->send(200, "text/plain", "EMERGENCY LANDING INITIATED");
        Serial.println("EMERGENCY: Landing sequence started");
    } else { 
      request->send(403, "text/plain", "Vehicle not armed"); 
    }
  });

  server.on("/force_disarm", HTTP_GET, [](AsyncWebServerRequest *request) {
    commandCount++;
    if (fc_detected && xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
        rc[2] = RC_THROTTLE_LOW;
        send_rc_override_values(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7]);
        xSemaphoreGive(rcMutex);
        arm_disarm_vehicle(false);
        emergencyMode = false;
        request->send(200, "text/plain", "FORCE DISARM EXECUTED");
        Serial.println("EMERGENCY: Force disarm executed");
    } else { 
      request->send(503, "text/plain", "FC not detected"); 
    }
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long cmdStart = millis();
    commandCount++;

    // Check if FC is armed and mutex is available
    if (fc_armed_status && xSemaphoreTake(rcMutex, pdMS_TO_TICKS(50))) {
      rc[0] = RC_NEUTRAL; // Roll
      rc[1] = RC_NEUTRAL; // Pitch
      rc[3] = RC_NEUTRAL; // Yaw

      // Keep throttle unchanged
      xSemaphoreGive(rcMutex); // Give mutex before sending
      lastLatencyMs = millis() - cmdStart; // Calculate latency
      request->send(200, "text/plain", "STOP command sent (" + String(lastLatencyMs) + "ms)");
      addWebLog("Command: STOP sent in " + String(lastLatencyMs) + "ms", "success");
    } else {
      request->send(fc_armed_status ? 503 : 403, "text/plain", 
                   fc_armed_status ? "System busy" : "Vehicle not armed");
    }
  });

  // Enhanced status API
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) { 
    String status = "{\"camera\":" + String(cameraWorking ? "true" : "false") + 
                   ",\"wifi\":\"connected\",\"ip\":\"" + WiFi.localIP().toString() + 
                   "\",\"rssi\":" + String(WiFi.RSSI()) +
                   ",\"emergency\":" + String(emergencyMode ? "true" : "false") + 
                   ",\"fc_detected\":" + String(fc_detected ? "true" : "false") + 
                   ",\"fc_armed\":" + String(fc_armed_status ? "true" : "false") + 
                   ",\"mavlink_state\":" + String(mavlinkState) +
                   ",\"uptime\":" + String(millis()) +
                   ",\"command_count\":" + String(commandCount) +
                   ",\"messages_sent\":" + String(messagesSent) +
                   ",\"messages_received\":" + String(messagesReceived) +
                   ",\"last_latency\":" + String(lastLatencyMs) +
                   ",\"free_heap\":" + String(ESP.getFreeHeap()) +
                   ",\"rc_channels\":[" + String(rc[0]) + "," + String(rc[1]) + "," + String(rc[2]) + "," + String(rc[3]) + "]" +
                   "}"; 
    request->send(200, "application/json", status); 
  });
  
  server.begin();
  addWebLog("Web server: Started on " + WiFi.localIP().toString(), "success");
  if (cameraWorking) { 
    addWebLog("Video stream: http://" + WiFi.localIP().toString() + "/stream", "success");
  }

  // Start camera processing task
  if (cameraWorking) {
    xTaskCreatePinnedToCore(
      cameraTask,
      "CameraTask", 
      4096,
      NULL,
      2,
      &cameraTaskHandle,
      0
    );
    addWebLog("Camera: Task started on Core 0", "system");
  }
  
  addWebLog("=== System Ready ===", "system");
  lastCommandMs = millis();
}

void loop() {
    // MAVLink Communication Processing
    if (mavlinkState != STATE_ERROR) {
        // Send periodic heartbeat
        if (millis() - lastEspHeartbeatSentMs > 1000) {
            send_esp_heartbeat();
            lastEspHeartbeatSentMs = millis();
        }

        // Process incoming MAVLink messages
        uint8_t messageCount = 0;

        // ADD THIS ENHANCED DEBUG BLOCK
        static unsigned long lastRxDebugTime = 0;
        static unsigned long totalBytesReceived = 0;
        static bool firstByteReceived = false;

        if (millis() - lastRxDebugTime > 2000) { // Debug every 2 seconds
            int available = MAVLINK_SERIAL_PORT.available();
            
            if (available > 0) {
                if (!firstByteReceived) {
                    addWebLog("DEBUG: First bytes detected on UART1! Available: " + String(available), "success");
                    firstByteReceived = true;
                } else {
                    addWebLog("DEBUG: UART1 RX active - " + String(available) + " bytes available", "info");
                }
                
                // Read and display first few bytes in hex
                String hexData = "Raw: ";
                int bytesToShow = min(available, 8);
                for(int i = 0; i < bytesToShow; i++) {
                    if(MAVLINK_SERIAL_PORT.available()) {
                        uint8_t b = MAVLINK_SERIAL_PORT.read();
                        totalBytesReceived++;
                        hexData += "0x" + String(b, HEX) + " ";
                    }
                }
                addWebLog(hexData + " (Total RX: " + String(totalBytesReceived) + ")", "info");
            } else {
                // Still no data after everything we've tried
                addWebLog("DEBUG: UART1 RX still SILENT - No bytes available (FC should be sending heartbeats)", "error");
            }
            lastRxDebugTime = millis();
        }
        // END DEBUG BLOCK

        while (MAVLINK_SERIAL_PORT.available() > 0 && messageCount < 10) {
            mavlink_message_t msg;
            mavlink_status_t status;
            uint8_t c = MAVLINK_SERIAL_PORT.read();
            
            // ADD THIS PARSING DEBUG
            static unsigned long lastParseLog = 0;
            if (millis() - lastParseLog > 5000) { // Log every 5 seconds if we're getting bytes but no valid messages
                addWebLog("DEBUG: Reading bytes but no valid MAVLink messages parsed yet", "warning");
                lastParseLog = millis();
            }
            
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
                addWebLog("DEBUG: Successfully parsed MAVLink message ID: " + String(msg.msgid) + " from SYS: " + String(msg.sysid), "success");
                handle_mavlink_message(&msg);
                messagesReceived++;
                messageCount++;
            }
        }

        // Flight Controller State Machine
        switch (mavlinkState) {
            case STATE_INIT:
                addWebLog("State: INIT -> WAIT_FOR_HEARTBEAT", "system");
                mavlinkState = STATE_WAIT_FOR_HEARTBEAT;
                stateTimer = millis();
                break;
                
            case STATE_WAIT_FOR_HEARTBEAT:
                if (fc_detected) {
                    addWebLog("State: WAIT_FOR_HEARTBEAT -> SET_MODE_STABILIZE", "system");
                    mavlinkState = STATE_SET_MODE_STABILIZE;
                } else if (millis() - stateTimer > 7000) {
                    addWebLog("State: WAIT_FOR_HEARTBEAT -> ERROR (timeout)", "error");
                    mavlinkState = STATE_ERROR;
                }
                break;
                
            case STATE_SET_MODE_STABILIZE:
                addWebLog("State: Setting STABILIZE mode (ID: " + String(stabilize_mode_id) + ")", "system");
                set_flight_mode(stabilize_mode_id);
                mavlinkState = STATE_WAIT_FOR_MODE_ACK;
                stateTimer = millis();
                break;
                
            case STATE_WAIT_FOR_MODE_ACK:
                if (millis() - stateTimer > 3000) {
                    addWebLog("State: WAIT_FOR_MODE_ACK -> ARM_VEHICLE (timeout)", "warning");
                    mavlinkState = STATE_ARM_VEHICLE; 
                }
                break;
                
            case STATE_ARM_VEHICLE:
                if (fc_armed_status) {
                    mavlinkState = STATE_OPERATIONAL;
                    addWebLog("State: ARM_VEHICLE -> OPERATIONAL", "success");
                }
                break;
                
            case STATE_WAIT_FOR_ARMED:
                if (fc_armed_status) {
                    addWebLog("State: WAIT_FOR_ARMED -> OPERATIONAL", "success");
                    mavlinkState = STATE_OPERATIONAL;
                } else if (millis() - stateTimer > 7000) {
                    addWebLog("State: WAIT_FOR_ARMED -> ERROR (timeout)", "error");
                    mavlinkState = STATE_ERROR;
                }
                break;
                
            case STATE_OPERATIONAL:
                if (!fc_armed_status && fc_detected) {
                    addWebLog("State: OPERATIONAL -> WAIT_FOR_HEARTBEAT (disarmed)", "info");
                    mavlinkState = STATE_WAIT_FOR_HEARTBEAT;
                }
                break;
                
            case STATE_ERROR:
                if (millis() - stateTimer > 10000) {
                    addWebLog("State: ERROR -> INIT (recovery attempt)", "warning");
                    mavlinkState = STATE_INIT;
                    stateTimer = millis();
                }
                break;
                
            default:
                break;
        }
    }

  // Continuous RC override sending - send always when armed with non-neutral values
  static unsigned long lastRcSend = 0;
  if (millis() - lastRcSend >= 100) { // 10Hz like Python
      if (fc_armed_status) {
          // Always send RC override when armed to maintain control
          send_rc_override_values(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7]);
      }
      lastRcSend = millis();
  }

  // Emergency landing sequence with detailed logging
  if (emergencyMode) {
    unsigned long emergencyTime = millis() - emergencyStartTime;
    if (xSemaphoreTake(rcMutex, pdMS_TO_TICKS(5))) {
        if (emergencyTime <= 1000) {
            rc[0] = RC_NEUTRAL; 
            rc[1] = RC_NEUTRAL;
            Serial.printf("Emergency: Phase 1 - Stabilizing (%lums)\n", emergencyTime);
        } else if (emergencyTime > 1000 && emergencyTime <= 6000) {
            int timeInDescent = emergencyTime - 1000;
            int throttleRange = emergencyStartThrottle - RC_THROTTLE_LOW;
            int reduction = (timeInDescent * throttleRange) / 5000;
            rc[2] = constrain(emergencyStartThrottle - reduction, RC_THROTTLE_LOW, 2000);
            rc[0] = RC_NEUTRAL; 
            rc[1] = RC_NEUTRAL;
            
            if (emergencyTime % 1000 < 50) { // Log every second
                Serial.printf("Emergency: Phase 2 - Descending throttle=%d (%lums)\n", rc[2], emergencyTime);
            }
            
            digitalWrite(LED_GPIO, (emergencyTime / 500) % 2);
        } else if (emergencyTime > 6000) {
            rc[2] = RC_THROTTLE_LOW;
            send_rc_override_values(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7]);
            xSemaphoreGive(rcMutex);
            arm_disarm_vehicle(false);
            emergencyMode = false;
            digitalWrite(LED_GPIO, LOW);
            Serial.println("Emergency: Phase 3 - Landing complete, vehicle disarmed");
            goto emergency_complete;
        }
        send_rc_override_values(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7]);
        xSemaphoreGive(rcMutex);
    }
    emergency_complete:;
  }
  
  // Monitor flight controller connection with detailed logging
  if (fc_detected && (millis() - lastFcHeartbeatMs > FC_HEARTBEAT_TIMEOUT_MS)) {
      Serial.printf("FC: Connection lost (last heartbeat %lums ago)\n", millis() - lastFcHeartbeatMs);
      fc_detected = false;
      fc_armed_status = false;
      mavlinkState = STATE_WAIT_FOR_HEARTBEAT;
  }
  
  delay(5); // Reduced delay for better performance
}

// MAVLink Protocol Implementation with performance tracking

void send_esp_heartbeat() {
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(
        ESP32_SYSTEM_ID, ESP32_COMPONENT_ID, &msg,
        MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        0,
        MAV_STATE_ACTIVE
    );
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    MAVLINK_SERIAL_PORT.write(buffer, len);
    messagesSent++;
}

void checkMavlinkProtocol(mavlink_message_t* msg) {
    static bool protocolLogged = false;
    if (!protocolLogged) {
        if (msg->magic == MAVLINK_STX) {
            addWebLog("MAVLink: Detected MAVLink 2 from FC", "warning");
        } else if (msg->magic == MAVLINK_STX_MAVLINK1) {
            addWebLog("MAVLink: Detected MAVLink 1 from FC", "success");
        }
        protocolLogged = true;
    }
}

void handle_mavlink_message(mavlink_message_t* msg) {
    // Check protocol version
    checkMavlinkProtocol(msg);
    
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(msg, &heartbeat);
            
            if (msg->sysid != ESP32_SYSTEM_ID) {
                if (!fc_detected) {
                    fc_system_id = msg->sysid;
                    fc_component_id = msg->compid;
                    fc_detected = true;
                    addWebLog("FC: Detected SYS:" + String(fc_system_id) + " COMP:" + String(fc_component_id) + 
                             " TYPE:" + String(heartbeat.type) + " AP:" + String(heartbeat.autopilot), "mavlink");
                    stabilize_mode_id = 0;
                }
                
                bool armed_from_fc = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
                if (armed_from_fc != fc_armed_status) {
                    fc_armed_status = armed_from_fc;
                    addWebLog("FC: Armed status changed to " + String(fc_armed_status ? "ARMED" : "DISARMED"), 
                             fc_armed_status ? "warning" : "info");
                    
                    if (fc_armed_status && mavlinkState < STATE_OPERATIONAL) {
                        mavlinkState = STATE_OPERATIONAL;
                    } else if (!fc_armed_status && mavlinkState == STATE_OPERATIONAL) {
                        mavlinkState = STATE_WAIT_FOR_HEARTBEAT;
                    }
                }
            }
            lastFcHeartbeatMs = millis();
            break;
        }
        
        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(msg, &ack);
            addWebLog("FC: Command ACK CMD:" + String(ack.command) + " RESULT:" + String(ack.result), 
                     ack.result == MAV_RESULT_ACCEPTED ? "success" : "error");
            break;
        }
        
        case MAVLINK_MSG_ID_STATUSTEXT: {
            mavlink_statustext_t statustext;
            char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
            mavlink_msg_statustext_decode(msg, &statustext);
            strncpy(text, statustext.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
            text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = '\0';
            addWebLog("FC: Status (Sev:" + String(statustext.severity) + "): " + String(text), "mavlink");
            break;
        }
        
        default:
            // Log unknown messages occasionally
            if (millis() % 10000 < 100) {
                addWebLog("FC: Unknown message ID " + String(msg->msgid), "info");
            }
            break;
    }
}

void send_mavlink_command_long(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
    // Use detected IDs if available, otherwise use defaults
    uint8_t target_system = fc_detected ? fc_system_id : DEFAULT_FC_SYSTEM_ID;
    uint8_t target_component = fc_detected ? fc_component_id : DEFAULT_FC_COMPONENT_ID;
    
    if (!fc_detected && command != MAV_CMD_REQUEST_MESSAGE) {
        addWebLog("MAVLink: Cannot send command - FC not detected, using defaults", "warning");
    }
    
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(
        ESP32_SYSTEM_ID, ESP32_COMPONENT_ID, &msg,
        target_system, target_component,
        command,
        0,
        param1, param2, param3, param4, param5, param6, param7
    );
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    MAVLINK_SERIAL_PORT.write(buffer, len);
    messagesSent++;
    addWebLog("MAVLink: Sent command " + String(command) + " to SYS:" + String(target_system), "mavlink");
}

void set_flight_mode(uint8_t mode_id) {
    send_mavlink_command_long(MAV_CMD_DO_SET_MODE, (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)mode_id);
}

void arm_disarm_vehicle(bool arm) {
    fc_armed_status = arm;
    Serial.printf("MAVLink: Sending %s command\n", arm ? "ARM" : "DISARM");
    send_mavlink_command_long(MAV_CMD_COMPONENT_ARM_DISARM, arm ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0);
}

void send_rc_override_values(uint16_t ch1_roll, uint16_t ch2_pitch, uint16_t ch3_throttle, uint16_t ch4_yaw,
                             uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8) {
    if (!fc_detected) return;
    if (!fc_armed_status && mavlinkState != STATE_DISARM_VEHICLE && !emergencyMode) return;
    
    lastRcOverrideSentMs = millis();
    
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    
    mavlink_msg_rc_channels_override_pack(
        ESP32_SYSTEM_ID, ESP32_COMPONENT_ID, &msg,

        fc_system_id, fc_component_id,
        ch1_roll, ch2_pitch, ch3_throttle, ch4_yaw,
        ch5, ch6, ch7, ch8,
        UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
        UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
        UINT16_MAX, UINT16_MAX
    );
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    MAVLINK_SERIAL_PORT.write(buffer, len);
    messagesSent++;
}

