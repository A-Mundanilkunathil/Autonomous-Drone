#!/bin/bash

echo "Starting display and VNC services..."

# Kill any existing X processes
pkill -f Xvfb || true
pkill -f x11vnc || true
pkill -f fluxbox || true
pkill -f websockify || true

# Start virtual display with proper settings for Gazebo
echo "Starting Xvfb..."
# Increased resolution for better viewing and centering
Xvfb :99 -screen 0 1920x1080x24 -ac +extension GLX +render +extension RANDR +extension COMPOSITE -noreset -dpi 96 -fp /usr/share/fonts/X11/misc/ &
export DISPLAY=:99
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR && chmod 700 $XDG_RUNTIME_DIR
export XAUTHORITY=/tmp/.X99-auth

# Set environment variables for better OpenGL support
export LIBGL_ALWAYS_SOFTWARE=1
export LIBGL_ALWAYS_INDIRECT=0
export MESA_GL_VERSION_OVERRIDE=3.3
export GALLIUM_DRIVER=llvmpipe
export QT_X11_NO_MITSHM=1
export GZ_IP=127.0.0.1
export IGN_IP=127.0.0.1

# Force software rendering and disable GPU features
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export __GLX_VENDOR_LIBRARY_NAME=mesa
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.x86_64.json
export LIBGL_DEBUG=verbose

# Force Gazebo to use software rendering
export GAZEBO_MASTER_URI=http://localhost:11345
export OGRE_RTShader_Write=0
export OGRE_SKIP_RenderSystem_GL3Plus=1

# Install a minimal Gazebo GUI configuration to avoid loading heavy plugins
MINIMAL_GUI_CONFIG='<?xml version="1.0"?>
<gui version="1.0">
    <window>
        <width>1280</width>
        <height>720</height>
    </window>
    <plugin filename="MinimalScene" name="3D View">
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.7 0.7 0.7</background_color>
    </plugin>
    <plugin filename="WorldControl" name="World control"/>
    <plugin filename="WorldStats" name="World stats"/>
</gui>'

# Overwrite the system default so Gazebo cannot restore the heavy layout
if [ -d /usr/share/gz/gz-sim7/gui ]; then
    echo "$MINIMAL_GUI_CONFIG" > /usr/share/gz/gz-sim7/gui/gui.config
fi

# Ensure the user config matches the minimal layout every startup
mkdir -p /root/.gz/sim/7
echo "$MINIMAL_GUI_CONFIG" > /root/.gz/sim/7/gui.config
export GZ_GUI_CONFIG=/root/.gz/sim/7/gui.config

# Wait for X server to start
echo "Waiting for X server to initialize..."
sleep 3

# Test X server
echo "Testing X server..."
xdpyinfo -display :99 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "X server is running successfully"
else
    echo "WARNING: X server may not be running properly"
fi

# Start window manager
echo "Starting window manager (fluxbox)..."

# Create fluxbox config directory
mkdir -p /root/.fluxbox

# Configure fluxbox for centered windows
cat > /root/.fluxbox/apps << 'EOF'
[app] (name=Gazebo) (class=gazebo)
  [Position] (CENTER) {0 0}
  [Maximized] {yes}
[end]

[app] (name=gz)
  [Position] (CENTER) {0 0}
  [Maximized] {yes}
[end]
EOF

# Configure fluxbox init settings for better window management
cat > /root/.fluxbox/init << 'EOF'
session.screen0.windowPlacement: RowSmartPlacement
session.screen0.defaultDeco: NORMAL
session.screen0.focusModel: ClickFocus
session.screen0.fullMaximization: true
session.screen0.maxIgnoreIncrement: true
session.screen0.maxDisableMove: false
session.screen0.maxDisableResize: false
session.screen0.workspaces: 1
EOF

# Start fluxbox in background and suppress config warnings
fluxbox 2>/dev/null &
sleep 2

# Start VNC server with authentication
echo "Starting VNC server..."
VNC_STARTED=false

# Try with authentication first
if [ -f /root/.vnc/passwd ]; then
    echo "Attempting VNC with password authentication..."
    x11vnc -display :99 -rfbauth /root/.vnc/passwd -listen 0.0.0.0 -rfbport 5900 -xkb -ncache 10 -ncache_cr -quiet -forever -shared -bg
    sleep 2
    
    # Check if VNC started
    if pgrep x11vnc > /dev/null; then
        echo "✓ VNC server started with authentication (PID: $(pgrep x11vnc))"
        VNC_STARTED=true
    fi
fi

# Fallback: try without authentication
if [ "$VNC_STARTED" = false ]; then
    echo "Trying VNC without password..."
    x11vnc -display :99 -nopw -listen 0.0.0.0 -rfbport 5900 -xkb -ncache 10 -ncache_cr -quiet -forever -shared -bg
    sleep 2
    
    if pgrep x11vnc > /dev/null; then
        echo "✓ VNC server started without authentication (PID: $(pgrep x11vnc))"
        VNC_STARTED=true
    else
        echo "✗ Failed to start VNC server"
    fi
fi

# Start noVNC web interface
echo "Starting noVNC web interface..."
NOVNC_STARTED=false

# Try different noVNC paths and commands
if [ -f /usr/share/novnc/utils/novnc_proxy ]; then
    echo "Using novnc_proxy..."
    /usr/share/novnc/utils/novnc_proxy --vnc localhost:5900 --listen 0.0.0.0:6080 &
    NOVNC_STARTED=true
elif [ -f /usr/share/novnc/utils/launch.sh ]; then
    echo "Using launch.sh..."
    /usr/share/novnc/utils/launch.sh --vnc localhost:5900 --listen 6080 &
    NOVNC_STARTED=true
elif command -v websockify &> /dev/null; then
    echo "Using websockify directly..."
    cd /usr/share/novnc && websockify --web=/usr/share/novnc/ 6080 localhost:5900 &
    NOVNC_STARTED=true
else
    echo "WARNING: noVNC not found, trying manual setup..."
    # Try to find and start websockify manually
    if [ -d /usr/share/novnc ]; then
        cd /usr/share/novnc
        python3 -m websockify.main --web=. 6080 localhost:5900 &
        NOVNC_STARTED=true
    fi
fi

sleep 3

if [ "$NOVNC_STARTED" = true ]; then
    echo "✓ noVNC web interface started successfully"
else
    echo "✗ Failed to start noVNC web interface"
fi

echo "=== VNC Setup Complete ==="
echo "VNC Server: localhost:5900 (password: gazebo123)"
echo "Web VNC: http://localhost:6080/vnc.html"
echo "=========================="

# Test what Gazebo versions are available
echo "Testing available Gazebo installations..."
if command -v gz &> /dev/null; then
    echo "Found 'gz' command"
    gz sim --help | head -10
elif command -v ign &> /dev/null; then
    echo "Found 'ign' command" 
    ign gazebo --help | head -10
elif command -v gazebo &> /dev/null; then
    echo "Found 'gazebo' command"
    gazebo --help | head -10
else
    echo "No Gazebo installation found!"
fi

# Create web directory
mkdir -p /gazebo/web

# Create HTML interface (keeping your existing HTML code)
cat > /gazebo/web/index.html << 'HTMLEOF'
<!DOCTYPE html>
<html>
<head>
    <title>Gazebo Web Interface</title>
    <style>
        body { font-family: Arial, sans-serif; max-width: 800px; margin: 0 auto; padding: 20px; background: #f5f5f5; }
        .container { background: white; padding: 30px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        button { background: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; margin: 5px; }
        button:hover { background: #45a049; }
        button:disabled { background: #ccc; cursor: not-allowed; }
        .status { padding: 10px; border-radius: 5px; margin: 10px 0; }
        .status.connected { background: #d4edda; color: #155724; }
        .status.disconnected { background: #f8d7da; color: #721c24; }
        .log { background: #f8f9fa; padding: 15px; height: 200px; overflow-y: auto; font-family: monospace; border: 1px solid #ddd; border-radius: 5px; }
        .vnc-link { background: #007bff; color: white; padding: 15px; border-radius: 5px; margin: 10px 0; text-align: center; }
        .vnc-link a { color: white; text-decoration: none; font-weight: bold; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Gazebo Web Interface</h1>
        
        <div class="vnc-link">
            <a href="http://localhost:6080/vnc.html" target="_blank">Open VNC Viewer (Visual Interface)</a>
            <br><small>VNC Password: <code>gazebo123</code></small>
        </div>
        
        <div id="status" class="status disconnected">Status: Disconnected</div>
        
        <div>
            <button id="connectBtn" onclick="connect()">Connect</button>
            <button id="startBtn" onclick="startSimulation()" disabled>Start Simulation</button>
            <button id="stopBtn" onclick="stopSimulation()" disabled>Stop Simulation</button>
        </div>
        
        <div class="log" id="log">Console output will appear here...</div>
        
        <p><strong>Instructions:</strong></p>
        <ol>
            <li>Click "Open VNC Viewer" above to see the desktop</li>
            <li>Enter password: <code>gazebo123</code> when prompted</li>
            <li>Click "Connect" in this panel</li>
            <li>Click "Start Simulation" to launch Gazebo</li>
            <li>Watch the Gazebo window appear in the VNC viewer!</li>
        </ol>
        
        <p><strong>Troubleshooting:</strong></p>
        <ul>
            <li>If VNC doesn't work, try refreshing the page</li>
            <li>Check that ports 6080 and 5900 are exposed</li>
            <li>Desktop should show a simple Fluxbox environment</li>
        </ul>
    </div>

    <script>
        let ws = null;
        let connected = false;

        function log(message) {
            const logDiv = document.getElementById('log');
            const timestamp = new Date().toLocaleTimeString();
            logDiv.innerHTML += `[${timestamp}] ${message}\n`;
            logDiv.scrollTop = logDiv.scrollHeight;
        }

        function updateStatus(message, isConnected) {
            const statusDiv = document.getElementById('status');
            statusDiv.textContent = `Status: ${message}`;
            statusDiv.className = `status ${isConnected ? 'connected' : 'disconnected'}`;
            
            document.getElementById('startBtn').disabled = !isConnected;
            document.getElementById('stopBtn').disabled = !isConnected;
            document.getElementById('connectBtn').textContent = isConnected ? 'Disconnect' : 'Connect';
            connected = isConnected;
        }

        function connect() {
            if (connected) {
                if (ws) ws.close();
                return;
            }

            ws = new WebSocket('ws://localhost:7681');
            
            ws.onopen = function() {
                log('Connected to Gazebo server');
                updateStatus('Connected', true);
            };
            
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                log(`${data.type}: ${data.message || 'No message'}`);
            };
            
            ws.onclose = function() {
                log('Disconnected from server');
                updateStatus('Disconnected', false);
            };
            
            ws.onerror = function(error) {
                log('WebSocket error: ' + error);
                updateStatus('Error', false);
            };
        }

        function sendCommand(command) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify(command));
                log(`Sent: ${command.type}`);
            }
        }

        function startSimulation() {
            sendCommand({ type: 'start_simulation' });
        }

        function stopSimulation() {
            sendCommand({ type: 'stop_simulation' });
        }

        window.onload = function() {
            log('Web interface loaded. Click Connect to start.');
        };
    </script>
</body>
</html>
HTMLEOF

echo "Starting Gazebo Web Interface..."
echo "Web UI: http://localhost:8080"
echo "VNC UI: http://localhost:6080/vnc.html"
echo "WebSocket: ws://localhost:7681"

echo ""
echo "Starting web server..."
# Start the web server
python3 /gazebo/web_server.py
