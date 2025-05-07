"""Web-based remote control interface for the drone."""
import threading
import asyncio
import socket
import json
from flask import Flask, Response, render_template_string, request, jsonify
import logging

# Disable Flask's verbose logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

class DroneRemoteServer:
    """Provides a web interface to control the drone."""
    
    def __init__(self, drone_controller, camera, port=5000):
        self.app = Flask(__name__)
        self.port = port
        self.drone = drone_controller
        self.camera = camera
        
        self.is_streaming = False
        self.stream_thread = None
        self.command_queue = asyncio.Queue()
        self.event_loop = asyncio.new_event_loop()
        
        # Configure Flask routes
        self._setup_routes()
    
    def _setup_routes(self):
        """Set up the Flask routes for the web interface."""
        @self.app.route('/')
        def index():
            ip = self._get_ip_address()
            return render_template_string('''
                <!DOCTYPE html>
                <html>
                <head>
                    <title>Drone Control</title>
                    <meta name="viewport" content="width=device-width, initial-scale=1">
                    <style>
                        body { margin: 0; padding: 0; font-family: Arial, sans-serif; background: #222; color: white; }
                        .container { display: flex; flex-direction: column; height: 100vh; }
                        .video-container { flex: 1; display: flex; justify-content: center; align-items: center; 
                                         background: #111; overflow: hidden; }
                        .video-feed { max-width: 100%; max-height: 80vh; }
                        .controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 5px; padding: 10px; 
                                  background: #333; }
                        button { padding: 15px; border: none; border-radius: 5px; background: #555; color: white; 
                                font-size: 16px; cursor: pointer; }
                        button:hover { background: #666; }
                        button:active { background: #777; }
                        #forward { grid-column: 2; }
                        #left { grid-column: 1; grid-row: 2; }
                        #stop { grid-column: 2; grid-row: 2; background: #888; }
                        #right { grid-column: 3; grid-row: 2; }
                        #backward { grid-column: 2; grid-row: 3; }
                        .action-buttons { grid-column: span 3; display: flex; justify-content: space-around; 
                                        margin-top: 10px; }
                        .takeoff { background: #4CAF50; }
                        .land { background: #f44336; }
                        .status { padding: 10px; background: #444; margin-bottom: 10px; border-radius: 4px; }
                    </style>
                </head>
                <body>
                    <div class="container">
                        <div class="video-container">
                            <img class="video-feed" src="{{ url_for('video_feed') }}" alt="Drone Camera">
                        </div>
                        
                        <div class="status" id="status">Status: Ready</div>
                        
                        <div class="controls">
                            <button id="forward" onmousedown="sendCommand('forward')" onmouseup="sendCommand('hover')"
                                  ontouchstart="sendCommand('forward')" ontouchend="sendCommand('hover')">Forward</button>
                            <button id="left" onmousedown="sendCommand('left')" onmouseup="sendCommand('hover')"
                                  ontouchstart="sendCommand('left')" ontouchend="sendCommand('hover')">Left</button>
                            <button id="stop" onclick="sendCommand('hover')">Hover</button>
                            <button id="right" onmousedown="sendCommand('right')" onmouseup="sendCommand('hover')"
                                  ontouchstart="sendCommand('right')" ontouchend="sendCommand('hover')">Right</button>
                            <button id="backward" onmousedown="sendCommand('backward')" onmouseup="sendCommand('hover')"
                                  ontouchstart="sendCommand('backward')" ontouchend="sendCommand('hover')">Backward</button>
                            
                            <div class="action-buttons">
                                <button class="takeoff" onclick="sendCommand('takeoff')">TAKE OFF</button>
                                <button onclick="sendCommand('photo')">PHOTO</button>
                                <button class="land" onclick="sendCommand('land')">LAND</button>
                            </div>
                        </div>
                    </div>
                    
                    <script>
                        function sendCommand(command) {
                            fetch('/control', {
                                method: 'POST',
                                headers: { 'Content-Type': 'application/json' },
                                body: JSON.stringify({ command: command })
                            })
                            .then(response => response.json())
                            .then(data => {
                                document.getElementById('status').textContent = 'Status: ' + data.status;
                            })
                            .catch(error => {
                                console.error('Error:', error);
                                document.getElementById('status').textContent = 'Status: Error';
                            });
                        }
                        
                        // Add keyboard controls
                        document.addEventListener('keydown', (e) => {
                            if (e.repeat) return;
                            
                            switch(e.key) {
                                case 'ArrowUp': sendCommand('forward'); break;
                                case 'ArrowDown': sendCommand('backward'); break;
                                case 'ArrowLeft': sendCommand('left'); break;
                                case 'ArrowRight': sendCommand('right'); break;
                                case ' ': sendCommand('hover'); break;
                                case 't': sendCommand('takeoff'); break;
                                case 'l': sendCommand('land'); break;
                                case 'p': sendCommand('photo'); break;
                            }
                        });
                        
                        document.addEventListener('keyup', (e) => {
                            if(['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
                                sendCommand('hover');
                            }
                        });
                    </script>
                </body>
                </html>
            ''', ip=ip)
        
        @self.app.route('/video_feed')
        def video_feed():
            # Return a streaming response for the video feed
            return Response(self._generate_frames(),
                           mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/control', methods=['POST'])
        def control():
            if request.method == 'POST':
                data = request.get_json()
                command = data.get('command')
                if command:
                    self._handle_command(command)
                    return jsonify({"status": f"Command: {command}"})
            return jsonify({"status": "Invalid command"}), 400
    
    def _get_ip_address(self):
        """Get the server's IP address."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "localhost"
    
    def _handle_command(self, command):
        """Handle incoming command by adding it to the queue."""
        print(f"Received command: {command}")
        asyncio.run_coroutine_threadsafe(
            self.command_queue.put(command), 
            self.event_loop
        )
    
    async def _command_processor(self):
        """Process commands from the queue."""
        while True:
            command = await self.command_queue.get()
            
            try:
                if command == 'takeoff':
                    await self.drone.takeoff()
                elif command == 'land':
                    await self.drone.land()
                elif command == 'forward':
                    await self.drone.move_forward()
                elif command == 'backward':
                    await self.drone.move_backward()
                elif command == 'left':
                    await self.drone.move_left()
                elif command == 'right':
                    await self.drone.move_right()
                elif command == 'hover':
                    await self.drone.hover()
                elif command == 'photo':
                    self.camera.capture_image()
            except Exception as e:
                print(f"Error processing command '{command}': {e}")
            
            self.command_queue.task_done()
    
    def _generate_frames(self):
        """Generate video frames for streaming."""
        import cv2
        
        while self.is_streaming:
            # Get frame from camera
            frame = self.camera.get_frame()
            
            # Encode to JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            
            # Send frame
            yield (b'--frame\r\n'
                  b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    
    def _run_command_loop(self):
        """Run the asyncio event loop for command processing."""
        asyncio.set_event_loop(self.event_loop)
        self.event_loop.create_task(self._command_processor())
        self.event_loop.run_forever()
    
    def _run_server(self):
        """Run the Flask web server."""
        self.app.run(host='0.0.0.0', port=self.port, threaded=True,
                    debug=False, use_reloader=False)
    
    def start(self):
        """Start the remote control server."""
        # Start command processing loop
        self.command_thread = threading.Thread(target=self._run_command_loop)
        self.command_thread.daemon = True
        self.command_thread.start()
        
        # Start streaming
        self.is_streaming = True
        
        # Start web server
        self.stream_thread = threading.Thread(target=self._run_server)
        self.stream_thread.daemon = True
        self.stream_thread.start()
        
        ip = self._get_ip_address()
        print(f"Remote control server started at: http://{ip}:{self.port}")
    
    def stop(self):
        """Stop the remote control server."""
        self.is_streaming = False
        
        # Stop the event loop
        if self.event_loop and self.event_loop.is_running():
            self.event_loop.call_soon_threadsafe(self.event_loop.stop)
        
        print("Remote control server stopped")