#!/usr/bin/env python3
import asyncio
import websockets
import json
import subprocess
import threading
import time
import os
import textwrap
from http.server import HTTPServer, SimpleHTTPRequestHandler

class GazeboWebServer:
    def __init__(self):
        self.gazebo_process = None
        self.clients = set()
        self.stdout_thread = None
        self.stderr_thread = None

    def _stream_output(self, stream, label):
        """Continuously read a subprocess stream to avoid deadlocks."""
        try:
            for line in iter(stream.readline, ''):
                if not line:
                    break
                print(f"[Gazebo {label}] {line.rstrip()}")
        finally:
            stream.close()
        
    def start_gazebo(self):
        """Start Gazebo simulation server."""
        try:
            commands = [
                ["gz", "sim", "-v", "4", "--gui-config", "/tmp/gui.config"],  # Try with GUI config
                ["gz", "sim", "-v", "4", "-g"],  # Force GUI mode
                ["gz", "sim", "-v", "4"],  # Basic GUI world
                ["ign", "gazebo", "-v", "4", "-g"],  # Ignition with GUI
                ["gazebo", "--verbose"],  # Classic Gazebo
            ]

            gui_config = textwrap.dedent(
                """
                <?xml version="1.0"?>
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
                </gui>
                """
            )
            with open('/tmp/gui.config', 'w') as f:
                f.write(gui_config)

            for cmd in commands:
                try:
                    print(f"Trying command: {' '.join(cmd)}")
                    env = dict(os.environ)
                    env.update({
                        'DISPLAY': ':99',
                        'QT_X11_NO_MITSHM': '1',
                        'GAZEBO_MODEL_PATH': '/usr/share/gazebo-11/models:/usr/share/ignition/fuel',
                        'GZ_SIM_RESOURCE_PATH': '/usr/share/ignition/fuel',
                        'GZ_GUI_CONFIG': '/root/.gz/sim/7/gui.config'
                    })

                    self.gazebo_process = subprocess.Popen(
                        cmd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        universal_newlines=True,
                        bufsize=1,
                        env=env
                    )
                    print(f"Command '{' '.join(cmd)}' started successfully")

                    if self.gazebo_process.stdout:
                        self.stdout_thread = threading.Thread(
                            target=self._stream_output,
                            args=(self.gazebo_process.stdout, 'STDOUT'),
                            daemon=True
                        )
                        self.stdout_thread.start()
                    if self.gazebo_process.stderr:
                        self.stderr_thread = threading.Thread(
                            target=self._stream_output,
                            args=(self.gazebo_process.stderr, 'STDERR'),
                            daemon=True
                        )
                        self.stderr_thread.start()

                    time.sleep(2)
                    if self.gazebo_process.poll() is None:
                        return True

                    print("Process exited quickly, trying next command...")
                except FileNotFoundError:
                    print(f"Command '{cmd[0]}' not found, trying next...")
                except Exception as e:
                    print(f"Error with command '{' '.join(cmd)}': {e}")

            return False
        except Exception as e:
            print(f"Failed to start Gazebo: {e}")
            return False
    
    async def handle_websocket(self, websocket, path):
        """Handle WebSocket connections"""
        self.clients.add(websocket)
        try:
            print(f"Client connected: {websocket.remote_address}")
            await websocket.send(json.dumps({
                "type": "status",
                "message": "Connected to Gazebo Web Server"
            }))
            
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.process_command(data, websocket)
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({
                        "type": "error",
                        "message": "Invalid JSON"
                    }))
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            print(f"Client disconnected")
    
    async def process_command(self, data, websocket):
        """Process commands from web client"""
        command_type = data.get("type")
        
        if command_type == "start_simulation":
            if not self.gazebo_process or self.gazebo_process.poll() is not None:
                if self.start_gazebo():
                    await websocket.send(json.dumps({
                        "type": "response",
                        "message": "Simulation started"
                    }))
                else:
                    await websocket.send(json.dumps({
                        "type": "error",
                        "message": "Failed to start simulation"
                    }))
        
        elif command_type == "stop_simulation":
            if self.gazebo_process and self.gazebo_process.poll() is None:
                self.gazebo_process.terminate()
                await websocket.send(json.dumps({
                    "type": "response",
                    "message": "Simulation stopped"
                }))
        
        elif command_type == "get_status":
            status = "running" if self.gazebo_process and self.gazebo_process.poll() is None else "stopped"
            await websocket.send(json.dumps({
                "type": "status",
                "simulation": status
            }))

def start_http_server():
    """Start HTTP server for web interface"""
    os.chdir('/gazebo/web')
    server = HTTPServer(('0.0.0.0', 8080), SimpleHTTPRequestHandler)
    print("HTTP server started on port 8080")
    server.serve_forever()

async def main():
    gazebo_server = GazeboWebServer()
    
    # Start HTTP server in a separate thread
    http_thread = threading.Thread(target=start_http_server)
    http_thread.daemon = True
    http_thread.start()
    
    # Start WebSocket server 
    print("Starting WebSocket server on port 7681")
    async with websockets.serve(gazebo_server.handle_websocket, "0.0.0.0", 7681):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())