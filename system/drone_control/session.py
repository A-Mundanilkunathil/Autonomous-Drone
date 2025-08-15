from pymavlink import mavutil
import time
from typing import Optional

class MavSession:
    def __init__(self, port: str = 'COM6', baud: int = 57600, heartbeat_hz: int = 1):
        self.port = port
        self.baud = baud
        self.heartbeat_period = 1 / heartbeat_hz # Heartbeat interval in seconds
        self._m: Optional[mavutil.mavfile] = None # MAVLink connection
        self._last_hb = 0.0 # Last heartbeat time
        
    def connect(self):
        print(f"Connecting {self.port}@{self.baud}...")
        self._m = mavutil.mavlink_connection(self.port, baud=self.baud)
        self._m.wait_heartbeat()
        print(f"Connected to system {self._m.target_system} component {self._m.target_component}")
        return self
    
    @property
    def conn(self):
        if not self._m:
            raise RuntimeError("MAVLink connection not established. Call connect() first.")
        return self._m
    
    def pump(self):
        message = self.conn.recv_match(blocking=False)
        if not message:
            return
        
        if message.get_type() == "STATUSTEXT":
            txt = getattr(message, "text", b"")
            try:
                print(f"Received STATUSTEXT: {txt.decode()}")
            except Exception:
                pass
            
    def send_heartbeat(self):  
        now = time.time()
        if now - self._last_hb >= self.heartbeat_period:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,  # Ground Control Station
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # Autopilot type
                0, 0, 0 # System status
            )
            self._last_hb = now
