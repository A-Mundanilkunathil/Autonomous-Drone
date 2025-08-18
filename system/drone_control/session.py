from pymavlink import mavutil
import time
from typing import Optional
import logging
import threading

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("MavSession")

class MavSession:
    def __init__(self, port: str = 'COM6', baud: int = 57600, heartbeat_hz: int = 1):
        self.port = port
        self.baud = baud
        self.heartbeat_period = 1 / heartbeat_hz # Heartbeat interval in seconds
        self._m: Optional[mavutil.mavfile] = None # MAVLink connection
        self._last_hb = 0.0 # Last heartbeat time
        self._hb_timer: Optional[threading.Timer] = None # Heartbeat timer
        
    def connect(self):
        logger.info(f"Connecting {self.port}@{self.baud}...")
        self._m = mavutil.mavlink_connection(self.port, baud=self.baud)
        self._m.wait_heartbeat()
        logger.info(f"Connected to system {self._m.target_system} component {self._m.target_component}")
        return self
    
    @property
    def conn(self):
        if not self._m:
            raise RuntimeError("MAVLink connection not established. Call connect() first.")
        return self._m
    
    def pump(self):
        message = self.recv(blocking=False)
        if not message:
            return
        
        if message.get_type() == "STATUSTEXT":
            txt = getattr(message, "text", b"")
            try:
                logger.info(f"Received STATUSTEXT: {txt.decode()}")
            except Exception:
                logger.info(f"Received STATUSTEXT: {txt}")
        else:
            logger.debug(f"Received message: {message}")

    def send_heartbeat(self):  
        now = time.time()
        if now - self._last_hb >= self.heartbeat_period:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,  # Ground Control Station 
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # Autopilot type
                0, 0, 0 # System status
            )
            self._last_hb = now
            logger.debug("Heartbeat sent")

    def close(self):
        if self._m:
            self._m.close()
            self.stop_heartbeat()
            logger.info("MAVLink connection closed")

    def recv(self, msg_type=None, blocking=True, timeout=1.0):
        return self._m.recv_match(type=msg_type, blocking=blocking, timeout=timeout)

    def start_heartbeat(self):
        def hb_loop():
            self.send_heartbeat()
            self._hb_timer = threading.Timer(self.heartbeat_period, hb_loop, daemon=True)
            self._hb_timer.start()
        
        if not hasattr(self, "_hb_timer") or not self._hb_timer.is_alive():
            hb_loop()
            logger.info("Heartbeat thread started")
    
    def stop_heartbeat(self):
        if hasattr(self, "_hb_timer"):
            self._hb_timer.cancel()
            logger.info("Heartbeat thread stopped")