from pymavlink import mavutil
import time
from typing import Optional
import logging
import threading

logger = logging.getLogger("MavSession")

class MavSession:
    def __init__(self, port: str = 'COM6', baud: int = 57600, heartbeat_hz: int = 1):
        self.port = port
        self.baud = baud
        self.heartbeat_period = 1 / heartbeat_hz # Heartbeat interval in seconds
        self._m: Optional[mavutil.mavfile] = None # MAVLink connection

        # Heartbeat control
        self._last_hb = 0.0 # Last heartbeat time
        self._hb_thread: Optional[threading.Thread] = None
        self._stop_hb_event = threading.Event()

        # Pump control
        self._pump_thread: Optional[threading.Thread] = None
        self._stop_pump_event = threading.Event()

        # Last warning storage
        self.last_warning: Optional[str] = None
        self.last_warning_time: float = 0.0
    
    # ---------------- Connection ---------------- 
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
    
    def close(self):
        self.stop_pump()
        self.stop_heartbeat()
        if self._m:
            self._m.close()
            self._m = None
            logger.info("MAVLink connection closed")
    
    # ---------------- Message Handling ----------------
    def recv(self, msg_type=None, blocking=True, timeout=1.0):
        """
        Receive a MAVLink message.

        :param msg_type: The type of MAVLink message to receive.
        :param blocking: If True, the function will block until a message is received.
        :param timeout: If blocking is True, the function will timeout after this many seconds.
        :return: The received MAVLink message, or None if no message was received.
        """
        return self._m.recv_match(type=msg_type, blocking=blocking, timeout=timeout)

    def pump(self):
        """
        Drain the MAVLink RX queue and capture key messages.

        - Reads only 'safe' types (STATUSTEXT, SYS_STATUS, HEARTBEAT)
        - Stores last STATUSTEXT warning/error into self.last_warning
        - Keeps the buffer clean to prevent backlog latency
        """
        if not self._m:
            return
        
        message = self._m.recv_match(
            type=["STATUSTEXT", "SYS_STATUS", "HEARTBEAT"],
            blocking=False, timeout=0.0
        )
        if not message:
            return
        
        mtype = message.get_type()
        if mtype == "STATUSTEXT":
            txt = getattr(message, "text", b"")
            try:
                txt = txt.decode() if isinstance(txt, bytes) else txt
            except Exception:
                txt = str(txt)

            # Save the last warning message
            self.last_warning = txt
            self.last_warning_time = time.time()

            # Log only significant ones
            if any(k in txt.lower() for k in ("error", "warn", "prearm", "ekf", "gps", "failsafe")):
                logger.warning(f"[STATUSTEXT] {txt}")
            else:
                logger.debug(f"[STATUSTEXT] {txt}")
        return 
    
    # ---------------- Pump Thread ----------------
    def start_pump(self, hz: float = 20.0):
        """
        Run pump() in a background thread to continuously drain and log warnings.
        Keeps RX buffer fresh without consuming navigation messages.
        """
        if self._pump_thread and self._pump_thread.is_alive():
            logger.debug("Pump thread is already running.")
            return  
        
        self._stop_pump_event.clear() # Clear the stop event
        period = max(0.005, 1 / float(hz)) # clamp rate to ≤ ~200 Hz (min sleep 5 ms)

        def _pump_loop():
            while not self._stop_pump_event.is_set():
                try:
                    # Drain messages
                    for _ in range(5):
                        self.pump()
                except Exception as e:
                    logger.debug(f"Pump failed: {e}")
                time.sleep(period)

        self._pump_thread = threading.Thread(target=_pump_loop, daemon=True)
        self._pump_thread.start()
        logger.info("Pump thread started")

    def stop_pump(self):
        if self._pump_thread and self._pump_thread.is_alive():
            self._stop_pump_event.set()
            self._pump_thread.join(timeout=0.5)
            logger.info("Pump thread stopped")
        self._pump_thread = None
    # ---------------- Heartbeat ----------------
    def send_heartbeat(self):  
        """
        Send a heartbeat message to the MAVLink connection.

        This function will send a heartbeat message if the time since the last heartbeat
        is greater than or equal to the heartbeat period.

        :return: None
        """
        now = time.time()
        if now - self._last_hb >= self.heartbeat_period:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,  # Ground Control Station 
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # Autopilot type
                0, 0, 0 # System status
            )
            self._last_hb = now
            logger.debug("Heartbeat sent")

    def start_heartbeat(self):
        if self._hb_thread and self._hb_thread.is_alive():
            logger.warning("Heartbeat thread is already running.")
            return
        
        self._stop_hb_event.clear() # Clear the stop event

        def _heartbeat_loop():
            """Main loop for the heartbeat thread."""
            while not self._stop_hb_event.is_set():
                try:
                    self.send_heartbeat()
                except Exception as e:
                    logger.warning(f"Heartbeat send failed: {e}")
                self._stop_hb_event.wait(self.heartbeat_period)

        self._hb_thread = threading.Thread(target=_heartbeat_loop, daemon=True)
        self._hb_thread.start()
        logger.info("Heartbeat thread started")
    
    def stop_heartbeat(self):
        if self._hb_thread and self._hb_thread.is_alive():
            self._stop_hb_event.set()
            self._hb_thread.join(timeout=self.heartbeat_period * 2)
            logger.info("Heartbeat thread stopped")
        self._hb_thread = None