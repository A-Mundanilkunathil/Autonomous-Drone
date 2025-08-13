from pymavlink import mavutil
import time
from session import MavSession
 
class Vehicle:
    def __init__(self, session: MavSession, wait_attempts: int = 10):
        self.s = session
        self.conn = session.conn
        self.wait_attempts = wait_attempts

    def set_mode(self, name: str = "STABILIZE"):
        mapping = self.conn.mode_mapping()
        if name not in mapping:
            raise RuntimeError(f"Mode {name} not available. Got: {list(mapping.keys())}")
        
        mode_id = mapping[name]
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        # Confirm via heartbeat
        if not self._wait(lambda hb: hb.custom_mode == mode_id, desc=f"mode {name}"):
            raise RuntimeError(f"Failed to confirm mode {name}")
        print(f"Mode set: {name}")
    
    def arm(self):
        self._command_arm_disarm(1, "arm")
        if not self._wait(self._is_armed, "arming"):
            raise RuntimeError("Failed to arm vehicle")
        print("Armed")
        
    def disarm(self):
        self._command_arm_disarm(0, "disarm")
        if not self._wait(lambda hb: not self._is_armed(hb), "disarming"):
            raise RuntimeError("Disarm failed")
        print("Disarmed")
        
    def _command_arm_disarm(self, arm_val: int, label: str):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm_val,
            0, 0, 0, 0, 0, 0
        )
        ack = self._wait_ack() 
        if ack is not None and ack.result not in (
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS
        ):
            raise RuntimeError(f"{label.capitalize()} command failed: {ack.result}")
    
    def _wait_ack(self):
        for _ in range(self.wait_attempts):
            message = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if message:
                return message
            time.sleep(0.2)
        return None
    
    def _wait(self, predicate, desc="condition"):
        for _ in range(self.wait_attempts):
            heartbeat = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if heartbeat and predicate(heartbeat):
                return True
        return False
    
    @staticmethod
    def _is_armed(heartbeat):
        return (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0