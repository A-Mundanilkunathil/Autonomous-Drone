from pymavlink import mavutil
import time
from session import MavSession
 
class Vehicle:
    def __init__(self, session: MavSession, wait_attempts: int = 10):
        self.s = session
        self.conn = session.conn
        self.wait_attempts = wait_attempts
        # RC override state
        self._rc_active = False
        self._last_rc = [0]*8 # 8 channels
        # PWM constants
        self.PWM_MIN = 1000
        self.PWM_MAX = 2000
        self.PWM_NEUTRAL = 1500
        # Default stick values
        self._default_sticks()
    
    def _default_sticks(self):
        self._last_rc[0] = self.PWM_NEUTRAL  # Roll
        self._last_rc[1] = self.PWM_NEUTRAL  # Pitch
        self._last_rc[2] = self.PWM_MIN  # Throttle
        self._last_rc[3] = self.PWM_NEUTRAL  # Yaw
        
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
        # Clear RC override cache
        self.disable_rc_override()

    # --- INTERNAL COMMAND / WAIT HELPERS ---
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
    
    # --- RC OVERRIDE CONTROL LAYER ---
    def enable_rc_override(self):
        if not self._rc_active:
            self._rc_active = True
            self._send_rc() # Send initial state
            print("RC override enabled")
    
    def disable_rc_override(self):
        if self._rc_active:
            # Sending zeros releases override
            self.conn.mav.rc_channels_override_send(
                self.conn.target_system,
                self.conn.target_component,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            self._rc_active = False
            print("RC override disabled")
    
    def _clamp(self, v):
        # Clamp the value between PWM_MIN and PWM_MAX
        return max(self.PWM_MIN, min(self.PWM_MAX, v))
    
    def _send_rc(self):
        if not self._rc_active:
            return
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,
            self.conn.target_component,
            *self._last_rc[:8] # Ensure we only send 8 channels
        )
        
    def set_sticks(self, *, roll=None, pitch=None, throttle=None, yaw=None):
        self.enable_rc_override()        
        if roll is not None:
            self._last_rc[0] = self._clamp(roll)
        if pitch is not None:
            self._last_rc[1] = self._clamp(pitch)
        if throttle is not None:
            self._last_rc[2] = self._clamp(throttle)
        if yaw is not None:
            self._last_rc[3] = self._clamp(yaw)
        self._send_rc()
        
    def inc_throttle(self, delta=25):
        self.enable_rc_override()
        self._last_rc[2] = self._clamp(self._last_rc[2] + delta)
        self._send_rc()
        return self._last_rc[2]
    
    def dec_throttle(self, delta=25):
        return self.inc_throttle(-delta)
    
    def roll_left(self, delta=50):
        self.enable_rc_override()
        self._last_rc[0] = self._clamp(self.PWM_NEUTRAL - delta)
        self._send_rc()

    def roll_right(self, delta=50):
        self.enable_rc_override()
        self._last_rc[0] = self._clamp(self.PWM_NEUTRAL + delta)
        self._send_rc()

    def pitch_forward(self, delta=50):
        self.enable_rc_override()
        self._last_rc[1] = self._clamp(self.PWM_NEUTRAL - delta)
        self._send_rc()

    def pitch_backward(self, delta=50):
        self.enable_rc_override()
        self._last_rc[1] = self._clamp(self.PWM_NEUTRAL + delta)
        self._send_rc()

    def yaw_left(self, delta=50):
        self.enable_rc_override()
        self._last_rc[3] = self._clamp(self.PWM_NEUTRAL - delta)
        self._send_rc()

    def yaw_right(self, delta=50):
        self.enable_rc_override()
        self._last_rc[3] = self._clamp(self.PWM_NEUTRAL + delta)
        self._send_rc()
    
    def center_attitude(self):
        self.enable_rc_override()
        self._last_rc[0] = self.PWM_NEUTRAL # Roll
        self._last_rc[1] = self.PWM_NEUTRAL # Pitch
        self._last_rc[3] = self.PWM_NEUTRAL # Yaw
        self._send_rc()

    def set_throttle_percent(self, pct: float):
        pct = max(0.0, min(100.0, pct))
        pwm = int(self.PWM_MIN + (pct/100.0)*(self.PWM_MAX - self.PWM_MIN))
        self.set_sticks(throttle=pwm)
        return pwm
