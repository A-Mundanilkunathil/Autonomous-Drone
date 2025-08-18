import time
from pymavlink import mavutil
from session import MavSession
from math import radians

class VehicleAuto:
    def __init__(self, session: MavSession):
        self.s = session
        self.conn = session.conn
        
    def set_mode(self, name: str):
        mapping = self.conn.mode_mapping()
        if name not in mapping:
            raise RuntimeError(f"Mode {name} not available. Got: {list(mapping.keys())}")
        
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[name]
        )
        
    def arm(self, timeout=2):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        end = time.time() + timeout # Wait for arm confirmation
        while time.time() < end:
            heartbeat = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if heartbeat and (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return
        raise RuntimeError("Arm timeout")
    
    def disarm(self, timeout=2):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        end = time.time() + timeout # Wait for disarm confirmation
        while time.time() < end:
            heartbeat = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if heartbeat and not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return
        raise RuntimeError("Disarm timeout")
    
    def guided_takeoff(self, alt_m: float):
        self.set_mode("GUIDED")
        self.arm()
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 
            0, 0, 0, 0, 
            0, 0,
            alt_m
        )
        
    def land(self):
        mapping = self.conn.mode_mapping()
        if "LAND" in mapping:
            self.set_mode("LAND")
        else:
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            
    def send_velocity_body(self, vx, vy, vz, yaw_rate_deg_s=0.0):
        type_mask = 0
        
        type_mask |= (1<<0)|(1<<1)|(1<<2) # ignore position
        type_mask |= (1<<6)|(1<<7)|(1<<8) # ignore acceleration
        type_mask |= (1<<9)  # ignore force
        type_mask |= (1<<10)  # ignore yaw rate
        
        self.conn.mav.set_position_target_local_ned_send(
            int(time.time() * 1e3),
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask, 
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, radians(yaw_rate_deg_s)
        )
        
    def send_attitude(self, roll_deg, pitch_deg, yaw_deg=None, thrust=0.5):
        import math
        def euler_to_quat(r, p, y):
            # Convert Euler angles (roll, pitch, yaw) to quaternion
            # Roll
            cr = math.cos(r/2) 
            sr = math.sin(r/ 2)
            # Pitch
            cp = math.cos(p/2)
            sp = math.sin(p/2)
            # Yaw
            cy = math.cos(y/2)
            sy = math.sin(y/2)
            return [
                cr*cp*cy + sr*sp*sy,
                sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*cy
            ]
        roll = radians(roll_deg)
        pitch = radians(pitch_deg)
        if yaw_deg is None:
            yaw = 0.0
            typemask = 0b00000101 # ignore roll rate, pitch rate, yaw rate + yaw angle bit
        else: 
            yaw = radians(yaw_deg)
            typemask = 0b00000111 # ignore all body rates
        q = euler_to_quat(roll, pitch, yaw)
        self.conn.mav.set_attitude_target_send(
            int(time.time() * 1e6),
            self.conn.target_system,
            self.conn.target_component,
            typemask,
            q,
            0.0, 0.0, 0.0,
            thrust
        )
            

        