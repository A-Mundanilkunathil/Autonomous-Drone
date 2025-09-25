import time
from pymavlink import mavutil
from session import MavSession
import math
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("VehicleAuto")

class VehicleAuto:
    def __init__(self, session: MavSession):
        self.s = session
        self.conn = session.conn
    
    def recv_msg(self, msg_type=None, timeout=1.0):
        return self.s.recv(msg_type, timeout=timeout)
    
    def wait_mode(self, name, timeout=3.0):
        mapping = self.conn.mode_mapping()
        want = mapping[name]
        end = time.time() + timeout
        while time.time() < end:
            hb = self.recv_msg('HEARTBEAT', timeout=0.3)
            if hb and getattr(hb, 'custom_mode', None) == want:
                return True
        raise RuntimeError(f"Mode change to {name} timed out")
        
    def set_mode(self, name: str):
        mapping = self.conn.mode_mapping()
        if name not in mapping:
            raise RuntimeError(f"Mode {name} not available. Got: {list(mapping.keys())}")
        
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[name]
        )

        self.wait_mode(name, timeout=3.0)

    def wait_command_ack(self, cmd, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            ack = self.recv_msg('COMMAND_ACK', timeout=0.3)
            if ack and ack.command == cmd:
                if ack.result not in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                      mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                    raise RuntimeError(f"Command {cmd} failed: {ack.result}")
                return
        raise RuntimeError(f"No COMMAND_ACK for {cmd}")

    def arm(self, wait=True, timeout=5):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        self.wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=3.0)
        if not wait:
            return
        
        end = time.time() + timeout # Wait for arm confirmation
        while time.time() < end:
            heartbeat = self.recv_msg('HEARTBEAT')
            if heartbeat and (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                logger.info("Vehicle armed successfully")
                return
        raise RuntimeError("Arm timeout")

    def disarm(self, wait=True, timeout=5):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        self.wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=3.0)
        if not wait:
            return
        
        end = time.time() + timeout # Wait for disarm confirmation
        while time.time() < end:
            heartbeat = self.recv_msg('HEARTBEAT')
            if heartbeat and not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                logger.info("Vehicle disarmed successfully")
                return
        raise RuntimeError("Disarm timeout")

    def guided_takeoff(self, target_alt: float, wait=True, timeout=15):
        self.set_mode("GUIDED")
        self.arm()
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_alt
        )
        
        self.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout=3.0)
        if not wait:
            return
        
        end = time.time() + timeout
        while time.time() < end:
            msg = self.recv_msg('GLOBAL_POSITION_INT')
            if msg and msg.relative_alt >= target_alt * 1000 * 0.95:
                logger.info(f"Takeoff reached {msg.relative_alt / 1000:.1f} m")
                return
        raise RuntimeError("Takeoff timeout")

    def land(self, wait=True, timeout=15):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        self.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_LAND, timeout=3.0)
        if not wait:
            return
        
        end = time.time() + timeout
        while time.time() < end:
            msg = self.recv_msg('GLOBAL_POSITION_INT')
            if msg and msg.relative_alt <= 500:
                logger.info("Landing complete")
                return
        raise RuntimeError("Landing timeout")

    def send_velocity_body(self, vx, vy, vz, yaw_rate_deg_s=0.0):
        """Stream a BODY_NED velocity setpoint (m/s) and yaw rate (deg/s).
        BODY_NED axes: 
            +x: forward
            +y: right
            +z: down
        """
        # Safety clamps
        vx = max(-3.0, min(3.0, vx))
        vy = max(-3.0, min(3.0, vy))
        vz = max(-2.0, min(2.0, vz))
        
        # Build type mask
        ignore_pos = (1<<0)|(1<<1)|(1<<2) 
        ignore_accel = (1<<6)|(1<<7)|(1<<8)
        ignore_force = (1<<9)  
        ignore_yaw = 0
        
        # if not using yaw rate
        if abs(yaw_rate_deg_s) < 1e-3:
            ignore_yaw |= (1<<10) | (1<<11) # ignore yaw angle and rate
            yaw_rate = 0.0
        else:
            ignore_yaw |= (1<<10) # ignore yaw angle
            yaw_rate = math.radians(yaw_rate_deg_s)
            
        type_mask = ignore_pos | ignore_accel | ignore_force | ignore_yaw
            
        self.conn.mav.set_position_target_local_ned_send(
            0, # ms, 0 = ignored
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask, 
            0, 0, 0, # position (ignored)
            vx, vy, vz, # velocity
            0, 0, 0, # acceleration (ignored)
            0, # yaw angle (ignored if bit 10 is set)
            yaw_rate # yaw rate (ignored if bit 11 is set)
        )
        
    def send_position_setpoint(self, x, y, z):
        self.conn.mav.set_position_target_local_ned_send(
            0, # ms, 0 = ignored
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            x, y, z,     # Position
            0, 0, 0,    # Velocity ignored
            0, 0, 0,    # Acceleration ignored
            0, 0        # Yaw, Yaw rate ignored
        )
        
    def send_attitude(self, roll_deg, pitch_deg, yaw_deg=None, thrust=0.5):
        # Safety clamps
        thrust = max(0.0, min(1.0, thrust))
        roll_deg = max(-35.0, min(35.0, roll_deg))
        pitch_deg = max(-35.0, min(35.0, pitch_deg))
        
        # Convert Euler angles (roll, pitch, yaw) to quaternion
        def euler_to_quat(r, p, y):
            # Roll
            cr, sr = math.cos(r/2), math.sin(r/2)
            # Pitch
            cp, sp = math.cos(p/2), math.sin(p/2)
            # Yaw
            cy, sy = math.cos(y/2), math.sin(y/2)
            return [
                cr*cp*cy + sr*sp*sy, # w
                sr*cp*cy - cr*sp*sy, # x
                cr*sp*cy + sr*cp*sy, # y
                cr*cp*sy - sr*sp*cy  # z
            ]
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)

        if yaw_deg is None:
            # Hold current yaw
            att = self.recv_msg('ATTITUDE')
            yaw = att.yaw if att else 0.0
        else: 
            yaw = math.radians(yaw_deg)
       
        q = euler_to_quat(roll, pitch, yaw)
        
        # Ignore all body rates
        typemask = (1<<0) | (1<<1) | (1<<2) # ignore roll, pitch, yaw rates
        self.conn.mav.set_attitude_target_send(
            0, # ms, 0 = ignored
            self.conn.target_system,
            self.conn.target_component,
            typemask,
            q, # quaternion [w,x,y,z]
            0.0, 0.0, 0.0, # body rates (ignored)
            thrust
        )
            
    def hold_position(self):
        # Switch to holding mode
        self.set_mode("LOITER")
        
        for _ in range(5):
            # Send zero velocities to stop motion
            self.send_velocity_body(0.0, 0.0, 0.0, yaw_rate_deg_s=0.0)
            time.sleep(0.1)
    
    def _stream_velocity_body(self, vx, vy, vz, yaw_rate_deg_s=0.0, duration=2.0, rate_hz=10):
        rate_hz = max(1, rate_hz) # Ensure rate is at least 1
        dt = 1.0 / rate_hz
        t_end = time.time() + duration
        while time.time() < t_end:
            self.send_velocity_body(vx, vy, vz, yaw_rate_deg_s=yaw_rate_deg_s)
            time.sleep(dt)

    def move_forward(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(vx=speed, vy=0.0, vz=0.0, duration=duration, rate_hz=rate_hz)

    def move_backward(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(vx=-speed, vy=0.0, vz=0.0, duration=duration, rate_hz=rate_hz)
        
    def move_right(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(vx=0.0, vy=speed, vz=0.0, duration=duration, rate_hz=rate_hz)
        
    def move_left(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(vx=0.0, vy=-speed, vz=0.0, duration=duration, rate_hz=rate_hz)
        
    def move_up(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(vx=0.0, vy=0.0, vz=-speed, duration=duration, rate_hz=rate_hz)

    def move_down(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(vx=0.0, vy=0.0, vz=speed, duration=duration, rate_hz=rate_hz)
        
    def rotate(self, yaw_rate_deg_s=30, duration=2.0, rate_hz=10):
        self._stream_velocity_body(vx=0.0, vy=0.0, vz=0.0, yaw_rate_deg_s=yaw_rate_deg_s, duration=duration, rate_hz=rate_hz)
        
    def move_diagonal_front_right(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(speed, speed, 0.0, duration=duration, rate_hz=rate_hz)

    def move_diagonal_front_left(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(speed, -speed, 0.0, duration=duration, rate_hz=rate_hz)

    def move_diagonal_back_right(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(-speed, speed, 0.0, duration=duration, rate_hz=rate_hz)

    def move_diagonal_back_left(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(-speed, -speed, 0.0, duration=duration, rate_hz=rate_hz)
    
    def stop(self, duration=0.3, rate_hz=10):
        self._stream_velocity_body(vx=0.0, vy=0.0, vz=0.0, duration=duration, rate_hz=rate_hz)
        
    def return_to_home(self):
        # Switch to RTL (Return to Launch) mode
        self.set_mode("RTL")
        
    def move_square(self, speed=1.0):
        self.move_forward(speed)
        self.move_right(speed)
        self.move_backward(speed)
        self.move_left(speed)
        
        self.hold_position()
        
    def move_circular_arc(self, radius=5.0, speed=1.0, duration=20):
        angular_velocity = speed / radius # rad/s
        start_time = time.time()
        
        while time.time() - start_time < duration:
            vx = speed                      # forward velocity
            vy = radius * angular_velocity  # rightward to curve
            self.send_velocity_body(vx, vy, 0.0)
            time.sleep(0.1)
            
        self.hold_position()
        
    def move_circle_global(self, radius=5.0, speed=1.0, duration=20, update_interval=0.1):
        # Get starting position
        pos = self.recv_msg('LOCAL_POSITION_NED')
        if not pos:
            raise RuntimeError("No LOCAL_POSITION_NED message received")
        
        xc, yc, z = pos.x, pos.y, pos.z
        angular_velocity = speed / radius # rad/s
        start_time = time.time()
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            theta = angular_velocity * t
            x_target = xc + radius * math.cos(theta)
            y_target = yc + radius * math.sin(theta)
            self.send_position_setpoint(x_target, y_target, z)
            time.sleep(update_interval)
        
        self.hold_position()