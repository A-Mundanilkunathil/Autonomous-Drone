import time, math, logging
from pymavlink import mavutil
from vehicle_base import VehicleBase

log = logging.getLogger("VehicleMotion")

MAX_VXY = 3.0  # m/s
MAX_VZ  = 2.0  # m/s

class VehicleMotion(VehicleBase):
    """Velocity/position/attitude setpoints + streaming helpers."""

    def _type_mask_velocity_only(self, use_yaw_rate: bool) -> int:
        ignore_pos   = (1<<0)|(1<<1)|(1<<2)
        ignore_accel = (1<<6)|(1<<7)|(1<<8)
        use_force    = (1<<9)  # irrelevant since accel ignored
        ignore_yaw   = (1<<10) | (0 if use_yaw_rate else (1<<11))
        return ignore_pos | ignore_accel | use_force | ignore_yaw

    def send_velocity_body(self, vx, vy, vz, yaw_rate_deg_s=0.0) -> None:
        # clamps
        vx = max(-MAX_VXY, min(MAX_VXY, vx))
        vy = max(-MAX_VXY, min(MAX_VXY, vy))
        vz = max(-MAX_VZ,  min(MAX_VZ,  vz))
        use_yaw_rate = abs(yaw_rate_deg_s) >= 1e-3
        type_mask = self._type_mask_velocity_only(use_yaw_rate)

        self.conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms ignored
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0,0,0,           # pos ignored
            vx,vy,vz,        # velocity
            0,0,0,           # accel ignored
            0,               # yaw angle ignored
            math.radians(yaw_rate_deg_s) if use_yaw_rate else 0.0,
        )

    def send_position_setpoint(self, x, y, z) -> None:
        self.conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms ignored
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # position only
            x, y, z,
            0,0,0,  0,0,0,  0,0
        )
    
    def send_position_target_global(self, lat_deg: float, lon_deg: float, alt_rel_m: float) -> None:
        """
        Send a GLOBAL_RELATIVE_ALT_INT position target (lat/lon in degrees, alt in meters AGL).
        Uses position-only mask (ignores vel/accel/yaw). Stream at 2–10 Hz until reached.
        """
        lat_i = int(lat_deg * 1e7)
        lon_i = int(lon_deg * 1e7)
        alt = float(alt_rel_m)

        # Use only position (ignore velocity, accel, yaw, yaw rate)
        type_mask = (
            (1<<3) | (1<<4) | (1<<5) | # ignore vx, vy, vz
            (1<<6) | (1<<7) | (1<<8) |  # ignore ax,ay,az
            (1<<9) |                     # is_force flag
            (1<<10)| (1<<11)             # ignore yaw, yaw_rate
        )

        self.conn.mav.set_position_target_global_int_send(
            0,  # time_boot_ms ignored
            self.conn.target_system, 
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            lat_i, lon_i, alt,
            0,0,0,  
            0,0,0,  
            0,0
        )

    def send_attitude(self, roll_deg, pitch_deg, yaw_deg=None, thrust=0.5) -> None:
        thrust = max(0.0, min(1.0, thrust))
        roll, pitch = math.radians(roll_deg), math.radians(pitch_deg)

        att = self.recv_msg('ATTITUDE', timeout=0.1) if yaw_deg is None else None
        yaw = att.yaw if att and yaw_deg is None else math.radians(yaw_deg or 0.0)

        def euler_to_quat(r, p, y):
            cr, sr = math.cos(r/2), math.sin(r/2)
            cp, sp = math.cos(p/2), math.sin(p/2)
            cy, sy = math.cos(y/2), math.sin(y/2)
            return [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                    cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]

        q = euler_to_quat(roll, pitch, yaw)
        typemask = (1<<0)|(1<<1)|(1<<2)  # ignore body rates

        self.conn.mav.set_attitude_target_send(
            0,  # time_boot_ms ignored
            self.conn.target_system, self.conn.target_component,
            typemask, q, 0.0, 0.0, 0.0, thrust
        )

    def _stream_velocity_body(self, vx, vy, vz, yaw_rate_deg_s=0.0,
                              duration=2.0, rate_hz=10) -> None:
        rate_hz = max(1.0, float(rate_hz))
        dt = 1.0 / rate_hz
        end = time.time() + duration
        while time.time() < end:
            self.send_velocity_body(vx, vy, vz, yaw_rate_deg_s)
            time.sleep(dt)

    def stop(self, duration=0.3, rate_hz=10) -> None:
        self._stream_velocity_body(0.0, 0.0, 0.0, 0.0, duration, rate_hz)

    def hold_position(self) -> None:
        self.set_mode("LOITER")
        for _ in range(5): 
            self.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            time.sleep(0.1)
