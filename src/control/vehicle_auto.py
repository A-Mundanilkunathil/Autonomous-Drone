import time, math, logging
from pymavlink import mavutil
from vehicle_motion import VehicleMotion

log = logging.getLogger("VehicleAuto")

class VehicleAuto(VehicleMotion):
    """High-level behaviors composed from motion + base primitives."""

    # Helpers
    def _meters_between(self, lat1, lon1, lat2, lon2) -> float:
        # Equirectangular approximation
        R = 6378137.0 # Earth radius in meters
        x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
        y = math.radians(lat2 - lat1)
        return math.sqrt(x*x + y*y) * R

    # Actions
    def goto_latlon(self,
                    lat_deg: float,
                    lon_deg: float,
                    alt_rel_m: float, # altitude relative to home
                    pos_tol_m: float = 1.5, # position tolerance
                    alt_tol_m: float = 0.7, # altitude tolerance
                    rate_hz: float = 10.0,
                    timeout: float = 90.0) -> None:
        """
        Fly to a global (lat,lon,alt_rel) in GUIDED by streaming GLOBAL_RELATIVE_ALT_INT position targets.
        """
        # Guided mode
        self.set_mode("GUIDED")
        
        dt = 1.0 / max(1.0, rate_hz)
        end = time.time() + timeout
        while time.time() < end:
            # Stream target position
            self.send_position_target_global(lat_deg, lon_deg, alt_rel_m)

            # Check current position
            gpi = self.recv_msg('GLOBAL_POSITION_INT', timeout=0.2)
            if gpi:
                cur_lat = gpi.lat / 1e7
                cur_lon = gpi.lon / 1e7
                cur_alt = gpi.relative_alt / 1000.0 # mm -> m

                dist_xy = self._meters_between(cur_lat, cur_lon, lat_deg, lon_deg)
                alt_err = abs(cur_alt - alt_rel_m)

                if dist_xy < pos_tol_m and alt_err < alt_tol_m:
                    return
                
            time.sleep(dt)

        self.hold_position()
        raise RuntimeError("goto_latlon timeout")

    def guided_takeoff(self, target_alt: float, wait: bool=True, timeout: float=30.0) -> None:
        self.set_mode("GUIDED")
        self.arm()
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0,0,0,0,0,0,0, target_alt
        )
        self.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 3.0)
        if not wait:
            return
        end = time.time() + timeout
        while time.time() < end:
            gpi = self.recv_msg('GLOBAL_POSITION_INT', timeout=0.3)
            if gpi and gpi.relative_alt >= target_alt * 1000 * 0.95:
                log.info("Takeoff complete")
                return
        raise RuntimeError("Takeoff timeout")

    def land(self, wait: bool=True, timeout: float=45.0) -> None:
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0,0,0,0,0,0,0,0
        )
        self.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_LAND, 3.0)
        if not wait:
            return
        end = time.time() + timeout
        while time.time() < end:
            gpi = self.recv_msg('GLOBAL_POSITION_INT', timeout=0.3)
            if gpi and gpi.relative_alt <= 500:  # 0.5 m
                log.info("Landing complete")
                return
        raise RuntimeError("Landing timeout")

    def return_to_home(self) -> None:
        self.set_mode("RTL")

    # Motion patterns (all streamed)
    def move_forward(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(speed, 0.0, 0.0, 0.0, duration, rate_hz)
    def move_backward(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(-speed, 0.0, 0.0, 0.0, duration, rate_hz)
    def move_right(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(0.0, speed, 0.0, 0.0, duration, rate_hz)
    def move_left(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(0.0,-speed, 0.0, 0.0, duration, rate_hz)
    def move_up(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(0.0, 0.0, -speed, 0.0, duration, rate_hz)  # up = -z
    def move_down(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(0.0, 0.0,  speed, 0.0, duration, rate_hz)
    def rotate(self, yaw_rate_deg_s=30, duration=2.0, rate_hz=10):
        self._stream_velocity_body(0.0,0.0,0.0,yaw_rate_deg_s,duration,rate_hz)

    def move_diagonal_front_right(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(speed, speed, 0.0, 0.0, duration, rate_hz)
    def move_diagonal_front_left(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(speed,-speed,0.0, 0.0, duration, rate_hz)
    def move_diagonal_back_right(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(-speed, speed,0.0, 0.0, duration, rate_hz)
    def move_diagonal_back_left(self, speed=1.0, duration=2.0, rate_hz=10):
        self._stream_velocity_body(-speed,-speed,0.0, 0.0, duration, rate_hz)

    def move_square(self, speed=1.0, leg_s=3.0, rate_hz=10):
        self.move_forward(speed, leg_s, rate_hz)
        self.move_right(speed,   leg_s, rate_hz)
        self.move_backward(speed,leg_s, rate_hz)
        self.move_left(speed,    leg_s, rate_hz)
        self.hold_position()

    def move_circle_global(self, radius=5.0, speed=1.0, duration=20, update_interval=0.1):
        pos = self.recv_msg('LOCAL_POSITION_NED', timeout=0.5)
        if not pos:
            raise RuntimeError("No LOCAL_POSITION_NED message received")
        xc, yc, z = pos.x, pos.y, pos.z
        omega = speed / radius
        start = time.time()
        while time.time() - start < duration:
            t = time.time() - start
            x = xc + radius * math.cos(omega * t)
            y = yc + radius * math.sin(omega * t)
            self.send_position_setpoint(x, y, z)
            time.sleep(update_interval)
        self.hold_position()
