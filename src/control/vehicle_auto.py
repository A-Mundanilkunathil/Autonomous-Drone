import time, math, logging
from pymavlink import mavutil
from vehicle_motion import VehicleMotion

log = logging.getLogger("VehicleAuto")

class VehicleAuto(VehicleMotion):
    """High-level behaviors composed from motion + base primitives."""

    # ------------------- Helpers -------------------
    def _meters_between(self, lat1, lon1, lat2, lon2) -> float:
        """
        Calculate the distance in meters between two lat/lon points using the Equirectangular approximation.

        :param lat1: Latitude of the first point in degrees
        :param lon1: Longitude of the first point in degrees
        :param lat2: Latitude of the second point in degrees
        :param lon2: Longitude of the second point in degrees
        :return: Distance between the two points in meters
        """
        R = 6378137.0 # Earth radius in meters
        x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
        y = math.radians(lat2 - lat1)
        return math.sqrt(x*x + y*y) * R

    # ------------------- Actions -------------------
    def goto_latlon(self,
                    lat_deg: float,
                    lon_deg: float,
                    alt_rel_m: float, # altitude relative to home
                    pos_tol_m: float = 1.5, # position tolerance
                    alt_tol_m: float = 0.7, # altitude tolerance
                    rate_hz: float = 10.0,
                    timeout: float = 90.0) -> None:
        """
        Go to a target position and altitude in GUIDED mode.

        Will timeout if the vehicle does not reach the target position
        within the given time limit.

        :param lat_deg: target latitude in degrees
        :param lon_deg: target longitude in degrees
        :param alt_rel_m: target altitude relative to home in meters
        :param pos_tol_m: position tolerance in meters
        :param alt_tol_m: altitude tolerance in meters
        :param rate_hz: rate at which to send position setpoints
        :param timeout: time limit in seconds
        :return: None
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

                # Check if we've reached the target
                dist_xy = self._meters_between(cur_lat, cur_lon, lat_deg, lon_deg)
                alt_err = abs(cur_alt - alt_rel_m)

                if dist_xy < pos_tol_m and alt_err < alt_tol_m:
                    return
                
            time.sleep(dt)

        self.hold_position()
        raise RuntimeError("goto_latlon timeout")

    def guided_takeoff(self, target_alt: float, wait: bool=True, timeout: float=30.0) -> None:
        """
        Takeoff in GUIDED mode.

        Will arm the vehicle, send a takeoff command and wait for the vehicle to reach the target altitude.

        Will timeout if the vehicle does not reach the target altitude within the given time limit.

        :param target_alt: target altitude in meters
        :param wait: whether to wait for the vehicle to reach the target altitude
        :param timeout: time limit in seconds
        :return: None
        """
        self.set_mode("GUIDED")
        self.arm()
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, target_alt  
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

    def land(self, wait: bool=True, timeout: float=60.0) -> None:
        """
        Land in LAND mode.

        Will send a land command and wait for the vehicle to land.

        Will timeout if the vehicle does not land within the given time limit.

        :param wait: whether to wait for the vehicle to land
        :param timeout: time limit in seconds
        :return: None
        """
        self.set_mode("LAND")
        if not wait:
            return
        end = time.time() + timeout
        while time.time() < end:
            # Check landed state if available
            ess = self.recv_msg('EXTENDED_SYS_STATE', timeout=0.3)
            if ess and getattr(ess, 'landed_state', None) == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                log.info("Landing complete (landed state)")
                time.sleep(2.0) # Wait for land to complete
                return
            
            # Fallback to GLOBAL_POSITION_INT altitude + vertical speed
            gpi = self.recv_msg('GLOBAL_POSITION_INT', timeout=0.3)
            if gpi:
                rel_alt_m = gpi.relative_alt / 1000.0 # mm -> m
                vz_mps = gpi.vz / 100.0 # cm/s -> m/s

                # If altitude < 0.5m and vertical speed < 0.2m/s
                if rel_alt_m <= 0.5 and abs(vz_mps) < 0.2:
                    log.info("Landing complete (alt + speed)")
                    time.sleep(2.0) # Wait for land to complete
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

    # TODO: Movements with rotation
    def turn_right(self):
        self.rotate(35, 2.0, 10)
    def turn_right_diagonal(self):
        self.rotate(45, 2.0, 10)
    def turn_left(self):
        self.rotate(-35, 2.0, 10)
    def turn_left_diagonal(self):
        self.rotate(-45, 2.0, 10)
    def turn_around(self):
        self.rotate(180, 2.0, 10)

    # TODO: add CW and CCW parameters
    def move_square(self, speed=1.0, leg_s=3.0, rate_hz=10):
        if self.get_mode() != 'GUIDED':
            self.set_mode('GUIDED')
        self.move_forward(speed, leg_s, rate_hz)
        self.move_right(speed,   leg_s, rate_hz)
        self.move_backward(speed,leg_s, rate_hz)
        self.move_left(speed,    leg_s, rate_hz)
        self.hold_position()

    # TODO: add CW and CCW parameters
    def move_circle_global(self, radius=5.0, speed=1.0, duration=20, update_interval=0.1):
        if self.get_mode() != 'GUIDED':
            self.set_mode('GUIDED')
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
