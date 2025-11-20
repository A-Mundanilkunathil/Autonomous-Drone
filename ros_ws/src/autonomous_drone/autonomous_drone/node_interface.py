import rclpy
from rclpy.node import Node
from autonomous_drone.publishers import MavrosPublishers
from autonomous_drone.subscribers import MavrosSubscribers
from autonomous_drone.perception.perception_subs import PerceptionSubscribers
from autonomous_drone.services import MavrosServices
from enum import Enum, auto
import time
import math

class DroneState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    MISSION = auto()
    AVOID = auto()
    FOLLOW = auto()
    RTL = auto()
    LAND = auto()

class AutonomousDroneNode(Node):
    def __init__(self):
        super().__init__('autonomous_drone_node')

        self.mavros_pubs = MavrosPublishers(self)
        self.mavros_subs = MavrosSubscribers(self)
        self.perception_subs = PerceptionSubscribers(self)
        self.mavros_srvs = MavrosServices(self)

        # Arbitration parameters
        self.avoid_enter_clear = 0.8  # meters
        self.avoid_exit_clear  = 1.5  # meters
        self.avoid_eps         = 0.05  # m/s
        self.fresh_age_s       = 1.0  # seconds

        # Follow mode parameters
        self.follow_lost_timeout = 5.0  # seconds
        self._follow_target_lost_time = None

        # Base mission speed
        self._mission_vx = 0.6

        # State & timer
        self.state = DroneState.IDLE
        self._control_rate_hz = 20.0
        self._blocking_rate_hz = 50.0
        self._control_timer = self.create_timer(1.0 / self._control_rate_hz, self.control_loop)

        # Mission target (GPS)
        self._mission_target = None  # tuple(lat, lon, alt) or None
        self._mission_target_tolerance_m = 1.5

        self.get_logger().info('Autonomous Drone Node initialized.')

    def _sleep_nonblocking(self, seconds: float, rate_hz: float = 50.0):
        """Non-blocking sleep that yields to ROS callbacks at specified rate"""
        dt = 1.0 / rate_hz
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

    def _vx_from_clear(self, fc: float) -> float:
        """Calculate forward velocity based on obstacle clearance"""
        stop = 0.5
        caut = 0.8
        
        if not math.isfinite(fc):
            return self._mission_vx
        if fc <= stop:
            return 0.0
        if fc < caut:
            t = (fc - stop) / max(1e-3, (caut - stop))
            return 0.20 + (self._mission_vx - 0.20) * t
        return self._mission_vx

    def _blend(self, mission_vx: float) -> tuple[float, float, float, float]:
        """Blend mission forward velocity with avoidance lateral/vertical commands"""
        avoidance_cmd = self.perception_subs.get_avoidance_cmd()
        if avoidance_cmd is None:
            return mission_vx, 0.0, 0.0, 0.0
        
        av = avoidance_cmd.twist.linear
        vx = mission_vx
        vy = float(av.y)
        vz = float(av.z)
        yaw_rate = 0.0
        return vx, vy, vz, yaw_rate

    def _get_yaw(self) -> float:
        """Return current vehicle yaw in radians from local_position quaternion"""
        local = self.mavros_subs.local_position
        if not local:
            return 0.0
        q = local.pose.orientation
        # Extract yaw from quaternion
        sin_yaw = +2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(sin_yaw, cos_yaw)
        return yaw

    def _bearing_to_target(self, lat1, lon1, lat2, lon2) -> float:
        """Return bearing (radians) from point1 to point2, measured clockwise from North"""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        y = math.sin(dlon) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
        brng = math.atan2(y, x)
        return brng

    def control_loop(self):
        if not self.mavros_subs.is_connected():
            return
        
        forward_clear = self.perception_subs.get_forward_clearance()
        
        # State transitions
        if self.state == DroneState.IDLE:
            pass

        elif self.state == DroneState.MISSION:
            if forward_clear < self.avoid_enter_clear:
                self.state = DroneState.AVOID

        elif self.state == DroneState.AVOID:
            if forward_clear > self.avoid_exit_clear and not self.perception_subs.is_avoidance_requesting(self.avoid_eps):
                self.state = DroneState.MISSION
        
        elif self.state == DroneState.FOLLOW:
            if forward_clear < self.avoid_enter_clear:
                self.state = DroneState.AVOID
            elif not self.perception_subs.has_target():
                if self._follow_target_lost_time is None:
                    self._follow_target_lost_time = time.monotonic()
                    self.get_logger().warn('Target lost, hovering and searching...')
                
                time_lost = time.monotonic() - self._follow_target_lost_time
                if time_lost > self.follow_lost_timeout:
                    self.get_logger().info(f'Target lost for {time_lost:.1f}s, returning to IDLE')
                    self.state = DroneState.IDLE
                    self._follow_target_lost_time = None
            else:
                if self._follow_target_lost_time is not None:
                    self.get_logger().info('Target reacquired!')
                    self._follow_target_lost_time = None

        # State actions
        if self.state == DroneState.MISSION:
            # If we have an active GPS mission target, compute velocity toward it
            if self._mission_target is not None:
                lat_t, lon_t, alt_t = self._mission_target
                curr_lat, curr_lon, _ = self.mavros_subs.get_global_position()
                curr_alt = self.mavros_subs.get_relative_altitude()
                
                # Distance to target
                distance = self._haversine_distance(curr_lat, curr_lon, lat_t, lon_t)

                if distance < self._mission_target_tolerance_m:
                    self.get_logger().info('Reached GPS mission target')
                    self._mission_target = None
                    vx = 0.0
                    vy = 0.0
                    vz = 0.0
                    yaw_rate = 0.0
                else:
                    # Bearing to target (radians from North)
                    bearing = self._bearing_to_target(curr_lat, curr_lon, lat_t, lon_t)
                    
                    # Current yaw (radians)
                    yaw = self._get_yaw()
                    
                    # Align frames: convert bearing (North, CW) -> ENU azimuth (East, CCW)
                    az_enu = (math.pi / 2) - bearing

                    # Body-frame angle error (target direction - current heading)
                    angle_to_target = az_enu - yaw

                    # Normalize to [-pi, pi]
                    angle_to_target = math.atan2(math.sin(angle_to_target), math.cos(angle_to_target))
                    
                    # Compute body-frame velocities
                    vx_base = self._vx_from_clear(forward_clear)
                    vx = vx_base * math.cos(angle_to_target)
                    vy = vx_base * math.sin(angle_to_target)
                    
                    # Altitude control 
                    vz = 0.0
                    if alt_t is not None:
                        alt_error = alt_t - curr_alt
                        vz = max(-0.5, min(0.5, alt_error * 0.3))
                    
                    # Blend with avoidance if active
                    if self.perception_subs.is_avoidance_fresh(self.fresh_age_s):
                        vx, vy, vz, yaw_rate = self._blend(vx)
                    else:
                        yaw_rate = 0.0
                
                self.mavros_pubs.publish_velocity_body(vx, vy, vz, yaw_rate)
            else:
                # No GPS target, just move forward with avoidance
                vx = self._vx_from_clear(forward_clear)
                vy = 0.0
                vz = 0.0
                yaw_rate = 0.0
                self.mavros_pubs.publish_velocity_body(vx, vy, vz, yaw_rate)
        
        elif self.state == DroneState.AVOID:
            vx_mission = self._vx_from_clear(forward_clear)
            if self.perception_subs.is_avoidance_fresh(self.fresh_age_s):
                vx, vy, vz, yaw_rate = self._blend(vx_mission)
            else:
                vx = vx_mission
                vy = 0.0
                vz = 0.0
                yaw_rate = 0.0
            self.mavros_pubs.publish_velocity_body(vx, vy, vz, yaw_rate)
        
        elif self.state == DroneState.FOLLOW:
            following_cmd = self.perception_subs.get_following_cmd()
            if following_cmd is not None and self.perception_subs.is_following_fresh(1.0):
                fol = following_cmd.twist
                
                fc = self.perception_subs.get_forward_clearance()
                vx_limit = self._vx_from_clear(fc)
                vx_cmd = float(fol.linear.x)
                vx = max(-vx_limit, min(vx_cmd, vx_limit))
                
                vy = float(fol.linear.y)
                vz = float(fol.linear.z)
                if self.perception_subs.is_avoidance_fresh(self.fresh_age_s):
                    av = self.perception_subs.get_avoidance_cmd().twist.linear
                    vy = float(av.y)
                    vz = float(av.z)
                
                yaw_rate = float(fol.angular.z)
                self.mavros_pubs.publish_velocity_body(vx, vy, vz, yaw_rate)
            else:
                self.mavros_pubs.publish_velocity_body(0.0, 0.0, 0.0, 0.0)

        elif self.state == DroneState.TAKEOFF:
            pass

        elif self.state in (DroneState.RTL, DroneState.LAND, DroneState.IDLE):
            self.mavros_pubs.publish_velocity_body(0.0, 0.0, 0.0, 0.0)

    def start_mission(self):
        """Enable autonomous mission mode with obstacle avoidance"""
        self.state = DroneState.MISSION
        self.get_logger().info('Mission mode started')

    def start_mission_gps(self, lat: float, lon: float, alt: float = None, tolerance_m: float = 1.5):
        """Start GPS mission to specific coordinates with obstacle avoidance
        
        Args:
            lat: Target latitude
            lon: Target longitude
            alt: Target altitude (AMSL), if None maintains current altitude
            tolerance_m: Distance tolerance to consider target reached
        """
        self._mission_target = (lat, lon, alt)
        self._mission_target_tolerance_m = tolerance_m
        self.state = DroneState.MISSION
        self.get_logger().info(f'GPS mission started to ({lat:.6f}, {lon:.6f}, alt={alt})')

    def stop_mission(self):
        """Stop autonomous mission, return to idle"""
        self.state = DroneState.IDLE
        self._mission_target = None
        self.get_logger().info('Mission mode stopped')
    
    def start_follow(self):
        """Enable object following mode"""
        self.state = DroneState.FOLLOW
        self._follow_target_lost_time = None
        self.get_logger().info('Follow mode started')
    
    def stop_follow(self):
        """Stop object following, return to idle"""
        self.state = DroneState.IDLE
        self._follow_target_lost_time = None
        self.get_logger().info('Follow mode stopped')

    def arm_and_takeoff(self, altitude: float, timeout: float = 30.0) -> bool:
        """Arm and takeoff to target altitude, waiting until reached"""
        self.get_logger().info(f'Starting takeoff to {altitude}m...')

        self.state = DroneState.TAKEOFF

        if not self.mavros_srvs.set_mode('GUIDED'):
            self.get_logger().error('Failed to set GUIDED mode')
            return False
        
        self._sleep_nonblocking(3.0, rate_hz=self._blocking_rate_hz)

        if not self.mavros_srvs.arm(True):
            self.get_logger().error('Failed to arm drone')
            return False
        
        self._sleep_nonblocking(3.0, rate_hz=self._blocking_rate_hz)

        if not self.mavros_srvs.takeoff(altitude):
            self.get_logger().error('Failed to send takeoff command')
            return False
        
        self.get_logger().info(f'Takeoff command sent. Waiting for altitude {altitude}m...')
        
        target_reached = False
        altitude_tolerance = 0.4 
        deadline = time.monotonic() + timeout
        last_log = time.monotonic()
        log_interval = 1.0
        rate_hz = self._blocking_rate_hz
        dt = 1.0 / rate_hz
        
        while time.monotonic() < deadline:
            current_alt = abs(self.mavros_subs.get_relative_altitude())  
            
            now = time.monotonic()
            if (now - last_log) >= log_interval:
                self.get_logger().info(
                    f'Current altitude: {current_alt:.2f}m / Target: {altitude}m'
                )
                last_log = now
            
            if abs(current_alt - altitude) < altitude_tolerance:
                target_reached = True
                break
            
            self._sleep_nonblocking(dt, rate_hz=rate_hz)
        
        if target_reached:
            final_alt = abs(self.mavros_subs.get_position()[2])
            self.get_logger().info(f'Takeoff successful! Reached {final_alt:.2f}m')
            self.state = DroneState.IDLE
            return True
        else:
            final_alt = abs(self.mavros_subs.get_position()[2])
            self.get_logger().warn(
                f'Takeoff timeout! Only reached {final_alt:.2f}m in {timeout}s'
            )
            self.state = DroneState.IDLE
            return False

    def move_body_velocity(self, vx: float, vy: float, vz: float, duration: float = 2.0, yaw_rate: float = 0.0):
        """Move with velocity for specified duration (BODY frame FLU convention)"""
        deadline = time.monotonic() + duration
        rate_hz = 20.0
        dt = 1.0 / rate_hz

        while time.monotonic() < deadline:
            self.mavros_pubs.publish_velocity_body(vx, vy, vz, yaw_rate)
            self._sleep_nonblocking(dt, rate_hz=rate_hz)
        
    def goto_position(self, x: float, y: float, z: float, duration: float = 5.0):
        """Go to position and hold (ENU frame)"""
        deadline = time.monotonic() + duration
        rate_hz = 20.0
        dt = 1.0 / rate_hz

        while time.monotonic() < deadline:
            self.mavros_pubs.publish_position(x, y, z)
            self._sleep_nonblocking(dt, rate_hz=rate_hz)
    
    def _haversine_distance(self, lat1, lon1, lat2, lon2):
        import math

        R = 6371000 # Earth radius in meters
        d_lat = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)
        a = (
            math.sin(d_lat / 2)**2 + 
            math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * 
            math.sin(d_lon / 2)**2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c # Distance in meters
        return distance

    def goto_gps(self, target_lat, target_lon, target_alt=None, timeout_s=90.0):
        """Go to GPS coordinates maintaining altitude"""
        deadline = time.monotonic() + timeout_s
        rate_hz = 20.0
        dt = 1.0 / rate_hz

        while time.monotonic() < deadline:
            curr_lat, curr_lon, _ = self.mavros_subs.get_global_position()
            curr_alt = self.mavros_subs.get_relative_altitude()

            distance = self._haversine_distance(
                curr_lat, curr_lon, target_lat, target_lon
            )
            
            if target_alt is not None:
                print(f"Current altitude: {curr_alt:.2f} m, Target altitude: {target_alt:.2f} m")
                alt_error = target_alt - curr_alt
            else:
                alt_error = 0.0

            self.mavros_pubs.publish_global_position(
                latitude=target_lat,
                longitude=target_lon,
                altitude_m=target_alt,
                relative_alt=True
            )

            self.get_logger().info(
                f"Distance to target: {distance:.2f} m | "
                f"Alt error: {alt_error:.2f} m"
            )

            if distance < 1.5 and alt_error < 0.8:
                self.get_logger().info("Target reached!")
                break

            self._sleep_nonblocking(dt, rate_hz=rate_hz)
            
    def land(self, timeout: float = 60.0) -> bool:
        """Command the drone to land and wait until on ground"""
        self.get_logger().info('Starting landing sequence...')
        
        if not self.mavros_srvs.land():
            self.get_logger().error('Land command failed')
            return False
        
        ground_threshold = 0.4
        deadline = time.monotonic() + timeout
        landed = False
        last_log = time.monotonic()
        log_interval = 1.0
        rate_hz = self._blocking_rate_hz
        dt = 1.0 / rate_hz
        
        while time.monotonic() < deadline:
            current_alt = abs(self.mavros_subs.get_relative_altitude())  
            
            now = time.monotonic()
            if (now - last_log) >= log_interval:
                self.get_logger().info(f'Landing... altitude: {current_alt:.2f}m')
                last_log = now
            
            if current_alt < ground_threshold:
                landed = True
                break
            
            self._sleep_nonblocking(dt, rate_hz=rate_hz)
        
        if landed:
            self.get_logger().info('Landed successfully')
            self._sleep_nonblocking(6.0, rate_hz=self._blocking_rate_hz)
            self.mavros_srvs.arm(False)
            return True
        else:
            self.get_logger().warn(f'Landing timeout after {timeout}s')
            return False
        
    def hover(self, duration: float = 5.0):
        """Hover in place for specified duration"""
        self.get_logger().info(f'Hovering for {duration}s...')
        deadline = time.monotonic() + duration
        rate_hz = 20.0
        dt = 1.0 / rate_hz

        while time.monotonic() < deadline:
            self.mavros_pubs.publish_velocity_body(0.0, 0.0, 0.0, 0.0)
            self._sleep_nonblocking(dt, rate_hz=rate_hz)
    
    def return_to_launch(self, pos_tol_m: float, alt_tol_m: float, timeout: float = 180.0, smart: bool = True) -> bool:
        """
        Command the drone to return to launch position

        Args:
            pos_tol_m: Position tolerance in meters
            alt_tol_m: Altitude tolerance in meters
            timeout: Maximum time to wait for RTL (seconds)
            smart: If True, try SMART_RTL first then fallback to RTL

        Why: Safe return procedure
        """
        home = self.mavros_subs.get_home_position()
        if home is None:
            self.get_logger().error('Home position unknown, cannot RTL.')
            return False
        
        lat, lon, alt = home
        self.get_logger().info(f'Home position: Lat {lat}, Lon {lon}, Alt {alt}m')
        
        if smart:
            from pymavlink import mavutil
            master = None
            try:
                master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
                master.wait_heartbeat(timeout=10)
                self.get_logger().info('Connected via UDP (simulation)')
            except Exception as e_udp:
                self.get_logger().warn(f'UDP connection failed: {e_udp}. Trying serial...')
                try:
                    master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
                    master.wait_heartbeat(timeout=10)
                    self.get_logger().info('Connected via serial (hardware)')
                except Exception as e_serial:
                    self.get_logger().error(f'Failed to connect via UDP and serial: {e_serial}')
                    return False
            master.set_mode(21)  # SMART_RTL
            master.close()
            self.get_logger().info('SMART_RTL mode activated')
        else:
            if not self.mavros_srvs.set_mode('RTL'):
                self.get_logger().error('Failed to set RTL mode.')
                return False
            self.get_logger().info('RTL mode activated')
            
        deadline = time.monotonic() + timeout
        rate_hz = 20.0
        dt = 1.0 / rate_hz
        
        while time.monotonic() < deadline:
            curr_lat, curr_lon, curr_alt = self.mavros_subs.get_global_position()
            distance = self._haversine_distance(curr_lat, curr_lon, lat, lon)
            alt_diff = abs(curr_alt - alt)

            self.get_logger().info(f'Distance to home: {distance:.2f} m | Alt diff: {alt_diff:.2f} m')
            
            if distance < pos_tol_m and alt_diff < alt_tol_m:
                self.get_logger().info('Reached home position.')
                return True
            
            self._sleep_nonblocking(dt, rate_hz=rate_hz)
            
        self.get_logger().warn(f'RTL timeout after {timeout}s.')
        return False

    # ------------------------- High-level missions -------------------------
    def move_circle_global(self, radius: float = 5.0, speed: float = 1.0, duration: float = 60.0):
        """
        Move in a circular pattern in the GLOBAL frame (ENU)
        
        Args:
            radius: Circle radius (meters)
            speed: Tangential speed (m/s)
            duration: Total duration to circle (seconds)
            update_interval: Time between position updates (seconds)
        
        Usage: node.move_circle_global(radius=5.0, speed=1.0, duration=60.0)
        """
        import math 

        center_pos = self.mavros_subs.get_position()
        center_x, center_y, center_z = center_pos

        rate_hz = 20.0
        dt = 1.0 / rate_hz
        deadline = time.monotonic() + duration
        angle_step = (speed / radius) * dt

        step = 0
        while time.monotonic() < deadline:
            angle = step * angle_step
            target_x = center_x + radius * math.cos(angle)
            target_y = center_y + radius * math.sin(angle)
            target_z = center_z

            self.goto_position(target_x, target_y, target_z, duration=dt)
            step += 1

    def move_square(self, speed: float = 1.0, leg_s: float = 3.0):
        """
        Move square in WORLD frame (ENU)
        Args:
            speed: Speed in m/s
            leg_s: Time to move each leg in seconds
        """
        pos = self.mavros_subs.get_position()  # (x, y, z)
        x_0, y_0, z_0 = pos
        distance = speed * leg_s if speed and leg_s else 3.0

        corners = [
            (x_0 + distance, y_0, z_0),
            (x_0 + distance, y_0 + distance, z_0),
            (x_0, y_0 + distance, z_0),
            (x_0, y_0, z_0)
        ]

        for corner in corners:
            self.goto_position(corner[0], corner[1], corner[2], duration=leg_s)
    
    # ------------------------- Low-level missions --------------------------
    # Basic movements in body frame (FLU)
    def move_forward(self, speed: float, duration: float):
        """Move forward in body frame"""
        self.get_logger().info(f'Moving forward at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=speed, vy=0.0, vz=0.0, duration=duration)
    def move_backward(self, speed: float, duration: float):
        """Move backward in body frame"""
        self.get_logger().info(f'Moving backward at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=0.0, vz=0.0, duration=duration)
    def move_right(self, speed: float, duration: float):
        """Move right in body frame"""
        self.get_logger().info(f'Moving right at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=-speed, vz=0.0, duration=duration)
    def move_left(self, speed: float, duration: float):
        """Move left in body frame"""
        self.get_logger().info(f'Moving left at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=speed, vz=0.0, duration=duration)
    def move_up(self, speed: float, duration: float):
        """Move up in body frame (FLU: negative Z is up)"""
        self.get_logger().info(f'Moving up at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=0.0, vz=-speed, duration=duration)
    def move_down(self, speed: float, duration: float):
        """Move down in body frame (FLU: positive Z is down)"""
        self.get_logger().info(f'Moving down at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=0.0, vz=speed, duration=duration)

    # Diagonal movements
    def move_diagonal_front_right(self, speed: float=1.0, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front right at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=+speed, vy=-speed, vz=0.0, duration=duration, yaw_rate=0.0)
    def move_diagonal_front_left(self, speed: float=1.0, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front left at {speed}m/s for {duration}s') 
        self.move_body_velocity(vx=+speed, vy=+speed, vz=0.0, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_right(self, speed: float=1.0, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back right at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=-speed, vz=0.0, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_left(self, speed: float=1.0, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back left at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=+speed, vz=0.0, duration=duration, yaw_rate=0.0)
    def move_diagonal_front_right_up(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front right up at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=+speed, vy=-speed, vz=-speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_front_right_down(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front right down at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=+speed, vy=-speed, vz=+speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_front_left_up(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front left up at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=+speed, vy=+speed, vz=-speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_front_left_down(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front left down at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=+speed, vy=+speed, vz=+speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_right_up(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back right up at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=-speed, vz=-speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_right_down(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back right down at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=-speed, vz=+speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_left_up(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back left up at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=+speed, vz=-speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_left_down(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back left down at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=+speed, vz=+speed, duration=duration, yaw_rate=0.0)

    # Rotation movements
    def rotate_left(self, yaw_rate: float, duration: float):
        """Rotate left (counter-clockwise)"""
        self.get_logger().info(f'Rotating left at {yaw_rate}°/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=0.0, vz=0.0, duration=duration, yaw_rate=yaw_rate)
    def rotate_right(self, yaw_rate: float, duration: float):
        """Rotate right (clockwise)"""
        self.get_logger().info(f'Rotating right at {yaw_rate}°/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=0.0, vz=0.0, duration=duration, yaw_rate=-yaw_rate)

# def main(args=None):
#     rclpy.init(args=args)
#     node = AutonomousDroneNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
