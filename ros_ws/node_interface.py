import rclpy
from rclpy.node import Node
from publishers import MavrosPublishers
from subscribers import MavrosSubscribers
from services import MavrosServices
from enum import Enum, auto
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
import time
import math

class DroneState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    MISSION = auto()
    AVOID = auto()
    RTL = auto()
    LAND = auto()

class AutonomousDroneNode(Node):
    def __init__(self):
        super().__init__('autonomous_drone_node')

        self.mavros_pubs = MavrosPublishers(self)
        self.mavros_subs = MavrosSubscribers(self)
        self.mavros_srvs = MavrosServices(self)

        # Avoidance input buffer
        self._avoid_cmd = TwistStamped()
        self._avoid_stamp = 0.0
        self._forward_clear_m = math.inf

        # Subscribe to avoidance velocity command
        self.create_subscription(
            TwistStamped,
            '/avoidance/cmd_vel',
            self._avoidance_callback,
            qos_profile=1
        )

        # Subscribe to forward clearance distance
        self.create_subscription(
            Float32,
            '/avoidance/forward_clearance',
            self._clearance_callback,
            qos_profile=1
        )

        # Arbitration tunables
        self.avoid_enter_clear = 0.8  # engage avoidance when clearance < this 
        self.avoid_exit_clear  = 1.5  # return to mission when clearance > this
        self.avoid_eps         = 0.05 # smallness of vy/vz to consider "quiet"
        self.fresh_age_s       = 1.0  # how recent avoidance msg must be

        # Base mission speed
        self._mission_vx = 0.6

        # State & timer
        self.state = DroneState.IDLE
        self._control_rate_hz = 20.0
        self._blocking_rate_hz = 50.0
        self._control_timer = self.create_timer(1.0 / self._control_rate_hz, self.control_loop)

        self.get_logger().info('Autonomous Drone Node initialized.')

    def _sleep_nonblocking(self, seconds: float, rate_hz: float = 50.0):
        """Non-blocking sleep that yields to ROS callbacks at specified rate"""
        dt = 1.0 / rate_hz
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

    def _avoidance_callback(self, msg: TwistStamped):
        """Receive avoidance velocity commands"""
        self._avoid_cmd = msg
        self._avoid_stamp = time.monotonic()

    def _clearance_callback(self, msg: Float32):
        try:
            fc = float(msg.data)
            if math.isnan(fc) or math.isinf(fc):
                self._forward_clear_m = math.inf
            else:
                self._forward_clear_m = fc
        except Exception as e:
            self._forward_clear_m = math.inf
            self.get_logger().warn(f'Clearance callback error: {e}')
    
    def _avoid_fresh(self) -> bool:
        return (time.monotonic() - self._avoid_stamp) < self.fresh_age_s
    
    def _avoid_requesting(self) -> bool:
        av = self._avoid_cmd.twist.linear
        return (abs(av.y) > self.avoid_eps) or (abs(av.z) > self.avoid_eps)

    def _vx_from_clear(self, fc: float) -> float:
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
        """
        Blend mission forward velocity with avoidance lateral/vertical
        Mission controls vx; avoidance controls vy/vz only
        Keep moving forward even during avoidance!
        """
        av = self._avoid_cmd.twist.linear
        vx = mission_vx  # Use calculated forward speed (don't hard cap anymore)
        vy = float(av.y)  # Avoidance controls lateral
        vz = float(av.z)  # Avoidance controls vertical
        yaw_rate = 0.0
        return vx, vy, vz, yaw_rate

    def control_loop(self):
        # Check connection
        if not self.mavros_subs.is_connected():
            return
        
        # ------ STATE TRANSITIONS ------
        if self.state == DroneState.IDLE:
            pass

        elif self.state == DroneState.MISSION:
            if self._forward_clear_m < self.avoid_enter_clear:
                self.state = DroneState.AVOID

        elif self.state == DroneState.AVOID:
            if self._forward_clear_m > self.avoid_exit_clear and not self._avoid_requesting():
                self.state = DroneState.MISSION

        # ------ STATE ACTIONS ------
        if self.state == DroneState.MISSION:
            vx = self._vx_from_clear(self._forward_clear_m)
            vy = 0.0
            vz = 0.0
            yaw_rate = 0.0
            self.mavros_pubs.publish_velocity_body(vx, vy, vz, yaw_rate)
        
        elif self.state == DroneState.AVOID:
            vx_mission = self._vx_from_clear(self._forward_clear_m)
            if self._avoid_fresh():
                vx, vy, vz, yaw_rate = self._blend(vx_mission)
            else:
                vx = vx_mission
                vy = 0.0
                vz = 0.0
                yaw_rate = 0.0
            self.mavros_pubs.publish_velocity_body(vx, vy, vz, yaw_rate)

        elif self.state == DroneState.TAKEOFF:
            pass

        elif self.state in (DroneState.RTL, DroneState.LAND, DroneState.IDLE):
            self.mavros_pubs.publish_velocity_body(0.0, 0.0, 0.0, 0.0)

    def start_mission(self):
        """Enable autonomous mission mode with obstacle avoidance"""
        self.state = DroneState.MISSION
        self.get_logger().info('Mission mode started')

    def stop_mission(self):
        """Stop autonomous mission, return to idle"""
        self.state = DroneState.IDLE
        self.get_logger().info('Mission mode stopped')

    def arm_and_takeoff(self, altitude: float, timeout: float = 30.0) -> bool:
        """
        Arm and takeoff to target altitude, waiting until reached
        
        Args:
            altitude: Target altitude in meters
            timeout: Maximum time to wait for altitude (seconds)
        
        Returns:
            True if successful and altitude reached
        """
        self.get_logger().info(f'Starting takeoff to {altitude}m...')

        self.state = DroneState.TAKEOFF

        if not self.mavros_srvs.set_mode('GUIDED'):
            self.get_logger().error('Failed to set GUIDED mode')
            return False
        
        self._sleep_nonblocking(1.0, rate_hz=self._blocking_rate_hz)

        if not self.mavros_srvs.arm(True):
            self.get_logger().error('Failed to arm drone')
            return False
        
        self._sleep_nonblocking(1.0, rate_hz=self._blocking_rate_hz)

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
            pos = self.mavros_subs.get_position()
            current_alt = abs(pos[2])  
            
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
        """
        Move with velocity for specified duration
        
        Uses BODY frame (FLU: Forward-Left-Up convention)
        
        Args:
            vx: Forward/backward velocity (m/s, positive = forward)
            vy: Left/right velocity (m/s, positive = LEFT, negative = RIGHT)
            vz: Up/down velocity (m/s, positive = DOWN, negative = UP)
            duration: How long to move (seconds)
            yaw_rate: Rotation rate (deg/s, positive = LEFT, negative = RIGHT)
        
        Usage: node.move_body_velocity(1.0, 0, 0, 5.0)  # Move forward 1m/s for 5s
        """
        deadline = time.monotonic() + duration
        rate_hz = 20.0
        dt = 1.0 / rate_hz

        while time.monotonic() < deadline:
            self.mavros_pubs.publish_velocity_body(vx, vy, vz, yaw_rate)
            self._sleep_nonblocking(dt, rate_hz=rate_hz)
        
    def goto_position(self, x: float, y: float, z: float, duration: float = 5.0):
        """
        Go to position and hold
        
        Uses ENU frame (East-North-Up)
        
        Args:
            x: East position (meters)
            y: North position (meters) 
            z: Altitude (meters, positive = up)
        
        Usage: node.goto_position(10, 5, 5.0, 10.0)  # Go 10m east, 5m north, 5m altitude
        """
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
        """
        Go to GPS coordinates maintaining altitude, then land vertically
        Args:
            target_lat: Target latitude (degrees)
            target_lon: Target longitude (degrees)
            target_alt: Target altitude (meters, relative to home)
            timeout_s: Maximum time to wait (seconds)
        """
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
        """
        Command the drone to land and wait until on ground
        
        Args:
            timeout: Maximum time to wait for landing (seconds)
        
        Returns:
            True if successfully landed
        """
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
            pos = self.mavros_subs.get_position()
            current_alt = abs(pos[2]) 
            
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
        """
        Hover in place for specified duration
        """
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