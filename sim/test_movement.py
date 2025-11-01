import logging
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src/control'))

from vehicle_auto import VehicleAuto
from session import MavSession

# Connect to SITL 
CONNECTION = 'udp:127.0.0.1:14550' 
# sess = MavSession(port="/dev/tty.usbserial-0001", baud=57600).connect()

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"             
    )
    sess = MavSession(CONNECTION).connect()
    auto = VehicleAuto(sess)

    # Hearbeat & Arm 
    sess.start_heartbeat()
    sess.start_pump()
    
    try:
        auto.disable_arm_check() # Disable arming check
        auto.guided_takeoff(2.0) # Takeoff to 1m
        time.sleep(2.0)
        
        # # Turn left
        # auto.turn_left()
        # time.sleep(2.0)
        
        # # Turn right
        # auto.turn_right()
        # auto.move_forward_world(0.5, 4.0, 10)

        # auto.turn_right()
        # auto.move_forward_world(0.5, 4.0, 10)
        
        # # Turn around
        # auto.turn_around()
        # time.sleep(2.0)

        # # Move forward then stop
        # auto.move_backward(speed=2, duration=8.0, rate_hz=10) 
        
        # # Move backward then stop
        # auto.move_forward(speed=2, duration=2.0, rate_hz=10)
    
        # # Move diagonal front right up
        # auto.move_diagonal_front_right_up(speed=0.5, duration=2.0, rate_hz=10)

        # # Move diagonal front left down
        # auto.move_diagonal_front_left_down(speed=0.5, duration=2.0, rate_hz=10)

        # # Move diagonal back right up
        # auto.move_diagonal_back_right_up(speed=0.5, duration=2.0, rate_hz=10)

        # # Move diagonal back left down
        # auto.move_diagonal_back_left_down(speed=0.5, duration=2.0, rate_hz=10)

        # # Move diagonal front left up
        # auto.move_diagonal_front_left_up(speed=0.5, duration=2.0, rate_hz=10)

        # # Move diagonal front right down
        # auto.move_diagonal_front_right_down(speed=0.5, duration=2.0, rate_hz=10)

        # # Move diagonal back right down
        # auto.move_diagonal_back_right_down(speed=0.5, duration=2.0, rate_hz=10)

        # # Move diagonal back left up
        # auto.move_diagonal_back_left_up(speed=0.5, duration=2.0, rate_hz=10)

        # # Move left 
        # auto.move_left(speed=0.5, duration=2.0, rate_hz=10)

        # # Move right 
        # auto.move_right(speed=0.5, duration=2.0, rate_hz=10)

        # # Move up
        # auto.move_up(speed=0.5, duration=2.0, rate_hz=10)

        # # Move down
        # auto.move_down(speed=0.5, duration=2.0, rate_hz=10)

        # # Rotate
        # auto.rotate(yaw_rate_deg_s=30, duration=2.0, rate_hz=10)

        # # Move diagonal front left
        # auto.move_diagonal_front_left(speed=0.5, duration=3.0, rate_hz=10)

        # # Move diagonal back right
        # auto.move_diagonal_back_right(speed=0.5, duration=3.0, rate_hz=10)

        # # Move diagonal front right 
        # auto.move_diagonal_front_right(speed=0.5, duration=3.0, rate_hz=10)

        # # Move diagonal back left
        # auto.move_diagonal_back_left(speed=0.5, duration=3.0, rate_hz=10)

        # Move circle
        # auto.move_circle_global(radius=6.0, speed=1.0, duration=60, update_interval=0.1)

        # Move square
        auto.move_square(speed=2.0, leg_s=3.0, rate_hz=10)

        # Land then disarm
        auto.hold_position()
        auto.land()

    except KeyboardInterrupt:
        print("Stopping drone control...")
    finally:
        try:
            auto.disarm(wait=True)
        except Exception as e:
            print(f"Failed to disarm: {e}")
        sess.stop_pump()
        sess.stop_heartbeat()
        sess.close()
        
if __name__ == "__main__":
    main()