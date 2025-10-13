import logging
import time
from vehicle_auto import VehicleAuto
from session import MavSession

# Connect to SITL 
CONNECTION = 'udp:127.0.0.1:14550'

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
        auto.guided_takeoff(3.0) # Takeoff to 3m
        
        # Hover for 2 seconds
        time.sleep(2.0)
        
        # Move forward then stop
        auto.move_backward(speed=0.5, duration=2.0, rate_hz=10)
        auto.stop(duration=0.3, rate_hz=10)  
        
        # Move backward then stop
        auto.move_forward(speed=0.5, duration=2.0, rate_hz=10)
        auto.stop(duration=0.3, rate_hz=10)

        # Move circle
        auto.move_circle_global(radius=2.0, speed=1.0, duration=10, update_interval=0.1)

        # Move square
        auto.move_square(speed=1.0, leg_s=3.0, rate_hz=10)

        # Land then disarm
        auto.hold_position()
        auto.land()
        time.sleep(2.0) # Wait for land to complete

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