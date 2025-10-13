from src.control.vehicle_auto import VehicleAuto
from src.control.session import MavSession
import time

def main():
    sess = MavSession(port="COM6", baud=57600).connect()
    auto = VehicleAuto(sess)
    
    sess.start_heartbeat()
    
    try:
        auto.guided_takeoff(3.0) # Takeoff to 3m
        
        # Hover for 2 seconds
        time.sleep(2.0)
        
        # Small forward then stop and hold
        auto.move_backward(speed=0.5, duration=2.0, rate_hz=10)
        auto.stop(duration=0.3, rate_hz=10)
        auto.hold_position()
        
        # Land then disarm
        auto.land()
        auto.disarm()

    except KeyboardInterrupt:
        print("Stopping drone control...")
    finally:
        try:
            auto.disarm(wait=True)
        except Exception as e:
            print(f"Failed to disarm: {e}")
        sess.stop_heartbeat()
        sess.close()
        
if __name__ == "__main__":
    main()