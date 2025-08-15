import time
from session import MavSession
from vehicle import Vehicle

def main():
    sess = MavSession().connect()
    vehicle = Vehicle(sess)

    vehicle.set_mode("STABILIZE")
    vehicle.arm()
    vehicle.set_throttle_percent(10)
    vehicle.pitch_forward(80)
    time.sleep(1)
    vehicle.center_attitude()
    vehicle.dec_throttle(50)
    vehicle.disarm()
    try:
        while True:
            sess.send_heartbeat()
            sess.pump()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping drone control...")
    finally:
        try:
            vehicle.disarm()
        except Exception as e:
            print(f"Error during disarm: {e}")

if __name__ == "__main__":
    main()