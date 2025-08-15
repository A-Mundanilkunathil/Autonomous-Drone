import time
from session import MavSession
from vehicle import Vehicle

def main():
    sess = MavSession().connect()
    vehicle = Vehicle(sess)

    vehicle.set_mode("STABILIZE")
    vehicle.arm()
    time.sleep(2)
    
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