import time
from session import MavSession
from vehicle import Vehicle
from vehicle_auto import VehicleAuto

def main():
    sess = MavSession(port="/dev/tty.usbserial-0001", baud=57600).connect()
    # vehicle = Vehicle(sess)
    auto = VehicleAuto(sess)

    # vehicle.set_mode("STABILIZE")
    # vehicle.arm()

    # Hearbeat & Arm 
    sess.start_heartbeat()

    # Guided Takeoff & Land
    auto.guided_takeoff(5.0)
    time.sleep(2)
    auto.land()
    auto.disarm()

    try:
        while True:
            sess.send_heartbeat()
            sess.pump()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping drone control...")
    finally:
        try:
            # vehicle.disarm()
            auto.disarm()
        except Exception as e:
            print(f"Error during disarm: {e}")

if __name__ == "__main__":
    main()