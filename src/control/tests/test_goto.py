import time, math
from session import MavSession
from vehicle_auto import VehicleAuto
import logging

def offset_latlon(lat_deg, lon_deg, d_north_m, d_east_m):
    R = 6378137.0
    dlat = d_north_m / R
    dlon = d_east_m / (R * math.cos(math.radians(lat_deg)))
    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"             
    )
    sess = MavSession(port="/dev/tty.usbserial-0001", baud=57600).connect()
    auto = VehicleAuto(sess)
    sess.start_heartbeat()
    sess.start_pump()

    try:
        auto.guided_takeoff(5.0)

        # Get current global position
        gpi = auto.recv_msg('GLOBAL_POSITION_INT', timeout=2.0)
        if not gpi:
            raise RuntimeError("No GLOBAL_POSITION_INT; EKF/GPS not ready?")
        cur_lat = gpi.lat / 1e7
        cur_lon = gpi.lon / 1e7

        # Target ~5 m North, 3 m East from current, same alt (5 m AGL)
        tgt_lat, tgt_lon = offset_latlon(cur_lat, cur_lon, d_north_m=5.0, d_east_m=3.0)
        auto.goto_latlon(tgt_lat, tgt_lon, alt_rel_m=5.0, pos_tol_m=1.5, alt_tol_m=0.7, rate_hz=5)

        auto.hold_position()
        auto.land()
        auto.disarm()

    except KeyboardInterrupt:
        print("Stopping drone control (user interrupt)...")
    finally:
        try:
            auto.disarm(wait=True)
        except Exception as e:
            print(f"Disarm cleanup error: {e}")
        sess.stop_pump()
        sess.stop_heartbeat()
        sess.close()

if __name__ == "__main__":
    main()
