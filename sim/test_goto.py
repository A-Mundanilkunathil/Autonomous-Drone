import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src/control'))

import time, math
from session import MavSession
from vehicle_auto import VehicleAuto
import logging

def offset_latlon(lat_deg, lon_deg, d_north_m, d_east_m):
    R = 6378137.0
    dlat = d_north_m / R
    dlon = d_east_m / (R * math.cos(math.radians(lat_deg)))
    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)

CONNECTION = 'udp:127.0.0.1:14550' 

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"             
    )
    sess = MavSession(CONNECTION).connect()
    auto = VehicleAuto(sess)
    sess.start_heartbeat()
    sess.start_pump()

    try:
        # Takeoff to 2 meters
        auto.guided_takeoff(2.0)
        
        # Wait a moment for stabilization
        time.sleep(2.0)

        # Get current global position
        gpi = auto.recv_msg('GLOBAL_POSITION_INT', timeout=2.0)
        if not gpi:
            raise RuntimeError("No GLOBAL_POSITION_INT; EKF/GPS not ready?")
        cur_lat = gpi.lat / 1e7
        cur_lon = gpi.lon / 1e7
        cur_alt = gpi.relative_alt / 1000.0
        
        print(f"Current position: lat={cur_lat:.7f}, lon={cur_lon:.7f}, alt={cur_alt:.2f}m")

        # Target: 10m north, 3m east, SAME altitude (4m to match takeoff)
        tgt_lat, tgt_lon = offset_latlon(cur_lat, cur_lon, d_north_m=-10.0, d_east_m=3.0)
        print(f"Target position:  lat={tgt_lat:.7f}, lon={tgt_lon:.7f}, alt=4.00m")
        
        # Go to target at current altitude (4m, not 5m)
        auto.goto_latlon(tgt_lat, tgt_lon, alt_rel_m=4.0, pos_tol_m=1.5, alt_tol_m=0.7, rate_hz=5)

        print("Waypoint reached! Holding position...")
        auto.hold_position()
        time.sleep(2.0)
        
        print("Landing...")
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
