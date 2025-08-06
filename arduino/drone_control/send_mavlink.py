from pymavlink import mavutil

master = mavutil.mavlink_connection('COM3', baud=115200)

# send hearbeat
master.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0, 0
)