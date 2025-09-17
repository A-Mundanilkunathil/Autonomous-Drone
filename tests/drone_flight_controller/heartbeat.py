from pymavlink import mavutil
import time

# Set your UNO serial port here
connection_string = '/dev/cu.usbserial-A5069RR4'
baud = 115200

# Connect to the UNO serial port
master = mavutil.mavlink_connection(connection_string, baud=baud)

print("⏳ Waiting for FC heartbeat through UNO bridge...")
master.wait_heartbeat()
print("✅ Heartbeat received from FC!")

def send_rc_override(roll=1900, pitch=1100, throttle=800, yaw=1100):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        roll, pitch, throttle, yaw,  # Channels 1-4
        0, 0, 0, 0                   # Channels 5-8
    )

print("🔁 Sending RC_CHANNELS_OVERRIDE every 0.2s")
try:
    while True:
        send_rc_override(roll=1600, throttle=1300)
        time.sleep(0.2)
except KeyboardInterrupt:
    print("🛑 Stopped")
