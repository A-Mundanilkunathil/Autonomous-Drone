# ---------- Script 1: Send MAVLink Commands via USB-C (direct to FC) ----------
# Save this as `mavlink_usb_direct.py`

from pymavlink import mavutil
import time

# Update this to your actual USB-C serial port on macOS
connection_string = '/dev/cu.usbmodem0x80000001' # e.g. SpeedyBee FC
baud = 115200

master = mavutil.mavlink_connection(connection_string, baud=baud)
print("Waiting for FC heartbeat (USB)...")
master.wait_heartbeat()
print("✅ Heartbeat received over USB")




while True:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1600, 1500, 1200, 1500,  # roll, pitch, throttle, yaw
        1000, 0, 0, 0            # aux1-4
    )
    time.sleep(0.1)

# ---------- Script 2: Send MAVLink via Arduino UNO Serial Bridge ----------
# Save this as `mavlink_uno_bridge.py`

from pymavlink import mavutil
import time

# Update to the port your UNO shows up as (USB serial bridge)
connection_string = '/dev/tty.usbserial-110'  # e.g. UNO connected to FC UART1
baud = 115200

master = mavutil.mavlink_connection(connection_string, baud=baud)
print("Waiting for FC heartbeat (UNO bridge)...")
master.wait_heartbeat()
print("✅ Heartbeat received over UNO bridge")

while True:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1600, 1500, 1200, 1500,
        1000, 0, 0, 0
    )
    time.sleep(0.1)
