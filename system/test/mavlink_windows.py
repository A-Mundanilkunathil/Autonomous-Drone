import time
from pymavlink import mavutil

# --- Configuration ---
MAVLINK_PORT = 'COM6'  # Your COM port
BAUD_RATE = 57600

# Neutral RC Values
RC_NEUTRAL = 1500
RC_THROTTLE_LOW = 1000 # Throttle for disarmed or neutral flight
RC_THROTTLE_HOVER_TEST = 1200 # A low throttle value for testing maneuvers (ADJUST WITH CAUTION)

# Maneuver RC Values (adjust intensity as needed)
# Pitch: Forward < 1500, Backward > 1500
# Roll: Left < 1500, Right > 1500
# Yaw: CCW < 1500, CW > 1500
PITCH_FORWARD = 1400
PITCH_BACKWARD = 1600
ROLL_LEFT = 1400
ROLL_RIGHT = 1600
YAW_CCW = 1400
YAW_CW = 1600

# Durations
MANEUVER_DURATION_S = 2.0  # Duration for each specific maneuver
PAUSE_DURATION_S = 1.0     # Pause between maneuvers

# --- Script ---
master = None

def send_rc_override(roll=RC_NEUTRAL, pitch=RC_NEUTRAL, throttle=RC_THROTTLE_LOW, yaw=RC_NEUTRAL, aux1=0, aux2=0, aux3=0, aux4=0):
    """Helper function to send RC override commands."""
    if master:
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            roll, pitch, throttle, yaw,
            aux1, aux2, aux3, aux4 # For 8 channels
            # If you need more channels, extend here with 0 or UINT16_MAX
        )
        # print(f"Sent RC: R:{roll} P:{pitch} T:{throttle} Y:{yaw}") # Uncomment for verbose output

def execute_maneuver(description, duration, roll=RC_NEUTRAL, pitch=RC_NEUTRAL, throttle=RC_THROTTLE_HOVER_TEST, yaw=RC_NEUTRAL):
    """Executes a maneuver for a given duration."""
    print(f"Executing: {description} for {duration}s (Throttle: {throttle})")
    start_time = time.time()
    while time.time() < start_time + duration:
        send_rc_override(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)
        time.sleep(0.1) # Send at ~10Hz
    send_rc_override(throttle=throttle) # Maintain throttle, neutralize roll/pitch/yaw
    print(f"Finished: {description}")

try:
    print(f"Connecting to MAVLink on {MAVLINK_PORT} at {BAUD_RATE} baud...")
    master = mavutil.mavlink_connection(MAVLINK_PORT, baud=BAUD_RATE)

    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

    stabilize_mode_id = master.mode_mapping().get('STABILIZE')
    if stabilize_mode_id is None:
        print("STABILIZE mode not found. Using ID 0 (Copter STABILIZE).")
        stabilize_mode_id = 0
    else:
        print(f"STABILIZE mode ID: {stabilize_mode_id}")

    print(f"Setting mode to STABILIZE (ID: {stabilize_mode_id})...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        stabilize_mode_id)

    # Wait for COMMAND_ACK for mode set
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack_msg:
        if ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Mode STABILIZE set successfully.")
            else:
                print(f"Set mode STABILIZE failed. Result: {ack_msg.result}")
                # Consider exiting if mode set fails
        else:
            # Received a COMMAND_ACK for a different command
            print(f"Received COMMAND_ACK for command {ack_msg.command}, expected {mavutil.mavlink.MAV_CMD_DO_SET_MODE}.")
    else:
        print("No COMMAND_ACK received for set_mode within timeout.")

    print("Attempting to arm motors...")
    master.arducopter_arm()
    print("Waiting for arming confirmation...")
    master.motors_armed_wait()
    print("FC reported ARMED. Motors are ready.")
    time.sleep(1) # Short pause after arming

    # --- Maneuver Sequence ---
    print(f"\n--- Starting Maneuver Sequence (Throttle: {RC_THROTTLE_HOVER_TEST}) ---")

    execute_maneuver("Hovering", MANEUVER_DURATION_S, throttle=RC_THROTTLE_HOVER_TEST)

    execute_maneuver("Forward", MANEUVER_DURATION_S, pitch=PITCH_FORWARD, throttle=RC_THROTTLE_HOVER_TEST)
    execute_maneuver("Neutral Hover", PAUSE_DURATION_S, throttle=RC_THROTTLE_HOVER_TEST)

    execute_maneuver("Backward", MANEUVER_DURATION_S, pitch=PITCH_BACKWARD, throttle=RC_THROTTLE_HOVER_TEST)
    execute_maneuver("Neutral Hover", PAUSE_DURATION_S, throttle=RC_THROTTLE_HOVER_TEST)

    execute_maneuver("Roll Left", MANEUVER_DURATION_S, roll=ROLL_LEFT, throttle=RC_THROTTLE_HOVER_TEST)
    execute_maneuver("Neutral Hover", PAUSE_DURATION_S, throttle=RC_THROTTLE_HOVER_TEST)

    execute_maneuver("Roll Right", MANEUVER_DURATION_S, roll=ROLL_RIGHT, throttle=RC_THROTTLE_HOVER_TEST)
    execute_maneuver("Neutral Hover", PAUSE_DURATION_S, throttle=RC_THROTTLE_HOVER_TEST)

    execute_maneuver("Yaw CCW (Turn Left)", MANEUVER_DURATION_S, yaw=YAW_CCW, throttle=RC_THROTTLE_HOVER_TEST)
    execute_maneuver("Neutral Hover", PAUSE_DURATION_S, throttle=RC_THROTTLE_HOVER_TEST)

    execute_maneuver("Yaw CW (Turn Right)", MANEUVER_DURATION_S, yaw=YAW_CW, throttle=RC_THROTTLE_HOVER_TEST)
    execute_maneuver("Neutral Hover", PAUSE_DURATION_S, throttle=RC_THROTTLE_HOVER_TEST)

    print("\n--- Maneuver Sequence Complete ---")
    # Lower throttle before disarming
    print("Lowering throttle to minimum...")
    send_rc_override(throttle=RC_THROTTLE_LOW)
    time.sleep(2) # Wait for throttle to take effect

    print("Attempting to disarm...")
    master.arducopter_disarm()
    print("Waiting for disarming confirmation...")
    master.motors_disarmed_wait()
    print("FC reported DISARMED. Test complete.")

except TimeoutError as te:
    print(f"TIMEOUT Error: {te}")
    print("If script hangs at 'Waiting for arming confirmation', the FC is not arming.")
    print("Check pre-arm checks in Mission Planner (e.g., ARMING_CHECK=0 for bench tests).")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    if master:
        print("Ensuring motors are stopped (sending neutral RC).")
        try:
            # Send neutral values with 0 throttle for all 8 channels
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component,
                RC_NEUTRAL, RC_NEUTRAL, 0, RC_NEUTRAL, # Ch 1-4 (Throttle 0)
                0, 0, 0, 0)                         # Ch 5-8
            print("Sent final neutral RC override (throttle 0).")
        except Exception as final_e:
            print(f"Error sending final neutral RC override: {final_e}")
        master.close()
        print("MAVLink connection closed.")