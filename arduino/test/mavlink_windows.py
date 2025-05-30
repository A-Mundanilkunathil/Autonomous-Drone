import time
from pymavlink import mavutil

# --- Configuration ---
MAVLINK_PORT = 'COM7'  # Your COM port
BAUD_RATE = 115200

RC_ROLL_VALUE = 1500 # Neutral value for RC roll channel
RC_PITCH_VALUE = 1500 # Set to 1500 for no pitch input
RC_YAW_VALUE = 1500 # Set to 1500 for no roll input
RC_THROTTLE_VALUE = 1200  # Set to 1200 for no throttle input
RC_AUX_CHANNELS_VALUE = 0 # Set to 0 for no AUX channel input
CONTROL_DURATION_S = 5  # Duration to control motors in seconds

# --- Script ---
master = None
try:
    print(f"Connecting to MAVLink on {MAVLINK_PORT} at {BAUD_RATE} baud...")
    master = mavutil.mavlink_connection(MAVLINK_PORT, baud=BAUD_RATE)

    print("Waiting for heartbeat...")
    master.wait_heartbeat() # Wait for the FC to respond
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

    stabilize_mode_id = master.mode_mapping()['STABILIZE']
    if stabilize_mode_id is None:
        print("STABILIZE mode not found. Please ensure your flight controller supports it.")
        stabilize_mode_id = 0  # Fallback to manual mode
        
    print(f"Setting mode to STABILIZE (ID: {stabilize_mode_id})...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        stabilize_mode_id)
    
    # Verify mode change
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Mode STABILIZE set successfully.")
        else:
            print(f"Set mode failed. Result: {ack_msg.result}")
            # You might want to exit or handle this error
    else:
        print("No COMMAND_ACK received for set_mode.")
        
    print("Attempting to arm motors...")
    master.arducopter_arm()  # Arm the motors
    print("Waiting for arming confirmation...")
    master.motors_armed_wait()  # Wait until motors are armed
    print("FC reported ARMED. Motors are ready.")
    
    
    print(f"Sending RC Override commands for {CONTROL_DURATION_S} seconds...")
    print(f"  Roll: {RC_ROLL_VALUE}, Pitch: {RC_PITCH_VALUE}, Throttle: {RC_THROTTLE_VALUE}, Yaw: {RC_YAW_VALUE}")


    start_time = time.time()
    while time.time() < start_time + CONTROL_DURATION_S:
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            RC_ROLL_VALUE,  # Roll channel
            RC_PITCH_VALUE,  # Pitch channel
            RC_THROTTLE_VALUE,  # Throttle channel
            RC_YAW_VALUE,  # Yaw channel
            RC_AUX_CHANNELS_VALUE,  # AUX channel 1
            RC_AUX_CHANNELS_VALUE,  # AUX channel 2
            RC_AUX_CHANNELS_VALUE,  # AUX channel 3
            RC_AUX_CHANNELS_VALUE,  # AUX channel 4
        )
        time.sleep(0.1)  # Send commands at a reasonable rate
    print("Motor test command sent.")

    print("Stopping RC Override commands...")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500,  # Roll channel
        1500,  # Pitch channel
        1000,  # Throttle channel
        1500,  # Yaw channel
        RC_AUX_CHANNELS_VALUE,  # AUX channel 1
        RC_AUX_CHANNELS_VALUE,  # AUX channel 2
        RC_AUX_CHANNELS_VALUE,  # AUX channel 3
        RC_AUX_CHANNELS_VALUE,  # AUX channel 4
    )
    time.sleep(5)   # Wait for a few seconds to ensure motors stop

    print("Attempting to disarm...")
    master.arducopter_disarm()
    print("Waiting for disarming confirmation...") # Added print statement
    master.motors_disarmed_wait() # Removed timeout argument
    print("FC reported DISARMED. Test complete.")

except TimeoutError:
    print("TIMEOUT: Failed to receive heartbeat (this shouldn't happen if previous step passed).")
    print("If script hangs at 'Waiting for arming confirmation', the FC is not arming.")
except Exception as e:
    print(f"An error occurred: {e}")
    print("If the script hangs at 'Waiting for arming confirmation', it means the FC is not arming due to pre-arm checks or an immediate failsafe.")
finally:
    if master:
        # Ensure we send a final neutral RC override to stop motors
        try:
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0) # Or UINT16_MAX for all
            print("Sent final neutral RC override.")
        except Exception as final_e:
            print(f"Error sending final neutral RC override: {final_e}")
        master.close()
        print("MAVLink connection closed.")
