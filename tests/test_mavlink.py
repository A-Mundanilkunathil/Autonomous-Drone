from pymavlink import mavutil
import time

def main():
    print("Simple MAVLink test - Stabilize + Arm commands")
    
    # Connect
    master = mavutil.mavlink_connection('COM6', baud=57600)
    master.wait_heartbeat()
    print(f"Connected to system {master.target_system} component {master.target_component}")
    
    # Request some data streams
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
        2, 1  # 2Hz
    )
    
    try:
        start_time = time.time()
        last_mode_cmd = 0
        last_arm_cmd = 0
        last_status = 0
        
        while True:
            now = time.time()
            
            # Send heartbeat every second
            if now - start_time >= 1.0:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                start_time = now
            
            # Send STABILIZE mode command every 2 seconds
            if now - last_mode_cmd >= 2.0:
                mode_mapping = master.mode_mapping()
                if 'STABILIZE' in mode_mapping:
                    stabilize_mode = mode_mapping['STABILIZE']
                    master.mav.set_mode_send(
                        master.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        stabilize_mode
                    )
                    print(f"[{now:.1f}s] Sent STABILIZE mode command")
                last_mode_cmd = now
            
            # Send ARM command every 3 seconds
            if now - last_arm_cmd >= 3.0:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,  # confirmation
                    1,  # arm (1) or disarm (0)
                    0, 0, 0, 0, 0, 0
                )
                print(f"[{now:.1f}s] Sent ARM command")
                last_arm_cmd = now
            
            # Check for messages and print status
            msg = master.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                
                if msg_type == 'HEARTBEAT':
                    armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    mode = msg.custom_mode
                    if now - last_status >= 1.0:
                        print(f"[{now:.1f}s] Status: Armed={armed} Mode={mode}")
                        last_status = now
                
                elif msg_type == 'COMMAND_ACK':
                    cmd = msg.command
                    result = msg.result
                    if cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            print(f"[{now:.1f}s] ARM command ACCEPTED")
                        else:
                            print(f"[{now:.1f}s] ARM command FAILED: {result}")
                
                elif msg_type == 'STATUSTEXT':
                    text = msg.text.decode('utf-8', errors='ignore')
                    print(f"[{now:.1f}s] STATUS: {text}")
            
            time.sleep(0.1)  # 10Hz loop
            
    except KeyboardInterrupt:
        print("\nStopping test...")
        
        # Send disarm before exit
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0  # disarm
        )
        print("Sent DISARM command")
        time.sleep(1)

if __name__ == "__main__":
    main()