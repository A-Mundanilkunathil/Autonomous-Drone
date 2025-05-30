import serial
import time
import struct
import serial.tools.list_ports

# MSP Commands
MSP_SET_RAW_RC = 200

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return None
    
    for port in ports:
        print(f"Found port: {port.device} - {port.description}")
    
    # Use the first available port or let user choose
    return ports[0].device

def connect(port=None, baud=115200):
    if not port:
        port = find_serial_port()
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud")
        return ser
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return None

def send_msp_set_rc(ser, channels):
    """Send MSP_SET_RAW_RC command with timing delays"""
    if not ser or not ser.is_open:
        print("Serial port not open")
        return False
        
    # Ensure we have 8 channels
    if len(channels) < 8:
        channels = channels + [1500] * (8 - len(channels))
    
    # Build MSP packet
    data = []
    for ch in channels:
        # Convert each channel value to two bytes (little endian)
        data.append(ch & 0xFF)          # Low byte
        data.append((ch >> 8) & 0xFF)   # High byte
    
    # Calculate packet length
    data_length = len(data)
    
    # Calculate checksum
    checksum = data_length ^ MSP_SET_RAW_RC
    for byte in data:
        checksum ^= byte
    
    # Send header with micro-delays
    ser.write(b'$')
    time.sleep(0.001)
    ser.write(b'M')
    time.sleep(0.001)
    ser.write(b'<')
    time.sleep(0.001)
    
    # Send payload size
    ser.write(bytes([data_length]))
    time.sleep(0.001)
    
    # Send command
    ser.write(bytes([MSP_SET_RAW_RC]))
    time.sleep(0.001)
    
    # Send data (with small delays)
    for byte in data:
        ser.write(bytes([byte]))
        time.sleep(0.0005)  # 0.5ms delay between bytes
    
    # Send checksum
    ser.write(bytes([checksum]))
    
    # Ensure everything is sent
    ser.flush()
    return True

def continuous_control(ser, channels, duration=5):
    """Send commands continuously for a period"""
    start_time = time.time()
    while time.time() - start_time < duration:
        send_msp_set_rc(ser, channels)
        time.sleep(0.02)  # 50Hz update rate

def simple_test(ser):
    """Very simple arming and throttle test"""
    channels = [1500, 1500, 1050, 1500, 1000, 2000, 1000, 1000]
    
    # Step 1: Disarm with throttle low
    print("Step 1: Disarming")
    channels[2] = 1000  # Throttle low
    channels[4] = 1000  # AUX1 - Disarm
    continuous_control(ser, channels, 2)
    
    # Step 2: Enable angle mode
    print("Step 2: Setting angle mode")
    channels[5] = 2000  # AUX2 - Angle mode
    continuous_control(ser, channels, 1)
    
    # Step 3: Arm with throttle still low
    print("Step 3: Arming")
    channels[4] = 2000  # AUX1 - Arm
    continuous_control(ser, channels, 3)  # Send longer to ensure arming
    
    # Step 4: Set very high throttle (many ESCs need >1700 first time)
    print("Step 4: Setting high throttle")
    channels[2] = 1800  # Very high throttle
    continuous_control(ser, channels, 5)  # Hold for 5 seconds
    
    # Step 5: Disarm
    print("Step 5: Disarming")
    channels[4] = 1000
    channels[2] = 1000
    continuous_control(ser, channels, 1)

def extreme_test(ser):
    channels = [1500, 1500, 1000, 1500, 1000, 2000, 1000, 1000]
    
    # Set angle mode first
    print("Setting angle mode")
    channels[5] = 2000
    continuous_control(ser, channels, 1)
    
    # Arm with throttle at MIN
    print("Arming...")
    channels[4] = 2000  # Arm
    continuous_control(ser, channels, 2)
    
    # Quick full throttle and back to medium
    print("Full throttle pulse!")
    channels[2] = 2000  # FULL throttle
    continuous_control(ser, channels, 0.2)
    
    # Medium throttle
    print("Medium throttle...")
    channels[2] = 1500
    continuous_control(ser, channels, 5)
    
    # Disarm
    channels[4] = 1000
    channels[2] = 1000
    continuous_control(ser, channels, 1)

def check_arming_flags(ser):
    """Send MSP command to get arming flags and display why motors won't spin"""
    # MSP_STATUS command
    ser.write(b'$M<\x00\x64\x64')
    time.sleep(0.1)
    
    # Read response
    response = b''
    while ser.in_waiting:
        response += ser.read()
    
    print(f"Debug response: {response.hex()}")
    return response

def check_fc_status(ser):
    """Get comprehensive flight controller status"""
    print("\n=== FLIGHT CONTROLLER STATUS ===")
    
    # Clear any pending data
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    
    # Request FC status (MSP_STATUS command)
    ser.write(b'$M<\x00\x64\x64')
    time.sleep(0.1)
    
    # Process response
    status_data = b''
    while ser.in_waiting:
        status_data += ser.read()
    
    print(f"Raw status response: {status_data.hex()}")
    
    # Request arming status (MSP_BOXIDS command - will show active modes)
    ser.write(b'$M<\x00\x34\x34')
    time.sleep(0.1)
    
    boxes_data = b''
    while ser.in_waiting:
        boxes_data += ser.read()
    
    print(f"Box status response: {boxes_data.hex()}")
    
    # Try sending simple version request
    ser.write(b'$M<\x00\x03\x03')  # MSP_IDENT
    time.sleep(0.1)
    
    ident_data = b''
    while ser.in_waiting:
        ident_data += ser.read()
    
    print(f"Version response: {ident_data.hex()}")
    
    # Check if the FC is stuck in CLI mode
    ser.write(b'\r\n')  # Send newline
    time.sleep(0.1)
    cli_check = b''
    while ser.in_waiting:
        cli_check += ser.read()
    
    if b'# ' in cli_check:
        print("ISSUE DETECTED: Flight controller is in CLI mode!")
        print("Try running the script with INAV Configurator closed")
    
    # Results analysis
    print("\n=== ANALYSIS ===")
    if len(status_data) < 5 and len(boxes_data) < 5:
        print("No valid responses received. Possible issues:")
        print("1. Flight controller not responding to MSP commands")
        print("2. Baud rate mismatch")
        print("3. MSP protocol version mismatch")
        print("4. Wrong serial port or connection issues")
    
    print("\nTry these steps:")
    print("1. Close INAV Configurator completely")
    print("2. Power cycle the flight controller")
    print("3. Try a different USB port")
    print("4. Check physical connections to ESCs and motors")
    print("===============================")
    
    return status_data

def main():
    # Connect to serial port
    ser = connect()
    if not ser:
        print("Failed to connect to serial port. Exiting.")
        return
    
    # Check and exit CLI mode if necessary
    detect_and_handle_cli_mode(ser)
    
    try:
        print("\n1. Simple Test (just tries to spin motors)")
        print("2. ESC Wake-Up Sequence (helps with stubborn ESCs)")
        print("3. Interactive Test (keyboard control)")
        print("4. Advanced Diagnostics (checks FC status)")
        choice = input("Select test (1-4): ")
        
        if choice == '1':
            simple_test(ser)
        elif choice == '2':
            extreme_test(ser)
        elif choice == '3':
            interactive_test(ser)
        elif choice == '4':
            diagnostic_test(ser)
        else:
            print("Invalid choice")
            
    except KeyboardInterrupt:
        # Safely disarm on Ctrl+C
        print("\nEmergency disarm!")
        channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        send_msp_set_rc(ser, channels)
    finally:
        # Close serial port
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed")

def interactive_test(ser):
    """Interactive keyboard-controlled test with better arming sequence"""
    import msvcrt  # Windows keyboard input
    
    channels = [1500, 1500, 1050, 1500, 1000, 2000, 1000, 1000]
    armed = False
    
    print("\n=== INTERACTIVE TEST MODE ===")
    print("Commands:")
    print("  a - Arm/Disarm")
    print("  m - Toggle Angle Mode")
    print("  + - Increase Throttle")
    print("  - - Decrease Throttle")
    print("  c - Calibration Pulse")
    print("  d - Diagnostics")
    print("  q - Quit")
    print("=====================")
    
    throttle = 1000
    
    while True:
        # Send current values
        send_msp_set_rc(ser, channels)
        
        if msvcrt.kbhit():
            key = msvcrt.getch().decode('utf-8', errors='ignore').lower()
            
            if key == 'q':
                # Disarm before quitting
                channels[4] = 1000  # AUX1 - Disarm
                channels[2] = 1000  # Throttle low
                for _ in range(10):  # Send multiple times to ensure it's received
                    send_msp_set_rc(ser, channels)
                    time.sleep(0.05)
                print("Disarmed and quitting")
                break
                
            elif key == 'a':
                if not armed and throttle > 1050:
                    print("Cannot arm with throttle raised! Lower throttle first.")
                else:
                    armed = not armed
                    channels[4] = 2000 if armed else 1000  # AUX1
                    if armed:
                        print("ARMING - sending continuous arm commands...")
                        # Send multiple arm commands to ensure it's received
                        for _ in range(15):
                            send_msp_set_rc(ser, channels)
                            time.sleep(0.05)
                    else:
                        print("DISARMING")
                        # Immediately set throttle to min when disarming
                        throttle = 1000
                        channels[2] = throttle
                
            elif key == 'm':
                channels[5] = 2000 if channels[5] == 1000 else 1000  # AUX2
                print(f"Angle mode: {'ON' if channels[5] == 2000 else 'OFF'}")
                
            elif key == '+':
                if not armed and throttle >= 1050:
                    print("Cannot raise throttle when disarmed!")
                else:
                    throttle = min(throttle + 50, 2000)
                    channels[2] = throttle  # Throttle
                    print(f"Throttle: {throttle}")
                
            elif key == '-':
                throttle = max(throttle - 50, 1000)
                channels[2] = throttle  # Throttle
                print(f"Throttle: {throttle}")
                
            elif key == 'c':
                # Send calibration pulse sequence
                print("Sending ESC calibration pulse sequence...")
                old_throttle = channels[2]
                old_arm = channels[4]
                
                # Disarm and set max throttle
                channels[4] = 1000  # Disarm
                channels[2] = 2000  # Max throttle
                for _ in range(20):
                    send_msp_set_rc(ser, channels)
                    time.sleep(0.05)
                
                # Set min throttle
                channels[2] = 1000  # Min throttle
                for _ in range(20):
                    send_msp_set_rc(ser, channels)
                    time.sleep(0.05)
                    
                # Restore previous state
                channels[2] = old_throttle
                channels[4] = old_arm
                print("Calibration sequence complete")
                
            elif key == 'd':
                # Enhanced diagnostics
                print("\n=== DIAGNOSTICS ===")
                print(f"Armed: {armed}")
                print(f"Throttle: {throttle}")
                print(f"Angle Mode: {'ON' if channels[5] == 2000 else 'OFF'}")
                print(f"All Channels: {channels}")
                print("Checking arming flags...")
                check_arming_flags(ser)
                print("===================\n")
        
        time.sleep(0.02)  # 50Hz update rate

def detect_and_handle_cli_mode(ser):
    """Check if FC is in CLI mode and fix it"""
    # Send a few newlines to stabilize any CLI state
    ser.write(b'\r\n\r\n')
    time.sleep(0.5)
    
    # Clear input buffer
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    
    # Send exit command to leave CLI mode
    ser.write(b'exit\r\n')
    time.sleep(0.5)
    
    # Read response to check if we were in CLI mode
    response = b''
    while ser.in_waiting:
        response += ser.read()
    
    if b'CLI mode' in response or b'# ' in response:
        print("Flight controller was in CLI mode. Attempting to exit...")
        ser.write(b'exit\r\n')
        time.sleep(0.5)
        ser.write(b'save\r\n')  # Save any changes
        time.sleep(1.0)
        
        # Clear buffer again
        while ser.in_waiting:
            ser.read(ser.in_waiting)
            
        print("CLI mode should be exited now.")
    
    # Now start MSP communication
    return True

def diagnostic_test(ser):
    """Test with intensive diagnostics"""
    channels = [1500, 1500, 1000, 1500, 1000, 2000, 1000, 1000]
    
    print("Starting diagnostic test...")
    
    # Step 1: Check FC status before starting
    print("Step 1: Checking FC status...")
    check_fc_status(ser)
    
    # Step 2: Try to exit CLI mode robustly
    print("Step 2: Ensuring FC is not in CLI mode...")
    for _ in range(3):  # Try multiple times
        ser.write(b'exit\r\n')
        time.sleep(0.2)
    
    # Step 3: Disarm and set throttle low
    print("Step 3: Disarming...")
    channels[2] = 1000  # Throttle low
    channels[4] = 1000  # AUX1 - Disarm
    for _ in range(20):  # Send more commands to ensure it's processed
        send_msp_set_rc(ser, channels)
        time.sleep(0.05)
    
    # Step 4: Enable angle mode
    print("Step 4: Setting angle mode...")
    channels[5] = 2000  # AUX2 - Angle mode
    for _ in range(20):
        send_msp_set_rc(ser, channels)
        time.sleep(0.05)
    
    # Step 5: Try to arm
    print("Step 5: Attempting to arm...")
    channels[4] = 2000  # AUX1 - Arm
    for _ in range(30):  # Send more commands to ensure it's processed
        send_msp_set_rc(ser, channels)
        time.sleep(0.05)
    
    # Step 6: Check if armed
    print("Step 6: Checking if armed...")
    check_fc_status(ser)
    
    # Step 7: Try raising throttle in steps
    print("Step 7: Raising throttle gradually...")
    for throttle in range(1000, 2001, 100):
        channels[2] = throttle
        print(f"  Setting throttle: {throttle}")
        for _ in range(10):
            send_msp_set_rc(ser, channels)
            time.sleep(0.05)
        time.sleep(0.5)
    
    # Step 8: Return to zero and disarm
    print("Step 8: Disarming...")
    channels[4] = 1000  # Disarm
    channels[2] = 1000  # Zero throttle
    for _ in range(20):
        send_msp_set_rc(ser, channels)
        time.sleep(0.05)
    
    check_fc_status(ser)
    return

def parse_msp_response(data, command_name):
    """Parse and interpret MSP response data"""
    if not data or len(data) < 5:
        return f"Invalid response for {command_name}"
    
    # Basic validation of MSP header
    if data[0:2] != b'$M':
        return f"Invalid MSP header in {command_name} response"
    
    # Check for common MSP error patterns
    if b'CLI mode' in data or b'# ' in data:
        return "FC is in CLI mode and not processing MSP commands"
    
    # For MSP_STATUS, try to extract arming flags
    if command_name == "MSP_STATUS" and len(data) > 6:
        try:
            # Extract bytes that might contain arming flags
            flags_bytes = data[5:9]
            flags = int.from_bytes(flags_bytes, byteorder='little')
            arming_status = "DISARMED"
            if flags & 0x01:
                arming_status = "ARMED"
            return f"FC Status: {arming_status}, Flags: {flags:08x}"
        except Exception as e:
            return f"Failed to parse status: {e}"
    
    return f"Response received for {command_name} ({len(data)} bytes)"

def fix_fc_configuration(ser):
    """Apply comprehensive fixes to FC configuration"""
    print("\n==== FIXING FC CONFIGURATION ====")
    
    # Step 1: Exit CLI mode definitively
    print("1. Exiting CLI mode...")
    ser.write(b'exit\r\n')
    time.sleep(0.5)
    ser.write(b'save\r\n')
    time.sleep(1.0)
    # Clear buffer
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    
    # Step 2: Send basic MSP initialization
    print("2. Sending MSP initialization sequence...")
    # MSP_API_VERSION request
    ser.write(b'$M<\x00\x01\x01')
    time.sleep(0.2)
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    
    # Step 3: Configure FC for MSP receiver
    print("3. Setting MSP as receiver via CLI...")
    ser.write(b'serial 0 1 115200 57600 0 115200\r\n')
    time.sleep(0.3)
    ser.write(b'set receiver_type = MSP\r\n')
    time.sleep(0.3)
    ser.write(b'set serialrx_provider = MSP\r\n')
    time.sleep(0.3)
    ser.write(b'set rx_min_usec = 885\r\n')
    time.sleep(0.3)
    ser.write(b'set rx_max_usec = 2115\r\n')
    time.sleep(0.3)
    ser.write(b'set min_throttle = 1000\r\n')
    time.sleep(0.3)
    ser.write(b'set motor_pwm_protocol = STANDARD\r\n')
    time.sleep(0.3)
    ser.write(b'save\r\n')
    time.sleep(2.0)
    
    # Clear buffer
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    
    print("FC configuration complete. Please power cycle your flight controller.")
    print("After restarting, run the ESC calibration test.")
    print("==============================\n")
    
    return True

if __name__ == "__main__":
    main()