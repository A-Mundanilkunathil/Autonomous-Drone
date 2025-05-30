import serial
import time
import serial.tools.list_ports

def find_com_port():
    """Find available COM ports"""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print(f"Found: {p}")
    
    if not ports:
        return None
    
    # Use first port or let user choose
    return ports[0].device

def send_cli_command(ser, command):
    """Send a command to the CLI and read response"""
    print(f"Sending: {command}")
    ser.write((command + '\r\n').encode('utf-8'))
    time.sleep(0.5)
    
    response = ""
    while ser.in_waiting:
        response += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        time.sleep(0.1)
    
    print(f"Response: {response}")
    return response

def test_motors():
    # Connect to FC
    port = find_com_port()
    if not port:
        print("No COM ports found!")
        return
    
    print(f"Connecting to {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(1)
        
        # Enter CLI mode
        ser.write(b'#\r\n')  # Send # character to enter CLI
        time.sleep(1)
        
        # Clear buffer
        if ser.in_waiting:
            ser.read(ser.in_waiting)
        
        # Test each motor individually
        for motor_num in range(4):
            # Start motor at low power
            send_cli_command(ser, f"motor {motor_num} 1100")
            print(f"Motor {motor_num+1} should be spinning at 10% power")
            time.sleep(3)
            
            # Stop motor
            send_cli_command(ser, f"motor {motor_num} 1000")
            print(f"Motor {motor_num+1} should now be stopped")
            time.sleep(1)
            
        # Exit CLI mode
        send_cli_command(ser, "exit")
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_motors()