import requests
import time
import cv2

# ESP32 IP - just the API, no streaming
ESP32_IP = "192.168.1.156"
API_URL = f"http://{ESP32_IP}/cv/action"

def test_cv_api():
    """Test CV API without camera streaming"""
    print(f"🚁 Testing CV API at {ESP32_IP}")
    print("Press Ctrl+C to stop")
    
    commands = ["hover", "up", "down", "left", "right", "forward", "backward"]
    
    while True:
        for cmd in commands:
            try:
                payload = {"action": cmd}
                print(f"📤 Sending: {cmd}")
                
                response = requests.post(API_URL, json=payload, timeout=10)
                print(f"✅ {cmd}: {response.status_code} - {response.text}")
                
                # Check ESP32 serial monitor for debug output
                
            except Exception as e:
                print(f"❌ {cmd} failed: {e}")
            
            time.sleep(2)  # Wait between commands
            
            # Check if user wants to quit
            try:
                # Non-blocking input check would go here
                pass
            except KeyboardInterrupt:
                print("\n👋 Stopping test")
                return

if __name__ == "__main__":
    test_cv_api()