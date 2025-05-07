import RPI.GPIO as GPIO
from sense_hat import SenseHat

# Motor configuration
MOTOR_PINS = [17, 18, 27, 22]  # Example GPIO pins for motors
PWM_FREQUENCY = 50  # Frequency for ESCs

class MotorController:
    def __init__(self):
        self.motors = None
        self.initialized = False
        
    def setup(self):
        if self.initialized: 
            return self.motors
        
        # Set up GPIO
        GPIO.setmode(GPIO.BCM)
        for pin in MOTOR_PINS:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
            
        # Create PWM instances for each motor
        self.motors = [GPIO.PWM(pin, PWM_FREQUENCY) for pin in MOTOR_PINS]
        self.initialized = True
        return self.motors
    
    def start_motors(self, initial_speed=0):
        if not self.initialized:
            self.setup()
        
        # Start motors with initial speed
        for motor in self.motors:
            motor.start(initial_speed)
            
    def set_speeds(self, speeds):
        if not self.initialized: 
            return
        
        # Set speeds for each motor
        for i, speed in enumerate(speeds):
            if i < len(self.motors):
                self.motors[i].ChangeDutyCycle(speed)
                
    def stop_motors(self):
        if not self.initialized: 
            return
        
        # Stop motors
        for motor in self.motors:
            motor.stop()
            
        # Clean up GPIO
        self.initialized = False
        
    def cleanup(self):
        if self.initialized: 
            self.stop_motors()
        GPIO.cleanup()
        
class SensorController:
    def __init__(self):
        self.sense = SenseHat()
        self.calibrated = False
        self.offset_pitch = 0
        self.offset_roll = 0
        self.offset_yaw = 0
        
    def calibrate(self):
        orientation = self.sense.get_orientation()
        self.offset_pitch = orientation['pitch']
        self.offset_roll = orientation['roll']
        self.offset_yaw = orientation['yaw']
        self.calibrated = True
        print("Calibration complete.")
        
    def get_orientation(self):
        orientation = self.sense.get_orientation()
        
        if self.calibrated:
            # Return values relative to calibration point
            return {
                "pitch": orientation["pitch"] - self.offset_pitch,
                "roll": orientation["roll"] - self.offset_roll,
                "yaw": orientation["yaw"] - self.offset_yaw
            }
        else:
            return orientation
        
    def get_acceleration(self):
        return self.sense.get_accelerometer_raw()
    
    def get_gyroscope(self):
        return self.sense.get_gyroscope_raw()
    
    def set_led_color(self, r, g, b):
        self.sense.clear((r, g, b))
            
        
        