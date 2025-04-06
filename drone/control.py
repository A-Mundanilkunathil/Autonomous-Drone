from sense_hat import SenseHat
import asyncio
import RPI.GPIO as GPIO # pip install RPI.GPIO

sense = SenseHat()

# Set up GPIO pins
MOTOR_PINS = [17, 18, 27, 22]  # Example GPIO pins for motors

def setup_gpio():
    GPIO.setmode(GPIO.BCM) # Use BCM pin numbering
    for pin in MOTOR_PINS:
        GPIO.setup(pin, GPIO.OUT)  # Set motor pins as output
        GPIO.output(pin, GPIO.LOW)  # Initialize motors to off
    
    # Create PWM instances for each motor
    motor_pwm = [GPIO.PWM(pin, 50) for pin in MOTOR_PINS] # 50Hz frequency
    return motor_pwm

async def takeoff():
    # Set up GPIO
    motors = setup_gpio()  
    
    print("Taking off...")
    
    # Initialize motors to off
    for motor in motors:
        motor.start(0) # Start PWM with 0% duty cycle (off)
        
    # Gradually increase speed to takeoff
    for speed in range(0, 40, 5): # 0% to 40% power
        for motor in motors:
            motor.ChangeDutyCycle(speed)
        await asyncio.sleep(0.5)
        
    await asyncio.sleep(2)  # Simulate takeoff duration
    print("Drone has taken off!")
    
    return motors

async def land(motors):
    print("Landing...")
    
    # Gradually decrease speed
    for speed in range(40, -1, -5): # 40% down to 0%
        for motor in motors:
            motor.ChangeDutyCycle(speed)
        await asyncio.sleep(0.5)
        
    # Stop motors
    for motor in motors:
        motor.stop()
        
    # Clean up GPIO
    GPIO.cleanup()
    print("Drone has landed and GPIO cleaned up.")
        
async def main():
    motors = await takeoff()
    await asyncio.sleep(5)  # Simulate flight duration
    await land(motors)
    
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        GPIO.cleanup()  # Clean up GPIO on exit
        print("Program interrupted. GPIO cleaned up.")