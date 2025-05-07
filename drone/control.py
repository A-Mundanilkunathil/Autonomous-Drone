import asyncio
from hardware import MotorController, SensorController

class DroneController:
    def __init__(self):
        self.motor_controller = MotorController()
        self.sensor_controller = SensorController()
        self.is_flying = False
        self.hover_speed = 30  # Base hover speed (%)
        self.motors = None
    
    async def takeoff(self):
        print("Taking off...")
        
        # Setup and start motors
        self.motors = self.motor_controller.setup()
        self.motor_controller.start_motors(0)
        
        # Gradually increase speed to takeoff
        for speed in range(0, self.hover_speed + 10, 5):
            self.motor_controller.set_speeds([speed] * 4)
            await asyncio.sleep(0.5)
        
        # Stabilize at hover speed
        self.motor_controller.set_speeds([self.hover_speed] * 4)
        await asyncio.sleep(2)
        
        self.is_flying = True
        print("Drone has taken off!")
        return self.motors
    
    async def land(self):
        if not self.is_flying:
            print("Drone is not flying")
            return
            
        print("Landing...")
        
        # Gradually decrease speed
        for speed in range(self.hover_speed, -1, -5):
            self.motor_controller.set_speeds([speed] * 4)
            await asyncio.sleep(0.5)
        
        # Stop motors and clean up
        self.motor_controller.stop_motors()
        self.motor_controller.cleanup()
        
        self.is_flying = False
        print("Drone has landed safely")
    
    async def move_forward(self, duration=1.0, speed_delta=10):
        if not self.is_flying:
            return
            
        print("Moving forward")
        # Tilt forward by adjusting motor speeds
        speeds = [
            self.hover_speed - speed_delta,  # Front-left lower
            self.hover_speed - speed_delta,  # Front-right lower
            self.hover_speed + speed_delta,  # Back-left higher
            self.hover_speed + speed_delta   # Back-right higher
        ]
        self.motor_controller.set_speeds(speeds)
        
        await asyncio.sleep(duration)
        
        # Return to hover
        self.motor_controller.set_speeds([self.hover_speed] * 4)
    
    async def move_backward(self, duration=1.0, speed_delta=10):
        if not self.is_flying:
            return
            
        print("Moving backward")
        # Tilt backward by adjusting motor speeds
        speeds = [
            self.hover_speed + speed_delta,  # Front-left higher
            self.hover_speed + speed_delta,  # Front-right higher
            self.hover_speed - speed_delta,  # Back-left lower
            self.hover_speed - speed_delta   # Back-right lower
        ]
        self.motor_controller.set_speeds(speeds)
        
        await asyncio.sleep(duration)
        
        # Return to hover
        self.motor_controller.set_speeds([self.hover_speed] * 4)
    
    async def move_left(self, duration=1.0, speed_delta=10):
        if not self.is_flying:
            return
            
        print("Moving left")
        # Tilt left by adjusting motor speeds
        speeds = [
            self.hover_speed - speed_delta,  # Front-left lower
            self.hover_speed + speed_delta,  # Front-right higher
            self.hover_speed + speed_delta,  # Back-left higher
            self.hover_speed - speed_delta   # Back-right lower
        ]
        self.motor_controller.set_speeds(speeds)
        
        await asyncio.sleep(duration)
        
        # Return to hover
        self.motor_controller.set_speeds([self.hover_speed] * 4)
    
    async def move_right(self, duration=1.0, speed_delta=10):
        if not self.is_flying:
            return
            
        print("Moving right")
        # Tilt right by adjusting motor speeds
        speeds = [
            self.hover_speed + speed_delta,  # Front-left higher
            self.hover_speed - speed_delta,  # Front-right lower
            self.hover_speed - speed_delta,  # Back-left lower
            self.hover_speed + speed_delta   # Back-right higher
        ]
        self.motor_controller.set_speeds(speeds)
        
        await asyncio.sleep(duration)
        
        # Return to hover
        self.motor_controller.set_speeds([self.hover_speed] * 4)
    
    async def hover(self):
        if not self.is_flying:
            return
            
        print("Hovering")
        self.motor_controller.set_speeds([self.hover_speed] * 4)
    
    def emergency_stop(self):
        print("EMERGENCY STOP")
        self.motor_controller.stop_motors()
        self.motor_controller.cleanup()
        self.is_flying = False