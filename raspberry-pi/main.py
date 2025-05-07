import asyncio
import signal 
import sys
from control import DroneController
from camera import DroneCamera
from remote_server import DroneRemoteServer

# Global reference to drone for clean shutdown
drone = None

def signal_handler(sig, frame):
    if drone:
        drone.emergency_stop()
    sys.exit(0)
    
async def main():
    global drone
    
    # Initialize drone components
    drone = DroneController()
    camera = DroneCamera()
    
    # Initialize remote server
    remote_server = DroneRemoteServer(drone, camera)
    remote_server.start()
    
    print("Drone is ready!")
    print("Press Ctrl+C to exit.")
    
    try: 
        # Keep the main thread alive
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError: # Handle cancellation
        pass
    finally:
        camera.stop()
        remote_server.stop()
        print("Drone components stopped.")

if __name__ == "__main__":
    # Register signal handlers for clean shutdown
    signal.signal(signal.SIGINT, signal_handler) # Signal for Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler) # Signal for termination
    
    # Run main function
    asyncio.run(main())