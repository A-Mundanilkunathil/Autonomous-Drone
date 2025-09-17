import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

drone_handle = sim.getObject('/Quadcopter')

def set_drone_position(x, y, z):
    sim.setObjectPosition(drone_handle, -1, [x, y, z])

def get_drone_position():
    return sim.getObjectPosition(drone_handle, -1)

def move_towards(target_pos, step=0.1, delay=0.05):
    """
    Smoothly move drone towards target_pos (x,y,z) in steps.
    """
    while True:
        current_pos = get_drone_position()
        dist = [target_pos[i] - current_pos[i] for i in range(3)]
        dist_mag = sum(d**2 for d in dist)**0.5
        if dist_mag < step:
            # Close enough, snap to target and break
            set_drone_position(*target_pos)
            break
        # Normalize step vector
        step_vec = [d / dist_mag * step for d in dist]
        new_pos = [current_pos[i] + step_vec[i] for i in range(3)]
        set_drone_position(*new_pos)
        time.sleep(delay)

# Define target position
target_x, target_y, target_z = 1.0, 1.0, 2.0

print("Taking off vertically...")
original_pos = get_drone_position() # Get original position
current_pos = get_drone_position() # Get current position
takeoff_pos = [current_pos[0], current_pos[1], target_z] # Set target z
move_towards(takeoff_pos) # Move to takeoff position

print("Moving horizontally to target x,y...")
horizontal_target = [target_x, target_y, target_z] # Set target z
move_towards(horizontal_target) # Move to horizontal target

print("Hovering at target...")
time.sleep(3)

print("Landing down at target...")
landing_target = [target_x, target_y, 0]
move_towards(landing_target)

print("Moving back to original position...")
move_towards(original_pos) # Move back to original position
print("Hovering at original position...")
time.sleep(3)

print("Landed. Done.")
