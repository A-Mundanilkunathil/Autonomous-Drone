"""
Utility functions for coordinate transforms and conversions
"""
import math
from geometry_msgs.msg import Quaternion

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    Convert Euler angles to Quaternion

    Why: ROS uses Quaternions for orientation
    """
    cr = math.cos(roll / 2) # roll cosine
    sr = math.sin(roll / 2) # roll sine
    cp = math.cos(pitch / 2) # pitch cosine
    sp = math.sin(pitch / 2) # pitch sine
    cy = math.cos(yaw / 2) # yaw cosine
    sy = math.sin(yaw / 2) # yaw sine

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy # scalar part
    q.x = sr * cp * cy - cr * sp * sy # x component
    q.y = cr * sp * cy + sr * cp * sy # y component
    q.z = cr * cp * sy - sr * sp * cy # z component
    return q

def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Clamp a value between min_val and max_val

    Why: Ensure values stay within safe/expected ranges
    """
    return max(min(value, max_val), min_val)