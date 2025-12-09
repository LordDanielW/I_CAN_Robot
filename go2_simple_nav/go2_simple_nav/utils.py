import math
from geometry_msgs.msg import Quaternion


def quaternion_to_yaw(q: Quaternion) -> float:
    # Convert quaternion to yaw (Z rotation) in radians.
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    # Wrap angle to [-pi, pi].
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))
