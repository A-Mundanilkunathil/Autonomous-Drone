"""Pure navigation calculations shared by ROS and simulation adapters."""

import math


EARTH_RADIUS_M = 6_371_000.0


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def haversine_distance_m(
    latitude_a: float,
    longitude_a: float,
    latitude_b: float,
    longitude_b: float,
) -> float:
    """Return great-circle distance between two WGS84 coordinates."""
    delta_latitude = math.radians(latitude_b - latitude_a)
    delta_longitude = math.radians(longitude_b - longitude_a)
    latitude_a_rad = math.radians(latitude_a)
    latitude_b_rad = math.radians(latitude_b)

    haversine = (
        math.sin(delta_latitude / 2.0) ** 2
        + math.cos(latitude_a_rad)
        * math.cos(latitude_b_rad)
        * math.sin(delta_longitude / 2.0) ** 2
    )
    return EARTH_RADIUS_M * 2.0 * math.atan2(
        math.sqrt(haversine), math.sqrt(1.0 - haversine)
    )


def target_reached(
    horizontal_distance_m: float,
    horizontal_tolerance_m: float,
    altitude_error_m: float = 0.0,
    altitude_tolerance_m: float | None = None,
) -> bool:
    """Return whether horizontal and optional altitude tolerances are met."""
    if horizontal_distance_m < 0.0 or horizontal_tolerance_m < 0.0:
        return False
    if horizontal_distance_m > horizontal_tolerance_m:
        return False
    if altitude_tolerance_m is None:
        return True
    return abs(altitude_error_m) <= altitude_tolerance_m
