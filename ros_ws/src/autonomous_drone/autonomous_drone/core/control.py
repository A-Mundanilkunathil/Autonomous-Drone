"""Pure control-law helpers using the application's FLU body convention."""


def damped_distance_speed(
    distance_error_m: float,
    distance_rate_mps: float,
    proportional_gain: float,
    derivative_gain: float,
) -> float:
    """Return forward speed; positive error means the target is too far away."""
    return (
        proportional_gain * distance_error_m
        + derivative_gain * distance_rate_mps
    )


def body_tracking_corrections(
    horizontal_image_error: float,
    vertical_image_error: float,
    lateral_gain: float,
    vertical_gain: float,
    yaw_gain: float,
) -> tuple[float, float, float]:
    """Return lateral, vertical, and yaw commands in Forward-Left-Up.

    Positive image X is right and positive image Y is down. FLU therefore
    requires negative commands to move or turn toward either positive error.
    """
    lateral_mps = -lateral_gain * horizontal_image_error
    vertical_mps = -vertical_gain * vertical_image_error
    yaw_rate_rad_s = -yaw_gain * horizontal_image_error
    return lateral_mps, vertical_mps, yaw_rate_rad_s
