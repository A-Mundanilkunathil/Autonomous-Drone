"""Pure safety policies for command arbitration."""

import math


def avoidance_yaw_rate(
    forward_clearance_m: float,
    left_clearance_m: float,
    right_clearance_m: float,
    *,
    stop_distance_m: float,
    caution_distance_m: float,
    max_yaw_rate_rps: float,
    turn_margin_m: float = 0.2,
    preferred_turn: int = 1,
) -> float:
    """Choose a bounded turn toward a genuinely open side corridor.

    Positive yaw turns left. If both open sides are effectively equal, a fixed
    preference prevents a symmetric deadlock. If neither side is known to be
    clear, the safe response is to hover rather than turn blindly.
    """
    values = (forward_clearance_m, stop_distance_m, caution_distance_m,
              max_yaw_rate_rps, turn_margin_m)
    if (not all(math.isfinite(value) for value in values)
            or stop_distance_m < 0.0
            or caution_distance_m <= stop_distance_m
            or max_yaw_rate_rps < 0.0):
        return 0.0
    if forward_clearance_m > caution_distance_m:
        return 0.0

    left = left_clearance_m if math.isfinite(left_clearance_m) else -math.inf
    right = right_clearance_m if math.isfinite(right_clearance_m) else -math.inf
    if max(left, right) <= caution_distance_m:
        return 0.0

    if left > right + turn_margin_m:
        direction = 1.0
    elif right > left + turn_margin_m:
        direction = -1.0
    else:
        direction = 1.0 if preferred_turn >= 0 else -1.0

    if forward_clearance_m <= stop_distance_m:
        intensity = 1.0
    else:
        intensity = (
            (caution_distance_m - forward_clearance_m)
            / (caution_distance_m - stop_distance_m)
        )
    return direction * max_yaw_rate_rps * max(0.0, min(1.0, intensity))


def forward_speed_for_clearance(
    clearance_m: float | None,
    mission_speed_mps: float,
    *,
    stop_distance_m: float,
    caution_distance_m: float,
    minimum_caution_speed_mps: float = 0.2,
) -> float:
    """Calculate a fail-closed forward speed from obstacle clearance.

    Missing, non-finite, or invalid clearance is treated as unsafe. This keeps
    loss of the perception stream from becoming permission to fly forward.
    """
    if (
        clearance_m is None
        or not math.isfinite(clearance_m)
        or clearance_m < 0.0
        or stop_distance_m < 0.0
        or caution_distance_m <= stop_distance_m
    ):
        return 0.0
    if clearance_m <= stop_distance_m:
        return 0.0
    if clearance_m >= caution_distance_m:
        return max(0.0, mission_speed_mps)

    interpolation = (clearance_m - stop_distance_m) / (
        caution_distance_m - stop_distance_m
    )
    caution_floor = min(
        max(0.0, minimum_caution_speed_mps), max(0.0, mission_speed_mps)
    )
    return caution_floor + (max(0.0, mission_speed_mps) - caution_floor) * interpolation
