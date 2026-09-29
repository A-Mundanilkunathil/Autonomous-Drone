"""Pure safety policies for command arbitration."""

import math


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
