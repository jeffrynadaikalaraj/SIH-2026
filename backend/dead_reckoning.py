"""Backend boundary for the team's dead-reckoning implementation.

Replace ``estimate_position`` with the teammate's trained/physics engine when
it is ready. The route layer should only depend on this function's contract.
"""

import math
from typing import Any


EARTH_RADIUS_M = 6_371_000.0


def haversine_distance(
    latitude_1: float,
    longitude_1: float,
    latitude_2: float,
    longitude_2: float,
) -> float:
    """Return the great-circle distance between two coordinates in metres."""
    latitude_1_rad = math.radians(latitude_1)
    latitude_2_rad = math.radians(latitude_2)
    delta_latitude = math.radians(latitude_2 - latitude_1)
    delta_longitude = math.radians(longitude_2 - longitude_1)
    arc = (
        math.sin(delta_latitude / 2) ** 2
        + math.cos(latitude_1_rad)
        * math.cos(latitude_2_rad)
        * math.sin(delta_longitude / 2) ** 2
    )
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(arc), math.sqrt(1 - arc))


def estimate_position(
    sensor_data: dict[str, Any],
    previous_state: dict[str, Any] | None,
) -> dict[str, float] | None:
    """Estimate the next position using the previous estimate and IMU inputs.

    This temporary implementation integrates speed along the supplied heading.
    It intentionally never reads latitude or longitude from ``sensor_data``.
    Return ``None`` until a previous GNSS-backed estimate exists.
    """
    if not previous_state or previous_state.get("latitude") is None:
        return None

    previous_timestamp = previous_state.get("timestamp")
    if previous_timestamp is None:
        return None

    delta_time = max(0.0, float(sensor_data["timestamp"]) - float(previous_timestamp))
    distance_m = max(0.0, float(sensor_data["speed"])) * delta_time
    heading_rad = math.radians(float(sensor_data["heading"]))
    north_m = distance_m * math.cos(heading_rad)
    east_m = distance_m * math.sin(heading_rad)

    latitude = float(previous_state["latitude"]) + math.degrees(
        north_m / EARTH_RADIUS_M
    )
    cosine_latitude = math.cos(math.radians(float(previous_state["latitude"])))
    if abs(cosine_latitude) < 1e-10:
        cosine_latitude = 1e-10
    longitude = float(previous_state["longitude"]) + math.degrees(
        east_m / (EARTH_RADIUS_M * cosine_latitude)
    )

    return {"latitude": latitude, "longitude": longitude}
