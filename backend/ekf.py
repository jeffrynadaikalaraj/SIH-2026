"""Small East/North constant-velocity EKF for the prototype.

State: [east_m, north_m, east_velocity_mps, north_velocity_mps].
GNSS and DR positions are converted to a local tangent plane around the first
GNSS fix, avoiding latitude/longitude as Cartesian filter state variables.
"""

import math
from typing import Any

import numpy as np

EARTH_RADIUS_M = 6_371_000.0


def gps_to_local(reference: tuple[float, float], position: tuple[float, float]) -> np.ndarray:
    ref_lat, ref_lon = reference
    lat, lon = position
    east = math.radians(lon - ref_lon) * EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    north = math.radians(lat - ref_lat) * EARTH_RADIUS_M
    return np.array([east, north], dtype=float)


def local_to_gps(reference: tuple[float, float], local: np.ndarray) -> tuple[float, float]:
    ref_lat, ref_lon = reference
    lat = ref_lat + math.degrees(float(local[1]) / EARTH_RADIUS_M)
    cosine = max(abs(math.cos(math.radians(ref_lat))), 1e-10)
    lon = ref_lon + math.degrees(float(local[0]) / (EARTH_RADIUS_M * cosine))
    return lat, lon


class NavigationEKF:
    """Constant-velocity EKF with position measurements from GNSS or DR."""

    def __init__(self) -> None:
        self.reference: tuple[float, float] | None = None
        self.state = np.zeros(4, dtype=float)
        self.covariance = np.eye(4, dtype=float) * 10.0
        self._last_timestamp = 0.0

    def reset(self) -> None:
        self.reference = None
        self.state.fill(0.0)
        self.covariance = np.eye(4, dtype=float) * 10.0
        self._last_timestamp = 0.0

    def update(
        self,
        position: tuple[float, float],
        timestamp: float,
        _measured_speed_mps: float,
        _heading_deg: float,
        measurement_type: str,
    ) -> dict[str, Any]:
        if self.reference is None:
            self.reference = position
            self.state[:2] = 0.0
            self._last_timestamp = timestamp
        dt = max(0.0, timestamp - self._last_timestamp)
        transition = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        process_noise = np.eye(4, dtype=float) * max(dt, 0.01) * 0.1
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process_noise

        measurement = gps_to_local(self.reference, position)
        observation = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        noise = 4.0 if measurement_type == "GNSS" else 25.0
        innovation_covariance = observation @ self.covariance @ observation.T + np.eye(2) * noise
        gain = self.covariance @ observation.T @ np.linalg.inv(innovation_covariance)
        self.state += gain @ (measurement - observation @ self.state)
        self.covariance = (np.eye(4) - gain @ observation) @ self.covariance
        self._last_timestamp = timestamp
        latitude, longitude = local_to_gps(self.reference, self.state[:2])
        return {"latitude": latitude, "longitude": longitude, "position_source": "EKF"}
