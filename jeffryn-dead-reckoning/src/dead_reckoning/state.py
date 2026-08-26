"""
Vehicle state dataclass for the Dead Reckoning engine.

Maintains the complete kinematic state of the vehicle including position,
velocity, heading, GNSS outage tracking, and calibration biases.

All positions are stored in both local East/North metres and geographic
latitude/longitude.  The ``to_dict`` method converts every field to a
native Python type so that the result is directly JSON-serializable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .models import NavigationMode


@dataclass
class VehicleState:
    """Full mutable state carried between Dead Reckoning updates.

    Attributes
    ----------
    initialized : bool
        ``True`` once the engine has received its first valid GNSS fix.
    timestamp : float
        Epoch-relative time of the most recent update (seconds).
    reference_latitude : float
        Latitude of the sequence origin (degrees).
    reference_longitude : float
        Longitude of the sequence origin (degrees).
    estimated_latitude : float
        Current best-estimate latitude (degrees).
    estimated_longitude : float
        Current best-estimate longitude (degrees).
    local_x_m : float
        East displacement from the reference point (metres).
    local_y_m : float
        North displacement from the reference point (metres).
    previous_velocity_mps : float
        Velocity at the previous timestep (m/s).
    current_velocity_mps : float
        Velocity at the current timestep (m/s).
    heading_deg : float
        Navigation heading in [0, 360) degrees.
    heading_rad : float
        Navigation heading in radians (kept in sync with *heading_deg*).
    previous_gnss_available : bool
        GNSS availability flag of the previous record.
    navigation_mode : NavigationMode
        Current state-machine mode.
    total_distance_m : float
        Cumulative distance travelled (metres).
    outage_elapsed_seconds : float
        Duration of the current GNSS outage (seconds).
    gyro_bias_z : float
        Static yaw-rate bias (deg/s or rad/s matching the configured unit).
    accel_bias_forward : float
        Static forward-acceleration bias (m/s²).
    last_reliable_gnss_latitude : float
        Latitude at the last valid GNSS reading before outage.
    last_reliable_gnss_longitude : float
        Longitude at the last valid GNSS reading before outage.
    last_reconnection_error_m : float
        Haversine distance between the DR estimate and the GNSS position
        at the most recent GNSS reacquisition (metres).
    """

    initialized: bool = False
    timestamp: float = 0.0
    reference_latitude: float = 0.0
    reference_longitude: float = 0.0
    estimated_latitude: float = 0.0
    estimated_longitude: float = 0.0
    local_x_m: float = 0.0
    local_y_m: float = 0.0
    previous_velocity_mps: float = 0.0
    current_velocity_mps: float = 0.0
    heading_deg: float = 0.0
    heading_rad: float = 0.0
    previous_gnss_available: bool = False
    navigation_mode: NavigationMode = NavigationMode.UNINITIALIZED
    total_distance_m: float = 0.0
    outage_elapsed_seconds: float = 0.0
    gyro_bias_z: float = 0.0
    accel_bias_forward: float = 0.0
    last_reliable_gnss_latitude: float = 0.0
    last_reliable_gnss_longitude: float = 0.0
    last_reconnection_error_m: float = 0.0

    # ── helpers ──────────────────────────────────────────────────────

    def sync_heading_rad(self) -> None:
        """Keep ``heading_rad`` consistent with ``heading_deg``."""
        self.heading_rad = math.radians(self.heading_deg)

    def to_dict(self) -> dict:
        """Return a JSON-serializable dictionary of the current state.

        NumPy/pandas scalar types are not used; every value is a native
        Python ``float``, ``bool``, ``str``, or ``int``.
        """
        return {
            "initialized": bool(self.initialized),
            "timestamp": float(self.timestamp),
            "reference_latitude": float(self.reference_latitude),
            "reference_longitude": float(self.reference_longitude),
            "estimated_latitude": float(self.estimated_latitude),
            "estimated_longitude": float(self.estimated_longitude),
            "local_x_m": float(self.local_x_m),
            "local_y_m": float(self.local_y_m),
            "previous_velocity_mps": float(self.previous_velocity_mps),
            "current_velocity_mps": float(self.current_velocity_mps),
            "heading_deg": float(self.heading_deg),
            "heading_rad": float(self.heading_rad),
            "previous_gnss_available": bool(self.previous_gnss_available),
            "navigation_mode": str(self.navigation_mode.value),
            "total_distance_m": float(self.total_distance_m),
            "outage_elapsed_seconds": float(self.outage_elapsed_seconds),
            "gyro_bias_z": float(self.gyro_bias_z),
            "accel_bias_forward": float(self.accel_bias_forward),
            "last_reliable_gnss_latitude": float(self.last_reliable_gnss_latitude),
            "last_reliable_gnss_longitude": float(self.last_reliable_gnss_longitude),
            "last_reconnection_error_m": float(self.last_reconnection_error_m),
        }
