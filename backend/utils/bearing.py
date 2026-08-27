"""
utils/bearing.py
================
Bearing / heading helpers shared across the IDR-X backend.
"""

import math


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return the forward azimuth (degrees, 0-360) from point 1 to point 2."""
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    dlon_r = math.radians(lon2 - lon1)
    y = math.sin(dlon_r) * math.cos(lat2_r)
    x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon_r)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def normalize_bearing(bearing: float) -> float:
    """Normalize any angle to [0, 360)."""
    return bearing % 360
