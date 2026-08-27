"""
utils/geo.py
============
Geographic helper functions shared across the IDR-X backend.
Consolidates the duplicated haversine implementations from
dead_reckoning.py and routing_service.py into a single source of truth.
"""

import math
from typing import Tuple

EARTH_RADIUS_M: float = 6_371_000.0


def haversine_distance(
    lat1: float, lon1: float, lat2: float, lon2: float
) -> float:
    """Return the great-circle distance between two coordinates in metres."""
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2) ** 2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def project_point_onto_segment(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> Tuple[float, float, float]:
    """
    Project point P onto segment AB (all in degrees lat/lon, treated as 2-D plane
    for small distances).  Returns (proj_lat, proj_lon, t) where t in [0, 1] is
    the fractional position along AB.
    """
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-18:
        return ax, ay, 0.0
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    return ax + t * dx, ay + t * dy, t
