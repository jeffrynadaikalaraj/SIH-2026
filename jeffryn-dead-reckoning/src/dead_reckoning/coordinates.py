"""
Geodetic and local-coordinate conversion utilities.

Coordinate convention
---------------------
* Local frame: X = East (metres), Y = North (metres).
* Navigation heading: 0° = North, 90° = East, 180° = South, 270° = West.
* Positive heading change = clockwise rotation.

Earth model
-----------
* Spherical Earth with configurable radius (default R = 6 371 000 m).

Functions
---------
haversine_distance    — Great-circle distance between two GPS points.
initial_bearing       — Forward azimuth from point A to point B.
gps_to_local          — Convert (lat, lon) to local (x_east, y_north).
local_to_gps          — Convert local (x_east, y_north) to (lat, lon).
normalize_heading_deg — Wrap any angle into [0, 360).
"""

from __future__ import annotations

import math

# Default Earth radius in metres
DEFAULT_EARTH_RADIUS_M: float = 6_371_000.0


# ── Heading normalisation ───────────────────────────────────────────────

def normalize_heading_deg(heading: float) -> float:
    """Return *heading* wrapped into the half-open interval [0, 360).

    Parameters
    ----------
    heading : float
        Heading in degrees (may be negative or ≥ 360).

    Returns
    -------
    float
        Normalised heading in [0, 360).
    """
    heading = heading % 360.0
    if heading < 0.0:
        heading += 360.0
    return heading


# ── Haversine distance ──────────────────────────────────────────────────

def haversine_distance(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
    earth_radius_m: float = DEFAULT_EARTH_RADIUS_M,
) -> float:
    """Great-circle distance in metres between two GPS coordinates.

    Parameters
    ----------
    lat1, lon1 : float
        First point in decimal degrees.
    lat2, lon2 : float
        Second point in decimal degrees.
    earth_radius_m : float, optional
        Radius of the Earth in metres.

    Returns
    -------
    float
        Distance in metres.
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2
    )
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return earth_radius_m * c


# ── Initial bearing ─────────────────────────────────────────────────────

def initial_bearing(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
) -> float:
    """Forward azimuth (navigation heading) from point 1 to point 2.

    Returns a heading in [0, 360) using the navigation convention
    (0° = North, 90° = East).

    Parameters
    ----------
    lat1, lon1, lat2, lon2 : float
        Coordinates in decimal degrees.

    Returns
    -------
    float
        Bearing in degrees [0, 360).
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_lambda = math.radians(lon2 - lon1)

    x = math.sin(d_lambda) * math.cos(phi2)
    y = (
        math.cos(phi1) * math.sin(phi2)
        - math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda)
    )
    bearing_rad = math.atan2(x, y)
    return normalize_heading_deg(math.degrees(bearing_rad))


# ── GPS ↔ Local conversions ─────────────────────────────────────────────

def gps_to_local(
    ref_lat: float,
    ref_lon: float,
    lat: float,
    lon: float,
    earth_radius_m: float = DEFAULT_EARTH_RADIUS_M,
) -> tuple[float, float]:
    """Convert a GPS coordinate to local East/North displacement in metres.

    Parameters
    ----------
    ref_lat, ref_lon : float
        Reference (origin) coordinate in decimal degrees.
    lat, lon : float
        Target coordinate in decimal degrees.
    earth_radius_m : float, optional
        Radius of the Earth in metres.

    Returns
    -------
    (x_east_m, y_north_m) : tuple[float, float]
        Local displacement in metres.
    """
    y_north = math.radians(lat - ref_lat) * earth_radius_m
    cos_ref = math.cos(math.radians(ref_lat))
    # Protect against division by zero near the poles
    if abs(cos_ref) < 1e-10:
        cos_ref = 1e-10
    x_east = math.radians(lon - ref_lon) * earth_radius_m * cos_ref
    return x_east, y_north


def local_to_gps(
    ref_lat: float,
    ref_lon: float,
    x_east_m: float,
    y_north_m: float,
    earth_radius_m: float = DEFAULT_EARTH_RADIUS_M,
) -> tuple[float, float]:
    """Convert local East/North displacement (metres) to GPS coordinates.

    Parameters
    ----------
    ref_lat, ref_lon : float
        Reference (origin) coordinate in decimal degrees.
    x_east_m : float
        East displacement in metres.
    y_north_m : float
        North displacement in metres.
    earth_radius_m : float, optional
        Radius of the Earth in metres.

    Returns
    -------
    (lat, lon) : tuple[float, float]
        GPS coordinate in decimal degrees.
    """
    lat = ref_lat + math.degrees(y_north_m / earth_radius_m)
    cos_ref = math.cos(math.radians(ref_lat))
    if abs(cos_ref) < 1e-10:
        cos_ref = 1e-10
    lon = ref_lon + math.degrees(x_east_m / (earth_radius_m * cos_ref))
    return lat, lon


# ── Displacement helpers ────────────────────────────────────────────────

def displacement_east_north(
    distance_m: float,
    heading_deg: float,
) -> tuple[float, float]:
    """East and North displacement for a given distance and nav heading.

    Uses the navigation convention:
        delta_east  = distance × sin(heading)
        delta_north = distance × cos(heading)

    Parameters
    ----------
    distance_m : float
        Scalar distance in metres (≥ 0).
    heading_deg : float
        Navigation heading in degrees.

    Returns
    -------
    (delta_east_m, delta_north_m) : tuple[float, float]
    """
    heading_rad = math.radians(heading_deg)
    delta_east = distance_m * math.sin(heading_rad)
    delta_north = distance_m * math.cos(heading_rad)
    return delta_east, delta_north
