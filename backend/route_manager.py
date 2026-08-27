"""
route_manager.py
================
RouteManager encapsulates all route-geometry concerns for IDR-X:
  - Full route storage and progress tracking
  - Rolling 5-km look-ahead window
  - Closest-point projection of vehicle position onto route
  - Next maneuver lookup

This is the clean seam for a future HMM map-matching replacement:
swap this module without touching NavigationEngine or the frontend contract.
"""

import math
from typing import Any, Dict, List, Optional, Tuple

from backend.utils.geo import haversine_distance, project_point_onto_segment
from backend.utils.bearing import calculate_bearing
from backend import config


class RouteManager:
    """Manages active route geometry and vehicle progress along it."""

    def __init__(self) -> None:
        self._geometry: List[List[float]] = []   # full [lat, lon] list
        self._steps: List[Dict[str, Any]] = []
        self._total_distance_m: float = 0.0
        self._total_duration_sec: float = 0.0

        # Vehicle progress on route
        self._current_idx: int = 0            # current segment start index
        self._segment_t: float = 0.0          # 0-1 fractional position on segment

        # Rolling cache window
        self._cache_start_idx: int = 0        # first kept index in active window

    # --- Public Properties ---------------------------------------------------

    @property
    def is_ready(self) -> bool:
        return len(self._geometry) >= 2

    @property
    def total_distance_m(self) -> float:
        return self._total_distance_m

    @property
    def total_duration_sec(self) -> float:
        return self._total_duration_sec

    @property
    def steps(self) -> List[Dict[str, Any]]:
        return self._steps

    @property
    def current_idx(self) -> int:
        return self._current_idx

    @property
    def segment_t(self) -> float:
        return self._segment_t

    @property
    def at_destination(self) -> bool:
        return self._current_idx >= len(self._geometry) - 1

    # --- Lifecycle -----------------------------------------------------------

    def load(self, route_data: Dict[str, Any]) -> None:
        """Load a route returned by RoutingService."""
        self._geometry = route_data.get("geometry", [])
        self._steps = route_data.get("steps", [])
        self._total_distance_m = route_data.get("distance_m", 0.0)
        self._total_duration_sec = route_data.get("duration_sec", 0.0)
        self._current_idx = 0
        self._segment_t = 0.0
        self._cache_start_idx = 0

    def reset(self) -> None:
        self.__init__()

    # --- Vehicle Progress ----------------------------------------------------

    def advance(self, distance_m: float) -> None:
        """
        Move vehicle forward by distance_m along the route geometry.
        Updates _current_idx and _segment_t.
        """
        if not self.is_ready or self.at_destination:
            return

        remaining = distance_m
        while remaining > 0 and self._current_idx < len(self._geometry) - 1:
            p_a = self._geometry[self._current_idx]
            p_b = self._geometry[self._current_idx + 1]
            seg_dist = haversine_distance(p_a[0], p_a[1], p_b[0], p_b[1])
            if seg_dist <= 0:
                self._current_idx += 1
                self._segment_t = 0.0
                continue

            available = seg_dist * (1.0 - self._segment_t)
            if remaining <= available:
                self._segment_t += remaining / seg_dist
                remaining = 0.0
            else:
                remaining -= available
                self._current_idx += 1
                self._segment_t = 0.0

        if self._current_idx >= len(self._geometry) - 1:
            self._current_idx = len(self._geometry) - 1
            self._segment_t = 0.0

        self._advance_cache_window()

    def _advance_cache_window(self) -> None:
        """
        Keep the rolling cache window starting just behind the current position.
        We allow going back a few indices for snap-back after DR correction.
        """
        if self._current_idx > self._cache_start_idx + 5:
            self._cache_start_idx = max(0, self._current_idx - 2)

    # --- Position Interpolation -----------------------------------------------

    def interpolated_position(self) -> Tuple[float, float]:
        """Return the interpolated (lat, lon) of the vehicle on the route."""
        if not self.is_ready:
            return (0.0, 0.0)
        if self.at_destination:
            p = self._geometry[-1]
            return (p[0], p[1])
        p_a = self._geometry[self._current_idx]
        p_b = self._geometry[self._current_idx + 1]
        lat = p_a[0] + (p_b[0] - p_a[0]) * self._segment_t
        lon = p_a[1] + (p_b[1] - p_a[1]) * self._segment_t
        return (lat, lon)

    def current_heading(self) -> float:
        """Return bearing (degrees) in the direction of travel."""
        if not self.is_ready or self.at_destination:
            return 0.0
        p_a = self._geometry[self._current_idx]
        p_b = self._geometry[self._current_idx + 1]
        return calculate_bearing(p_a[0], p_a[1], p_b[0], p_b[1])

    # --- Rolling 5-km Cache Window --------------------------------------------

    def active_window(self) -> List[List[float]]:
        """
        Return the slice of route geometry within the rolling cache window.
        At navigation start this is the first 5 km; it slides forward as the
        vehicle progresses.  Only this window is needed for DR projection and
        local re-routing.  The full geometry is still stored for metrics.
        """
        start = self._cache_start_idx
        accumulated = 0.0
        end = start
        while end < len(self._geometry) - 1:
            p_a = self._geometry[end]
            p_b = self._geometry[end + 1]
            accumulated += haversine_distance(p_a[0], p_a[1], p_b[0], p_b[1])
            end += 1
            if accumulated >= config.ROLLING_CACHE_KM * 1000:
                break
        return self._geometry[start:end + 1]

    # --- Closest Route Point (for DR snap-back) -------------------------------

    def project_onto_route(
        self, lat: float, lon: float
    ) -> Tuple[float, float, int, float]:
        """
        Find the closest point on the active route window to (lat, lon).
        Returns (proj_lat, proj_lon, best_seg_idx, best_t).
        Searches only within the active window for efficiency.
        """
        window = self.active_window()
        if len(window) < 2:
            return lat, lon, self._current_idx, self._segment_t

        best_dist = math.inf
        best_lat, best_lon = lat, lon
        best_idx = self._cache_start_idx
        best_t = 0.0

        for i in range(len(window) - 1):
            a = window[i]
            b = window[i + 1]
            pl, plo, t = project_point_onto_segment(lat, lon, a[0], a[1], b[0], b[1])
            d = haversine_distance(lat, lon, pl, plo)
            if d < best_dist:
                best_dist = d
                best_lat, best_lon = pl, plo
                best_idx = self._cache_start_idx + i
                best_t = t

        return best_lat, best_lon, best_idx, best_t

    # --- Maneuver Tracking ---------------------------------------------------

    def next_maneuver_info(
        self, current_lat: float, current_lon: float, step_idx: int
    ) -> Tuple[int, float, str, str, str, str]:
        """
        Given the vehicle position and current step index, return:
        (updated_step_idx, dist_to_maneuver_m, instruction,
         road_name, maneuver_type, maneuver_icon)
        """
        if not self._steps:
            remaining = max(
                0.0,
                self._total_distance_m - self._distance_travelled_approx(current_lat, current_lon),
            )
            return step_idx, remaining, "Proceed to destination", "", "STRAIGHT", "straight"

        if step_idx >= len(self._steps):
            return step_idx, 0.0, "Arrived at destination", "", "ARRIVE", "arrive"

        step = self._steps[step_idx]
        loc = step.get("location", [current_lat, current_lon])
        dist = haversine_distance(current_lat, current_lon, loc[0], loc[1])

        # Auto-advance if vehicle passed this maneuver point
        if dist < config.MANEUVER_ARRIVE_THRESHOLD_M and step_idx < len(self._steps) - 1:
            step_idx += 1
            step = self._steps[step_idx]
            loc = step.get("location", [current_lat, current_lon])
            dist = haversine_distance(current_lat, current_lon, loc[0], loc[1])

        road = step.get("road_name", "Main Road")
        icon = step.get("icon", "straight")
        m_type = step.get("type", "turn")
        modifier = step.get("modifier", "")

        # Format instruction
        if dist <= config.MANEUVER_NOW_THRESHOLD_M:
            if m_type == "arrive":
                instruction = "Arriving at destination now"
            else:
                mod_str = modifier.replace("_", " ").title()
                instruction = f"Turn {mod_str} Now onto {road}" if mod_str else f"Turn Now onto {road}"
        elif dist < 1000:
            base = step.get("instruction", "Continue straight")
            instruction = f"In {int(dist)} m, {base.lower()}"
        else:
            base = step.get("instruction", "Continue straight")
            instruction = f"In {round(dist / 1000, 1)} km, {base.lower()}"

        # Map icon ? spec maneuver enum
        maneuver_enum = _icon_to_enum(icon)

        return step_idx, round(dist, 1), instruction, road, maneuver_enum, icon

    def _distance_travelled_approx(self, lat: float, lon: float) -> float:
        """Rough remaining distance from current position to destination."""
        if not self.is_ready:
            return 0.0
        dest = self._geometry[-1]
        return haversine_distance(lat, lon, dest[0], dest[1])


# --- Helper -------------------------------------------------------------------

def _icon_to_enum(icon: str) -> str:
    """Map routing icon string to spec maneuver enum."""
    mapping = {
        "turn-right": "TURN_RIGHT",
        "turn-left": "TURN_LEFT",
        "slight-right": "KEEP_RIGHT",
        "slight-left": "KEEP_LEFT",
        "sharp-right": "TURN_RIGHT",
        "sharp-left": "TURN_LEFT",
        "arrive": "ARRIVE",
        "straight": "STRAIGHT",
        "u-turn": "U_TURN",
        "roundabout": "ROUNDABOUT",
        "merge": "MERGE",
        "exit": "EXIT",
    }
    return mapping.get(icon, "STRAIGHT")
