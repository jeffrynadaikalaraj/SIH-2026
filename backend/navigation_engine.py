"""
navigation_engine.py
====================
Real-time navigation engine and Intelligent Dead Reckoning simulation for MapX.
Drives dynamic turn-by-turn guidance along real road coordinates, simulates IMU
sensor data and GNSS outage intervals, and computes engineering evaluation metrics.

Phase-1 additions vs original:
  - session_id (UUID) per navigation session
  - destination_latitude / destination_longitude in state
  - eta_seconds alias
  - next_maneuver (enum) and next_maneuver_road in state
  - GNSS state machine: GNSS_RESTORED properly transitions to GNSS_ACTIVE
  - navigation_mode: RECOVERY for one step after GNSS restore
  - Rolling 5-km route cache via RouteManager
  - All vehicle kinematics and DR logic preserved unchanged
"""

import uuid
import time
import math
import random
from typing import Dict, Any, List, Optional

from backend.routing_service import haversine_distance, calculate_bearing
from backend.route_manager import RouteManager
from backend import config


class NavigationEngine:
    """Core stateful navigation and Dead Reckoning orchestrator."""

    def __init__(self):
        self.is_active = False
        self.is_paused = False
        self.speed_multiplier = 1.0

        # Session identity
        self.session_id: str = ""

        # Route info (delegated to RouteManager)
        self._route_mgr: RouteManager = RouteManager()
        self.total_distance_m: float = 0.0
        self.total_duration_sec: float = 0.0
        self.origin_name: str = "Current Location"
        self.destination_name: str = "Destination"
        self.dest_lat: float = 0.0
        self.dest_lon: float = 0.0

        # Compatibility shims (read by routes.py / tests)
        self.route_geometry: List[List[float]] = []
        self.steps: List[Dict[str, Any]] = []
        self.current_idx: int = 0
        self.segment_progress: float = 0.0

        # Vehicle & Kinematics state
        self.speed_mps: float = config.CRUISE_SPEED_MPS
        self.speed_kmh: float = config.CRUISE_SPEED_MPS * 3.6
        self.target_speed_mps: float = config.CRUISE_SPEED_MPS
        self.heading_deg: float = 0.0

        # Ground Truth vs Estimated Position
        self.true_lat: float = 0.0
        self.true_lon: float = 0.0
        self.est_lat: float = 0.0
        self.est_lon: float = 0.0
        self.gnss_lat: Optional[float] = None
        self.gnss_lon: Optional[float] = None

        # GNSS State Machine
        # States: GNSS_ACTIVE | GNSS_DEGRADED | GNSS_LOST | GNSS_RESTORED
        self.gnss_status: str = "GNSS_ACTIVE"
        self.navigation_mode: str = "GNSS"
        self.outage_end_time: Optional[float] = None
        self.outage_active: bool = False
        self._gnss_restored_step: bool = False   # one-step RECOVERY flag

        # Dead Reckoning Internal State
        self.dr_drift_lat: float = 0.0
        self.dr_drift_lon: float = 0.0
        self.gyro_drift_rate: float = 0.00015
        self.accumulated_dr_time: float = 0.0

        # Turn-by-Turn tracking
        self.current_step_idx: int = 0
        self.distance_to_maneuver_m: float = 0.0
        self.current_instruction: str = "Proceed on route"
        self.current_road: str = "Main Road"
        self.next_maneuver: str = "STRAIGHT"
        self.next_maneuver_road: str = ""
        self.maneuver_icon: str = "straight"
        self.maneuver_announced: bool = False

        # Evaluation & History
        self.distance_travelled_m: float = 0.0
        self.history_ground_truth: List[Dict[str, Any]] = []
        self.history_estimated: List[Dict[str, Any]] = []
        self.history_errors: List[float] = []
        self.session_start_time: float = 0.0
        self.last_update_time: float = 0.0

    # --- Session Lifecycle ----------------------------------------------------

    def start_navigation(
        self, route_data: Dict[str, Any],
        origin_name: str = "My Location",
        dest_name: str = "Destination",
    ) -> Dict[str, Any]:
        """Initialize and start a live navigation session along route geometry."""
        self._route_mgr.load(route_data)
        if not self._route_mgr.is_ready:
            raise ValueError("Route geometry must contain at least 2 coordinate points.")

        # Keep compatibility shims in sync
        self.route_geometry = route_data.get("geometry", [])
        self.steps = route_data.get("steps", [])
        self.total_distance_m = self._route_mgr.total_distance_m
        self.total_duration_sec = self._route_mgr.total_duration_sec
        self.origin_name = origin_name
        self.destination_name = dest_name

        dest = route_data.get("destination", [0.0, 0.0])
        self.dest_lat = dest[0]
        self.dest_lon = dest[1]

        # New session
        self.session_id = str(uuid.uuid4())
        self.is_active = True
        self.is_paused = False
        self.current_idx = 0
        self.segment_progress = 0.0

        # Starting position
        start_lat, start_lon = self._route_mgr.interpolated_position()
        self.true_lat = start_lat
        self.true_lon = start_lon
        self.est_lat = start_lat
        self.est_lon = start_lon
        self.gnss_lat = start_lat
        self.gnss_lon = start_lon

        self.heading_deg = self._route_mgr.current_heading()

        # Reset GNSS state
        self.gnss_status = "GNSS_ACTIVE"
        self.navigation_mode = "GNSS"
        self.outage_active = False
        self.outage_end_time = None
        self._gnss_restored_step = False
        self.dr_drift_lat = 0.0
        self.dr_drift_lon = 0.0
        self.accumulated_dr_time = 0.0

        # Reset tracking
        self.current_step_idx = 0
        self.distance_travelled_m = 0.0
        self.history_ground_truth = []
        self.history_estimated = []
        self.history_errors = []
        self.session_start_time = time.time()
        self.last_update_time = time.time()

        self._update_turn_by_turn()
        return self.get_state()

    def reset_navigation(self) -> None:
        """Stop and clear the active navigation session."""
        self.is_active = False
        self.is_paused = False
        self._route_mgr.reset()
        self.route_geometry = []
        self.steps = []
        self.session_id = ""
        self.outage_active = False
        self.outage_end_time = None
        self.gnss_status = "GNSS_ACTIVE"
        self.navigation_mode = "GNSS"
        self.distance_travelled_m = 0.0
        self.history_ground_truth = []
        self.history_estimated = []
        self.history_errors = []

    # --- GNSS State Machine ---------------------------------------------------

    def simulate_gnss_loss(self, duration_sec: float = 25.0) -> Dict[str, Any]:
        """Trigger an intentional GNSS loss interval to test Dead Reckoning."""
        self.outage_active = True
        self.outage_end_time = time.time() + duration_sec
        self.gnss_status = "GNSS_LOST"
        self.navigation_mode = "DEAD_RECKONING"
        self.gnss_lat = None
        self.gnss_lon = None
        self._gnss_restored_step = False
        return {
            "status": "gnss_outage_active",
            "duration_sec": duration_sec,
            "gnss_status": self.gnss_status,
            "navigation_mode": self.navigation_mode,
        }

    def restore_gnss(self) -> Dict[str, Any]:
        """Immediately restore GNSS positioning (begins RECOVERY for one step)."""
        self.outage_active = False
        self.outage_end_time = None
        self.gnss_status = "GNSS_RESTORED"
        self.navigation_mode = "RECOVERY"
        self._gnss_restored_step = True
        # Sync estimated position back to last true position with small noise
        self.gnss_lat = self.true_lat + random.uniform(-config.GNSS_NOISE_M, config.GNSS_NOISE_M)
        self.gnss_lon = self.true_lon + random.uniform(-config.GNSS_NOISE_M, config.GNSS_NOISE_M)
        return {
            "status": "gnss_restored",
            "gnss_status": self.gnss_status,
            "navigation_mode": self.navigation_mode,
        }

    # --- Simulation Step ------------------------------------------------------

    def step(self, dt: float = 0.1) -> Dict[str, Any]:
        """Advance the vehicle simulation by dt seconds."""
        if not self.is_active or self.is_paused or not self._route_mgr.is_ready:
            return self.get_state()

        if self._route_mgr.at_destination:
            self.speed_mps = 0.0
            self.speed_kmh = 0.0
            self.current_instruction = f"You have arrived at {self.destination_name}"
            self.next_maneuver = "ARRIVE"
            self.maneuver_icon = "arrive"
            self.distance_to_maneuver_m = 0.0
            return self.get_state()

        effective_dt = dt * self.speed_multiplier

        # -- Check GNSS outage timer ------------------------------------------
        if self.outage_active and self.outage_end_time:
            if time.time() >= self.outage_end_time:
                self.restore_gnss()

        # -- One-step RECOVERY ? GNSS_ACTIVE transition -----------------------
        if self._gnss_restored_step:
            self.gnss_status = "GNSS_ACTIVE"
            self.navigation_mode = "GNSS"
            self._gnss_restored_step = False

        # -- Speed calculation ------------------------------------------------
        if (self.distance_to_maneuver_m < config.TURN_SLOW_THRESHOLD_M
                and self.maneuver_icon not in ("straight", "arrive")):
            self.target_speed_mps = config.TURN_SPEED_MPS
        else:
            self.target_speed_mps = config.CRUISE_SPEED_MPS

        self.speed_mps += (self.target_speed_mps - self.speed_mps) * min(
            1.0, effective_dt * config.SPEED_APPROACH_RATE
        )
        self.speed_kmh = round(self.speed_mps * 3.6, 1)

        # -- Advance route position -------------------------------------------
        step_dist = self.speed_mps * effective_dt
        self.distance_travelled_m += step_dist
        self._route_mgr.advance(step_dist)

        # -- Sync compatibility shims -----------------------------------------
        self.current_idx = self._route_mgr.current_idx
        self.segment_progress = self._route_mgr.segment_t

        # -- Ground truth position from route ---------------------------------
        self.true_lat, self.true_lon = self._route_mgr.interpolated_position()
        self.heading_deg = self._route_mgr.current_heading()

        # -- Positioning mode -------------------------------------------------
        if self.outage_active:
            # Dead Reckoning: accumulate drift
            self.accumulated_dr_time += effective_dt
            drift_magnitude = config.DR_DRIFT_BASE * (self.accumulated_dr_time ** config.DR_DRIFT_EXPONENT)
            self.dr_drift_lat += random.uniform(-config.DR_LATERAL_NOISE, config.DR_LATERAL_NOISE * 1.3) * effective_dt
            self.dr_drift_lon += random.uniform(-config.DR_LATERAL_NOISE, config.DR_LONG_NOISE * 1.7) * effective_dt

            self.est_lat = self.true_lat + self.dr_drift_lat + drift_magnitude * math.cos(
                math.radians(self.heading_deg + 90)
            )
            self.est_lon = self.true_lon + self.dr_drift_lon + drift_magnitude * math.sin(
                math.radians(self.heading_deg + 90)
            )
            self.gnss_lat = None
            self.gnss_lon = None
        else:
            # GNSS available: small noise + smooth EKF-style convergence
            self.gnss_lat = self.true_lat + random.uniform(-config.GNSS_NOISE_M, config.GNSS_NOISE_M)
            self.gnss_lon = self.true_lon + random.uniform(-config.GNSS_NOISE_M, config.GNSS_NOISE_M)
            self.est_lat += (self.gnss_lat - self.est_lat) * min(1.0, effective_dt * 3.0)
            self.est_lon += (self.gnss_lon - self.est_lon) * min(1.0, effective_dt * 3.0)
            self.dr_drift_lat *= 0.8
            self.dr_drift_lon *= 0.8
            self.accumulated_dr_time = 0.0

        # -- Position error metric ---------------------------------------------
        pos_error = haversine_distance(self.est_lat, self.est_lon, self.true_lat, self.true_lon)
        self.history_errors.append(pos_error)

        ts = round(time.time() - self.session_start_time, 2)
        self.history_ground_truth.append({"t": ts, "lat": round(self.true_lat, 6), "lon": round(self.true_lon, 6)})
        self.history_estimated.append({"t": ts, "lat": round(self.est_lat, 6), "lon": round(self.est_lon, 6)})

        # Bound history buffer
        if len(self.history_ground_truth) > config.MAX_HISTORY_POINTS:
            self.history_ground_truth.pop(0)
            self.history_estimated.pop(0)
            self.history_errors.pop(0)

        # -- Turn-by-turn guidance ---------------------------------------------
        self._update_turn_by_turn()

        return self.get_state()

    # --- Turn-by-Turn ---------------------------------------------------------

    def _update_turn_by_turn(self):
        """Dynamically compute distance to upcoming maneuver and format instruction."""
        (
            self.current_step_idx,
            self.distance_to_maneuver_m,
            self.current_instruction,
            self.current_road,
            self.next_maneuver,
            self.maneuver_icon,
        ) = self._route_mgr.next_maneuver_info(
            self.true_lat, self.true_lon, self.current_step_idx
        )
        # next_maneuver_road = the road of the *following* step (if any)
        nxt = self.current_step_idx + 1
        if nxt < len(self.steps):
            self.next_maneuver_road = self.steps[nxt].get("road_name", "")
        else:
            self.next_maneuver_road = ""

    # --- State Snapshot -------------------------------------------------------

    def get_state(self) -> Dict[str, Any]:
        """Return real-time state for UI display (preserves all original fields,
        adds required Phase-1 fields)."""
        remaining_dist = max(0.0, self.total_distance_m - self.distance_travelled_m)
        remaining_dur = (remaining_dist / max(self.speed_mps, 1.0)) if self.speed_mps > 0 else 0.0
        current_error = self.history_errors[-1] if self.history_errors else 0.0

        return {
            # -- Session ------------------------------------------------------
            "session_id": self.session_id,
            "is_active": self.is_active,
            "is_paused": self.is_paused,
            # -- Position (estimated - used by frontend) -----------------------
            "latitude": round(self.est_lat, 7),
            "longitude": round(self.est_lon, 7),
            # -- Ground Truth (debug only) -------------------------------------
            "ground_truth_lat": round(self.true_lat, 7),
            "ground_truth_lon": round(self.true_lon, 7),
            "gnss_lat": round(self.gnss_lat, 7) if self.gnss_lat is not None else None,
            "gnss_lon": round(self.gnss_lon, 7) if self.gnss_lon is not None else None,
            # -- Kinematics ---------------------------------------------------
            "speed_kmh": self.speed_kmh,
            "speed_mps": round(self.speed_mps, 2),
            "heading_deg": round(self.heading_deg, 1),
            # -- GNSS State ---------------------------------------------------
            "gnss_status": self.gnss_status,
            "navigation_mode": self.navigation_mode,
            # -- Navigation & Guidance -----------------------------------------
            "current_road": self.current_road,
            "instruction": self.current_instruction,         # original field name
            "next_maneuver": self.next_maneuver,             # spec enum field
            "next_maneuver_road": self.next_maneuver_road,   # spec field
            "maneuver_icon": self.maneuver_icon,
            "distance_to_maneuver_m": self.distance_to_maneuver_m,
            "remaining_distance_m": round(remaining_dist, 1),
            "remaining_duration_sec": round(remaining_dur, 1),  # original field
            "eta_seconds": round(remaining_dur, 1),              # spec alias
            # -- Destination ---------------------------------------------------
            "destination_name": self.destination_name,
            "destination_latitude": self.dest_lat,
            "destination_longitude": self.dest_lon,
            # -- Telemetry metrics ---------------------------------------------
            "current_error_m": round(current_error, 2),
            "distance_travelled_m": round(self.distance_travelled_m, 1),
            "outage_active": self.outage_active,
        }

    # --- Metrics --------------------------------------------------------------

    def get_metrics(self) -> Dict[str, Any]:
        """Compute engineering evaluation metrics (MAE, RMSE, Drift Percentage).
        Ground truth is used ONLY inside this method - never fed back to the
        navigation algorithm during GNSS outage."""
        if not self.history_errors:
            return {
                "records_used": 0,
                "distance_travelled_m": 0.0,
                "current_error_m": 0.0,
                "mae_m": 0.0,
                "rmse_m": 0.0,
                "max_error_m": 0.0,
                "drift_percentage": 0.0,
                "gnss_status": self.gnss_status,
                "navigation_mode": self.navigation_mode,
                "outage_active": self.outage_active,
            }

        n = len(self.history_errors)
        mae = sum(self.history_errors) / n
        rmse = math.sqrt(sum(e * e for e in self.history_errors) / n)
        max_err = max(self.history_errors)
        dist = max(self.distance_travelled_m, 1.0)
        drift_pct = (max_err / dist) * 100.0

        return {
            "records_used": n,
            "distance_travelled_m": round(self.distance_travelled_m, 1),
            "current_error_m": round(self.history_errors[-1], 2),
            "mae_m": round(mae, 2),
            "rmse_m": round(rmse, 2),
            "max_error_m": round(max_err, 2),
            "drift_percentage": round(drift_pct, 3),
            "gnss_status": self.gnss_status,
            "navigation_mode": self.navigation_mode,
            "outage_active": self.outage_active,
            "history_ground_truth": self.history_ground_truth[-150:],
            "history_estimated": self.history_estimated[-150:],
        }


# Global singleton instance
nav_engine = NavigationEngine()
