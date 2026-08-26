"""
Core Dead Reckoning engine.

Implements the complete GNSS state machine:

    UNINITIALIZED → GNSS → DEAD_RECKONING → GNSS_REACQUIRED → GNSS → …

Public API
----------
``DeadReckoningEngine``
    • ``reset()``
    • ``initialize(lat, lon, speed, heading, timestamp)``
    • ``update_heading(gyro_z, dt) -> heading_deg``
    • ``update_velocity(speed, accel, dt, gnss_avail) -> velocity``
    • ``propagate_position(velocity, heading, dt)``
    • ``reset_with_gnss(lat, lon, speed, timestamp)``
    • ``process_sensor_record(record) -> dict``
    • ``process_dataframe(df) -> pd.DataFrame``

Coordinate convention
---------------------
Local frame: X = East, Y = North (metres).
Navigation heading: 0° = North, 90° = East.
"""

from __future__ import annotations

import logging
import math
from typing import Any

import numpy as np
import pandas as pd

from .coordinates import (
    displacement_east_north,
    gps_to_local,
    haversine_distance,
    initial_bearing,
    local_to_gps,
    normalize_heading_deg,
)
from .exceptions import EngineError
from .models import NavigationMode, OUTPUT_COLUMNS
from .preprocessing import preprocess_dataframe
from .state import VehicleState
from .validation import (
    is_valid_latitude,
    is_valid_longitude,
    safe_float,
    validate_config,
    validate_dataframe,
)

logger = logging.getLogger(__name__)


class DeadReckoningEngine:
    """Physics-based Dead Reckoning engine for GNSS-denied navigation.

    Parameters
    ----------
    config : dict
        Parsed and validated configuration (see ``config.yaml``).
    """

    # ── construction / reset ────────────────────────────────────────

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = validate_config(config)
        self.state = VehicleState()
        self._start_timestamp: float = 0.0
        self._previous_lat: float | None = None
        self._previous_lon: float | None = None
        self._filtered_accel: float = 0.0
        self._outage_count: int = 0
        self._outage_durations: list[float] = []
        self._calibration_samples_gyro: list[float] = []
        self._calibration_samples_accel: list[float] = []
        self._calibration_done: bool = False
        logger.info("DeadReckoningEngine created.")

    def reset(self) -> None:
        """Reset the engine to its initial un-initialised state."""
        self.state = VehicleState()
        self._start_timestamp = 0.0
        self._previous_lat = None
        self._previous_lon = None
        self._filtered_accel = 0.0
        self._outage_count = 0
        self._outage_durations = []
        self._calibration_samples_gyro = []
        self._calibration_samples_accel = []
        self._calibration_done = False
        logger.info("Engine reset.")

    # ── initialisation ──────────────────────────────────────────────

    def initialize(
        self,
        latitude: float,
        longitude: float,
        speed_mps: float,
        heading_deg: float,
        timestamp: float,
    ) -> None:
        """Initialise the engine with the first valid GNSS fix.

        Parameters
        ----------
        latitude, longitude : float
            First reliable GNSS coordinate (degrees).
        speed_mps : float
            Initial vehicle speed (m/s).
        heading_deg : float
            Initial navigation heading (degrees, 0 = North).
        timestamp : float
            Record timestamp (seconds).
        """
        s = self.state
        s.initialized = True
        s.timestamp = timestamp
        self._start_timestamp = timestamp
        s.reference_latitude = latitude
        s.reference_longitude = longitude
        s.estimated_latitude = latitude
        s.estimated_longitude = longitude
        s.local_x_m = 0.0
        s.local_y_m = 0.0
        s.previous_velocity_mps = speed_mps
        s.current_velocity_mps = speed_mps
        s.heading_deg = normalize_heading_deg(heading_deg)
        s.sync_heading_rad()
        s.previous_gnss_available = True
        s.navigation_mode = NavigationMode.GNSS
        s.last_reliable_gnss_latitude = latitude
        s.last_reliable_gnss_longitude = longitude

        # Apply configured biases
        cal = self.config.get("calibration", {})
        s.gyro_bias_z = cal.get("gyro_bias_z", 0.0)
        s.accel_bias_forward = cal.get("forward_acceleration_bias", 0.0)

        logger.info(
            "Engine initialised at (%.6f, %.6f), heading=%.1f°, speed=%.2f m/s.",
            latitude, longitude, s.heading_deg, speed_mps,
        )

    # ── heading update ──────────────────────────────────────────────

    def update_heading(self, gyro_z: float, dt: float) -> float:
        """Integrate gyroscope yaw rate to update the heading.

        Parameters
        ----------
        gyro_z : float
            Raw yaw-rate reading.
        dt : float
            Time step (seconds).

        Returns
        -------
        float
            Updated heading in degrees [0, 360).
        """
        s = self.state
        cfg_sensors = self.config.get("sensors", {})
        cfg_heading = self.config.get("heading", {})
        cfg_vehicle = self.config.get("vehicle", {})

        # Bias correction
        corrected = gyro_z - s.gyro_bias_z

        # Unit conversion
        gyro_unit = cfg_sensors.get("gyroscope_unit", "degrees_per_second")
        if gyro_unit == "radians_per_second":
            corrected_dps = math.degrees(corrected)
        else:
            corrected_dps = corrected

        # Clip extreme yaw rates
        max_yaw = cfg_vehicle.get("maximum_yaw_rate_deg_s", 120.0)
        if abs(corrected_dps) > max_yaw:
            logger.warning(
                "Yaw rate %.2f°/s clipped to ±%.1f°/s.",
                corrected_dps, max_yaw,
            )
            corrected_dps = max(-max_yaw, min(max_yaw, corrected_dps))

        # Apply yaw sign and integrate
        yaw_sign = cfg_heading.get("gyro_yaw_sign", 1.0)
        heading_change = corrected_dps * dt * yaw_sign
        s.heading_deg = normalize_heading_deg(s.heading_deg + heading_change)
        s.sync_heading_rad()

        return s.heading_deg

    # ── velocity update ─────────────────────────────────────────────

    def update_velocity(
        self,
        sensor_speed_mps: float | None,
        forward_acceleration_mps2: float | None,
        dt: float,
        gnss_available: bool,
    ) -> float:
        """Compute the current velocity based on the configured mode.

        Parameters
        ----------
        sensor_speed_mps : float or None
            Speed reading from the sensor CSV (m/s).
        forward_acceleration_mps2 : float or None
            Corrected forward acceleration (m/s²).
        dt : float
            Time step (seconds).
        gnss_available : bool
            Whether GNSS is currently available.

        Returns
        -------
        float
            Updated velocity (m/s), clamped to vehicle limits.
        """
        s = self.state
        vel_cfg = self.config.get("velocity", {})
        veh_cfg = self.config.get("vehicle", {})
        mode = vel_cfg.get("mode", "hybrid")
        speed_is_gnss = vel_cfg.get("speed_is_gnss_derived", False)
        max_speed = veh_cfg.get("maximum_speed_mps", 60.0)

        # Determine whether sensor speed is usable
        speed_usable = (
            sensor_speed_mps is not None
            and math.isfinite(sensor_speed_mps)
            and not (speed_is_gnss and not gnss_available)
        )

        accel = forward_acceleration_mps2 if (
            forward_acceleration_mps2 is not None
            and math.isfinite(forward_acceleration_mps2)
        ) else 0.0

        if mode == "sensor_speed":
            if speed_usable:
                velocity = sensor_speed_mps
            else:
                # Fallback to accelerometer integration
                velocity = s.current_velocity_mps + accel * dt
        elif mode == "acceleration":
            velocity = s.current_velocity_mps + accel * dt
        else:  # hybrid
            alpha = vel_cfg.get("hybrid_speed_weight", 0.7)
            v_accel = s.current_velocity_mps + accel * dt
            if speed_usable:
                velocity = alpha * sensor_speed_mps + (1.0 - alpha) * v_accel
            else:
                velocity = v_accel

        # Clamp
        velocity = max(veh_cfg.get("minimum_speed_mps", 0.0), velocity)
        velocity = min(max_speed, velocity)

        s.previous_velocity_mps = s.current_velocity_mps
        s.current_velocity_mps = velocity
        return velocity

    # ── position propagation ────────────────────────────────────────

    def propagate_position(
        self,
        velocity_mps: float,
        heading_deg: float,
        dt: float,
    ) -> None:
        """Propagate local position using trapezoidal distance integration.

        Parameters
        ----------
        velocity_mps : float
            Current velocity (m/s).
        heading_deg : float
            Current heading (degrees).
        dt : float
            Time step (seconds).
        """
        s = self.state
        # Trapezoidal distance
        distance = ((s.previous_velocity_mps + velocity_mps) / 2.0) * dt
        distance = max(0.0, distance)

        delta_e, delta_n = displacement_east_north(distance, heading_deg)
        s.local_x_m += delta_e
        s.local_y_m += delta_n
        s.total_distance_m += distance

        # Convert to lat/lon
        earth_r = self.config["earth"].get("radius_m", 6_371_000.0)
        lat, lon = local_to_gps(
            s.reference_latitude,
            s.reference_longitude,
            s.local_x_m,
            s.local_y_m,
            earth_r,
        )
        s.estimated_latitude = lat
        s.estimated_longitude = lon

        return distance  # used for output

    # ── GNSS reacquisition ──────────────────────────────────────────

    def reset_with_gnss(
        self,
        latitude: float,
        longitude: float,
        speed_mps: float | None,
        timestamp: float,
    ) -> None:
        """Reset the estimated position to a new GNSS fix.

        Calculates the reconnection error before resetting.

        Parameters
        ----------
        latitude, longitude : float
            New GNSS coordinate.
        speed_mps : float or None
            Speed reading (m/s), may be ``None``.
        timestamp : float
            Record timestamp (seconds).
        """
        s = self.state
        earth_r = self.config["earth"].get("radius_m", 6_371_000.0)

        # Reconnection error
        error = haversine_distance(
            s.estimated_latitude, s.estimated_longitude,
            latitude, longitude,
            earth_r,
        )
        s.last_reconnection_error_m = error

        # Store outage duration
        if s.outage_elapsed_seconds > 0:
            self._outage_durations.append(s.outage_elapsed_seconds)

        # Reset position to GNSS
        s.estimated_latitude = latitude
        s.estimated_longitude = longitude
        s.last_reliable_gnss_latitude = latitude
        s.last_reliable_gnss_longitude = longitude

        x, y = gps_to_local(
            s.reference_latitude, s.reference_longitude,
            latitude, longitude, earth_r,
        )
        s.local_x_m = x
        s.local_y_m = y
        s.outage_elapsed_seconds = 0.0
        s.timestamp = timestamp

        if speed_mps is not None and math.isfinite(speed_mps):
            s.previous_velocity_mps = speed_mps
            s.current_velocity_mps = speed_mps

        s.navigation_mode = NavigationMode.GNSS_REACQUIRED
        logger.info(
            "GNSS reacquired at t=%.2f s. Reconnection error: %.2f m.",
            timestamp, error,
        )

    # ── auto-calibration ────────────────────────────────────────────

    def _collect_calibration(self, record: dict) -> None:
        """Collect stationary IMU samples for bias estimation."""
        cal = self.config.get("calibration", {})
        if not cal.get("enable_auto_bias_estimation", False):
            return
        if self._calibration_done:
            return
        threshold = cal.get("stationary_speed_threshold_mps", 0.15)
        speed = safe_float(record.get("speed_mps"), 0.0)
        if speed < threshold:
            gyro_z = safe_float(record.get("gyro_z"), 0.0)
            self._calibration_samples_gyro.append(gyro_z)
            fwd = self._extract_forward_accel(record)
            self._calibration_samples_accel.append(fwd)
        elif len(self._calibration_samples_gyro) >= cal.get("minimum_stationary_samples", 10):
            self._apply_calibration()

    def _apply_calibration(self) -> None:
        """Apply auto-bias estimation if enough samples were collected."""
        cal = self.config.get("calibration", {})
        if not cal.get("enable_auto_bias_estimation", False):
            self._calibration_done = True
            return
        min_samples = cal.get("minimum_stationary_samples", 10)
        if len(self._calibration_samples_gyro) >= min_samples:
            self.state.gyro_bias_z = float(
                np.mean(self._calibration_samples_gyro)
            )
            self.state.accel_bias_forward = float(
                np.mean(self._calibration_samples_accel)
            )
            logger.info(
                "Auto-calibration applied: gyro_bias_z=%.4f, accel_bias=%.4f "
                "(%d samples).",
                self.state.gyro_bias_z,
                self.state.accel_bias_forward,
                len(self._calibration_samples_gyro),
            )
        else:
            logger.info(
                "Auto-calibration: only %d stationary samples; using configured biases.",
                len(self._calibration_samples_gyro),
            )
        self._calibration_done = True

    # ── acceleration extraction ─────────────────────────────────────

    def _extract_forward_accel(self, record: dict) -> float:
        """Extract and correct forward acceleration from a record."""
        cfg = self.config.get("sensors", {})

        # Prefer explicit forward_accel_mps2 column
        if "forward_accel_mps2" in record:
            val = safe_float(record["forward_accel_mps2"], None)
            if val is not None:
                return val

        axis = cfg.get("forward_acceleration_axis", "accel_x")
        sign = cfg.get("forward_acceleration_sign", 1.0)
        raw = safe_float(record.get(axis), 0.0) * sign
        return raw

    def _correct_acceleration(self, raw_accel: float) -> float:
        """Apply bias subtraction, clipping, and low-pass filter."""
        s = self.state
        cfg = self.config.get("sensors", {})
        veh = self.config.get("vehicle", {})

        corrected = raw_accel - s.accel_bias_forward

        # Clip
        a_min = veh.get("minimum_acceleration_mps2", -8.0)
        a_max = veh.get("maximum_acceleration_mps2", 6.0)
        if corrected < a_min or corrected > a_max:
            logger.debug(
                "Acceleration %.2f m/s² clipped to [%.1f, %.1f].",
                corrected, a_min, a_max,
            )
            corrected = max(a_min, min(a_max, corrected))

        # Exponential low-pass filter
        alpha = cfg.get("acceleration_filter_alpha", 0.25)
        self._filtered_accel = (
            alpha * corrected + (1.0 - alpha) * self._filtered_accel
        )
        return self._filtered_accel

    # ── heading estimation from GNSS ────────────────────────────────

    def _estimate_heading_from_gnss(
        self, lat: float, lon: float, record: dict,
    ) -> float | None:
        """Attempt to estimate heading from GNSS or a heading column.

        Returns
        -------
        float or None
            Heading in degrees or ``None`` if estimation is not possible.
        """
        cfg = self.config.get("heading", {})
        min_disp = cfg.get("minimum_gps_displacement_m", 1.5)
        earth_r = self.config["earth"].get("radius_m", 6_371_000.0)

        # Priority 1: explicit heading column
        if "heading_deg" in record:
            h = safe_float(record.get("heading_deg"), None)
            if h is not None and math.isfinite(h):
                return normalize_heading_deg(h)

        # Priority 2: bearing from previous GNSS
        if self._previous_lat is not None and self._previous_lon is not None:
            dist = haversine_distance(
                self._previous_lat, self._previous_lon,
                lat, lon, earth_r,
            )
            if dist >= min_disp:
                bearing = initial_bearing(
                    self._previous_lat, self._previous_lon, lat, lon,
                )
                return bearing

        # Cannot estimate → None (caller uses previous heading or default)
        return None

    # ── process a single sensor record ──────────────────────────────

    def process_sensor_record(self, record: dict) -> dict:
        """Process one sensor record and return the estimated state.

        This is the primary streaming API intended for FastAPI integration.

        Parameters
        ----------
        record : dict
            A single row of sensor data as a dictionary.

        Returns
        -------
        dict
            JSON-serializable estimated state dictionary.
        """
        s = self.state
        quality_flags: list[str] = []
        earth_r = self.config["earth"].get("radius_m", 6_371_000.0)

        # ── Parse key fields ────────────────────────────────────────
        timestamp = safe_float(record.get("timestamp"), s.timestamp)
        gnss_avail = bool(record.get("gnss_available", False))
        gps_lat = safe_float(record.get("gps_latitude"), None)
        gps_lon = safe_float(record.get("gps_longitude"), None)
        speed = safe_float(record.get("speed_mps"), None)
        gyro_z = safe_float(record.get("gyro_z"), 0.0)

        gps_valid = (
            gps_lat is not None
            and gps_lon is not None
            and is_valid_latitude(gps_lat)
            and is_valid_longitude(gps_lon)
        )

        # If marked GNSS ON but GPS coordinates are invalid ⇒ treat as OFF
        effective_gnss = gnss_avail and gps_valid
        if gnss_avail and not gps_valid:
            quality_flags.append("GNSS_ON_but_invalid_GPS")
            effective_gnss = False

        # Handle missing sensor values
        if record.get("gyro_z") is None or (
            isinstance(record.get("gyro_z"), float)
            and math.isnan(record["gyro_z"])
        ):
            gyro_z = 0.0
            quality_flags.append("missing_gyro_z")

        raw_accel = self._extract_forward_accel(record)
        corrected_accel = self._correct_acceleration(raw_accel)

        # ── dt ──────────────────────────────────────────────────────
        dt = safe_float(record.get("dt"), 0.0)
        if dt <= 0 and s.initialized:
            dt = timestamp - s.timestamp
            if dt < 0:
                dt = 0.0

        # Flag large time gaps as unreliable instead of silently clamping
        max_dt = self.config.get("timing", {}).get("maximum_dt_seconds", 2.0)
        if dt > max_dt:
            quality_flags.append(
                f"large_dt_{dt:.3f}s_exceeds_{max_dt:.1f}s"
            )
            logger.warning(
                "Large dt=%.4f s at t=%.2f s; record flagged as unreliable.",
                dt, timestamp,
            )

        # ── Variables to fill for output ────────────────────────────
        corrected_gyro_z_out = 0.0
        distance_step = 0.0
        reconnection_error = 0.0

        # ════════════════════════════════════════════════════════════
        #  STATE MACHINE
        # ════════════════════════════════════════════════════════════

        if not s.initialized:
            # ── UNINITIALIZED ───────────────────────────────────────
            self._collect_calibration(record)

            if effective_gnss:
                default_heading = self.config.get("heading", {}).get(
                    "default_heading_deg", 0.0
                )
                heading = default_heading

                # Try to get heading from record
                if "heading_deg" in record:
                    h = safe_float(record.get("heading_deg"), None)
                    if h is not None:
                        heading = h

                self.initialize(
                    gps_lat, gps_lon,
                    speed if speed is not None else 0.0,
                    heading,
                    timestamp,
                )
                self._previous_lat = gps_lat
                self._previous_lon = gps_lon
            else:
                s.timestamp = timestamp
                # Remain UNINITIALIZED
                pass

        elif effective_gnss:
            # ── GNSS AVAILABLE ──────────────────────────────────────

            # Continue collecting calibration samples during GNSS-ON
            # stationary period before the first outage
            if not self._calibration_done:
                self._collect_calibration(record)

            was_dr = (
                s.navigation_mode == NavigationMode.DEAD_RECKONING
            )

            if was_dr:
                # Transition: DR → GNSS (reacquisition)
                self._outage_count += 1
                self.reset_with_gnss(gps_lat, gps_lon, speed, timestamp)
                reconnection_error = s.last_reconnection_error_m
            else:
                # Continuing GNSS or after GNSS_REACQUIRED
                s.navigation_mode = NavigationMode.GNSS
                s.estimated_latitude = gps_lat
                s.estimated_longitude = gps_lon
                s.last_reliable_gnss_latitude = gps_lat
                s.last_reliable_gnss_longitude = gps_lon
                x, y = gps_to_local(
                    s.reference_latitude, s.reference_longitude,
                    gps_lat, gps_lon, earth_r,
                )
                s.local_x_m = x
                s.local_y_m = y

            # Heading estimation from GNSS
            est_heading = self._estimate_heading_from_gnss(
                gps_lat, gps_lon, record,
            )
            if est_heading is not None:
                s.heading_deg = est_heading
                s.sync_heading_rad()

            # Velocity update
            self.update_velocity(speed, corrected_accel, dt, True)

            # Distance bookkeeping (trapezoidal with GNSS speed)
            if dt > 0:
                distance_step = (
                    (s.previous_velocity_mps + s.current_velocity_mps) / 2.0
                ) * dt
                distance_step = max(0.0, distance_step)
                s.total_distance_m += distance_step

            s.timestamp = timestamp
            s.outage_elapsed_seconds = 0.0
            s.previous_gnss_available = True
            self._previous_lat = gps_lat
            self._previous_lon = gps_lon

        else:
            # ── GNSS NOT AVAILABLE (Dead Reckoning) ─────────────────

            if s.previous_gnss_available:
                # Transition ON → OFF
                if not self._calibration_done:
                    self._apply_calibration()
                logger.info(
                    "GNSS outage started at t=%.2f s.", timestamp,
                )

            s.navigation_mode = NavigationMode.DEAD_RECKONING

            # 1. Heading
            corrected_gz = gyro_z - s.gyro_bias_z
            gyro_unit = self.config.get("sensors", {}).get(
                "gyroscope_unit", "degrees_per_second"
            )
            if gyro_unit == "radians_per_second":
                corrected_gyro_z_out = math.degrees(corrected_gz)
            else:
                corrected_gyro_z_out = corrected_gz
            # Determine usable speed for velocity update (anti-leakage rule)
            vel_cfg = self.config.get("velocity", {})
            speed_for_vel = speed
            if vel_cfg.get("speed_is_gnss_derived", False):
                speed_for_vel = None  # Block GPS speed leakage

            # If dt exceeds maximum_dt_seconds, subdivide into smaller sub-steps
            # to maintain numerical integration stability while preserving total elapsed time
            max_dt = self.config.get("timing", {}).get("maximum_dt_seconds", 2.0)
            n_steps = max(1, math.ceil(dt / max_dt)) if (max_dt > 0 and dt > max_dt) else 1
            dt_sub = dt / n_steps

            distance_step = 0.0
            for _ in range(n_steps):
                self.update_heading(gyro_z, dt_sub)
                self.update_velocity(speed_for_vel, corrected_accel, dt_sub, False)
                step_dist = self.propagate_position(
                    s.current_velocity_mps, s.heading_deg, dt_sub,
                )
                if step_dist:
                    distance_step += step_dist

            # 4. Outage timer
            s.outage_elapsed_seconds += dt
            s.timestamp = timestamp
            s.previous_gnss_available = False

        # ── Build output record ─────────────────────────────────────
        elapsed = timestamp - self._start_timestamp if s.initialized else 0.0
        result = {
            "timestamp": float(timestamp),
            "elapsed_time_s": float(elapsed),
            "gnss_available": bool(effective_gnss),
            "navigation_mode": str(s.navigation_mode.value),
            "estimated_latitude": float(s.estimated_latitude),
            "estimated_longitude": float(s.estimated_longitude),
            "estimated_x_m": float(s.local_x_m),
            "estimated_y_m": float(s.local_y_m),
            "estimated_velocity_mps": float(s.current_velocity_mps),
            "estimated_speed_kmph": float(s.current_velocity_mps * 3.6),
            "estimated_heading_deg": float(s.heading_deg),
            "corrected_gyro_z": float(corrected_gyro_z_out),
            "corrected_forward_accel_mps2": float(corrected_accel),
            "distance_step_m": float(distance_step) if distance_step else 0.0,
            "cumulative_distance_m": float(s.total_distance_m),
            "outage_elapsed_seconds": float(s.outage_elapsed_seconds),
            "gnss_reconnection_error_m": float(reconnection_error),
            "data_quality_flags": ";".join(quality_flags) if quality_flags else "",
        }
        return result

    # ── batch processing ────────────────────────────────────────────

    def process_dataframe(self, dataframe: pd.DataFrame) -> pd.DataFrame:
        """Process an entire sensor DataFrame and return the trajectory.

        The engine is reset before processing so that results are
        deterministic.

        Parameters
        ----------
        dataframe : pd.DataFrame
            Raw or preprocessed sensor data.

        Returns
        -------
        pd.DataFrame
            Output trajectory with the same number of rows as the input.
        """
        self.reset()

        # Validate
        warnings = validate_dataframe(dataframe)
        for w in warnings:
            logger.warning(w)

        # Preprocess
        df = preprocess_dataframe(dataframe, self.config)

        results: list[dict] = []
        for idx, row in df.iterrows():
            record = row.to_dict()
            result = self.process_sensor_record(record)
            results.append(result)

        out = pd.DataFrame(results, columns=OUTPUT_COLUMNS)
        return out

    # ── summary statistics ──────────────────────────────────────────

    def get_summary(self) -> dict[str, Any]:
        """Return a summary of the processing run.

        Returns
        -------
        dict
            Human-readable summary values.
        """
        s = self.state
        longest_outage = (
            max(self._outage_durations) if self._outage_durations else 0.0
        )
        return {
            "total_records": 0,  # placeholder; filled by caller
            "gnss_records": 0,
            "dead_reckoning_records": 0,
            "gnss_outages_detected": self._outage_count,
            "longest_outage_seconds": longest_outage,
            "estimated_distance_m": s.total_distance_m,
            "maximum_speed_kmph": 0.0,
            "last_reconnection_error_m": s.last_reconnection_error_m,
        }
