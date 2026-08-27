"""
Data models and enumerations for the Dead Reckoning engine.

Defines:
- NavigationMode   — GNSS state-machine modes.
- REQUIRED_COLUMNS — Mandatory input sensor columns.
- GNSS_TRUE_VALUES / GNSS_FALSE_VALUES — Accepted GNSS status literals.
"""

from enum import Enum
from typing import Set


class NavigationMode(str, Enum):
    """Navigation mode used by the Dead Reckoning state machine.

    Values
    ------
    UNINITIALIZED   : Engine has not yet received a valid GNSS fix.
    GNSS            : Navigating with live GNSS position.
    DEAD_RECKONING  : Estimating position via IMU integration (GNSS lost).
    GNSS_REACQUIRED : First record after GNSS returns; reconnection error
                      has been computed and the position reset.
    """
    UNINITIALIZED = "UNINITIALIZED"
    GNSS = "GNSS"
    DEAD_RECKONING = "DEAD_RECKONING"
    GNSS_REACQUIRED = "GNSS_REACQUIRED"


# ── Required input columns ──────────────────────────────────────────────
REQUIRED_COLUMNS: list[str] = [
    "timestamp",
    "gps_latitude",
    "gps_longitude",
    "speed_mps",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "gnss_available",
]

# ── Optional input columns (not required but recognised) ────────────────
OPTIONAL_COLUMNS: list[str] = [
    "sequence_id",
    "ground_truth_latitude",
    "ground_truth_longitude",
    "forward_accel_mps2",
    "heading_deg",
    "speed_source",
]

# ── GNSS status normalisation look-up tables ────────────────────────────
GNSS_TRUE_VALUES: Set[str] = {
    "on", "true", "1", "1.0", "available", "active", "yes",
}

GNSS_FALSE_VALUES: Set[str] = {
    "off", "false", "0", "0.0", "lost", "inactive", "no",
}

# ── Output trajectory columns ───────────────────────────────────────────
OUTPUT_COLUMNS: list[str] = [
    "timestamp",
    "elapsed_time_s",
    "gnss_available",
    "navigation_mode",
    "estimated_latitude",
    "estimated_longitude",
    "estimated_x_m",
    "estimated_y_m",
    "estimated_velocity_mps",
    "estimated_speed_kmph",
    "estimated_heading_deg",
    "corrected_gyro_z",
    "corrected_forward_accel_mps2",
    "distance_step_m",
    "cumulative_distance_m",
    "outage_elapsed_seconds",
    "gnss_reconnection_error_m",
    "data_quality_flags",
]
