"""
Configuration and sensor-data validation for the Dead Reckoning engine.

Responsibilities
----------------
1. Validate the structure and value ranges of ``config.yaml``.
2. Check that the input DataFrame contains all required columns.
3. Flag individual sensor values that are out of physical bounds.
"""

from __future__ import annotations

import logging
import math
from typing import Any

import numpy as np
import pandas as pd

from .exceptions import ConfigurationError, ValidationError
from .models import REQUIRED_COLUMNS

logger = logging.getLogger(__name__)


# ── Configuration validation ────────────────────────────────────────────

_REQUIRED_CONFIG_SECTIONS = [
    "earth",
    "timing",
    "heading",
    "sensors",
    "velocity",
    "calibration",
    "vehicle",
]


def validate_config(config: dict[str, Any]) -> dict[str, Any]:
    """Validate and normalise a configuration dictionary.

    Parameters
    ----------
    config : dict
        Parsed YAML configuration.

    Returns
    -------
    dict
        The same *config* object (potentially with defaults filled in).

    Raises
    ------
    ConfigurationError
        If required sections or keys are missing or have invalid values.
    """
    for section in _REQUIRED_CONFIG_SECTIONS:
        if section not in config:
            raise ConfigurationError(
                f"Missing required configuration section: '{section}'"
            )

    # Earth
    r = config["earth"].get("radius_m", 6_371_000.0)
    if not (1_000_000 < r < 10_000_000):
        raise ConfigurationError(f"earth.radius_m out of range: {r}")

    # Timing
    t = config["timing"]
    if t.get("minimum_dt_seconds", 0.001) <= 0:
        raise ConfigurationError("timing.minimum_dt_seconds must be > 0")
    if t.get("maximum_dt_seconds", 2.0) <= 0:
        raise ConfigurationError("timing.maximum_dt_seconds must be > 0")

    # Sensors
    s = config["sensors"]
    valid_gyro_units = {"degrees_per_second", "radians_per_second"}
    unit = s.get("gyroscope_unit", "degrees_per_second")
    if unit not in valid_gyro_units:
        raise ConfigurationError(
            f"sensors.gyroscope_unit must be one of {valid_gyro_units}, got '{unit}'"
        )

    # Velocity
    v = config["velocity"]
    valid_modes = {"sensor_speed", "acceleration", "hybrid"}
    mode = v.get("mode", "hybrid")
    if mode not in valid_modes:
        raise ConfigurationError(
            f"velocity.mode must be one of {valid_modes}, got '{mode}'"
        )

    # Vehicle
    veh = config["vehicle"]
    if veh.get("maximum_speed_mps", 60.0) <= 0:
        raise ConfigurationError("vehicle.maximum_speed_mps must be > 0")

    # Fill output defaults
    config.setdefault("output", {})
    config["output"].setdefault("include_quality_flags", True)
    config["output"].setdefault("floating_point_precision", 8)

    return config


# ── DataFrame validation ────────────────────────────────────────────────

def validate_dataframe(df: pd.DataFrame) -> list[str]:
    """Validate that a sensor DataFrame has the required columns.

    Parameters
    ----------
    df : pd.DataFrame
        Raw sensor data.

    Returns
    -------
    list[str]
        List of warnings (non-fatal).

    Raises
    ------
    ValidationError
        If the DataFrame is empty or is missing required columns.
    """
    warnings_list: list[str] = []

    if df is None or df.empty:
        raise ValidationError("Input DataFrame is empty or None.")

    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise ValidationError(
            f"Missing required columns: {missing}"
        )

    # Check for entirely null required columns
    for col in REQUIRED_COLUMNS:
        if df[col].isna().all():
            warnings_list.append(f"Column '{col}' is entirely NaN/null.")

    return warnings_list


# ── Per-record value validation ─────────────────────────────────────────

def is_valid_latitude(lat: Any) -> bool:
    """Return ``True`` if *lat* is a finite number in [-90, 90]."""
    try:
        v = float(lat)
        return math.isfinite(v) and -90.0 <= v <= 90.0
    except (TypeError, ValueError):
        return False


def is_valid_longitude(lon: Any) -> bool:
    """Return ``True`` if *lon* is a finite number in [-180, 180]."""
    try:
        v = float(lon)
        return math.isfinite(v) and -180.0 <= v <= 180.0
    except (TypeError, ValueError):
        return False


def safe_float(value: Any, default: float = 0.0) -> float:
    """Convert *value* to ``float``, returning *default* on failure.

    Also replaces NaN and Inf with *default*.
    """
    try:
        v = float(value)
        if math.isfinite(v):
            return v
        return default
    except (TypeError, ValueError):
        return default
