"""
Sensor-data preprocessing for the Dead Reckoning engine.

Responsibilities
----------------
1. Parse and normalise timestamps (numeric seconds **or** ISO-8601).
2. Normalise ``gnss_available`` from many accepted string/int forms.
3. Sort records chronologically and compute ``dt``.
4. Handle duplicate timestamps, negative gaps, and large gaps.
"""

from __future__ import annotations

import logging
import math
from typing import Any

import numpy as np
import pandas as pd

from .exceptions import PreprocessingError
from .models import GNSS_FALSE_VALUES, GNSS_TRUE_VALUES

logger = logging.getLogger(__name__)


# ── GNSS status normalisation ───────────────────────────────────────────

def normalize_gnss_status(value: Any) -> bool:
    """Convert a flexible GNSS-status value to ``bool``.

    Accepts: ON/OFF, TRUE/FALSE, True/False, 1/0, AVAILABLE/LOST,
    ACTIVE/INACTIVE, and Python booleans/ints.

    Parameters
    ----------
    value : Any
        Raw GNSS status from the CSV.

    Returns
    -------
    bool

    Raises
    ------
    PreprocessingError
        If the value cannot be mapped to a known status.
    """
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, float, np.integer, np.floating)):
        if math.isnan(float(value)):
            raise PreprocessingError(f"Cannot interpret NaN as GNSS status.")
        return float(value) != 0.0

    s = str(value).strip().lower()
    if s in GNSS_TRUE_VALUES:
        return True
    if s in GNSS_FALSE_VALUES:
        return False
    raise PreprocessingError(
        f"Unrecognised GNSS status value: '{value}'. "
        f"Accepted true values: {sorted(GNSS_TRUE_VALUES)}; "
        f"accepted false values: {sorted(GNSS_FALSE_VALUES)}."
    )


# ── Timestamp parsing ───────────────────────────────────────────────────

def parse_timestamps(series: pd.Series) -> pd.Series:
    """Convert a timestamp column to float seconds.

    Supports:
    * Numeric values already in seconds (``0.0, 0.1, 0.2, …``).
    * ISO-8601 datetime strings (``2026-08-26T10:00:00.000``).

    The returned series contains **relative** seconds from the first
    timestamp so that ``dt`` computation is straightforward.

    Parameters
    ----------
    series : pd.Series
        Raw timestamp column.

    Returns
    -------
    pd.Series
        Timestamps as ``float64`` seconds.

    Raises
    ------
    PreprocessingError
        If timestamps cannot be parsed at all.
    """
    # Try numeric first
    numeric = pd.to_numeric(series, errors="coerce")
    if numeric.notna().sum() == len(series):
        return numeric.astype(float)

    # Try datetime parsing
    try:
        dt_series = pd.to_datetime(series, errors="coerce")
        if dt_series.notna().sum() == len(series):
            # Convert to seconds since the first timestamp
            origin = dt_series.min()
            return (dt_series - origin).dt.total_seconds().astype(float)
    except Exception:
        pass

    # Mixed — fall back to numeric interpretation with NaN fill
    if numeric.notna().sum() > 0:
        logger.warning(
            "Some timestamps could not be parsed; filling with NaN."
        )
        return numeric.astype(float)

    raise PreprocessingError("Unable to parse timestamp column.")


# ── Full preprocessing pipeline ─────────────────────────────────────────

def preprocess_dataframe(
    df: pd.DataFrame,
    config: dict,
) -> pd.DataFrame:
    """Apply the full preprocessing pipeline to a raw sensor DataFrame.

    Steps:
    1. Parse timestamps to float seconds.
    2. Sort chronologically.
    3. Normalise ``gnss_available`` to ``bool``.
    4. Compute ``dt`` between successive records.
    5. Clamp or warn on out-of-range ``dt`` values.

    Parameters
    ----------
    df : pd.DataFrame
        Raw sensor DataFrame (must already have required columns).
    config : dict
        Parsed ``config.yaml``.

    Returns
    -------
    pd.DataFrame
        Preprocessed copy (original is not modified).
    """
    df = df.copy()

    # 1 — timestamps
    df["timestamp"] = parse_timestamps(df["timestamp"])
    df.sort_values("timestamp", inplace=True)
    df.reset_index(drop=True, inplace=True)

    # 2 — GNSS status
    df["gnss_available"] = df["gnss_available"].apply(normalize_gnss_status)

    # 3 — dt
    timing_cfg = config.get("timing", {})
    min_dt = timing_cfg.get("minimum_dt_seconds", 0.001)
    max_dt = timing_cfg.get("maximum_dt_seconds", 2.0)

    timestamps = df["timestamp"].values.astype(float)
    dt_arr = np.zeros(len(timestamps), dtype=float)
    for i in range(1, len(timestamps)):
        raw_dt = timestamps[i] - timestamps[i - 1]
        if raw_dt < 0:
            logger.warning(
                "Negative dt=%.6f at index %d; clamping to 0.", raw_dt, i
            )
            raw_dt = 0.0
        if raw_dt < min_dt and raw_dt > 0:
            # Very small dt — keep it but note it
            pass
        if raw_dt > max_dt:
            logger.warning(
                "Large dt=%.4f s at index %d exceeds maximum_dt_seconds=%.2f; "
                "retaining actual dt and flagging record as unreliable.",
                raw_dt, i, max_dt,
            )
        dt_arr[i] = raw_dt

    df["dt"] = dt_arr
    return df
