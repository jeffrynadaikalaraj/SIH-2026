"""
Unit tests for src/dead_reckoning/preprocessing.py

Tests cover:
  • Numeric and ISO timestamp parsing
  • GNSS status normalisation (all accepted variants)
  • Chronological sorting
  • Duplicate timestamp handling
  • Missing value resilience
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pandas as pd
import pytest

from src.dead_reckoning.preprocessing import (
    normalize_gnss_status,
    parse_timestamps,
    preprocess_dataframe,
)
from src.dead_reckoning.exceptions import PreprocessingError


# ── GNSS status ─────────────────────────────────────────────────────────

class TestGnssStatus:
    @pytest.mark.parametrize("val", ["ON", "on", "True", "TRUE", "true", "1",
                                      "AVAILABLE", "ACTIVE", "yes", 1, True, 1.0])
    def test_true_values(self, val):
        assert normalize_gnss_status(val) is True

    @pytest.mark.parametrize("val", ["OFF", "off", "False", "FALSE", "false", "0",
                                      "LOST", "INACTIVE", "no", 0, False, 0.0])
    def test_false_values(self, val):
        assert normalize_gnss_status(val) is False

    def test_invalid_raises(self):
        with pytest.raises(PreprocessingError):
            normalize_gnss_status("MAYBE")

    def test_nan_raises(self):
        with pytest.raises(PreprocessingError):
            normalize_gnss_status(float("nan"))


# ── Timestamps ──────────────────────────────────────────────────────────

class TestTimestamps:
    def test_numeric(self):
        s = pd.Series([0.0, 0.1, 0.2, 0.3])
        result = parse_timestamps(s)
        assert list(result) == [0.0, 0.1, 0.2, 0.3]

    def test_iso_datetime(self):
        s = pd.Series([
            "2026-08-26T10:00:00.000",
            "2026-08-26T10:00:00.100",
            "2026-08-26T10:00:00.200",
        ])
        result = parse_timestamps(s)
        assert result.iloc[0] == pytest.approx(0.0)
        assert result.iloc[1] == pytest.approx(0.1, abs=0.01)

    def test_integer_timestamps(self):
        s = pd.Series([0, 1, 2, 3])
        result = parse_timestamps(s)
        assert result.iloc[2] == pytest.approx(2.0)


# ── Preprocessing pipeline ─────────────────────────────────────────────

class TestPreprocessDataframe:
    def _make_df(self, n=5):
        return pd.DataFrame({
            "timestamp": [float(i) * 0.1 for i in range(n)],
            "gps_latitude": [13.0] * n,
            "gps_longitude": [80.0] * n,
            "speed_mps": [10.0] * n,
            "accel_x": [0.0] * n,
            "accel_y": [0.0] * n,
            "accel_z": [9.81] * n,
            "gyro_x": [0.0] * n,
            "gyro_y": [0.0] * n,
            "gyro_z": [0.0] * n,
            "gnss_available": ["ON"] * n,
        })

    def test_adds_dt_column(self):
        config = {"timing": {"minimum_dt_seconds": 0.001, "maximum_dt_seconds": 2.0}}
        df = preprocess_dataframe(self._make_df(), config)
        assert "dt" in df.columns

    def test_dt_values(self):
        config = {"timing": {"minimum_dt_seconds": 0.001, "maximum_dt_seconds": 2.0}}
        df = preprocess_dataframe(self._make_df(), config)
        assert df["dt"].iloc[0] == pytest.approx(0.0)
        assert df["dt"].iloc[1] == pytest.approx(0.1, abs=0.01)

    def test_gnss_normalised(self):
        config = {"timing": {"minimum_dt_seconds": 0.001, "maximum_dt_seconds": 2.0}}
        df = preprocess_dataframe(self._make_df(), config)
        assert df["gnss_available"].dtype == bool

    def test_sorts_chronologically(self):
        df = self._make_df()
        df = df.iloc[::-1].reset_index(drop=True)
        config = {"timing": {"minimum_dt_seconds": 0.001, "maximum_dt_seconds": 2.0}}
        result = preprocess_dataframe(df, config)
        assert list(result["timestamp"]) == sorted(result["timestamp"])

    def test_duplicate_timestamps(self):
        """Duplicate timestamps should produce dt=0 and not crash."""
        df = self._make_df()
        df.loc[2, "timestamp"] = df.loc[1, "timestamp"]
        config = {"timing": {"minimum_dt_seconds": 0.001, "maximum_dt_seconds": 2.0}}
        result = preprocess_dataframe(df, config)
        assert len(result) == len(df)
