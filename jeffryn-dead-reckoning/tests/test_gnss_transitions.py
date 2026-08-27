"""
GNSS state-machine transition tests.

Verifies the full GNSS ON → GNSS OFF → GNSS ON cycle:
  • Initialisation during GNSS ON
  • Mode switch to DEAD_RECKONING
  • Position continues updating during outage
  • Reconnection error is calculated
  • Position resets on GNSS return
  • Mode becomes GNSS_REACQUIRED then GNSS
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pandas as pd
import pytest

from src.dead_reckoning import DeadReckoningEngine
from src.dead_reckoning.models import NavigationMode


def _config():
    return {
        "earth": {"radius_m": 6_371_000.0},
        "timing": {"minimum_dt_seconds": 0.001, "maximum_dt_seconds": 2.0},
        "heading": {"default_heading_deg": 0.0, "minimum_gps_displacement_m": 1.5,
                     "gyro_yaw_sign": 1.0},
        "sensors": {"gyroscope_unit": "degrees_per_second",
                     "forward_acceleration_axis": "accel_x",
                     "forward_acceleration_sign": 1.0,
                     "acceleration_filter_alpha": 1.0},
        "velocity": {"mode": "sensor_speed", "speed_is_gnss_derived": False,
                      "hybrid_speed_weight": 0.7, "maximum_speed_mps": 60.0},
        "calibration": {"enable_auto_bias_estimation": False,
                         "gyro_bias_z": 0.0, "forward_acceleration_bias": 0.0,
                         "minimum_stationary_samples": 10,
                         "stationary_speed_threshold_mps": 0.15},
        "vehicle": {"minimum_speed_mps": 0.0, "maximum_speed_mps": 60.0,
                     "minimum_acceleration_mps2": -8.0,
                     "maximum_acceleration_mps2": 6.0,
                     "maximum_yaw_rate_deg_s": 120.0},
        "output": {"include_quality_flags": True, "floating_point_precision": 8},
    }


def _build_transition_df():
    """Build a small DataFrame with GNSS ON → OFF → ON."""
    rows = []
    lat, lon = 13.0, 80.0

    # 5 records GNSS ON, vehicle moving North at 10 m/s
    for i in range(5):
        rows.append({
            "timestamp": float(i),
            "gps_latitude": lat + i * 0.0001,
            "gps_longitude": lon,
            "speed_mps": 10.0,
            "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
            "gnss_available": True,
        })

    # 5 records GNSS OFF
    for i in range(5, 10):
        rows.append({
            "timestamp": float(i),
            "gps_latitude": lat + i * 0.0001,  # ground truth (should NOT be used)
            "gps_longitude": lon,
            "speed_mps": 10.0,
            "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
            "gnss_available": False,
        })

    # 5 records GNSS ON again
    for i in range(10, 15):
        rows.append({
            "timestamp": float(i),
            "gps_latitude": lat + i * 0.0001,
            "gps_longitude": lon,
            "speed_mps": 10.0,
            "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
            "gnss_available": True,
        })

    return pd.DataFrame(rows)


class TestGnssTransitions:
    def test_full_cycle(self):
        engine = DeadReckoningEngine(_config())
        df = _build_transition_df()
        result = engine.process_dataframe(df)

        assert len(result) == len(df)

        # First records should be GNSS
        assert result.iloc[0]["navigation_mode"] == "GNSS"

        # Middle records should be DEAD_RECKONING
        dr_rows = result[result["navigation_mode"] == "DEAD_RECKONING"]
        assert len(dr_rows) >= 4  # at least some DR rows

        # GNSS_REACQUIRED should appear exactly once
        reacq = result[result["navigation_mode"] == "GNSS_REACQUIRED"]
        assert len(reacq) == 1

        # Records after reacquisition should be GNSS
        reacq_idx = reacq.index[0]
        for idx in range(reacq_idx + 1, len(result)):
            assert result.iloc[idx]["navigation_mode"] == "GNSS"

    def test_reconnection_error_calculated(self):
        engine = DeadReckoningEngine(_config())
        df = _build_transition_df()
        result = engine.process_dataframe(df)

        reacq = result[result["navigation_mode"] == "GNSS_REACQUIRED"]
        assert len(reacq) == 1
        error = reacq.iloc[0]["gnss_reconnection_error_m"]
        # Error should be a finite positive number (DR != GNSS position)
        assert error >= 0.0

    def test_position_updates_during_outage(self):
        engine = DeadReckoningEngine(_config())
        df = _build_transition_df()
        result = engine.process_dataframe(df)

        dr_rows = result[result["navigation_mode"] == "DEAD_RECKONING"]
        if len(dr_rows) >= 2:
            # Position should change since vehicle is moving
            first_y = dr_rows.iloc[0]["estimated_y_m"]
            last_y = dr_rows.iloc[-1]["estimated_y_m"]
            assert abs(last_y - first_y) > 1.0

    def test_position_resets_on_reacquisition(self):
        engine = DeadReckoningEngine(_config())
        df = _build_transition_df()
        result = engine.process_dataframe(df)

        reacq = result[result["navigation_mode"] == "GNSS_REACQUIRED"]
        assert len(reacq) == 1
        # After reacquisition, position should match the GNSS position
        reacq_row = reacq.iloc[0]
        expected_lat = df.loc[df["timestamp"] == reacq_row["timestamp"], "gps_latitude"]
        if len(expected_lat) > 0:
            assert abs(reacq_row["estimated_latitude"] - expected_lat.iloc[0]) < 0.001

    def test_outage_elapsed_seconds(self):
        engine = DeadReckoningEngine(_config())
        df = _build_transition_df()
        result = engine.process_dataframe(df)

        dr_rows = result[result["navigation_mode"] == "DEAD_RECKONING"]
        if len(dr_rows) > 0:
            last_outage = dr_rows.iloc[-1]["outage_elapsed_seconds"]
            assert last_outage > 0.0
