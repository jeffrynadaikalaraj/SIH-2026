"""
No-GPS-leakage verification test.

CRITICAL TEST — ensures the Dead Reckoning engine does NOT use any GPS
data during a GNSS outage.

Method
------
1. Create a sensor sequence with GNSS ON → OFF → ON.
2. Run the engine → Collect DR-only outputs (Run A).
3. Create an identical copy but **corrupt** all GPS coordinates during
   the GNSS OFF period (set them to Antarctica).
4. Run the engine again → Collect DR-only outputs (Run B).
5. Assert that every DR output field is **bitwise identical** between
   Run A and Run B.

If GPS data leaked into the DR calculation, the corrupted coordinates
would produce different results.
"""

import copy
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pandas as pd
import pytest

from src.dead_reckoning import DeadReckoningEngine


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


def _build_df():
    """Create a sensor DataFrame: 5 GNSS ON, 10 GNSS OFF, 5 GNSS ON."""
    rows = []
    lat, lon = 13.0, 80.0

    for i in range(5):
        rows.append({
            "timestamp": float(i),
            "gps_latitude": lat + i * 0.0001,
            "gps_longitude": lon,
            "speed_mps": 10.0,
            "accel_x": 0.5, "accel_y": 0.0, "accel_z": 9.81,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 2.0,
            "gnss_available": True,
        })

    for i in range(5, 15):
        rows.append({
            "timestamp": float(i),
            "gps_latitude": lat + i * 0.0001,   # ground truth GPS
            "gps_longitude": lon + i * 0.00005,
            "speed_mps": 10.0,
            "accel_x": 0.5, "accel_y": 0.0, "accel_z": 9.81,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 2.0,
            "gnss_available": False,
        })

    for i in range(15, 20):
        rows.append({
            "timestamp": float(i),
            "gps_latitude": lat + i * 0.0001,
            "gps_longitude": lon + i * 0.00005,
            "speed_mps": 10.0,
            "accel_x": 0.5, "accel_y": 0.0, "accel_z": 9.81,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 2.0,
            "gnss_available": True,
        })

    return pd.DataFrame(rows)


class TestNoGpsLeakage:
    def test_corrupted_gps_during_outage_has_no_effect(self):
        """
        Run A: Normal GPS during GNSS OFF (ground truth present but unused).
        Run B: GPS coordinates set to Antarctica during GNSS OFF.

        DR outputs during GNSS OFF must be identical.
        """
        df_a = _build_df()
        df_b = df_a.copy()

        # Corrupt GPS during GNSS OFF in Run B
        off_mask = df_b["gnss_available"] == False
        df_b.loc[off_mask, "gps_latitude"] = -80.0    # Antarctica
        df_b.loc[off_mask, "gps_longitude"] = 0.0

        # Run A
        engine_a = DeadReckoningEngine(_config())
        result_a = engine_a.process_dataframe(df_a)

        # Run B
        engine_b = DeadReckoningEngine(_config())
        result_b = engine_b.process_dataframe(df_b)

        # Compare DR-only rows
        dr_a = result_a[result_a["navigation_mode"] == "DEAD_RECKONING"].reset_index(drop=True)
        dr_b = result_b[result_b["navigation_mode"] == "DEAD_RECKONING"].reset_index(drop=True)

        assert len(dr_a) == len(dr_b), "Different number of DR rows"
        assert len(dr_a) > 0, "No DR rows found"

        compare_cols = [
            "estimated_latitude", "estimated_longitude",
            "estimated_x_m", "estimated_y_m",
            "estimated_velocity_mps", "estimated_heading_deg",
            "distance_step_m", "cumulative_distance_m",
        ]

        for col in compare_cols:
            for idx in range(len(dr_a)):
                assert dr_a.loc[idx, col] == pytest.approx(
                    dr_b.loc[idx, col], abs=1e-12
                ), (
                    f"GPS LEAKAGE DETECTED in column '{col}' at DR row {idx}: "
                    f"Run A = {dr_a.loc[idx, col]}, Run B = {dr_b.loc[idx, col]}"
                )
