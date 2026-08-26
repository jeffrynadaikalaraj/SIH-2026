"""
Unit tests for the DeadReckoningEngine — kinematics and numerical acceptance.

Tests cover:
  1.  Stationary vehicle remains stationary.
  2.  Straight North movement.
  3.  Straight East movement.
  4.  Constant velocity produces expected distance.
  5.  Acceleration increases velocity.
  6.  Braking decreases velocity.
  7.  Velocity never becomes negative.
  8.  Left turn updates heading correctly.
  9.  Right turn updates heading correctly.
  10. Heading stays within [0, 360).
  11. Extreme acceleration is clipped.
  12. Extreme yaw rate is clipped.
"""

import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pandas as pd
import pytest

from src.dead_reckoning import DeadReckoningEngine, load_config


def _default_config():
    """Minimal config for unit tests."""
    return {
        "earth": {"radius_m": 6_371_000.0},
        "timing": {"minimum_dt_seconds": 0.001, "maximum_dt_seconds": 2.0},
        "heading": {"default_heading_deg": 0.0, "minimum_gps_displacement_m": 1.5,
                     "gyro_yaw_sign": 1.0},
        "sensors": {"gyroscope_unit": "degrees_per_second",
                     "forward_acceleration_axis": "accel_x",
                     "forward_acceleration_sign": 1.0,
                     "acceleration_filter_alpha": 1.0},  # no filter
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


def _make_engine(config=None):
    cfg = config or _default_config()
    engine = DeadReckoningEngine(cfg)
    engine.initialize(13.0, 80.0, 0.0, 0.0, 0.0)
    return engine


# ── 1. Stationary ───────────────────────────────────────────────────────

class TestStationary:
    def test_position_unchanged(self):
        engine = _make_engine()
        for _ in range(100):
            engine.propagate_position(0.0, 0.0, 0.1)
        assert engine.state.local_x_m == pytest.approx(0.0, abs=1e-10)
        assert engine.state.local_y_m == pytest.approx(0.0, abs=1e-10)


# ── 2–3. Straight movement ─────────────────────────────────────────────

class TestStraightMovement:
    def test_north_100m(self):
        """10 m/s North for 10 s → ~100 m North, ~0 m East."""
        engine = _make_engine()
        engine.state.previous_velocity_mps = 10.0
        engine.state.current_velocity_mps = 10.0
        engine.state.heading_deg = 0.0
        engine.state.sync_heading_rad()
        for _ in range(100):
            engine.propagate_position(10.0, 0.0, 0.1)
        assert engine.state.local_y_m == pytest.approx(100.0, abs=0.5)
        assert abs(engine.state.local_x_m) < 0.01

    def test_east_100m(self):
        """10 m/s East for 10 s → ~100 m East, ~0 m North."""
        engine = _make_engine()
        engine.state.heading_deg = 90.0
        engine.state.sync_heading_rad()
        engine.state.previous_velocity_mps = 10.0
        engine.state.current_velocity_mps = 10.0
        for _ in range(100):
            engine.propagate_position(10.0, 90.0, 0.1)
        assert engine.state.local_x_m == pytest.approx(100.0, abs=0.5)
        assert abs(engine.state.local_y_m) < 0.01

    def test_latitude_changes_for_north(self):
        engine = _make_engine()
        initial_lat = engine.state.estimated_latitude
        engine.state.previous_velocity_mps = 10.0
        engine.state.current_velocity_mps = 10.0
        for _ in range(100):
            engine.propagate_position(10.0, 0.0, 0.1)
        assert engine.state.estimated_latitude > initial_lat

    def test_longitude_changes_for_east(self):
        engine = _make_engine()
        initial_lon = engine.state.estimated_longitude
        engine.state.heading_deg = 90.0
        engine.state.sync_heading_rad()
        engine.state.previous_velocity_mps = 10.0
        engine.state.current_velocity_mps = 10.0
        for _ in range(100):
            engine.propagate_position(10.0, 90.0, 0.1)
        assert engine.state.estimated_longitude > initial_lon


# ── 4. Constant velocity distance ──────────────────────────────────────

class TestConstantVelocity:
    def test_distance(self):
        engine = _make_engine()
        engine.state.previous_velocity_mps = 15.0
        engine.state.current_velocity_mps = 15.0
        total = 0.0
        for _ in range(200):  # 20 seconds
            d = engine.propagate_position(15.0, 45.0, 0.1)
            total += d if d else 0
        assert total == pytest.approx(300.0, abs=1.0)


# ── 5–7. Velocity update ───────────────────────────────────────────────

class TestVelocity:
    def test_acceleration_increases(self):
        cfg = _default_config()
        cfg["velocity"]["mode"] = "acceleration"
        engine = _make_engine(cfg)
        v = engine.update_velocity(None, 2.0, 1.0, False)
        assert v > 0.0

    def test_braking_decreases(self):
        cfg = _default_config()
        cfg["velocity"]["mode"] = "acceleration"
        engine = _make_engine(cfg)
        engine.state.current_velocity_mps = 10.0
        v = engine.update_velocity(None, -3.0, 1.0, False)
        assert v < 10.0

    def test_velocity_never_negative(self):
        cfg = _default_config()
        cfg["velocity"]["mode"] = "acceleration"
        engine = _make_engine(cfg)
        engine.state.current_velocity_mps = 1.0
        v = engine.update_velocity(None, -50.0, 1.0, False)
        assert v >= 0.0


# ── 8–10. Heading update ───────────────────────────────────────────────

class TestHeading:
    def test_right_turn(self):
        engine = _make_engine()
        engine.state.heading_deg = 0.0
        h = engine.update_heading(10.0, 1.0)  # 10 deg/s for 1 s
        assert h == pytest.approx(10.0, abs=0.1)

    def test_left_turn(self):
        engine = _make_engine()
        engine.state.heading_deg = 10.0
        h = engine.update_heading(-10.0, 1.0)
        assert h == pytest.approx(0.0, abs=0.1)

    def test_heading_wraps(self):
        engine = _make_engine()
        engine.state.heading_deg = 350.0
        h = engine.update_heading(20.0, 1.0)  # 350 + 20 = 370 → 10
        assert 0.0 <= h < 360.0
        assert h == pytest.approx(10.0, abs=0.1)


# ── 11. Extreme acceleration clipping ──────────────────────────────────

class TestClipping:
    def test_accel_clipped(self):
        """Extreme acceleration through the full pipeline is clipped."""
        cfg = _default_config()
        cfg["velocity"]["mode"] = "acceleration"
        engine = DeadReckoningEngine(cfg)
        # Initialize via a GNSS record
        init_record = {
            "timestamp": 0.0, "dt": 0.0,
            "gps_latitude": 13.0, "gps_longitude": 80.0,
            "speed_mps": 0.0, "accel_x": 0.0, "accel_y": 0.0,
            "accel_z": 9.81, "gyro_x": 0.0, "gyro_y": 0.0,
            "gyro_z": 0.0, "gnss_available": True,
        }
        engine.process_sensor_record(init_record)
        # Now send a DR record with extreme acceleration
        dr_record = {
            "timestamp": 1.0, "dt": 1.0,
            "gps_latitude": 13.0, "gps_longitude": 80.0,
            "speed_mps": 0.0, "accel_x": 100.0, "accel_y": 0.0,
            "accel_z": 9.81, "gyro_x": 0.0, "gyro_y": 0.0,
            "gyro_z": 0.0, "gnss_available": False,
        }
        result = engine.process_sensor_record(dr_record)
        # Acceleration should have been clipped to max 6.0 m/s²
        # So velocity ≈ 0 + 6.0 * 1.0 = 6.0 m/s (not 100 m/s)
        assert result["estimated_velocity_mps"] <= 7.0
        assert result["estimated_velocity_mps"] < 100.0

    def test_yaw_clipped(self):
        engine = _make_engine()
        engine.state.heading_deg = 0.0
        h = engine.update_heading(500.0, 1.0)  # 500 deg/s >> max 120
        # Should be clipped to 120 * 1 * 1 = 120
        assert h == pytest.approx(120.0, abs=1.0)


# ── Turning scenario ───────────────────────────────────────────────────

class TestTurning:
    def test_curved_trajectory(self):
        """Constant speed + constant yaw → heading changes, curved path."""
        engine = _make_engine()
        engine.state.heading_deg = 0.0
        engine.state.previous_velocity_mps = 10.0
        engine.state.current_velocity_mps = 10.0
        headings = []
        for _ in range(100):
            engine.update_heading(5.0, 0.1)  # 5 deg/s
            engine.propagate_position(10.0, engine.state.heading_deg, 0.1)
            headings.append(engine.state.heading_deg)
        # Heading should have changed significantly
        assert headings[-1] != pytest.approx(0.0, abs=5.0)
        # Should be curved — both x and y should have changed
        assert abs(engine.state.local_x_m) > 1.0
        assert abs(engine.state.local_y_m) > 1.0
