"""
Unit tests for src/dead_reckoning/coordinates.py

Tests cover:
  • Haversine distance
  • Initial bearing
  • GPS ↔ local round-trip conversion
  • Heading normalisation
  • Displacement calculations
"""

import math
import pytest

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.dead_reckoning.coordinates import (
    displacement_east_north,
    gps_to_local,
    haversine_distance,
    initial_bearing,
    local_to_gps,
    normalize_heading_deg,
)


# ── Heading normalisation ───────────────────────────────────────────────

class TestNormalizeHeading:
    def test_zero(self):
        assert normalize_heading_deg(0.0) == pytest.approx(0.0)

    def test_positive(self):
        assert normalize_heading_deg(90.0) == pytest.approx(90.0)

    def test_full_circle(self):
        assert normalize_heading_deg(360.0) == pytest.approx(0.0)

    def test_negative(self):
        assert normalize_heading_deg(-90.0) == pytest.approx(270.0)

    def test_large_positive(self):
        assert normalize_heading_deg(720.0) == pytest.approx(0.0)

    def test_large_negative(self):
        assert normalize_heading_deg(-450.0) == pytest.approx(270.0)


# ── Haversine distance ──────────────────────────────────────────────────

class TestHaversine:
    def test_same_point(self):
        d = haversine_distance(13.0, 80.0, 13.0, 80.0)
        assert d == pytest.approx(0.0, abs=1e-6)

    def test_known_distance(self):
        """New York to London ≈ 5 570 km."""
        d = haversine_distance(40.7128, -74.0060, 51.5074, -0.1278)
        assert 5_500_000 < d < 5_700_000

    def test_short_distance_north(self):
        """~111 km per degree of latitude at the equator."""
        d = haversine_distance(0.0, 0.0, 1.0, 0.0)
        assert d == pytest.approx(111_195, abs=200)

    def test_symmetry(self):
        d1 = haversine_distance(10.0, 20.0, 30.0, 40.0)
        d2 = haversine_distance(30.0, 40.0, 10.0, 20.0)
        assert d1 == pytest.approx(d2, abs=1e-6)


# ── Initial bearing ─────────────────────────────────────────────────────

class TestBearing:
    def test_due_north(self):
        b = initial_bearing(0.0, 0.0, 1.0, 0.0)
        assert b == pytest.approx(0.0, abs=0.1)

    def test_due_east(self):
        b = initial_bearing(0.0, 0.0, 0.0, 1.0)
        assert b == pytest.approx(90.0, abs=0.1)

    def test_due_south(self):
        b = initial_bearing(1.0, 0.0, 0.0, 0.0)
        assert b == pytest.approx(180.0, abs=0.1)

    def test_due_west(self):
        b = initial_bearing(0.0, 1.0, 0.0, 0.0)
        assert b == pytest.approx(270.0, abs=0.1)


# ── GPS ↔ Local conversions ─────────────────────────────────────────────

class TestGpsLocalConversion:
    REF_LAT = 13.0827
    REF_LON = 80.2707

    def test_origin_is_zero(self):
        x, y = gps_to_local(self.REF_LAT, self.REF_LON, self.REF_LAT, self.REF_LON)
        assert x == pytest.approx(0.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)

    def test_north_displacement(self):
        """1 degree north ≈ 111 km."""
        x, y = gps_to_local(self.REF_LAT, self.REF_LON, self.REF_LAT + 1.0, self.REF_LON)
        assert abs(x) < 10  # mostly North, negligible East
        assert y == pytest.approx(111_195, abs=300)

    def test_east_displacement(self):
        x, y = gps_to_local(self.REF_LAT, self.REF_LON, self.REF_LAT, self.REF_LON + 1.0)
        assert x > 100_000
        assert abs(y) < 10

    def test_round_trip(self):
        """GPS → local → GPS should recover the original coordinates."""
        target_lat = self.REF_LAT + 0.005
        target_lon = self.REF_LON + 0.003
        x, y = gps_to_local(self.REF_LAT, self.REF_LON, target_lat, target_lon)
        lat2, lon2 = local_to_gps(self.REF_LAT, self.REF_LON, x, y)
        assert lat2 == pytest.approx(target_lat, abs=1e-6)
        assert lon2 == pytest.approx(target_lon, abs=1e-6)

    def test_round_trip_large(self):
        target_lat = self.REF_LAT + 0.1
        target_lon = self.REF_LON - 0.05
        x, y = gps_to_local(self.REF_LAT, self.REF_LON, target_lat, target_lon)
        lat2, lon2 = local_to_gps(self.REF_LAT, self.REF_LON, x, y)
        assert lat2 == pytest.approx(target_lat, abs=1e-4)
        assert lon2 == pytest.approx(target_lon, abs=1e-4)


# ── Displacement ────────────────────────────────────────────────────────

class TestDisplacement:
    def test_north(self):
        de, dn = displacement_east_north(100.0, 0.0)
        assert de == pytest.approx(0.0, abs=1e-10)
        assert dn == pytest.approx(100.0, abs=1e-10)

    def test_east(self):
        de, dn = displacement_east_north(100.0, 90.0)
        assert de == pytest.approx(100.0, abs=1e-10)
        assert dn == pytest.approx(0.0, abs=1e-10)

    def test_south(self):
        de, dn = displacement_east_north(100.0, 180.0)
        assert de == pytest.approx(0.0, abs=1e-8)
        assert dn == pytest.approx(-100.0, abs=1e-8)

    def test_west(self):
        de, dn = displacement_east_north(100.0, 270.0)
        assert de == pytest.approx(-100.0, abs=1e-8)
        assert dn == pytest.approx(0.0, abs=1e-8)

    def test_northeast(self):
        de, dn = displacement_east_north(100.0, 45.0)
        expected = 100.0 * math.sin(math.radians(45))
        assert de == pytest.approx(expected, abs=1e-8)
        assert dn == pytest.approx(expected, abs=1e-8)
