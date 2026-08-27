"""
test_routing.py
===============
Tests for RoutingService -- fallback router and OSRM parsing.
"""
import asyncio
import sys
sys.path.insert(0, ".")

import pytest
from backend.routing_service import RoutingService, haversine_distance, calculate_bearing


def test_haversine_known_distance():
    # Chennai Central to Anna Nagar -- roughly 7.5 km
    d = haversine_distance(13.0827, 80.2707, 13.0900, 80.2100)
    assert 5000 < d < 10000, f"Unexpected distance: {d}"


def test_haversine_same_point():
    assert haversine_distance(12.0, 77.0, 12.0, 77.0) == pytest.approx(0.0, abs=1e-3)


def test_bearing_north():
    b = calculate_bearing(10.0, 80.0, 11.0, 80.0)
    assert abs(b) < 2 or abs(b - 360) < 2, f"Expected ~0 deg (north), got {b}"


def test_bearing_east():
    b = calculate_bearing(13.0, 80.0, 13.0, 81.0)
    assert 85 < b < 95, f"Expected ~90 deg (east), got {b}"


def test_fallback_router_returns_geometry():
    svc = RoutingService()
    # Use two arbitrary locations (not hard-coded specific route)
    result = svc._generate_realistic_fallback_route(13.0335, 77.5640, 12.9592, 77.6668)
    assert result["status"] == "success"
    assert result["source"] == "fallback_router"
    assert len(result["geometry"]) >= 2
    assert result["distance_m"] > 0
    assert len(result["steps"]) >= 2


def test_fallback_router_arbitrary_coords():
    svc = RoutingService()
    # Different arbitrary pair
    result = svc._generate_realistic_fallback_route(28.6139, 77.2090, 28.5355, 77.3910)
    assert result["status"] == "success"
    assert len(result["geometry"]) >= 2


def test_route_has_depart_and_arrive():
    svc = RoutingService()
    result = svc._generate_realistic_fallback_route(13.0, 80.0, 13.1, 80.1)
    types = [s["type"] for s in result["steps"]]
    assert "depart" in types
    assert "arrive" in types


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
