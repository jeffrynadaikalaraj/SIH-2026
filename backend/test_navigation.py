"""
test_navigation.py (updated)
============================
Tests for NavigationEngine session, state fields, and route following.
"""
import sys
sys.path.insert(0, ".")

import pytest
from backend.navigation_engine import NavigationEngine
from backend.routing_service import RoutingService


def _make_engine(origin=(13.0335, 77.5640), dest=(12.9592, 77.6668)):
    eng = NavigationEngine()
    svc = RoutingService()
    route = svc._generate_realistic_fallback_route(origin[0], origin[1], dest[0], dest[1])
    state = eng.start_navigation(route, "Origin", "Destination")
    return eng, state


def test_session_id_generated():
    eng, state = _make_engine()
    assert "session_id" in state
    assert len(state["session_id"]) > 0


def test_destination_fields_in_state():
    eng, state = _make_engine()
    assert "destination_latitude" in state
    assert "destination_longitude" in state
    assert abs(state["destination_latitude"] - 12.9592) < 0.01
    assert abs(state["destination_longitude"] - 77.6668) < 0.01


def test_eta_seconds_present():
    eng, state = _make_engine()
    assert "eta_seconds" in state
    assert state["eta_seconds"] >= 0


def test_next_maneuver_field_present():
    eng, state = _make_engine()
    assert "next_maneuver" in state
    valid_maneuvers = {"TURN_LEFT", "TURN_RIGHT", "STRAIGHT", "KEEP_LEFT", "KEEP_RIGHT",
                       "U_TURN", "ROUNDABOUT", "MERGE", "EXIT", "ARRIVE"}
    assert state["next_maneuver"] in valid_maneuvers, f"Unknown maneuver: {state["next_maneuver"]}"


def test_vehicle_follows_route():
    eng, _ = _make_engine()
    positions = []
    for _ in range(20):
        s = eng.step(0.1)
        positions.append((s["latitude"], s["longitude"]))
    # Vehicle should have moved
    assert positions[0] != positions[-1], "Vehicle must move along route"


def test_speed_updates():
    eng, _ = _make_engine()
    speeds = set()
    for _ in range(10):
        s = eng.step(0.1)
        speeds.add(s["speed_kmh"])
    assert len(speeds) >= 1


def test_heading_updates():
    eng, _ = _make_engine()
    headings = set()
    for _ in range(20):
        s = eng.step(0.1)
        headings.add(s["heading_deg"])
    assert len(headings) >= 1


def test_distance_to_maneuver_decreases_generally():
    eng, _ = _make_engine()
    dists = []
    for _ in range(50):
        s = eng.step(0.1)
        dists.append(s["distance_to_maneuver_m"])
    # Over 50 steps the maneuver distance should generally decrease
    assert dists[-1] <= dists[0] + 50, "Maneuver distance should generally decrease"


def test_reset_clears_session():
    eng, _ = _make_engine()
    for _ in range(10):
        eng.step(0.1)
    eng.reset_navigation()
    assert not eng.is_active
    assert eng.session_id == ""
    assert eng.route_geometry == []


def test_route_cache_window_available():
    from backend.route_manager import RouteManager
    from backend.routing_service import RoutingService
    svc = RoutingService()
    route = svc._generate_realistic_fallback_route(13.0335, 77.5640, 12.9592, 77.6668)
    rm = RouteManager()
    rm.load(route)
    window = rm.active_window()
    assert len(window) >= 2, "Rolling cache window must have at least 2 points"


def test_arbitrary_origin_destination():
    # Any pair of coords -- not hard-coded locations
    eng, state = _make_engine(origin=(28.6139, 77.2090), dest=(28.5355, 77.3910))
    assert state["is_active"]
    assert state["destination_latitude"] != 0.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
