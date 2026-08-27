"""
routes.py
=========
FastAPI Router exposing endpoints for MapX Navigation, Geocoding, Routing,
Dead Reckoning Simulation, WebSocket Live Streaming, and Evaluation.
"""

import asyncio
import json
import logging
from itertools import pairwise
from math import sqrt
from typing import Any, Dict, Optional

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect, status
from pydantic import BaseModel, Field

from backend.dead_reckoning import haversine_distance
from backend.dr_engine import BackendDeadReckoning
from backend.ekf import NavigationEKF
from backend.geocoding_service import GeocodingService
from backend.navigation_engine import nav_engine
from backend.preprocessing import preprocess_sensor_records
from backend.routing_service import RoutingService
from backend.schemas import (
    GNSSLossRequest, SensorData, SimulationRequest,
    GNSSLossToggleRequest, NavigationResetRequest,
)
from backend.state import simulation
from backend.websocket_manager import connection_manager

logger = logging.getLogger(__name__)

router = APIRouter()

routing_svc = RoutingService()
geocoding_svc = GeocodingService()


# -----------------------------------------------------------------------------
# Pydantic Request Models for New MapX Navigation API
# -----------------------------------------------------------------------------
class RouteRequest(BaseModel):
    origin_lat: float = Field(..., ge=-90, le=90)
    origin_lon: float = Field(..., ge=-180, le=180)
    dest_lat: float = Field(..., ge=-90, le=90)
    dest_lon: float = Field(..., ge=-180, le=180)


class StartNavigationRequest(BaseModel):
    origin_lat: float = Field(13.0335, ge=-90, le=90)
    origin_lon: float = Field(77.5640, ge=-180, le=180)
    dest_lat: float = Field(12.9592, ge=-90, le=90)
    dest_lon: float = Field(77.6668, ge=-180, le=180)
    origin_name: str = "ISRO HQ (Antariksh Bhavan)"
    destination_name: str = "URSC (Satellite Centre)"
    speed_multiplier: float = Field(1.0, ge=0.1, le=10.0)


class GNSSLossSimRequest(BaseModel):
    duration_sec: float = Field(25.0, gt=0, le=600)


# -----------------------------------------------------------------------------
# Destination & Geocoding Endpoints
# -----------------------------------------------------------------------------
@router.get("/destinations/search")
async def search_destinations(q: str = "", limit: int = 6):
    """Search for locations matching the query or return popular presets."""
    results = await geocoding_svc.search(q, limit=limit)
    return {"query": q, "results": results}


@router.get("/destinations/saved")
def get_saved_destinations():
    """Return preconfigured popular landmarks (ISRO, tech corridors, etc.)."""
    return {"saved_places": geocoding_svc.get_saved_places()}


# -----------------------------------------------------------------------------
# MapX Navigation Endpoints
# -----------------------------------------------------------------------------
@router.post("/navigation/route")
async def calculate_route(req: RouteRequest):
    """Calculate actual road route with turn maneuvers."""
    route_data = await routing_svc.get_route(
        req.origin_lat, req.origin_lon, req.dest_lat, req.dest_lon
    )
    return route_data


@router.post("/navigation/start")
async def start_navigation(req: StartNavigationRequest):
    """Calculate route and launch real-time navigation session."""
    route_data = await routing_svc.get_route(
        req.origin_lat, req.origin_lon, req.dest_lat, req.dest_lon
    )
    nav_engine.speed_multiplier = req.speed_multiplier
    state = nav_engine.start_navigation(
        route_data, origin_name=req.origin_name, dest_name=req.destination_name
    )
    return {
        "status": "navigation_started",
        "route": route_data,
        "state": state,
    }


@router.post("/navigation/simulate-gnss-loss")
def simulate_gnss_loss_action(req: GNSSLossSimRequest = GNSSLossSimRequest()):
    """Trigger GNSS outage to demonstrate Intelligent Dead Reckoning fallback."""
    return nav_engine.simulate_gnss_loss(req.duration_sec)


@router.post("/navigation/restore-gnss")
def restore_gnss_action():
    """Manually restore GNSS signal."""
    return nav_engine.restore_gnss()


@router.post("/navigation/gnss-loss")
def gnss_loss_toggle(req: GNSSLossToggleRequest = GNSSLossToggleRequest()):
    """Toggle GNSS loss on/off (spec API contract: { session_id, enabled })."""
    if req.enabled:
        from backend import config
        return nav_engine.simulate_gnss_loss(config.GNSS_DEFAULT_OUTAGE_SEC)
    else:
        return nav_engine.restore_gnss()


@router.post("/navigation/reset")
def reset_navigation(req: NavigationResetRequest = NavigationResetRequest()):
    """Stop and clear the active navigation session."""
    nav_engine.reset_navigation()
    return {"status": "navigation_reset", "session_id": req.session_id}


@router.post("/navigation/step")
def step_simulation(dt: float = 0.1):
    """Step the simulation by dt seconds."""
    return nav_engine.step(dt)


@router.get("/navigation/state")
def get_navigation_state():
    """Return live navigation state (for consumer UI)."""
    return nav_engine.get_state()


@router.get("/navigation/metrics")
def get_navigation_metrics():
    """Return engineering evaluation metrics (for debug modal)."""
    return nav_engine.get_metrics()


# -----------------------------------------------------------------------------
# Centralized Simulation Loop & WebSocket Live Navigation Streamer (/navigation/live)
# -----------------------------------------------------------------------------
async def simulation_loop():
    """Centralized background task using actual elapsed time to step the simulation
    at exactly real-time speed, regardless of loop frequency or timing jitter."""
    import time
    logger.info("Centralized real-time simulation loop started")
    from backend import config
    sleep_time = 1.0 / getattr(config, "WS_UPDATE_HZ", 10.0)
    
    last_time = time.time()
    while True:
        try:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Cap dt to avoid massive jumps if the server hangs or during debugging
            dt = min(0.5, max(0.001, dt))
            
            if nav_engine.is_active and not nav_engine.is_paused:
                # Step using the actual elapsed time!
                nav_engine.step(dt)
                logger.info(f"SIM_STEP: dt={dt:.4f}, speed_mps={nav_engine.speed_mps:.2f}, dist={nav_engine.distance_travelled_m:.2f}")
                
                # Fetch state and metrics
                state = nav_engine.get_state()
                metrics = nav_engine.get_metrics()
                
                # Broadcast
                packet = {
                    "type": "LIVE_STATE",
                    "state": state,
                    "metrics": {
                        "mae_m": metrics["mae_m"],
                        "rmse_m": metrics["rmse_m"],
                        "drift_percentage": metrics["drift_percentage"],
                        "current_error_m": metrics["current_error_m"],
                        "distance_travelled_m": metrics["distance_travelled_m"],
                        "outage_active": metrics["outage_active"],
                    },
                }
                await connection_manager.broadcast(packet)
            else:
                # Keep last_time updated even when navigation is not active
                pass
        except Exception as e:
            logger.error("Error in centralized simulation loop: %s", e)
        await asyncio.sleep(sleep_time)


@router.on_event("startup")
async def startup_event():
    """Start the centralized simulation task on application startup."""
    asyncio.create_task(simulation_loop())


@router.websocket("/navigation/live")
async def websocket_live_navigation(websocket: WebSocket):
    """Stream live vehicle state and dead reckoning data at 10 Hz."""
    await connection_manager.connect(websocket)
    try:
        # Send initial state immediately
        state = nav_engine.get_state()
        metrics = nav_engine.get_metrics()
        packet = {
            "type": "LIVE_STATE",
            "state": state,
            "metrics": {
                "mae_m": metrics["mae_m"],
                "rmse_m": metrics["rmse_m"],
                "drift_percentage": metrics["drift_percentage"],
                "current_error_m": metrics["current_error_m"],
                "distance_travelled_m": metrics["distance_travelled_m"],
                "outage_active": metrics["outage_active"],
            },
        }
        await connection_manager.send(websocket, packet)
        
        # Keep connection alive; broadcast loop handles updates
        while True:
            # Block waiting for any client message (or disconnect)
            await websocket.receive_text()
    except WebSocketDisconnect:
        connection_manager.disconnect(websocket)
    except Exception as exc:
        logger.debug("WebSocket closed: %s", exc)
        connection_manager.disconnect(websocket)


# -----------------------------------------------------------------------------
# Legacy / Test Compatibility Endpoints
# -----------------------------------------------------------------------------
def _require_running() -> None:
    if not simulation["running"]:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Start the simulation with POST /start first",
        )


def _is_in_outage(timestamp: float) -> bool:
    start = simulation["outage_start"]
    end = simulation["outage_end"]
    return start is not None and end is not None and start <= timestamp < end


@router.post("/start")
def start_legacy_simulation():
    simulation["running"] = True
    simulation["gnss_status"] = "GNSS_AVAILABLE"
    simulation["sensor_data"] = []
    simulation["trajectory"] = []
    simulation["last_position"] = None
    simulation["current_position"] = None
    simulation["outage_start"] = None
    simulation["outage_end"] = None
    simulation["previous_sensor"] = None
    simulation["previous_estimate"] = None
    simulation["dr_engine"] = BackendDeadReckoning()
    simulation["ekf"] = NavigationEKF()

    return {
        "status": "started",
        "gnss_status": "GNSS_AVAILABLE",
        "message": "Simulation started successfully",
    }


@router.post("/sensor-data")
def receive_sensor_data(data: SensorData):
    _require_running()

    in_outage = _is_in_outage(data.timestamp)
    gnss_status = "GNSS_LOST" if in_outage else "GNSS_AVAILABLE"
    mode = "DEAD_RECKONING" if in_outage else "GNSS_INS"
    sensor_record = preprocess_sensor_records([data.model_dump()])[0]

    engine_result = simulation["dr_engine"].process(sensor_record, not in_outage)
    engine_position = {
        "latitude": engine_result["estimated_latitude"],
        "longitude": engine_result["estimated_longitude"],
    }
    fused = simulation["ekf"].update(
        (engine_position["latitude"], engine_position["longitude"]),
        data.timestamp,
        engine_result["estimated_velocity_mps"],
        engine_result["estimated_heading_deg"],
        "GNSS" if not in_outage else "DR",
    )
    estimate = {"latitude": fused["latitude"], "longitude": fused["longitude"]}

    if estimate is not None:
        simulation["current_position"] = estimate
        simulation["previous_estimate"] = {
            **estimate,
            "timestamp": data.timestamp,
        }

    trajectory_point = {
        "timestamp": data.timestamp,
        "ground_truth_latitude": data.latitude,
        "ground_truth_longitude": data.longitude,
        "estimated_latitude": estimate["latitude"] if estimate else None,
        "estimated_longitude": estimate["longitude"] if estimate else None,
        "gnss_status": gnss_status,
        "mode": mode,
        "dr_engine": engine_result,
        "ekf_position": estimate,
    }

    simulation["sensor_data"].append({**sensor_record, "effective_gnss_status": gnss_status})
    simulation["trajectory"].append(trajectory_point)
    simulation["previous_sensor"] = sensor_record
    simulation["gnss_status"] = gnss_status

    return {
        "status": "received",
        "timestamp": data.timestamp,
        "gnss_status": gnss_status,
        "mode": mode,
        "dr_engine": engine_result,
        "ekf_position": estimate,
        "position": estimate,
    }


@router.post("/gnss-loss")
def simulate_gnss_loss(data: GNSSLossRequest):
    _require_running()

    simulation["outage_start"] = data.start
    simulation["outage_end"] = data.start + data.duration
    simulation["gnss_status"] = "GNSS_LOST"

    return {
        "status": "gnss_loss_configured",
        "outage_start": data.start,
        "outage_end": data.start + data.duration,
        "message": "GNSS outage configured successfully",
    }


@router.get("/position")
def get_position():
    _require_running()
    if not simulation["trajectory"]:
        raise HTTPException(status_code=404, detail="No sensor data received yet")

    latest = simulation["trajectory"][-1]
    return {
        "latitude": latest["estimated_latitude"],
        "longitude": latest["estimated_longitude"],
        "mode": latest["mode"],
        "gnss_status": latest["gnss_status"],
        "timestamp": latest["timestamp"],
    }


@router.get("/trajectory")
def get_trajectory():
    _require_running()
    return {
        "ground_truth": [
            {
                "timestamp": point["timestamp"],
                "latitude": point["ground_truth_latitude"],
                "longitude": point["ground_truth_longitude"],
            }
            for point in simulation["trajectory"]
        ],
        "estimated": [
            {
                "timestamp": point["timestamp"],
                "latitude": point["estimated_latitude"],
                "longitude": point["estimated_longitude"],
            }
            for point in simulation["trajectory"]
            if point["estimated_latitude"] is not None
        ],
        "points": simulation["trajectory"],
    }


@router.get("/metrics")
def get_metrics():
    _require_running()
    points = [point for point in simulation["trajectory"] if point["estimated_latitude"] is not None]
    errors = [
        haversine_distance(
            point["estimated_latitude"],
            point["estimated_longitude"],
            point["ground_truth_latitude"],
            point["ground_truth_longitude"],
        )
        for point in points
    ]
    estimated_points = [(point["estimated_latitude"], point["estimated_longitude"]) for point in points]
    distance_travelled = sum(haversine_distance(*previous, *current) for previous, current in pairwise(estimated_points))
    ground_truth_points = [(point["ground_truth_latitude"], point["ground_truth_longitude"]) for point in points]
    ground_truth_distance = sum(haversine_distance(*previous, *current) for previous, current in pairwise(ground_truth_points))

    if not errors:
        return {
            "total_samples": len(simulation["trajectory"]),
            "gnss_available_samples": sum(point["gnss_status"] == "GNSS_AVAILABLE" for point in simulation["trajectory"]),
            "gnss_outage_samples": sum(point["gnss_status"] == "GNSS_LOST" for point in simulation["trajectory"]),
            "dr_samples": sum(point["mode"] == "DEAD_RECKONING" for point in simulation["trajectory"]),
            "records_used": 0,
            "distance_travelled_m": None,
            "mae_m": None,
            "rmse_m": None,
            "maximum_position_error_m": None,
            "drift_percentage": None,
        }

    return {
        "total_samples": len(simulation["trajectory"]),
        "gnss_available_samples": sum(point["gnss_status"] == "GNSS_AVAILABLE" for point in simulation["trajectory"]),
        "gnss_outage_samples": sum(point["gnss_status"] == "GNSS_LOST" for point in simulation["trajectory"]),
        "dr_samples": sum(point["mode"] == "DEAD_RECKONING" for point in simulation["trajectory"]),
        "records_used": len(errors),
        "distance_travelled_m": distance_travelled,
        "mae_m": sum(errors) / len(errors),
        "rmse_m": sqrt(sum(error * error for error in errors) / len(errors)),
        "maximum_position_error_m": max(errors),
        "drift_percentage": max(errors) / ground_truth_distance * 100 if ground_truth_distance > 0 else None,
    }


@router.post("/simulate")
def simulate(req: SimulationRequest = None):
    if req is None:
        req = SimulationRequest()

    simulation["running"] = True
    simulation["gnss_status"] = "GNSS_AVAILABLE"
    simulation["sensor_data"] = []
    simulation["trajectory"] = []
    simulation["last_position"] = None
    simulation["current_position"] = None
    simulation["outage_start"] = req.outage_start
    simulation["outage_end"] = req.outage_start + req.outage_duration
    simulation["previous_sensor"] = None
    simulation["previous_estimate"] = None
    simulation["dr_engine"] = BackendDeadReckoning()
    simulation["ekf"] = NavigationEKF()

    try:
        from backend.dataset_adapter import load_synchronized_smartphone
        records = load_synchronized_smartphone(req.journey_path)
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Failed to load dataset: {str(e)}",
        )

    if req.limit > 0:
        records = records[: req.limit]

    for r in records:
        ts = r["timestamp"]
        in_outage = req.outage_start <= ts < (req.outage_start + req.outage_duration)
        r["gnss_status"] = "GNSS_LOST" if in_outage else "GNSS_AVAILABLE"

    try:
        processed_records = preprocess_sensor_records(records)
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Preprocessing failed: {str(e)}",
        )

    for sensor_record in processed_records:
        ts = sensor_record["timestamp"]
        in_outage = req.outage_start <= ts < (req.outage_start + req.outage_duration)
        gnss_status = "GNSS_LOST" if in_outage else "GNSS_AVAILABLE"
        mode = "DEAD_RECKONING" if in_outage else "GNSS_INS"

        engine_result = simulation["dr_engine"].process(sensor_record, not in_outage)
        engine_position = {
            "latitude": engine_result["estimated_latitude"],
            "longitude": engine_result["estimated_longitude"],
        }

        fused = simulation["ekf"].update(
            (engine_position["latitude"], engine_position["longitude"]),
            ts,
            engine_result["estimated_velocity_mps"],
            engine_result["estimated_heading_deg"],
            "GNSS" if not in_outage else "DR",
        )
        estimate = {"latitude": fused["latitude"], "longitude": fused["longitude"]}

        if estimate is not None:
            simulation["current_position"] = estimate
            simulation["previous_estimate"] = {
                **estimate,
                "timestamp": ts,
            }

        trajectory_point = {
            "timestamp": ts,
            "ground_truth_latitude": sensor_record["latitude"],
            "ground_truth_longitude": sensor_record["longitude"],
            "estimated_latitude": estimate["latitude"] if estimate else None,
            "estimated_longitude": estimate["longitude"] if estimate else None,
            "gnss_status": gnss_status,
            "mode": mode,
            "dr_engine": engine_result,
            "ekf_position": estimate,
        }

        simulation["sensor_data"].append({**sensor_record, "effective_gnss_status": gnss_status})
        simulation["trajectory"].append(trajectory_point)
        simulation["previous_sensor"] = sensor_record
        simulation["gnss_status"] = gnss_status

    return {
        "status": "simulation_completed",
        "total_records": len(processed_records),
        "trajectory": get_trajectory(),
        "metrics": get_metrics(),
    }