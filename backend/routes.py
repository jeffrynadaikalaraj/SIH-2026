from itertools import pairwise

from fastapi import APIRouter, HTTPException, status

from backend.dead_reckoning import haversine_distance
from backend.dr_engine import BackendDeadReckoning
from backend.ekf import NavigationEKF
from backend.preprocessing import preprocess_sensor_records
from backend.schemas import GNSSLossRequest, SensorData, SimulationRequest
from backend.state import simulation

router = APIRouter()


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
def start_simulation():

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
        "message": "Simulation started successfully"
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
        "message": "GNSS outage configured successfully"
    }


@router.get("/position")
def get_position():
    """Return the latest estimated position and effective navigation mode."""
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
    """Return plot-ready ground-truth and estimated trajectory points."""
    _require_running()
    return {
        "ground_truth": [
            {"timestamp": point["timestamp"], "latitude": point["ground_truth_latitude"], "longitude": point["ground_truth_longitude"]}
            for point in simulation["trajectory"]
        ],
        "estimated": [
            {"timestamp": point["timestamp"], "latitude": point["estimated_latitude"], "longitude": point["estimated_longitude"]}
            for point in simulation["trajectory"]
            if point["estimated_latitude"] is not None
        ],
        "points": simulation["trajectory"],
    }


@router.get("/metrics")
def get_metrics():
    """Calculate position errors from records containing estimates and truth."""
    _require_running()
    from math import sqrt

    points = [point for point in simulation["trajectory"] if point["estimated_latitude"] is not None]
    errors = [
        haversine_distance(point["estimated_latitude"], point["estimated_longitude"], point["ground_truth_latitude"], point["ground_truth_longitude"])
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
            detail=f"Failed to load dataset: {str(e)}"
        )

    if req.limit > 0:
        records = records[:req.limit]

    # Preprocess records
    for r in records:
        ts = r["timestamp"]
        in_outage = req.outage_start <= ts < (req.outage_start + req.outage_duration)
        r["gnss_status"] = "GNSS_LOST" if in_outage else "GNSS_AVAILABLE"

    try:
        processed_records = preprocess_sensor_records(records)
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Preprocessing failed: {str(e)}"
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
        
        # update EKF
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
        "metrics": get_metrics()
    }