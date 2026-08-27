import sys
from pathlib import Path

# Add project root to sys.path to find backend module
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from fastapi.testclient import TestClient
from backend.main import app
from backend.state import simulation

client = TestClient(app)


def test_home():
    response = client.get("/")
    assert response.status_code == 200
    assert response.json() == {
        "message": "IDR-X Backend is running",
        "status": "success",
    }


def test_require_running():
    # Make sure endpoint requires simulation to be running
    simulation["running"] = False
    response = client.get("/position")
    assert response.status_code == 409


def test_start_simulation():
    response = client.post("/start")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "started"
    assert data["gnss_status"] == "GNSS_AVAILABLE"
    assert simulation["running"] is True


def test_gnss_loss_configuration():
    # Start the simulation first
    client.post("/start")

    response = client.post("/gnss-loss", json={"start": 10.0, "duration": 5.0})
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "gnss_loss_configured"
    assert data["outage_start"] == 10.0
    assert data["outage_end"] == 15.0
    assert simulation["outage_start"] == 10.0
    assert simulation["outage_end"] == 15.0


def test_sensor_data_processing():
    client.post("/start")
    client.post("/gnss-loss", json={"start": 10.0, "duration": 5.0})

    # Test timestamp 5.0 (GNSS available)
    sensor_packet_gnss = {
        "timestamp": 5.0,
        "latitude": 52.40166,
        "longitude": -1.50529,
        "speed": 1.5,
        "accel_x": 0.1,
        "accel_y": 0.2,
        "accel_z": 9.8,
        "gyro_x": 0.0,
        "gyro_y": 0.0,
        "gyro_z": 0.0,
        "heading": 90.0,
        "gnss_status": "GNSS_AVAILABLE",
    }
    response = client.post("/sensor-data", json=sensor_packet_gnss)
    assert response.status_code == 200
    data = response.json()
    assert data["gnss_status"] == "GNSS_AVAILABLE"
    assert data["mode"] == "GNSS_INS"
    assert "position" in data

    # Test timestamp 12.0 (GNSS lost -> DR fallback)
    sensor_packet_dr = {
        "timestamp": 12.0,
        "latitude": 52.40166,
        "longitude": -1.50529,
        "speed": 1.5,
        "accel_x": 0.1,
        "accel_y": 0.2,
        "accel_z": 9.8,
        "gyro_x": 0.0,
        "gyro_y": 0.0,
        "gyro_z": 0.0,
        "heading": 90.0,
        "gnss_status": "GNSS_AVAILABLE",
    }
    response = client.post("/sensor-data", json=sensor_packet_dr)
    assert response.status_code == 200
    data = response.json()
    assert data["gnss_status"] == "GNSS_LOST"
    assert data["mode"] == "DEAD_RECKONING"

    # Test position endpoint
    pos_response = client.get("/position")
    assert pos_response.status_code == 200
    pos_data = pos_response.json()
    assert "latitude" in pos_data
    assert "longitude" in pos_data
    assert pos_data["gnss_status"] == "GNSS_LOST"
    assert pos_data["mode"] == "DEAD_RECKONING"

    # Test trajectory endpoint
    traj_response = client.get("/trajectory")
    assert traj_response.status_code == 200
    traj_data = traj_response.json()
    assert len(traj_data["ground_truth"]) == 2
    assert len(traj_data["estimated"]) == 2

    # Test metrics endpoint
    metrics_response = client.get("/metrics")
    assert metrics_response.status_code == 200
    metrics_data = metrics_response.json()
    assert metrics_data["total_samples"] == 2
    assert metrics_data["gnss_available_samples"] == 1
    assert metrics_data["gnss_outage_samples"] == 1
    assert metrics_data["dr_samples"] == 1
    assert metrics_data["records_used"] == 2


def test_simulate_endpoint():
    # Test the end-to-end /simulate route with the actual S1 journey (first 50 points to be quick)
    journey_path = str(project_root / "IO-VNBD" / "Synchronised V abd S datasets" / "Categorised IOVNB Dataset" / "S (Driver A)" / "S1" / "S-S1.csv")
    response = client.post(
        "/simulate",
        json={
            "journey_path": journey_path,
            "outage_start": 30.0,
            "outage_duration": 10.0,
            "limit": 50,
        },
    )
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "simulation_completed"
    assert data["total_records"] == 50
    assert "trajectory" in data
    assert "metrics" in data
    
    # Check that outage is handled and simulated
    metrics = data["metrics"]
    assert metrics["total_samples"] == 50
    assert "mae_m" in metrics
