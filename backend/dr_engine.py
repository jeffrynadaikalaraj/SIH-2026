"""Adapter for the existing standalone dead-reckoning engine."""

from pathlib import Path
import sys
from typing import Any

_ENGINE_ROOT = Path(__file__).resolve().parent.parent / "jeffryn-dead-reckoning"
if str(_ENGINE_ROOT) not in sys.path:
    sys.path.insert(0, str(_ENGINE_ROOT))

from src.dead_reckoning import DeadReckoningEngine, load_config


class BackendDeadReckoning:
    """Translate FastAPI sensor packets to the teammate engine contract."""

    def __init__(self) -> None:
        config_path = _ENGINE_ROOT / "config.yaml"
        config = load_config(str(config_path))
        if "sensors" not in config:
            config["sensors"] = {}
        config["sensors"]["gyroscope_unit"] = "radians_per_second"
        self.engine = DeadReckoningEngine(config)

    def reset(self) -> None:
        self.engine.reset()

    def process(self, sensor: dict[str, Any], gnss_available: bool) -> dict[str, Any]:
        record = {
            "timestamp": sensor["timestamp"],
            "gps_latitude": sensor["latitude"],
            "gps_longitude": sensor["longitude"],
            "speed_mps": sensor["speed"],
            "accel_x": sensor["accel_x"],
            "accel_y": sensor["accel_y"],
            "accel_z": sensor["accel_z"],
            "gyro_x": sensor["gyro_x"],
            "gyro_y": sensor["gyro_y"],
            "gyro_z": sensor["gyro_z"],
            "heading_deg": sensor["heading"],
            "gnss_available": gnss_available,
        }
        return self.engine.process_sensor_record(record)
