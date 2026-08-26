"""IO-VNBD adapter for one synchronized smartphone/vehicle journey pair."""

from pathlib import Path
from typing import Any

import pandas as pd


SMARTPHONE_COLUMNS = {}  # Retained for backward compatibility if imported elsewhere

def load_synchronized_smartphone(path: str | Path) -> list[dict[str, Any]]:
    """Load documented smartphone columns without changing the source file."""
    frame = pd.read_csv(path, encoding="cp1252")
    frame.columns = [column.strip() for column in frame.columns]
    
    column_mapping = {}
    for col in frame.columns:
        col_upper = col.upper()
        if 'TIME SINCE START' in col_upper:
            column_mapping[col] = 'timestamp'
        elif 'GPS LATITUDE' in col_upper:
            column_mapping[col] = 'latitude'
        elif 'GPS LONGITUDE' in col_upper:
            column_mapping[col] = 'longitude'
        elif 'GPS ALTITUDE' in col_upper:
            column_mapping[col] = 'altitude'
        elif 'ACCELEROMETER X' in col_upper:
            column_mapping[col] = 'accel_x'
        elif 'ACCELEROMETER Y' in col_upper:
            column_mapping[col] = 'accel_y'
        elif 'ACCELEROMETER Z' in col_upper:
            column_mapping[col] = 'accel_z'
        elif 'GYROSCOPE X' in col_upper or 'GYROSCOPE YAW' in col_upper:
            column_mapping[col] = 'gyro_x'
        elif 'GYROSCOPE Y' in col_upper or 'GYROSCOPE PITCH' in col_upper:
            column_mapping[col] = 'gyro_y'
        elif 'GYROSCOPE Z' in col_upper or 'GYROSCOPE ROLL' in col_upper:
            column_mapping[col] = 'gyro_z'
        elif 'GPS SPEED' in col_upper:
            column_mapping[col] = 'speed'
        elif 'GPS ORIENTATION' in col_upper:
            column_mapping[col] = 'heading'

    required = ['timestamp', 'latitude', 'longitude', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'speed', 'heading']
    missing = [req for req in required if req not in column_mapping.values()]
    if missing:
        raise ValueError(f"Could not map columns dynamically: missing {missing} from columns {frame.columns}")
        
    frame = frame.rename(columns=column_mapping)
    frame["timestamp"] = frame["timestamp"] / 1000.0
    frame["speed"] = frame["speed"].fillna(0.0) / 3.6
    frame["heading"] = frame["heading"].fillna(0.0) % 360.0
    
    # Keep only the mapped columns
    final_cols = required + (['altitude'] if 'altitude' in column_mapping.values() else [])
    frame = frame[final_cols]
    
    return frame.to_dict(orient="records")
