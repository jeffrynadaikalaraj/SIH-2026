"""Small, dataset-agnostic preprocessing helpers for the FastAPI prototype.

The official SIH dataset is not currently in the workspace. This module does
not guess its column names. It can inspect a CSV and normalize records that
already match the backend SensorData contract.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any, Iterable, Mapping


NUMERIC_SENSOR_FIELDS = (
    "timestamp",
    "latitude",
    "longitude",
    "speed",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "heading",
)


def inspect_csv(path: str | Path) -> dict[str, Any]:
    """Inspect a CSV's actual columns and missing values without mapping them."""
    csv_path = Path(path)
    if not csv_path.is_file():
        raise FileNotFoundError(f"Dataset file not found: {csv_path}")

    with csv_path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        columns = reader.fieldnames or []
        missing_values = {column: 0 for column in columns}
        row_count = 0

        for row in reader:
            row_count += 1
            for column in columns:
                if row.get(column, "").strip() == "":
                    missing_values[column] += 1

    return {
        "path": str(csv_path),
        "columns": columns,
        "row_count": row_count,
        "missing_values": missing_values,
    }


def preprocess_sensor_records(
    records: Iterable[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """Validate and chronologically normalize records matching SensorData.

    This function deliberately requires the existing backend field names. A
    future dataset adapter should map real dataset columns explicitly after
    the dataset has been inspected.
    """
    processed: list[dict[str, Any]] = []
    previous_timestamp: float | None = None

    for index, record in enumerate(records):
        normalized = dict(record)
        for field in NUMERIC_SENSOR_FIELDS:
            if field not in normalized:
                raise ValueError(f"Record {index} is missing '{field}'")
            try:
                value = float(normalized[field])
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f"Record {index} has a non-numeric '{field}'"
                ) from error
            if not math.isfinite(value):
                raise ValueError(f"Record {index} has a non-finite '{field}'")
            normalized[field] = value

        if not normalized.get("gnss_status"):
            raise ValueError(f"Record {index} is missing 'gnss_status'")
        processed.append(normalized)

    processed.sort(key=lambda record: record["timestamp"])
    for record in processed:
        timestamp = record["timestamp"]
        record["dt"] = (
            0.0 if previous_timestamp is None else timestamp - previous_timestamp
        )
        if record["dt"] < 0:
            raise ValueError("Records could not be sorted by timestamp")
        previous_timestamp = timestamp

    return processed
