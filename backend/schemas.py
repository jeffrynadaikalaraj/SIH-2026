import math

from pydantic import BaseModel, Field, field_validator


class SensorData(BaseModel):
    timestamp: float = Field(..., ge=0)

    latitude: float = Field(..., ge=-90, le=90)
    longitude: float = Field(..., ge=-180, le=180)

    speed: float = Field(..., ge=0)

    accel_x: float
    accel_y: float
    accel_z: float

    gyro_x: float
    gyro_y: float
    gyro_z: float

    heading: float = Field(..., ge=0, lt=360)

    gnss_status: str

    @field_validator(
        "timestamp", "latitude", "longitude", "speed", "accel_x", "accel_y",
        "accel_z", "gyro_x", "gyro_y", "gyro_z", "heading",
    )
    @classmethod
    def values_must_be_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be a finite number")
        return value


class GNSSLossRequest(BaseModel):
    start: float = Field(..., ge=0)
    duration: float = Field(..., gt=0)

    @field_validator("start", "duration")
    @classmethod
    def values_must_be_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be a finite number")
        return value


class SimulationRequest(BaseModel):
    journey_path: str = Field(r"C:\Users\NISHITHA\SIH-2026\IO-VNBD\Synchronised V abd S datasets\Categorised IOVNB Dataset\S (Driver A)\S1\S-S1.csv")
    outage_start: float = Field(30.0, ge=0)
    outage_duration: float = Field(30.0, gt=0)
    limit: int = Field(1000, ge=1)

    @field_validator("outage_start", "outage_duration")
    @classmethod
    def values_must_be_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be a finite number")
        return value


# ─── New Phase-1 Schemas ──────────────────────────────────────────────────────

class GNSSLossToggleRequest(BaseModel):
    """Toggle GNSS loss on/off for a named session (spec API contract)."""
    session_id: str = ""
    enabled: bool = True


class NavigationResetRequest(BaseModel):
    """Stop and clear the active navigation session."""
    session_id: str = ""