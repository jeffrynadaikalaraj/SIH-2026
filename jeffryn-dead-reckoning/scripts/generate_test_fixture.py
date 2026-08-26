#!/usr/bin/env python3
"""
Generate a deterministic 90-second synthetic sensor dataset for testing.

Journey profile
---------------
  0–10 s   : Vehicle stationary, GNSS ON
 10–30 s   : Vehicle accelerates straight North, GNSS ON
 30–60 s   : GNSS OFF, vehicle goes straight then makes a gradual right turn
 60–75 s   : GNSS ON again, vehicle continues
 75–90 s   : Vehicle brakes and stops, GNSS ON

Output
------
  data/sample_sensor_data.csv

All values are deterministic (fixed random seed for optional sensor noise).
Ground-truth GPS is retained during GNSS OFF **only** for later evaluation;
the Dead Reckoning engine must never use it.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pandas as pd

# Fixed seed for repeatability
RNG = np.random.default_rng(42)

# ── Constants ───────────────────────────────────────────────────────────
DT = 0.1  # seconds per sample
START_LAT = 13.0827  # Chennai, India
START_LON = 80.2707
EARTH_R = 6_371_000.0

# Noise amplitudes
GYRO_NOISE_STD = 0.3      # deg/s
ACCEL_NOISE_STD = 0.15     # m/s²
SPEED_NOISE_STD = 0.05     # m/s


def _local_to_gps(ref_lat, ref_lon, x_e, y_n):
    lat = ref_lat + math.degrees(y_n / EARTH_R)
    cos_ref = math.cos(math.radians(ref_lat))
    lon = ref_lon + math.degrees(x_e / (EARTH_R * cos_ref))
    return lat, lon


def generate() -> pd.DataFrame:
    """Generate the synthetic sensor DataFrame."""
    records = []
    t = 0.0
    heading_deg = 0.0  # Start facing North
    speed = 0.0
    x, y = 0.0, 0.0
    gyro_bias = 0.1  # slight bias in deg/s
    accel_bias = 0.05  # slight bias in m/s²

    while t <= 90.0 + 1e-9:
        # ── Phase logic ─────────────────────────────────────────────
        if t < 10.0:
            # Stationary, GNSS ON
            target_accel = 0.0
            target_gyro = 0.0
            gnss = True
        elif t < 30.0:
            # Accelerate to ~15 m/s heading North, GNSS ON
            if speed < 15.0:
                target_accel = 2.0
            else:
                target_accel = 0.0
            target_gyro = 0.0
            gnss = True
        elif t < 45.0:
            # GNSS OFF — straight
            target_accel = 0.0
            target_gyro = 0.0
            gnss = False
        elif t < 60.0:
            # GNSS OFF — gradual right turn (~3 deg/s clockwise)
            target_accel = 0.0
            target_gyro = 3.0
            gnss = False
        elif t < 75.0:
            # GNSS ON again
            target_accel = 0.0
            target_gyro = 0.0
            gnss = True
        else:
            # Braking to stop, GNSS ON
            if speed > 0.5:
                target_accel = -3.0
            else:
                target_accel = 0.0
                speed = 0.0
            target_gyro = 0.0
            gnss = True

        # ── Update ground truth ─────────────────────────────────────
        heading_deg += target_gyro * DT
        heading_deg = heading_deg % 360.0

        speed += target_accel * DT
        speed = max(0.0, speed)

        dist = speed * DT
        heading_rad = math.radians(heading_deg)
        x += dist * math.sin(heading_rad)
        y += dist * math.cos(heading_rad)

        gt_lat, gt_lon = _local_to_gps(START_LAT, START_LON, x, y)

        # ── Sensor readings (with noise) ────────────────────────────
        gyro_z_noisy = target_gyro + gyro_bias + RNG.normal(0, GYRO_NOISE_STD)
        accel_x_noisy = target_accel + accel_bias + RNG.normal(0, ACCEL_NOISE_STD)
        accel_y_noisy = RNG.normal(0, ACCEL_NOISE_STD)
        accel_z_noisy = 9.81 + RNG.normal(0, ACCEL_NOISE_STD)
        speed_noisy = max(0.0, speed + RNG.normal(0, SPEED_NOISE_STD))

        # GPS coordinates (ground truth stored even during GNSS OFF)
        gps_lat = gt_lat
        gps_lon = gt_lon

        records.append({
            "timestamp": round(t, 3),
            "gps_latitude": round(gps_lat, 8),
            "gps_longitude": round(gps_lon, 8),
            "speed_mps": round(speed_noisy, 4),
            "accel_x": round(accel_x_noisy, 4),
            "accel_y": round(accel_y_noisy, 4),
            "accel_z": round(accel_z_noisy, 4),
            "gyro_x": round(RNG.normal(0, 0.1), 4),
            "gyro_y": round(RNG.normal(0, 0.1), 4),
            "gyro_z": round(gyro_z_noisy, 4),
            "gnss_available": "ON" if gnss else "OFF",
            "ground_truth_latitude": round(gt_lat, 8),
            "ground_truth_longitude": round(gt_lon, 8),
        })

        t = round(t + DT, 3)

    return pd.DataFrame(records)


def main() -> None:
    df = generate()
    out_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "sample_sensor_data.csv")
    df.to_csv(out_path, index=False)
    print(f"Generated {len(df)} records -> {out_path}")


if __name__ == "__main__":
    main()
