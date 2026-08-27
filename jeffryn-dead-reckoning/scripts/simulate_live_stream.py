#!/usr/bin/env python3
"""
Simulate a live real-time sensor stream into the Dead Reckoning Engine.

Demonstrates how Nishitha's FastAPI backend or a real-time websocket
can feed sensor records one-by-one into ``engine.process_sensor_record()``
and receive immediate JSON-serializable navigation updates.

Usage
-----
    python scripts/simulate_live_stream.py
    python scripts/simulate_live_stream.py --speed 20.0   # 20x faster than real-time
    python scripts/simulate_live_stream.py --limit 50     # stream first 50 records
"""

from __future__ import annotations

import argparse
import os
import sys
import time

# Ensure project root is on sys.path
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import pandas as pd
from src.dead_reckoning import DeadReckoningEngine, load_config


def run_stream(
    sensor_path: str,
    config_path: str,
    playback_speed: float = 20.0,
    limit: int | None = None,
) -> None:
    """Stream records from CSV one-by-one into the engine."""
    if not os.path.exists(sensor_path):
        print(f"Error: Sensor file not found at {sensor_path}")
        print("Run `python scripts/generate_test_fixture.py` first.")
        return

    df = pd.read_csv(sensor_path)
    if limit is not None:
        df = df.iloc[:limit]

    config = load_config(config_path)
    engine = DeadReckoningEngine(config)

    print("=" * 72)
    print("JEFFRYN DEAD RECKONING ENGINE - LIVE STREAMING SIMULATION")
    print(f"Playback speed: {playback_speed}x real-time | Total records: {len(df)}")
    print("=" * 72)
    print(
        f"{'Time (s)':<9} | {'GNSS':<5} | {'Mode':<15} | {'Lat, Lon':<22} | "
        f"{'Speed (km/h)':<12} | {'Heading':<7} | {'Outage (s)':<10}"
    )
    print("-" * 72)

    prev_time = 0.0
    for idx, row in df.iterrows():
        record = row.to_dict()
        curr_time = float(record.get("timestamp", 0.0))

        # Compute delay for realistic playback
        if idx > 0 and playback_speed > 0:
            delay = (curr_time - prev_time) / playback_speed
            if delay > 0:
                time.sleep(delay)
        prev_time = curr_time

        # Process record through streaming interface
        result = engine.process_sensor_record(record)

        # Format output line
        t_str = f"{result['timestamp']:.1f}s"
        gnss_str = "ON" if result["gnss_available"] else "OFF"
        mode_str = result["navigation_mode"]
        coord_str = f"{result['estimated_latitude']:.5f}, {result['estimated_longitude']:.5f}"
        spd_str = f"{result['estimated_speed_kmph']:.1f}"
        hdg_str = f"{result['estimated_heading_deg']:.1f} deg"
        outage_str = f"{result['outage_elapsed_seconds']:.1f}s" if result['outage_elapsed_seconds'] > 0 else "-"

        # Highlight transitions
        prefix = ""
        if mode_str == "DEAD_RECKONING" and result["outage_elapsed_seconds"] <= 0.15:
            prefix = " [!] OUTAGE START -> "
        elif mode_str == "GNSS_REACQUIRED":
            prefix = f" [*] RECONNECTED (Error: {result['gnss_reconnection_error_m']:.2f}m) -> "

        print(
            f"{t_str:<9} | {gnss_str:<5} | {mode_str:<15} | {coord_str:<22} | "
            f"{spd_str:<12} | {hdg_str:<7} | {outage_str:<10}{prefix}"
        )

    print("-" * 72)
    print("Streaming simulation finished successfully.")
    print(f"Total distance estimated: {engine.state.total_distance_m:.2f} metres")
    print(f"Last reconnection error:  {engine.state.last_reconnection_error_m:.2f} metres")
    print("=" * 72)


def main() -> None:
    parser = argparse.ArgumentParser(description="Live streaming simulation.")
    parser.add_argument("--sensor-data", default="data/sample_sensor_data.csv")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--speed", type=float, default=20.0, help="Playback speed multiplier (e.g. 20 for 20x).")
    parser.add_argument("--limit", type=int, default=None, help="Limit number of records to stream.")
    args = parser.parse_args()

    sensor_path = os.path.join(ROOT, args.sensor_data)
    config_path = os.path.join(ROOT, args.config)
    run_stream(sensor_path, config_path, playback_speed=args.speed, limit=args.limit)


if __name__ == "__main__":
    main()
