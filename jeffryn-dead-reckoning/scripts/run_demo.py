#!/usr/bin/env python3
"""
End-to-end demonstration of the Jeffryn Dead Reckoning Engine.

Steps:
1. Generate the 90-second test fixture.
2. Run the Dead Reckoning engine on it.
3. Export the estimated trajectory CSV.
4. Generate the trajectory plot.
"""

from __future__ import annotations

import os
import sys

# Ensure project root is on path
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import pandas as pd

from scripts.generate_test_fixture import generate
from scripts.plot_estimated_trajectory import plot_trajectory
from src.dead_reckoning import DeadReckoningEngine, load_config


def main() -> None:
    data_dir = os.path.join(ROOT, "data")
    out_dir = os.path.join(ROOT, "outputs")
    os.makedirs(data_dir, exist_ok=True)
    os.makedirs(out_dir, exist_ok=True)

    sensor_path = os.path.join(data_dir, "sample_sensor_data.csv")
    traj_path = os.path.join(out_dir, "estimated_trajectory.csv")
    plot_path = os.path.join(out_dir, "estimated_trajectory.png")
    config_path = os.path.join(ROOT, "config.yaml")

    # 1 - Generate fixture
    print("[1/4] Generating test fixture ...")
    df = generate()
    df.to_csv(sensor_path, index=False)
    print(f"      {len(df)} records -> {sensor_path}")

    # 2 - Process
    print("[2/4] Running Dead Reckoning engine ...")
    config = load_config(config_path)
    engine = DeadReckoningEngine(config)
    result = engine.process_dataframe(df)
    print(f"      Output rows: {len(result)}")

    # 3 - Save CSV
    print("[3/4] Saving trajectory ...")
    precision = config.get("output", {}).get("floating_point_precision", 8)
    result.to_csv(traj_path, index=False, float_format=f"%.{precision}f")
    print(f"      -> {traj_path}")

    # 4 - Plot
    print("[4/4] Generating plot ...")
    plot_trajectory(
        trajectory_path=traj_path,
        output_path=plot_path,
        sensor_data_path=sensor_path,
        show_ground_truth=True,
    )

    # Summary
    dr_count = int((result["navigation_mode"] == "DEAD_RECKONING").sum())
    gnss_count = len(result) - dr_count
    print()
    print("=" * 50)
    print("DEMO COMPLETE")
    print("=" * 50)
    print(f"Total records:          {len(result)}")
    print(f"GNSS records:           {gnss_count}")
    print(f"Dead Reckoning records: {dr_count}")
    print(f"Reconnection error:     {engine.state.last_reconnection_error_m:.2f} m")
    print("=" * 50)


if __name__ == "__main__":
    main()
