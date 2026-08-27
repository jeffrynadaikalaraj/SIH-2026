#!/usr/bin/env python3
"""
Velocity Mode Comparison Benchmark.

Runs the 90-second journey under all three velocity estimation modes:
  1. sensor_speed (independent non-GNSS speed)
  2. acceleration (pure forward accelerometer integration)
  3. hybrid (blended speed and accelerometer estimate)

Generates a comparative plot and console performance table.

Usage
-----
    python scripts/compare_velocity_modes.py
"""

from __future__ import annotations

import copy
import os
import sys

# Ensure project root is on sys.path
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

from src.dead_reckoning import DeadReckoningEngine, load_config


def run_comparison() -> None:
    sensor_path = os.path.join(ROOT, "data", "sample_sensor_data.csv")
    config_path = os.path.join(ROOT, "config.yaml")
    out_dir = os.path.join(ROOT, "outputs")
    os.makedirs(out_dir, exist_ok=True)

    if not os.path.exists(sensor_path):
        print(f"Generating fixture data at {sensor_path} ...")
        from scripts.generate_test_fixture import main as gen_main
        gen_main()

    df = pd.read_csv(sensor_path)
    base_config = load_config(config_path)

    modes = ["sensor_speed", "acceleration", "hybrid"]
    results = {}
    summaries = []

    for mode in modes:
        cfg = copy.deepcopy(base_config)
        cfg["velocity"]["mode"] = mode

        engine = DeadReckoningEngine(cfg)
        res_df = engine.process_dataframe(df)
        results[mode] = res_df

        # Calculate metrics
        dist = float(res_df["cumulative_distance_m"].iloc[-1])
        reacq_row = res_df[res_df["navigation_mode"] == "GNSS_REACQUIRED"]
        err = float(reacq_row["gnss_reconnection_error_m"].iloc[0]) if len(reacq_row) > 0 else 0.0
        max_spd = float(res_df["estimated_speed_kmph"].max())

        summaries.append({
            "Mode": mode,
            "Total Distance (m)": f"{dist:.2f}",
            "Max Speed (km/h)": f"{max_spd:.2f}",
            "Reconnection Error (m)": f"{err:.2f}",
        })

    # Print summary table
    print("=" * 65)
    print("DEAD RECKONING ENGINE - VELOCITY MODES BENCHMARK")
    print("=" * 65)
    print(f"{'Mode':<15} | {'Total Dist (m)':<15} | {'Max Speed':<12} | {'Recon Error (m)':<15}")
    print("-" * 65)
    for s in summaries:
        print(f"{s['Mode']:<15} | {s['Total Distance (m)']:<15} | {s['Max Speed (km/h)']:<12} | {s['Reconnection Error (m)']:<15}")
    print("=" * 65)

    # Plot comparative trajectories
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    colors = {"sensor_speed": "tab:blue", "acceleration": "tab:orange", "hybrid": "tab:green"}
    styles = {"sensor_speed": "-", "acceleration": "--", "hybrid": "-."}

    for mode in modes:
        res = results[mode]
        ax1.plot(
            res["estimated_x_m"], res["estimated_y_m"],
            label=f"Mode: {mode}",
            color=colors[mode],
            linestyle=styles[mode],
            linewidth=1.5,
        )

        ax2.plot(
            res["elapsed_time_s"], res["estimated_speed_kmph"],
            label=f"Mode: {mode}",
            color=colors[mode],
            linestyle=styles[mode],
            linewidth=1.2,
        )

    # Ground truth trajectory overlay if available
    if "ground_truth_latitude" in df.columns:
        import math
        ref_lat = df["ground_truth_latitude"].iloc[0]
        ref_lon = df["ground_truth_longitude"].iloc[0]
        R = 6_371_000.0
        cos_ref = math.cos(math.radians(ref_lat))
        gt_x = (df["ground_truth_longitude"] - ref_lon).apply(math.radians) * R * cos_ref
        gt_y = (df["ground_truth_latitude"] - ref_lat).apply(math.radians) * R
        ax1.plot(gt_x, gt_y, "k:", label="Ground Truth", linewidth=1.5, alpha=0.8)

    ax1.set_title("Trajectory Comparison (Local East-North)")
    ax1.set_xlabel("East (metres)")
    ax1.set_ylabel("North (metres)")
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    ax2.set_title("Speed Profile Comparison Over Time")
    ax2.set_xlabel("Elapsed Time (seconds)")
    ax2.set_ylabel("Speed (km/h)")
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    plot_out = os.path.join(out_dir, "velocity_modes_comparison.png")
    fig.tight_layout()
    fig.savefig(plot_out, dpi=150)
    plt.close(fig)
    print(f"Comparison plot saved -> {plot_out}")


def main() -> None:
    run_comparison()


if __name__ == "__main__":
    main()
