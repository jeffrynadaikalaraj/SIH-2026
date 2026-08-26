#!/usr/bin/env python3
"""
Plot the estimated trajectory from the Dead Reckoning engine output.

Colours GNSS and Dead Reckoning sections differently, marks outage
start/end, and optionally overlays ground truth.

Usage
-----
    python scripts/plot_estimated_trajectory.py \\
        --trajectory outputs/estimated_trajectory.csv \\
        --output outputs/estimated_trajectory.png

    # With ground truth overlay:
    python scripts/plot_estimated_trajectory.py \\
        --trajectory outputs/estimated_trajectory.csv \\
        --sensor-data data/sample_sensor_data.csv \\
        --show-ground-truth \\
        --output outputs/estimated_trajectory.png
"""

from __future__ import annotations

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")  # non-interactive backend
import matplotlib.pyplot as plt
import pandas as pd


def plot_trajectory(
    trajectory_path: str,
    output_path: str = "outputs/estimated_trajectory.png",
    sensor_data_path: str | None = None,
    show_ground_truth: bool = False,
) -> None:
    """Generate and save the trajectory plot.

    Parameters
    ----------
    trajectory_path : str
        Path to the estimated_trajectory.csv.
    output_path : str
        Path to save the PNG.
    sensor_data_path : str or None
        Path to the original sensor CSV (needed for ground truth).
    show_ground_truth : bool
        If True, overlay ground-truth trajectory.
    """
    df = pd.read_csv(trajectory_path)

    fig, ax = plt.subplots(figsize=(10, 8))

    # Split into GNSS and DR sections
    gnss_mask = df["navigation_mode"].isin(["GNSS", "GNSS_REACQUIRED"])
    dr_mask = df["navigation_mode"] == "DEAD_RECKONING"

    # Plot GNSS sections
    if gnss_mask.any():
        ax.plot(
            df.loc[gnss_mask, "estimated_x_m"],
            df.loc[gnss_mask, "estimated_y_m"],
            "b.", markersize=2, label="GNSS",
        )

    # Plot DR sections
    if dr_mask.any():
        ax.plot(
            df.loc[dr_mask, "estimated_x_m"],
            df.loc[dr_mask, "estimated_y_m"],
            "r.", markersize=2, label="Dead Reckoning",
        )

    # Full trajectory line
    ax.plot(
        df["estimated_x_m"], df["estimated_y_m"],
        "k-", linewidth=0.5, alpha=0.4,
    )

    # Mark outage transitions
    for i in range(1, len(df)):
        prev_mode = df.loc[i - 1, "navigation_mode"]
        curr_mode = df.loc[i, "navigation_mode"]
        if prev_mode != "DEAD_RECKONING" and curr_mode == "DEAD_RECKONING":
            ax.plot(
                df.loc[i, "estimated_x_m"], df.loc[i, "estimated_y_m"],
                "rv", markersize=10, label="Outage Start" if i < 5 or True else "",
            )
        if prev_mode == "DEAD_RECKONING" and curr_mode in ("GNSS", "GNSS_REACQUIRED"):
            ax.plot(
                df.loc[i, "estimated_x_m"], df.loc[i, "estimated_y_m"],
                "g^", markersize=10, label="Outage End",
            )

    # Start and End
    ax.plot(
        df.iloc[0]["estimated_x_m"], df.iloc[0]["estimated_y_m"],
        "ks", markersize=10, label="Start",
    )
    ax.plot(
        df.iloc[-1]["estimated_x_m"], df.iloc[-1]["estimated_y_m"],
        "kD", markersize=10, label="End",
    )

    # Optional ground truth
    if show_ground_truth and sensor_data_path:
        sd = pd.read_csv(sensor_data_path)
        if "ground_truth_latitude" in sd.columns and "ground_truth_longitude" in sd.columns:
            # Quick local conversion using first row as reference
            import math
            ref_lat = sd["ground_truth_latitude"].iloc[0]
            ref_lon = sd["ground_truth_longitude"].iloc[0]
            R = 6_371_000.0
            cos_ref = math.cos(math.radians(ref_lat))
            gt_x = (sd["ground_truth_longitude"] - ref_lon).apply(math.radians) * R * cos_ref
            gt_y = (sd["ground_truth_latitude"] - ref_lat).apply(math.radians) * R
            ax.plot(gt_x, gt_y, "g--", linewidth=1.0, alpha=0.7, label="Ground Truth")

    ax.set_xlabel("East (metres)")
    ax.set_ylabel("North (metres)")
    ax.set_title("Jeffryn Dead Reckoning Engine — Estimated Trajectory")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # De-duplicate legend
    handles, labels = ax.get_legend_handles_labels()
    seen = {}
    unique_h, unique_l = [], []
    for h, l in zip(handles, labels):
        if l not in seen:
            seen[l] = True
            unique_h.append(h)
            unique_l.append(l)
    ax.legend(unique_h, unique_l, loc="best")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Plot saved -> {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot estimated trajectory.")
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--sensor-data", default=None)
    parser.add_argument("--show-ground-truth", action="store_true")
    parser.add_argument("--output", default="outputs/estimated_trajectory.png")
    args = parser.parse_args()

    plot_trajectory(
        trajectory_path=args.trajectory,
        output_path=args.output,
        sensor_data_path=args.sensor_data,
        show_ground_truth=args.show_ground_truth,
    )


if __name__ == "__main__":
    main()
