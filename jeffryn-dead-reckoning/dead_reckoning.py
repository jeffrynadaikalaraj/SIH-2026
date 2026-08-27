#!/usr/bin/env python3
"""
Jeffryn Dead Reckoning Engine — CLI entry point.
PS 26168 — Intelligent Vehicle Navigation During GNSS Outages

Usage
-----
    python dead_reckoning.py \\
        --input  data/sample_sensor_data.csv \\
        --output outputs/estimated_trajectory.csv \\
        --config config.yaml \\
        --plot
"""

from __future__ import annotations

import argparse
import logging
import os
import sys

import pandas as pd

# Ensure the project root is on sys.path so ``src`` is importable
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.dead_reckoning import DeadReckoningEngine, load_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Jeffryn Dead Reckoning Engine — estimate vehicle "
                    "position during GNSS outages.",
    )
    parser.add_argument(
        "--input", "-i",
        required=True,
        help="Path to the input sensor CSV file.",
    )
    parser.add_argument(
        "--output", "-o",
        default="outputs/estimated_trajectory.csv",
        help="Path to the output trajectory CSV file.",
    )
    parser.add_argument(
        "--config", "-c",
        default="config.yaml",
        help="Path to the YAML configuration file.",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity.",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Generate a trajectory plot after processing.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # Logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(levelname)s: %(message)s",
    )

    # Load configuration
    config = load_config(args.config)

    # Read input
    df = pd.read_csv(args.input)
    logging.info("Loaded %d records from %s", len(df), args.input)

    # Process
    engine = DeadReckoningEngine(config)
    result = engine.process_dataframe(df)

    # Ensure output directory exists
    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    # Save
    precision = config.get("output", {}).get("floating_point_precision", 8)
    result.to_csv(args.output, index=False, float_format=f"%.{precision}f")
    logging.info("Trajectory saved to %s", args.output)

    # ── Summary ─────────────────────────────────────────────────────
    total = len(result)
    gnss_count = int((result["navigation_mode"].isin(["GNSS", "GNSS_REACQUIRED"])).sum())
    dr_count = int((result["navigation_mode"] == "DEAD_RECKONING").sum())
    outages = engine._outage_count
    longest = max(engine._outage_durations) if engine._outage_durations else 0.0
    total_dist = float(result["cumulative_distance_m"].iloc[-1]) if total else 0.0
    max_speed = float(result["estimated_speed_kmph"].max()) if total else 0.0
    recon_error = float(engine.state.last_reconnection_error_m)

    print()
    print("=" * 50)
    print("JEFFRYN DEAD RECKONING ENGINE")
    print("=" * 50)
    print(f"Input file:             {args.input}")
    print(f"Output file:            {args.output}")
    print(f"Total records:          {total}")
    print(f"GNSS records:           {gnss_count}")
    print(f"Dead Reckoning records: {dr_count}")
    print(f"GNSS outages detected:  {outages}")
    print(f"Longest outage:         {longest:.2f} seconds")
    print(f"Estimated distance:     {total_dist:.2f} metres")
    print(f"Maximum speed:          {max_speed:.2f} km/h")
    print(f"Reconnection error:     {recon_error:.2f} metres")
    print("Processing completed successfully")
    print("=" * 50)

    # ── Optional plot ───────────────────────────────────────────────
    if args.plot:
        plot_out = os.path.splitext(args.output)[0] + ".png"
        try:
            from scripts.plot_estimated_trajectory import plot_trajectory
            plot_trajectory(
                trajectory_path=args.output,
                output_path=plot_out,
            )
            print(f"Plot saved to {plot_out}")
        except Exception as exc:
            logging.warning("Could not generate plot: %s", exc)


if __name__ == "__main__":
    main()
