#!/usr/bin/env python3
"""
SIH26168 (ISRO — AI-ML based Intelligent Dead Reckoning system for seamless navigation)
Main End-to-End Simulation & Validation Script.

Executes the primary validation benchmark:
- Trajectory: 70-second 2D vehicle maneuver at 1 Hz (15 m/s)
- GNSS Schedule:
    * 0  - 20s : GNSS Lock (Active, ~1.5m noise, DR calibrating)
    * 20 - 50s : GNSS Denied / Outage (DR active fallback, gyro drift & noise)
    * 50 - 70s : GNSS Reacquisition (Smooth blend over 2.0s, no position jump)
- Outputs:
    * Validation summary table with MAE, RMSE, Max Error, Drift %, Jump magnitude
    * 2D Trajectory overlay plot (output/trajectory_plot.png)
    * Time-series error analysis & diagnostics dashboard (output/error_analysis_plot.png)
"""

from pathlib import Path
import sys

from src.trajectory_generator import generate_synthetic_trajectory
from gnss_outage_simulator import run_simulation


def main():
    print("=" * 84)
    print(" ISRO SIH26168: GNSS-DENIED DEAD RECKONING NAVIGATION SIMULATION & VALIDATION")
    print("=" * 84)
    print("Scenario Configuration:")
    print("  - Total Duration    : 70.0 seconds (1 Hz sampling)")
    print("  - Cruising Speed    : 15.0 m/s (~54 km/h)")
    print("  - Maneuver Profile  : S-Curve Vehicle Maneuver (Turning dynamics)")
    print("  - Outage Window     : 20.0s to 50.0s (30 seconds GNSS-denied window)")
    print("  - Reacquisition     : 50.0s to 70.0s with 2.0s smoothing filter")
    print("  - GNSS Noise Std    : 1.5 meters")
    print("  - IMU Gyro Drift    : 0.003 rad/s bias + 0.005 rad/s noise")
    print("=" * 84)

    output_directory = Path(__file__).parent / "output"
    output_directory.mkdir(parents=True, exist_ok=True)

    # Define the required schedule: 0-20s ON, 20-50s OFF, 50-70s ON
    outage_schedule = [
        (0.0, 20.0, "ON"),
        (20.0, 50.0, "OFF"),
        (50.0, 70.0, "ON"),
    ]

    trajectory, sensors, fusion, metrics = run_simulation(
        duration=70.0,
        dt=1.0,
        nominal_speed=15.0,
        profile="s_curve",
        outage_windows=outage_schedule,
        gnss_pos_noise=1.5,
        imu_speed_noise=0.15,
        imu_yaw_noise=0.005,
        imu_yaw_bias=0.003,
        blend_duration=2.0,
        random_seed=42,
        output_dir=output_directory,
        save_plots=True,
        print_report=True,
    )

    print("\n[SUCCESS] Simulation & validation completed successfully.")
    print(f"[SUCCESS] Artifacts generated in: {output_directory.resolve()}\n")


if __name__ == "__main__":
    main()
