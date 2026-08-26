#!/usr/bin/env python3
"""
GNSS Outage Simulator & Dead Reckoning Validation Suite.

SIH26168: ISRO — AI-ML based Intelligent Dead Reckoning system for seamless navigation.

Main configurable simulator and CLI tool. Supports synthetic trajectories or CSV logs,
configurable GNSS outage schedules, IMU noise modeling, Dead Reckoning estimation,
switching/fusion logic with reacquisition smoothing, metrics evaluation, and plotting.
"""

import argparse
from pathlib import Path
import sys
from typing import List, Optional, Tuple, Union

from src.trajectory_generator import Trajectory, generate_synthetic_trajectory, load_trajectory_from_csv
from src.sensor_simulator import create_sensor_streams, OutageWindow, SensorStreams
from src.dead_reckoning import BaseDeadReckoningEstimator, KinematicDeadReckoningEstimator, MLDeadReckoningPlaceholder
from src.fusion_engine import NavigationFusionEngine, FusionResult
from src.metrics import OutageMetricsCalculator, SimulationMetricsReport
from src.visualizer import Visualizer


def run_simulation(
    trajectory: Optional[Trajectory] = None,
    csv_path: Optional[Union[str, Path]] = None,
    duration: float = 70.0,
    dt: float = 1.0,
    nominal_speed: float = 15.0,
    profile: str = "s_curve",
    outage_windows: Optional[List[Union[Tuple[float, float, str], Tuple[float, float], OutageWindow]]] = None,
    gnss_pos_noise: float = 1.5,
    imu_speed_noise: float = 0.15,
    imu_yaw_noise: float = 0.005,
    imu_yaw_bias: float = 0.003,
    blend_duration: float = 2.0,
    dr_estimator: Optional[BaseDeadReckoningEstimator] = None,
    random_seed: Optional[int] = 42,
    output_dir: Optional[Union[str, Path]] = "output",
    save_plots: bool = True,
    print_report: bool = True,
) -> Tuple[Trajectory, SensorStreams, FusionResult, SimulationMetricsReport]:
    """
    Execute an end-to-end GNSS-denied navigation simulation.

    Parameters
    ----------
    trajectory : Trajectory, optional
        Pre-built Trajectory object. If None, loaded from CSV or generated synthetically.
    csv_path : str or Path, optional
        Path to GPS CSV log file.
    duration : float
        Total synthetic trajectory duration in seconds (default: 70.0s).
    dt : float
        Sampling interval in seconds (default: 1.0s for 1 Hz).
    nominal_speed : float
        Cruising vehicle speed in m/s (default: 15.0 m/s).
    profile : str
        Maneuver profile ("s_curve", "urban_maneuver", "circle", "straight").
    outage_windows : list, optional
        List of outage specs, e.g. [(0, 20, "ON"), (20, 50, "OFF"), (50, 70, "ON")].
    gnss_pos_noise : float
        GNSS horizontal 1-sigma standard deviation in meters (default: 1.5m).
    imu_speed_noise : float
        IMU speed measurement noise std in m/s (default: 0.15 m/s).
    imu_yaw_noise : float
        Gyroscope yaw rate noise std in rad/s (default: 0.005 rad/s).
    imu_yaw_bias : float
        Gyroscope constant yaw rate bias drift in rad/s (default: 0.003 rad/s).
    blend_duration : float
        Reacquisition smoothing duration in seconds (default: 2.0s).
    dr_estimator : BaseDeadReckoningEstimator, optional
        Dead Reckoning estimator instance (defaults to KinematicDeadReckoningEstimator).
    random_seed : int, optional
        RNG seed for reproducibility.
    output_dir : str or Path, optional
        Directory to save plots and artifacts.
    save_plots : bool
        Whether to generate and save Matplotlib plots.
    print_report : bool
        Whether to print the ASCII validation metrics table to console.

    Returns
    -------
    Tuple[Trajectory, SensorStreams, FusionResult, SimulationMetricsReport]
    """
    # 1. Obtain Ground Truth Trajectory
    if trajectory is None:
        if csv_path is not None:
            trajectory = load_trajectory_from_csv(csv_path)
        else:
            trajectory = generate_synthetic_trajectory(
                duration=duration,
                dt=dt,
                nominal_speed=nominal_speed,
                profile=profile,
            )

    # 2. Default Outage Schedule if not specified: 0-20s ON, 20-50s OFF, 50-70s ON
    if outage_windows is None:
        outage_windows = [(0.0, 20.0, "ON"), (20.0, 50.0, "OFF"), (50.0, duration, "ON")]

    # 3. Simulate Sensor Streams (GNSS with noise & outages, IMU with drift)
    sensor_streams = create_sensor_streams(
        trajectory=trajectory,
        outage_windows=outage_windows,
        gnss_pos_noise=gnss_pos_noise,
        imu_speed_noise=imu_speed_noise,
        imu_yaw_noise=imu_yaw_noise,
        imu_yaw_bias=imu_yaw_bias,
        random_seed=random_seed,
    )

    # 4. Initialize Dead Reckoning Estimator (Default: Kinematic)
    if dr_estimator is None:
        dr_estimator = KinematicDeadReckoningEstimator()

    # 5. Run Fusion Engine
    fusion_engine = NavigationFusionEngine(
        dr_estimator=dr_estimator,
        blend_duration=blend_duration,
    )
    fusion_result = fusion_engine.run(sensor_streams)

    # 6. Compute Validation Metrics
    metrics_report = OutageMetricsCalculator.compute(
        ground_truth=trajectory,
        fusion_result=fusion_result,
        sensor_streams=sensor_streams,
    )

    if print_report:
        print(metrics_report.print_summary())

    # 7. Generate Visual Deliverables
    if save_plots:
        out_dir = Path(output_dir) if output_dir else Path("output")
        out_dir.mkdir(parents=True, exist_ok=True)
        visualizer = Visualizer()

        traj_plot_path = out_dir / "trajectory_plot.png"
        visualizer.plot_trajectory(
            ground_truth=trajectory,
            fusion_result=fusion_result,
            sensor_streams=sensor_streams,
            output_path=traj_plot_path,
        )

        error_plot_path = out_dir / "error_analysis_plot.png"
        visualizer.plot_error_analysis(
            ground_truth=trajectory,
            fusion_result=fusion_result,
            sensor_streams=sensor_streams,
            metrics_report=metrics_report,
            output_path=error_plot_path,
        )

        if print_report:
            print(f"[Visualizer] Trajectory map saved to: {traj_plot_path}")
            print(f"[Visualizer] Error diagnostics saved to: {error_plot_path}\n")

    return trajectory, sensor_streams, fusion_result, metrics_report


def build_cli_parser() -> argparse.ArgumentParser:
    """Build command line argument parser."""
    parser = argparse.ArgumentParser(
        description="SIH26168: GNSS Outage Simulator & Dead Reckoning Validation Suite",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--duration", type=float, default=70.0, help="Total simulation duration (seconds)")
    parser.add_argument("--dt", type=float, default=1.0, help="Sampling time interval dt (seconds)")
    parser.add_argument("--speed", type=float, default=15.0, help="Cruising speed in m/s")
    parser.add_argument("--profile", type=str, default="s_curve", choices=["s_curve", "urban_maneuver", "circle", "straight"], help="Synthetic trajectory maneuver profile")
    parser.add_argument("--csv", type=str, default=None, help="Path to ground-truth GPS CSV log file")
    parser.add_argument("--outage", action="append", nargs=2, type=float, metavar=("START", "END"), help="GNSS outage interval in seconds (can be repeated, e.g. --outage 20 50)")
    parser.add_argument("--gnss-noise", type=float, default=1.5, help="GNSS position Gaussian noise std dev (meters)")
    parser.add_argument("--imu-speed-noise", type=float, default=0.15, help="IMU speed noise std dev (m/s)")
    parser.add_argument("--imu-yaw-noise", type=float, default=0.005, help="IMU yaw rate noise std dev (rad/s)")
    parser.add_argument("--imu-yaw-bias", type=float, default=0.003, help="IMU constant yaw rate bias drift (rad/s)")
    parser.add_argument("--blend-duration", type=float, default=2.0, help="Reacquisition blending window duration (seconds)")
    parser.add_argument("--estimator", type=str, default="kinematic", choices=["kinematic", "ml_placeholder"], help="Dead reckoning estimator model")
    parser.add_argument("--output-dir", type=str, default="output", help="Directory to save output plots and reports")
    parser.add_argument("--no-plots", action="store_true", help="Disable plot generation")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for repeatable noise simulation")

    return parser


def main():
    parser = build_cli_parser()
    args = parser.parse_args()

    # Build outage windows list
    if args.outage:
        outages = [(start, end, "OFF") for start, end in args.outage]
    else:
        outages = [(0.0, 20.0, "ON"), (20.0, 50.0, "OFF"), (50.0, args.duration, "ON")]

    # Select estimator
    if args.estimator == "ml_placeholder":
        dr_estimator = MLDeadReckoningPlaceholder()
    else:
        dr_estimator = KinematicDeadReckoningEstimator()

    run_simulation(
        csv_path=args.csv,
        duration=args.duration,
        dt=args.dt,
        nominal_speed=args.speed,
        profile=args.profile,
        outage_windows=outages,
        gnss_pos_noise=args.gnss_noise,
        imu_speed_noise=args.imu_speed_noise,
        imu_yaw_noise=args.imu_yaw_noise,
        imu_yaw_bias=args.imu_yaw_bias,
        blend_duration=args.blend_duration,
        dr_estimator=dr_estimator,
        random_seed=args.seed,
        output_dir=args.output_dir,
        save_plots=not args.no_plots,
        print_report=True,
    )


if __name__ == "__main__":
    main()
