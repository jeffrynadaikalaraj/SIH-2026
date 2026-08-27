"""
plot_sensor_data.py
===================
Sensor Dashboard Visualizer
SIH 2026 -- Problem Statement 26168
"AI-ML based Intelligent Dead Reckoning System"

Generates a 7-panel dashboard from sensor_data.csv:

  Panel 1  : Speed vs Time
  Panel 2  : Acceleration vs Time (Ax, Ay, Az)
  Panel 3  : Gyroscope / Yaw Rate vs Time (Gx, Gy, Gz)
  Panel 4  : Heading vs Time
  Panel 5  : GNSS Availability (ON / OFF bar)
  Panel 6  : Ground Truth GPS Trajectory
  Panel 7  : GNSS Measured Trajectory (gaps during outage)

GNSS outage period is highlighted in RED on all time-series panels.
Ground truth trajectory remains fully visible for post-run evaluation.

Usage:
  pip install matplotlib pandas numpy
  python plot_sensor_data.py
  python plot_sensor_data.py --file sensor_data.csv --out sensor_dashboard.png
"""

import argparse
import os
import sys

try:
    import matplotlib
    matplotlib.use("Agg")   # headless backend -- writes PNG without a display
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import matplotlib.gridspec as gridspec
    import pandas as pd
    import numpy as np
except ImportError:
    print("\n  [!] Missing dependencies. Install with:")
    print("      pip install matplotlib pandas numpy\n")
    sys.exit(1)


# =============================================================================
# COLOUR PALETTE (dark theme)
# =============================================================================
C = {
    "bg":         "#0d1117",
    "surface":    "#161b22",
    "border":     "#30363d",
    "text":       "#e6edf3",
    "muted":      "#8b949e",
    "gnss_on":    "#3fb950",   # green
    "gnss_off":   "#f85149",   # red
    "gt_traj":    "#58a6ff",   # blue  -- ground truth
    "meas_traj":  "#3fb950",   # green -- GNSS measured
    "speed":      "#58a6ff",
    "ax":         "#ff7b72",
    "ay":         "#ffa657",
    "az":         "#d2a8ff",
    "gx":         "#79c0ff",
    "gy":         "#56d364",
    "gz":         "#e3b341",
    "heading":    "#f0f6fc",
    "outage_bg":  "#f85149",
}


# =============================================================================
# HELPERS
# =============================================================================

def style_ax(ax):
    """Apply dark theme to a single axes object."""
    ax.set_facecolor(C["surface"])
    ax.tick_params(colors=C["muted"], labelsize=8)
    ax.xaxis.label.set_color(C["muted"])
    ax.yaxis.label.set_color(C["muted"])
    ax.title.set_color(C["text"])
    for spine in ax.spines.values():
        spine.set_edgecolor(C["border"])


def shade_outage(ax, df):
    """
    Shade every GNSS-OFF period on a time-series axes with a translucent
    red band and a vertical dashed line at each transition.
    """
    in_off = False
    start_t = None
    for _, row in df.iterrows():
        if row["gnss_status"] == "OFF" and not in_off:
            in_off  = True
            start_t = row["time_sec"]
        elif row["gnss_status"] == "ON" and in_off:
            in_off  = False
            ax.axvspan(start_t, row["time_sec"],
                       alpha=0.12, color=C["outage_bg"], zorder=0)
            ax.axvline(start_t,          color=C["gnss_off"], lw=0.8,
                       linestyle="--", alpha=0.7)
            ax.axvline(row["time_sec"],  color=C["gnss_on"],  lw=0.8,
                       linestyle="--", alpha=0.7)
    if in_off and start_t is not None:
        ax.axvspan(start_t, df["time_sec"].iloc[-1],
                   alpha=0.12, color=C["outage_bg"], zorder=0)


def legend(ax, **kwargs):
    """Styled legend helper."""
    ax.legend(fontsize=7,
              facecolor=C["surface"],
              edgecolor=C["border"],
              labelcolor=C["text"],
              **kwargs)


# =============================================================================
# INDIVIDUAL PANEL PLOTTERS
# =============================================================================

def plot_speed(ax, df):
    """Panel 1 -- Speed vs Time."""
    ax.plot(df["time_sec"], df["speed_kmh"],
            color=C["speed"], linewidth=1.8, label="Speed (km/h)")
    shade_outage(ax, df)
    ax.set_title("Speed vs Time", fontsize=10, fontweight="bold")
    ax.set_xlabel("Time (s)", fontsize=8)
    ax.set_ylabel("Speed (km/h)", fontsize=8)
    legend(ax)


def plot_acceleration(ax, df):
    """Panel 2 -- Accelerometer Ax / Ay / Az vs Time."""
    ax.plot(df["time_sec"], df["accel_x"], color=C["ax"],
            linewidth=1.2, label="Ax  longitudinal (m/s2)")
    ax.plot(df["time_sec"], df["accel_y"], color=C["ay"],
            linewidth=1.2, label="Ay  lateral (m/s2)")
    ax.plot(df["time_sec"], df["accel_z"], color=C["az"],
            linewidth=1.0, label="Az  vertical incl. ~9.81 m/s2 gravity", alpha=0.8)
    shade_outage(ax, df)
    ax.axhline(9.81, color=C["az"], linewidth=0.6, linestyle=":", alpha=0.5)
    ax.set_title("Accelerometer vs Time  [Az includes gravity ~9.81 m/s2]",
                 fontsize=10, fontweight="bold")
    ax.set_xlabel("Time (s)", fontsize=8)
    ax.set_ylabel("m/s2", fontsize=8)
    legend(ax)


def plot_gyroscope(ax, df):
    """Panel 3 -- Gyroscope Gx / Gy / Gz vs Time."""
    ax.plot(df["time_sec"], df["gyro_x"], color=C["gx"],
            linewidth=1.0, label="Gx  roll (rad/s)", alpha=0.8)
    ax.plot(df["time_sec"], df["gyro_y"], color=C["gy"],
            linewidth=1.0, label="Gy  pitch (rad/s)", alpha=0.8)
    ax.plot(df["time_sec"], df["gyro_z"], color=C["gz"],
            linewidth=1.6, label="Gz  yaw rate (rad/s)")
    shade_outage(ax, df)
    ax.set_title("Gyroscope / Yaw Rate vs Time", fontsize=10, fontweight="bold")
    ax.set_xlabel("Time (s)", fontsize=8)
    ax.set_ylabel("rad/s", fontsize=8)
    legend(ax)


def plot_heading(ax, df):
    """Panel 4 -- Heading vs Time."""
    ax.plot(df["time_sec"], df["heading_deg"],
            color=C["heading"], linewidth=1.6, label="Heading (deg)")
    shade_outage(ax, df)
    ax.set_ylim(-5, 365)
    ax.set_yticks([0, 90, 180, 270, 360])
    ax.set_title("Heading vs Time", fontsize=10, fontweight="bold")
    ax.set_xlabel("Time (s)", fontsize=8)
    ax.set_ylabel("Degrees", fontsize=8)
    legend(ax)


def plot_gnss_status(ax, df):
    """Panel 5 -- GNSS Availability bar."""
    numeric = (df["gnss_status"] == "ON").astype(int)
    colors  = [C["gnss_on"] if v == 1 else C["gnss_off"] for v in numeric]
    bar_w   = (df["time_sec"].iloc[1] - df["time_sec"].iloc[0]) * 0.9 if len(df) > 1 else 0.9
    ax.bar(df["time_sec"], numeric, width=bar_w, color=colors, align="center")
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["OFF", "ON"], fontsize=8)
    ax.set_title("GNSS Availability", fontsize=10, fontweight="bold")
    ax.set_xlabel("Time (s)", fontsize=8)
    on_p  = mpatches.Patch(color=C["gnss_on"],  label="GNSS ON")
    off_p = mpatches.Patch(color=C["gnss_off"], label="GNSS OFF")
    ax.legend(handles=[on_p, off_p], fontsize=7,
              facecolor=C["surface"], edgecolor=C["border"],
              labelcolor=C["text"])


def plot_ground_truth_trajectory(ax, df):
    """
    Panel 6 -- Ground Truth GPS Trajectory.

    Shows the COMPLETE path including periods when GNSS is OFF.
    This is the reference for evaluating Dead Reckoning accuracy.
    NOT available to the navigation algorithm during GNSS outage.
    """
    gt = df[["true_latitude", "true_longitude", "gnss_status"]].copy()
    gt["true_latitude"]  = pd.to_numeric(gt["true_latitude"],  errors="coerce")
    gt["true_longitude"] = pd.to_numeric(gt["true_longitude"], errors="coerce")

    # Segment by GNSS status for colouring
    gt_on  = gt[gt["gnss_status"] == "ON"]
    gt_off = gt[gt["gnss_status"] == "OFF"]

    ax.plot(gt["true_longitude"], gt["true_latitude"],
            color=C["muted"], linewidth=0.8, linestyle=":", label="Full path", alpha=0.5)
    ax.plot(gt_on["true_longitude"],  gt_on["true_latitude"],
            color=C["gt_traj"],  linewidth=1.8, label="GNSS ON period")
    ax.plot(gt_off["true_longitude"], gt_off["true_latitude"],
            color=C["gnss_off"], linewidth=1.8, label="GNSS OFF period (DR zone)")

    if not gt.empty:
        ax.scatter(gt["true_longitude"].iloc[0], gt["true_latitude"].iloc[0],
                   color="#f0f6fc", s=60, zorder=5, label="Start")
        ax.scatter(gt["true_longitude"].iloc[-1], gt["true_latitude"].iloc[-1],
                   color=C["gz"], s=60, zorder=5, label="End")

    ax.set_title("Ground Truth Trajectory  [Evaluation Only]",
                 fontsize=10, fontweight="bold")
    ax.set_xlabel("Longitude (deg)", fontsize=8)
    ax.set_ylabel("Latitude (deg)", fontsize=8)
    legend(ax, loc="best")


def plot_gnss_trajectory(ax, df):
    """
    Panel 7 -- GNSS Measured Trajectory.

    Shows only the positions reported by GNSS. During the outage window
    no position is plotted (blank gap). This is what the navigation
    algorithm actually receives.
    """
    meas = df[["gnss_latitude", "gnss_longitude"]].copy()
    meas["gnss_latitude"]  = pd.to_numeric(meas["gnss_latitude"],  errors="coerce")
    meas["gnss_longitude"] = pd.to_numeric(meas["gnss_longitude"], errors="coerce")

    valid = meas.dropna(subset=["gnss_latitude", "gnss_longitude"])

    # Split into continuous segments (before / after outage) to avoid
    # drawing a line across the gap
    segments = []
    seg = []
    prev_valid = False
    for idx, row in meas.iterrows():
        is_valid = not (pd.isna(row["gnss_latitude"]) or pd.isna(row["gnss_longitude"]))
        if is_valid:
            seg.append((row["gnss_longitude"], row["gnss_latitude"]))
            prev_valid = True
        else:
            if seg:
                segments.append(seg)
                seg = []
            prev_valid = False
    if seg:
        segments.append(seg)

    for i, seg in enumerate(segments):
        lons = [p[0] for p in seg]
        lats = [p[1] for p in seg]
        label = "GNSS measured" if i == 0 else "_nolegend_"
        ax.plot(lons, lats, color=C["meas_traj"], linewidth=1.8, label=label)

    # Annotate the gap
    if df[df["gnss_status"] == "OFF"].shape[0] > 0:
        mid_row = valid.iloc[len(valid) // 2] if len(valid) > 0 else None
        if mid_row is not None:
            ax.annotate(
                "GNSS OFF\n(gap -- DR period)",
                xy=(mid_row["gnss_longitude"], mid_row["gnss_latitude"]),
                xytext=(12, 0), textcoords="offset points",
                fontsize=7, color=C["gnss_off"],
                arrowprops=dict(arrowstyle="->", color=C["gnss_off"], lw=0.8)
            )

    if not valid.empty:
        ax.scatter(valid["gnss_longitude"].iloc[0],  valid["gnss_latitude"].iloc[0],
                   color="#f0f6fc", s=60, zorder=5, label="Start")
        ax.scatter(valid["gnss_longitude"].iloc[-1], valid["gnss_latitude"].iloc[-1],
                   color=C["gz"], s=60, zorder=5, label="End (last fix)")

    ax.set_title("GNSS Measured Trajectory  [Navigation Input]",
                 fontsize=10, fontweight="bold")
    ax.set_xlabel("Longitude (deg)", fontsize=8)
    ax.set_ylabel("Latitude (deg)", fontsize=8)
    legend(ax, loc="best")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="SIH 2026 DR Simulator -- Sensor Dashboard Plotter"
    )
    parser.add_argument("--file", default="sensor_data.csv",
                        help="Path to sensor_data.csv (default: sensor_data.csv)")
    parser.add_argument("--out",  default="sensor_dashboard.png",
                        help="Output image path (default: sensor_dashboard.png)")
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print("\n  [!] File not found: " + args.file)
        print("      Run generate_sensor_data.py first.\n")
        sys.exit(1)

    print("\n  Loading: " + args.file)
    df = pd.read_csv(args.file)

    # Numeric coercions
    for col in ["time_sec", "speed_kmh", "accel_x", "accel_y", "accel_z",
                "gyro_x", "gyro_y", "gyro_z", "heading_deg",
                "true_latitude", "true_longitude",
                "true_speed_mps", "true_heading_deg"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    print("  Rows loaded: " + str(len(df)))
    on_count  = (df["gnss_status"] == "ON").sum()
    off_count = (df["gnss_status"] == "OFF").sum()
    print("  GNSS ON: " + str(on_count) + "  |  GNSS OFF: " + str(off_count))

    # -------------------------------------------------------------------------
    # 7-panel layout
    # -------------------------------------------------------------------------
    fig = plt.figure(figsize=(18, 22), dpi=110)
    fig.patch.set_facecolor(C["bg"])

    fig.suptitle(
        "SIH 2026 -- Intelligent Dead Reckoning System\n"
        "Vehicle Sensor Simulation Dashboard",
        fontsize=13, fontweight="bold", color=C["text"], y=0.995
    )

    gs = gridspec.GridSpec(
        4, 2, figure=fig,
        hspace=0.55, wspace=0.32,
        left=0.07, right=0.97,
        top=0.96, bottom=0.04
    )

    # Row 0 : trajectories
    ax_gt   = fig.add_subplot(gs[0, 0])
    ax_gnss = fig.add_subplot(gs[0, 1])
    # Row 1 : speed + GNSS availability
    ax_spd  = fig.add_subplot(gs[1, 0])
    ax_avail= fig.add_subplot(gs[1, 1])
    # Row 2 : accel (full width)
    ax_acc  = fig.add_subplot(gs[2, :])
    # Row 3 : gyro + heading
    ax_gyr  = fig.add_subplot(gs[3, 0])
    ax_head = fig.add_subplot(gs[3, 1])

    all_axes = [ax_gt, ax_gnss, ax_spd, ax_avail, ax_acc, ax_gyr, ax_head]
    for ax in all_axes:
        style_ax(ax)

    plot_ground_truth_trajectory(ax_gt,   df)
    plot_gnss_trajectory(ax_gnss,         df)
    plot_speed(ax_spd,                    df)
    plot_gnss_status(ax_avail,            df)
    plot_acceleration(ax_acc,             df)
    plot_gyroscope(ax_gyr,                df)
    plot_heading(ax_head,                 df)

    # Footer
    on_off_note = "Red = GNSS OFF  |  Green = GNSS ON  |  "
    note = (
        on_off_note +
        "Ground Truth is for evaluation only -- NOT fed to the DR engine  |  "
        "Simulated data -- not real sensor measurements"
    )
    fig.text(0.5, 0.005, note,
             ha="center", fontsize=6.5, color=C["muted"])

    plt.savefig(args.out, dpi=110, bbox_inches="tight",
                facecolor=C["bg"])
    print("  [OK] Dashboard saved -> " + os.path.abspath(args.out) + "\n")


if __name__ == "__main__":
    main()
