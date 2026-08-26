# -*- coding: utf-8 -*-
"""
generate_sensor_data.py
=======================
Simulated Vehicle Sensor Data Generator
SIH 2026 -- Problem Statement 26168
"AI-ML based Intelligent Dead Reckoning System"

PURPOSE
-------
Provides a physically consistent, evaluation-safe sensor stream for the
Dead Reckoning prototype before real smartphone sensors are integrated.

This module generates two categories of data in every row:

  1. GROUND TRUTH  (true_*)
     -- Internal reference computed from the kinematic model.
     -- Always available for post-run evaluation and drift measurement.
     -- MUST NOT be passed to the Dead Reckoning navigation engine.

  2. SENSOR / GNSS MEASUREMENTS  (gnss_*, accel_*, gyro_*)
     -- Data available to the navigation algorithm.
     -- gnss_latitude / gnss_longitude are BLANK during GNSS outage.
     -- IMU (accel, gyro) remains active at all times.

Data Flow:
  Sensor Data (gnss + IMU)
    --> Dead Reckoning Engine --> Estimated Position
    --> compared against true_latitude / true_longitude
    --> Drift / Error Evaluation

==================================================
COORDINATE FRAME ASSUMPTION (Prototype Scope)
==================================================

  The smartphone coordinate frame is assumed to be ALREADY ALIGNED
  with the vehicle body frame.

  X axis --> forward  (longitudinal)
  Y axis --> lateral  (left / right)
  Z axis --> vertical (up)

  A real implementation will require:
    - Phone orientation estimation (pitch / roll / yaw)
    - Phone-to-vehicle alignment calibration
    - Dynamic gravity subtraction from the Z channel

  These calibration steps are out of scope for this prototype.

==================================================
GRAVITY MODEL ASSUMPTION (Prototype Scope)
==================================================

  Raw accelerometer output = true_linear_acceleration
                           + gravity_component
                           + sensor_noise

  For this prototype:
    - Phone Z axis is assumed perfectly vertical
    - gravity component is fixed at 9.81 m/s2 on the Z channel
    - accel_z ~ 9.81 + noise  (does NOT represent pure linear acceleration)

  In a real implementation:
    - Gravity must be subtracted before integrating accel for velocity
    - This requires accurate orientation estimation (e.g., Madgwick filter)

OUTPUT FILES
------------
  sensor_data.csv   -- complete sensor timeline (CSV)
  sensor_data.json  -- same data in JSON format

USAGE
-----
  python generate_sensor_data.py
  python generate_sensor_data.py --rate 10 --duration 90
  python generate_sensor_data.py --rate 10 --duration 90 --outage-start 30 --outage-duration 20
"""

import argparse
import csv
import json
import math
import os
import random
from datetime import datetime, timezone, timedelta


# =============================================================================
# SIMULATION CONFIGURATION  -- edit these values to change the simulation
# =============================================================================

EARTH_RADIUS_M  = 6_378_137.0   # WGS-84 semi-major axis (meters)
GRAVITY         = 9.81           # m/s2 -- gravitational acceleration

# Default GPS origin (Bangalore, India)
ORIGIN_LAT = 12.9716            # decimal degrees
ORIGIN_LON = 77.5946            # decimal degrees

# --------------------------------------------------
# NOISE PARAMETERS  (all Gaussian, zero-mean)
# --------------------------------------------------
# Increase to simulate noisier / cheaper sensors.
# Decrease to simulate higher-quality sensors.
# Do not set to zero -- the DR engine must demonstrate accumulated error.
ACCEL_NOISE_STD = 0.05          # m/s2  -- accelerometer white noise std-dev
GYRO_NOISE_STD  = 0.002         # rad/s -- gyroscope white noise std-dev
GNSS_NOISE_STD  = 0.000005      # degrees (~0.5 m horizontal 1-sigma error)

# Default simulation rate (Hz)
# 10 Hz is the recommended prototype rate.
# 1 Hz is available for quick visualization.
DEFAULT_RATE_HZ = 10

# Default GNSS outage window (seconds)
DEFAULT_OUTAGE_START    = 30.0  # seconds from t=0
DEFAULT_OUTAGE_DURATION = 20.0  # seconds of GNSS blackout


# =============================================================================
# COORDINATE CONVERSION
# =============================================================================

def enu_to_latlon(east_m, north_m, origin_lat, origin_lon):
    """
    Convert local ENU (East-North-Up) offsets in metres to
    geographic (latitude, longitude) in decimal degrees (WGS-84).

    east_m  -- displacement east of origin in metres
    north_m -- displacement north of origin in metres
    """
    lat = origin_lat + math.degrees(north_m / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
    )
    return lat, lon


# =============================================================================
# VEHICLE MOTION PROFILES
# =============================================================================

def speed_profile(t, duration):
    """
    Piecewise true vehicle speed (m/s) as a function of elapsed time.

    The profile is designed to exercise:
      - Acceleration phase        (0 to 10 s)
      - Steady cruise             (10 to 30 s)  ~50 km/h
      - GNSS-OFF driving period   (30 to 50 s)  ramp to ~60 km/h
      - Continued cruise          (50 to 70 s)
      - Urban deceleration        (70 to 80 s)
      - Final braking to stop     (80 to 90 s)

    For durations beyond 90 s the vehicle remains stationary.
    """
    if t < 10.0:
        return 1.39 * t                              # 0 -> 13.9 m/s over 10 s
    elif t < 30.0:
        return 13.9                                  # cruise ~50 km/h
    elif t < 45.0:
        return 13.9 + (16.7 - 13.9) * (t - 30.0) / 15.0  # ramp to ~60 km/h
    elif t < 70.0:
        return 16.7                                  # cruise ~60 km/h
    elif t < 80.0:
        return 16.7 - (16.7 - 8.0) * (t - 70.0) / 10.0   # decelerate
    elif t < 90.0:
        return max(0.0, 8.0 - 8.0 * (t - 80.0) / 10.0)   # brake to stop
    else:
        return 0.0


def yaw_rate_profile(t):
    """
    True vehicle yaw rate omega (rad/s) -- represents actual steering angle.

    Three turns are included:
      10-18 s : gentle right turn  (+0.05 rad/s)
      25-32 s : gentle left turn   (-0.04 rad/s)
      60-68 s : moderate right turn(+0.07 rad/s)
      All other times: straight road (0 rad/s)

    Heading is integrated from this profile, so there are NO discontinuities.
    """
    if 10.0 <= t < 18.0:
        return 0.05
    elif 25.0 <= t < 32.0:
        return -0.04
    elif 60.0 <= t < 68.0:
        return 0.07
    else:
        return 0.0


# =============================================================================
# GNSS AVAILABILITY
# =============================================================================

def is_gnss_available(t, outage_start, outage_duration):
    """
    Return True if GNSS signal is available at time t.

    The GNSS outage window is [outage_start, outage_start + outage_duration).
    Outside this window the signal is always ON.

    outage_start    -- seconds from t=0 when outage begins
    outage_duration -- seconds of continuous outage
    """
    outage_end = outage_start + outage_duration
    return not (outage_start <= t < outage_end)


# =============================================================================
# NOISE HELPER
# =============================================================================

def gauss(std):
    """Gaussian (normal) random sample, zero mean, given std deviation."""
    return random.gauss(0.0, std)


# =============================================================================
# VALIDATION
# =============================================================================

def validate(records, outage_start, outage_duration, dt):
    """
    Perform consistency checks on the generated records.

    Checks performed:
      1. Timestamp monotonicity
      2. Speed is non-negative
      3. Heading in [0, 360)
      4. GNSS OFF window matches configured parameters
      5. True lat/lon always present and valid
      6. GNSS lat/lon blank during outage, present outside outage
      7. IMU values finite at all times
    """
    errors = []
    outage_end = outage_start + outage_duration

    for i, r in enumerate(records):
        t = r["time_sec"]

        # 1. Speed >= 0
        if r["true_speed_mps"] < 0.0:
            errors.append("Row " + str(i) + ": negative speed at t=" + str(t))

        # 2. Heading in [0, 360)
        h = r["heading_deg"]
        if not (0.0 <= h < 360.0):
            errors.append("Row " + str(i) + ": heading out of range at t=" + str(t))

        # 3. True lat/lon must always be valid numbers
        if r["true_latitude"] == "" or r["true_longitude"] == "":
            errors.append("Row " + str(i) + ": true_lat/lon missing at t=" + str(t))

        # 4. GNSS fields during outage
        if outage_start <= t < outage_end:
            if r["gnss_status"] != "OFF":
                errors.append("Row " + str(i) + ": gnss_status should be OFF at t=" + str(t))
            if r["gnss_latitude"] != "" or r["gnss_longitude"] != "":
                errors.append("Row " + str(i) + ": gnss lat/lon should be blank at t=" + str(t))
        else:
            if r["gnss_status"] != "ON":
                errors.append("Row " + str(i) + ": gnss_status should be ON at t=" + str(t))
            if r["gnss_latitude"] == "" or r["gnss_longitude"] == "":
                errors.append("Row " + str(i) + ": gnss lat/lon missing during ON period at t=" + str(t))

        # 5. IMU must always be finite
        for imu_col in ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]:
            val = r[imu_col]
            try:
                if not math.isfinite(float(val)):
                    errors.append("Row " + str(i) + ": non-finite " + imu_col + " at t=" + str(t))
            except (TypeError, ValueError):
                errors.append("Row " + str(i) + ": non-numeric " + imu_col + " at t=" + str(t))

    # 6. Timestamp monotonicity
    for i in range(1, len(records)):
        if records[i]["time_sec"] <= records[i - 1]["time_sec"]:
            errors.append("Timestamps not monotonic at row " + str(i))

    return errors


# =============================================================================
# MAIN SIMULATION LOOP
# =============================================================================

def simulate(duration=90.0, sample_rate=10.0,
             origin_lat=ORIGIN_LAT, origin_lon=ORIGIN_LON,
             outage_start=DEFAULT_OUTAGE_START,
             outage_duration=DEFAULT_OUTAGE_DURATION,
             seed=42):
    """
    Run the kinematic vehicle simulation and return a list of records.

    Parameters
    ----------
    duration        : Total simulation time in seconds.
    sample_rate     : Samples per second (Hz). 10 Hz recommended for prototype.
    origin_lat      : GPS origin latitude (decimal degrees).
    origin_lon      : GPS origin longitude (decimal degrees).
    outage_start    : Time (seconds) when GNSS outage begins.
    outage_duration : Duration (seconds) of the GNSS outage.
    seed            : Random seed for reproducible noise.

    Returns
    -------
    list of dicts  -- one dict per sample step.

    GROUND TRUTH RULE
    -----------------
    true_latitude / true_longitude are computed at every step.
    They represent the exact vehicle position from the kinematic model.
    They MUST NOT be passed to the Dead Reckoning navigation algorithm.
    They exist only so the evaluation module can compute position error.

    gnss_latitude / gnss_longitude are the SENSOR MEASUREMENTS.
    They are blank (empty string) during the GNSS outage window.
    """
    random.seed(seed)

    dt = 1.0 / sample_rate
    num_steps = int(round(duration * sample_rate))
    start_utc = datetime(2025, 1, 15, 6, 0, 0, tzinfo=timezone.utc)

    # -------------------------------------------------------------------------
    # Vehicle state (kinematic integration)
    # -------------------------------------------------------------------------
    x_enu     = 0.0   # East displacement (metres)
    y_enu     = 0.0   # North displacement (metres)
    heading   = 0.0   # Heading angle (radians); integrated from yaw rate
                       # 0 rad = pointing North (positive Y direction)

    records = []

    for step in range(num_steps + 1):
        t = round(step * dt, 6)   # avoid float drift
        timestamp = start_utc + timedelta(seconds=t)

        # -------------------------------------------------------------------------
        # True kinematics
        # -------------------------------------------------------------------------
        v_true     = speed_profile(t, duration)
        omega_true = yaw_rate_profile(t)

        # Longitudinal acceleration: finite difference of true speed
        if step == 0:
            v_prev = speed_profile(0.0, duration)
        else:
            v_prev = speed_profile(round((step - 1) * dt, 6), duration)

        long_accel_true = (v_true - v_prev) / dt   # m/s2
        lat_accel_true  = v_true * omega_true       # centripetal m/s2

        # Integrate heading (no discontinuities)
        heading += omega_true * dt
        # Normalise to [0, 2*pi) so heading_deg is always in [0, 360)
        heading = heading % (2.0 * math.pi)

        # Integrate position in ENU frame
        # heading=0 points North (aligned with +Y), so:
        #   dx = v * sin(heading)   (East component)
        #   dy = v * cos(heading)   (North component)
        x_enu += v_true * math.sin(heading) * dt
        y_enu += v_true * math.cos(heading) * dt

        # -------------------------------------------------------------------------
        # GROUND TRUTH position (always computed; for evaluation only)
        # -------------------------------------------------------------------------
        true_lat, true_lon = enu_to_latlon(x_enu, y_enu, origin_lat, origin_lon)

        # -------------------------------------------------------------------------
        # GNSS MEASUREMENT (may be unavailable during outage)
        # -------------------------------------------------------------------------
        gnss_on = is_gnss_available(t, outage_start, outage_duration)

        if gnss_on:
            # Add realistic GNSS position noise (~0.5 m horizontal 1-sigma)
            gnss_lat = true_lat + gauss(GNSS_NOISE_STD)
            gnss_lon = true_lon + gauss(GNSS_NOISE_STD)
            gnss_lat_str = str(round(gnss_lat, 7))
            gnss_lon_str = str(round(gnss_lon, 7))
            gnss_status_str = "ON"
        else:
            # GNSS signal lost -- Dead Reckoning period
            # gnss_latitude and gnss_longitude are BLANK.
            # true_latitude and true_longitude are still computed but
            # must not be visible to the DR navigation algorithm.
            gnss_lat_str    = ""
            gnss_lon_str    = ""
            gnss_status_str = "OFF"

        # -------------------------------------------------------------------------
        # IMU MEASUREMENTS
        # (always active -- not affected by GNSS outage)
        # -------------------------------------------------------------------------

        # -- Accelerometer (body frame) --
        # IMPORTANT: raw accelerometer includes the gravity component.
        # accel_z ~ 9.81 m/s2 because gravity acts on the Z axis.
        # The DR engine must compensate for gravity before integrating accel_z.
        # For this prototype the phone Z axis is assumed perfectly vertical.
        #
        # accel_x: longitudinal (forward/back) = true linear acceleration + noise
        # accel_y: lateral (left/right)        = centripetal acceleration + noise
        # accel_z: vertical (up)               = gravity + road vibration + noise
        ax = long_accel_true + gauss(ACCEL_NOISE_STD)
        ay = lat_accel_true  + gauss(ACCEL_NOISE_STD)
        az = GRAVITY         + gauss(ACCEL_NOISE_STD * 2.0)  # road roughness noise

        # -- Gyroscope (body frame) --
        # gyro_z (yaw rate) is the most important for DR heading estimation.
        # gyro_x (roll rate) and gyro_y (pitch rate) are noise-only for a
        # flat road assumption used in this prototype.
        gx = gauss(GYRO_NOISE_STD)               # roll rate (noise only)
        gy = gauss(GYRO_NOISE_STD)               # pitch rate (noise only)
        gz = omega_true + gauss(GYRO_NOISE_STD)  # yaw rate = true + noise

        records.append({
            # -- Timestamp --
            "timestamp":        timestamp.strftime("%Y-%m-%dT%H:%M:%S.") +
                                "{:03d}".format(int((t % 1) * 1000)) + "Z",
            "time_sec":         round(t, 3),

            # -- GROUND TRUTH (evaluation use only; DO NOT feed to DR engine) --
            "true_latitude":    round(true_lat, 7),
            "true_longitude":   round(true_lon, 7),
            "true_speed_mps":   round(v_true, 4),
            "true_heading_deg": round(math.degrees(heading) % 360.0, 3),

            # -- GNSS SENSOR MEASUREMENT (available to navigation algorithm) --
            "gnss_latitude":    gnss_lat_str,
            "gnss_longitude":   gnss_lon_str,
            "gnss_status":      gnss_status_str,

            # -- IMU MEASUREMENTS (always available to navigation algorithm) --
            # accel_z includes ~9.81 m/s2 gravity component (see notes above)
            "accel_x":          round(ax, 5),
            "accel_y":          round(ay, 5),
            "accel_z":          round(az, 5),
            "gyro_x":           round(gx, 6),
            "gyro_y":           round(gy, 6),
            "gyro_z":           round(gz, 6),

            # -- Derived / convenience columns --
            "heading_deg":      round(math.degrees(heading) % 360.0, 3),
            "speed_kmh":        round(v_true * 3.6, 4),
        })

    return records


# =============================================================================
# EXPORT FUNCTIONS
# =============================================================================

CSV_FIELDS = [
    "timestamp", "time_sec",
    "true_latitude", "true_longitude", "true_speed_mps", "true_heading_deg",
    "gnss_latitude", "gnss_longitude", "gnss_status",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "heading_deg", "speed_kmh",
]


def export_csv(records, path):
    """Write records to a UTF-8 CSV file with the required schema."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(records)
    print("  [OK] CSV  exported -> " + os.path.abspath(path))


def export_json(records, path):
    """Write records to a UTF-8 JSON file (pretty-printed)."""
    with open(path, "w", encoding="utf-8") as f:
        json.dump(records, f, indent=2)
    print("  [OK] JSON exported -> " + os.path.abspath(path))


# =============================================================================
# SUMMARY & VALIDATION REPORT
# =============================================================================

def print_summary(records, outage_start, outage_duration, dt):
    """Print validation results and a preview table to stdout."""

    total       = len(records)
    on_count    = sum(1 for r in records if r["gnss_status"] == "ON")
    off_count   = sum(1 for r in records if r["gnss_status"] == "OFF")
    max_spd_kmh = max(r["speed_kmh"] for r in records)

    # GNSS transitions
    transitions = []
    for i in range(1, total):
        if records[i]["gnss_status"] != records[i - 1]["gnss_status"]:
            transitions.append((records[i]["time_sec"], records[i]["gnss_status"]))

    print("\n" + "=" * 65)
    print("  VEHICLE SENSOR DATA -- SIMULATION SUMMARY")
    print("=" * 65)
    print("  Total samples        : " + str(total))
    print("  Duration             : " + str(records[-1]["time_sec"]) + " s")
    print("  GNSS ON  samples     : " + str(on_count))
    print("  GNSS OFF samples     : " + str(off_count))
    print("  Peak speed           : " + str(round(max_spd_kmh, 1)) + " km/h")
    print("  GNSS outage window   : " + str(outage_start) + " s to " +
          str(outage_start + outage_duration) + " s")
    print("  GNSS transitions     : " + str(len(transitions)))
    for ts, st in transitions:
        print("      @ t=" + str(ts) + " s  ->  GNSS " + st)
    print("=" * 65)

    # Validation
    print("\n  Running validation checks...")
    errors = validate(records, outage_start, outage_duration, dt)
    if errors:
        print("  [FAIL] Validation found " + str(len(errors)) + " issue(s):")
        for e in errors[:10]:
            print("    - " + e)
    else:
        print("  [PASS] All " + str(total) + " rows passed validation.")
    print()

    # Preview table header
    print("  {:<8}  {:<12}  {:<12}  {:>9}  {:>7}  {:>7}  {:>7}  {:>8}  {:>5}".format(
        "Time(s)", "TrueLat", "TrueLon", "Spd km/h", "Ax", "Ay", "Az", "Gz", "GNSS"
    ))
    print("  " + "-" * 80)

    # First 10 rows
    for r in records[:10]:
        print("  {:<8}  {:<12}  {:<12}  {:>9.2f}  {:>7.3f}  {:>7.3f}  {:>7.3f}  {:>8.5f}  {:>5}".format(
            r["time_sec"],
            str(r["true_latitude"]), str(r["true_longitude"]),
            r["speed_kmh"], r["accel_x"], r["accel_y"], r["accel_z"],
            r["gyro_z"], r["gnss_status"]
        ))

    # Show a mid-outage row
    off_rows = [r for r in records if r["gnss_status"] == "OFF"]
    if off_rows:
        print("  ...")
        r = off_rows[len(off_rows) // 2]
        gps_label = "[GNSS OFF -- blank]"
        print("  {:<8}  {:<25}  {:>9.2f}  {:>7.3f}  {:>7.3f}  {:>7.3f}  {:>8.5f}  {:>5}".format(
            r["time_sec"], gps_label,
            r["speed_kmh"], r["accel_x"], r["accel_y"], r["accel_z"],
            r["gyro_z"], r["gnss_status"]
        ))
    print("  ...")
    print()


# =============================================================================
# ARGUMENT PARSING
# =============================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="SIH 2026 -- Intelligent Dead Reckoning System: Vehicle Sensor Simulator"
    )
    parser.add_argument(
        "--duration", type=float, default=90.0,
        help="Total simulation duration in seconds (default: 90)"
    )
    parser.add_argument(
        "--rate", type=float, default=float(DEFAULT_RATE_HZ),
        help="Sample rate in Hz (default: " + str(DEFAULT_RATE_HZ) + " Hz)"
    )
    parser.add_argument(
        "--outage-start", type=float, default=DEFAULT_OUTAGE_START,
        help="Time (seconds) when GNSS outage begins (default: " + str(DEFAULT_OUTAGE_START) + ")"
    )
    parser.add_argument(
        "--outage-duration", type=float, default=DEFAULT_OUTAGE_DURATION,
        help="Duration (seconds) of GNSS outage (default: " + str(DEFAULT_OUTAGE_DURATION) + ")"
    )
    parser.add_argument(
        "--origin-lat", type=float, default=ORIGIN_LAT,
        help="GPS origin latitude (default: " + str(ORIGIN_LAT) + ")"
    )
    parser.add_argument(
        "--origin-lon", type=float, default=ORIGIN_LON,
        help="GPS origin longitude (default: " + str(ORIGIN_LON) + ")"
    )
    parser.add_argument(
        "--seed", type=int, default=42,
        help="Random seed for reproducible noise (default: 42)"
    )
    parser.add_argument(
        "--out-dir", type=str, default=".",
        help="Output directory for CSV and JSON (default: current directory)"
    )
    return parser.parse_args()


# =============================================================================
# ENTRY POINT
# =============================================================================

def main():
    args = parse_args()
    dt = 1.0 / args.rate

    print("\n  SIH 2026 -- Intelligent Dead Reckoning System")
    print("  Vehicle Sensor Data Simulator")
    print("  " + "-" * 50)
    print("  Duration       : " + str(args.duration) + " s")
    print("  Sample rate    : " + str(args.rate) + " Hz  (" + str(round(dt * 1000)) + " ms intervals)")
    print("  GNSS outage    : t=" + str(args.outage_start) + " s  for " +
          str(args.outage_duration) + " s  (until t=" +
          str(args.outage_start + args.outage_duration) + " s)")
    print("  GPS origin     : (" + str(args.origin_lat) + ", " + str(args.origin_lon) + ")")
    print("  Seed           : " + str(args.seed))
    print("  Output dir     : " + os.path.abspath(args.out_dir))
    print()

    os.makedirs(args.out_dir, exist_ok=True)

    print("  Simulating vehicle motion...")
    records = simulate(
        duration        = args.duration,
        sample_rate     = args.rate,
        origin_lat      = args.origin_lat,
        origin_lon      = args.origin_lon,
        outage_start    = args.outage_start,
        outage_duration = args.outage_duration,
        seed            = args.seed,
    )

    csv_path  = os.path.join(args.out_dir, "sensor_data.csv")
    json_path = os.path.join(args.out_dir, "sensor_data.json")

    export_csv(records, csv_path)
    export_json(records, json_path)

    print_summary(records, args.outage_start, args.outage_duration, dt)
    print("  Done. Use plot_sensor_data.py to visualize the output.\n")


if __name__ == "__main__":
    main()
