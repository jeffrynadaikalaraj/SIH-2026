# SIH 2026 -- Intelligent Dead Reckoning System
## Vehicle Sensor Simulator

**Problem Statement 26168** | Smart India Hackathon 2026

---

## Purpose

This simulator provides a physically consistent, evaluation-safe vehicle
sensor stream for the **Dead Reckoning prototype** before real smartphone
sensors are integrated.

The generated data feeds directly into the next module:

```
Sensor Data (GNSS + IMU)
    --> Dead Reckoning Engine
        --> Estimated Position
            --> Ground Truth Comparison
                --> Drift / Error Evaluation
```

---

## Prototype Assumptions

### Phone / Vehicle Coordinate Frame

> **Assumption: The smartphone coordinate frame is already aligned with the
> vehicle body frame.**

For this prototype:

| Axis | Direction       |
|------|-----------------|
| X    | Forward (longitudinal) |
| Y    | Lateral (left / right) |
| Z    | Vertical (up)          |

A real implementation will require:
- Phone orientation estimation (pitch / roll / yaw)
- Phone-to-vehicle alignment calibration
- Dynamic gravity subtraction from the Z channel

These calibration steps are **out of scope for the current prototype**.

---

## Sensor Model

### Accelerometer

```
accel_x  =  true_longitudinal_acceleration  +  noise
accel_y  =  true_lateral_acceleration       +  noise
accel_z  =  gravity (~9.81 m/s2)            +  noise
```

Raw accelerometer output includes the gravity component on the Z axis.

The Dead Reckoning engine **must subtract gravity before integrating accel_z**
to obtain vertical linear acceleration. For a flat-road prototype, accel_z
can be ignored for horizontal position estimation.

### Gyroscope

```
gyro_x  =  roll rate   +  noise  (near zero for flat road)
gyro_y  =  pitch rate  +  noise  (near zero for flat road)
gyro_z  =  yaw rate    +  noise  (primary DR input for heading)
```

Heading is integrated from the true yaw rate. Gyro_z is the most important
signal for estimating heading change during GNSS outage.

### Gravity Assumption

- accel_z approximately equals 9.81 m/s2 because gravity acts on the Z axis.
- This is valid ONLY when the phone Z axis is vertical.
- Real smartphone data will have gravity components on all three axes
  depending on phone orientation.

---

## GNSS Simulation

GNSS positions include realistic horizontal noise (~0.5 m, 1-sigma).
During the outage window, **gnss_latitude and gnss_longitude are blank**.

The outage simulates:
- Tunnel entry/exit
- Urban canyon signal blockage
- Any environment that blocks satellite signals

---

## Ground Truth vs GNSS

| Column              | Available to DR Engine? | Purpose               |
|---------------------|------------------------|-----------------------|
| `true_latitude`     | NO (evaluation only)   | Compute position error|
| `true_longitude`    | NO (evaluation only)   | Compute position error|
| `true_speed_mps`    | NO (evaluation only)   | Compute speed error   |
| `true_heading_deg`  | NO (evaluation only)   | Compute heading error |
| `gnss_latitude`     | YES (when GNSS ON)     | Navigation input      |
| `gnss_longitude`    | YES (when GNSS ON)     | Navigation input      |
| `gnss_status`       | YES                    | Navigation input      |
| `accel_x/y/z`       | YES (always)           | Navigation input      |
| `gyro_x/y/z`        | YES (always)           | Navigation input      |

**Critical Rule:**  
True values are for evaluation only. Do NOT pass `true_*` columns to the  
Dead Reckoning engine. This prevents data leakage in accuracy measurements.

---

## CSV Schema

| Column            | Unit     | Description                          |
|-------------------|----------|--------------------------------------|
| `timestamp`       | ISO 8601 | UTC timestamp                        |
| `time_sec`        | seconds  | Elapsed simulation time              |
| `true_latitude`   | degrees  | WGS-84 latitude (ground truth)       |
| `true_longitude`  | degrees  | WGS-84 longitude (ground truth)      |
| `true_speed_mps`  | m/s      | True vehicle speed                   |
| `true_heading_deg`| degrees  | True heading [0, 360)                |
| `gnss_latitude`   | degrees  | GNSS measured latitude (blank if OFF)|
| `gnss_longitude`  | degrees  | GNSS measured longitude (blank if OFF)|
| `gnss_status`     | ON / OFF | Satellite lock status                |
| `accel_x`         | m/s2     | Longitudinal acceleration + noise    |
| `accel_y`         | m/s2     | Lateral acceleration + noise         |
| `accel_z`         | m/s2     | Vertical acceleration + gravity + noise|
| `gyro_x`          | rad/s    | Roll rate + noise                    |
| `gyro_y`          | rad/s    | Pitch rate + noise                   |
| `gyro_z`          | rad/s    | Yaw rate + noise (DR heading input)  |
| `heading_deg`     | degrees  | True heading [0, 360)                |
| `speed_kmh`       | km/h     | True speed (convenience column)      |

---

## GNSS ON -> OFF -> ON Scenario

Default configuration (outage-start=30, outage-duration=20):

```
t =  0 s  -->  t = 29.9 s   GNSS ON   (normal GPS lock)
t = 30 s  -->  t = 49.9 s   GNSS OFF  (tunnel / blackout)
t = 50 s  -->  t = 90.0 s   GNSS ON   (signal restored)
```

During GNSS OFF:
- `gnss_latitude`, `gnss_longitude` are blank
- `accel_x/y/z`, `gyro_x/y/z` remain active
- `true_latitude`, `true_longitude` are still computed but hidden from DR engine

---

## Noise Parameters

Defined at the top of `generate_sensor_data.py`:

| Parameter        | Default      | Description                          |
|------------------|--------------|--------------------------------------|
| `ACCEL_NOISE_STD`| 0.05 m/s2    | Accelerometer white noise            |
| `GYRO_NOISE_STD` | 0.002 rad/s  | Gyroscope white noise                |
| `GNSS_NOISE_STD` | 0.000005 deg | GNSS position noise (~0.5 m)         |

---

## Running Commands

### Quick 1 Hz run (90 seconds, default outage)
```bash
python generate_sensor_data.py
```

### Recommended prototype run (10 Hz, 90 seconds)
```bash
python generate_sensor_data.py --rate 10 --duration 90
```

### Custom outage window
```bash
python generate_sensor_data.py --rate 10 --duration 90 --outage-start 30 --outage-duration 20
```

### Custom GPS origin (New Delhi)
```bash
python generate_sensor_data.py --rate 10 --origin-lat 28.6139 --origin-lon 77.2090
```

### Longer run (120 seconds)
```bash
python generate_sensor_data.py --rate 10 --duration 120
```

### Regenerate visualizations
```bash
python plot_sensor_data.py --file sensor_data.csv --out sensor_dashboard.png
```

---

## Output Files

| File                   | Description                                    |
|------------------------|------------------------------------------------|
| `sensor_data.csv`      | Sensor timeline (CSV, UTF-8)                   |
| `sensor_data.json`     | Same data in JSON format (API integration)     |
| `sensor_dashboard.png` | 7-panel visualization dashboard                |

---

## Prototype Scope

This module is **Phase 1 only**.

**Not implemented** (reserved for Phase 2):
- CNN / GRU / LSTM neural network models
- Extended / Unscented Kalman Filter (EKF / UKF)
- Map matching
- Real smartphone sensor APIs (Android / iOS)
- Cloud services
- Phone orientation calibration

---

## Dependencies

```bash
pip install matplotlib pandas numpy
```

Python 3.10+ recommended.
