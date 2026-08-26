# Jeffryn Dead Reckoning Engine

**PS 26168 -- Intelligent Vehicle Navigation During GNSS Outages**

> Author: **Jeffryn Adaikalaraj A**
> Module: Basic Dead Reckoning Engine (Prototype 30-40%)

---

## 1. Problem Statement

When a vehicle loses GNSS (GPS) signal -- in tunnels, urban canyons, or areas
with poor satellite coverage -- the navigation system has no external position
reference.  This module provides a **physics-based Dead Reckoning (DR) engine**
that estimates the vehicle's position using inertial sensors (accelerometer,
gyroscope) and the last known GNSS fix, until the GNSS signal is restored.

## 2. Jeffryn's Responsibility

Design, implement, test, and document a **modular, integration-ready Dead
Reckoning engine** that:

- Initialises from GNSS.
- Detects GNSS ON/OFF transitions.
- Integrates gyroscope data to update heading.
- Estimates velocity from sensor speed and/or accelerometer.
- Propagates vehicle position in local East/North metres.
- Converts local displacement back to latitude/longitude.
- Measures reconnection error when GNSS returns.
- Exports a complete estimated trajectory CSV.
- Provides a clean Python API for FastAPI integration.

## 3. Module Architecture

```
src/dead_reckoning/
  __init__.py         Public API: DeadReckoningEngine, load_config
  engine.py           Core state machine and DR logic
  state.py            VehicleState dataclass
  coordinates.py      Geodetic conversions (Haversine, GPS <-> local)
  preprocessing.py    Timestamp parsing, GNSS normalisation, dt
  validation.py       Config and sensor-data validation
  models.py           NavigationMode enum, column schemas
  exceptions.py       Custom exception hierarchy
```

## 4. Input Data Schema

| Column           | Type         | Unit     | Description              |
|------------------|--------------|----------|--------------------------|
| `timestamp`      | float/string | s / ISO  | Sensor timestamp         |
| `gps_latitude`   | float        | degrees  | GPS latitude             |
| `gps_longitude`  | float        | degrees  | GPS longitude            |
| `speed_mps`      | float        | m/s      | Vehicle speed            |
| `accel_x`        | float        | m/s^2    | Acceleration X-axis      |
| `accel_y`        | float        | m/s^2    | Acceleration Y-axis      |
| `accel_z`        | float        | m/s^2    | Acceleration Z-axis      |
| `gyro_x`         | float        | config   | Angular velocity X       |
| `gyro_y`         | float        | config   | Angular velocity Y       |
| `gyro_z`         | float        | config   | Vehicle yaw rate         |
| `gnss_available` | bool/str/int | --       | ON/OFF, 1/0, True/False  |

**Optional columns:** `heading_deg`, `forward_accel_mps2`, `ground_truth_*`, etc.

## 5. Output Data Schema

The output `estimated_trajectory.csv` contains:

| Column                         | Description                    |
|--------------------------------|--------------------------------|
| `timestamp`                    | Original timestamp             |
| `elapsed_time_s`               | Time from sequence start       |
| `gnss_available`               | Normalised GNSS status         |
| `navigation_mode`              | GNSS / DEAD_RECKONING / etc.   |
| `estimated_latitude`           | Estimated latitude (degrees)   |
| `estimated_longitude`          | Estimated longitude (degrees)  |
| `estimated_x_m`                | Local East displacement (m)    |
| `estimated_y_m`                | Local North displacement (m)   |
| `estimated_velocity_mps`       | Estimated velocity (m/s)       |
| `estimated_speed_kmph`         | Estimated speed (km/h)         |
| `estimated_heading_deg`        | Navigation heading (degrees)   |
| `corrected_gyro_z`             | Bias-corrected yaw rate        |
| `corrected_forward_accel_mps2` | Corrected acceleration         |
| `distance_step_m`              | Distance per step (m)          |
| `cumulative_distance_m`        | Total distance (m)             |
| `outage_elapsed_seconds`       | Current outage duration (s)    |
| `gnss_reconnection_error_m`    | Error at GNSS reacquisition    |
| `data_quality_flags`           | Warning flags                  |

---

## 6. Installation

### Prerequisites

- Python 3.11 or later

### Create a Virtual Environment

```bash
python -m venv venv

# Windows
venv\Scripts\activate

# macOS / Linux
source venv/bin/activate
```

### Install Dependencies

```bash
pip install -r requirements.txt
```

---

## 7. Quick Start

### Generate Test Fixture

```bash
python scripts/generate_test_fixture.py
```

Creates `data/sample_sensor_data.csv` (901 records, 90 seconds).

### Run Dead Reckoning

```bash
python dead_reckoning.py \
  --input data/sample_sensor_data.csv \
  --output outputs/estimated_trajectory.csv \
  --config config.yaml
```

### Generate Development Debugging Plot

> **Note:** Official benchmark comparisons (Ground Truth vs Prediction, MAE, RMSE, Drift %) are the responsibility of **Hari's evaluation module**. The plot below is provided strictly for **development and debugging visualization** of Jeffryn's Dead Reckoning trajectory.

```bash
python scripts/plot_estimated_trajectory.py \
  --trajectory outputs/estimated_trajectory.csv \
  --sensor-data data/sample_sensor_data.csv \
  --show-ground-truth \
  --output outputs/estimated_trajectory.png
```

### Run Tests

```bash
pytest tests/ -v
```

---

## 8. Mathematical Explanation

### Coordinate Convention

- **Local frame:** X = East (metres), Y = North (metres)
- **Heading:** 0 deg = North, 90 deg = East, 180 deg = South, 270 deg = West
- **Positive heading change** = clockwise rotation

### Time Difference

```
dt = current_time - previous_time
```

### Heading Update (Gyroscope Integration)

```
corrected_gyro_z = gyro_z - gyro_bias_z
heading_new = heading_previous + corrected_gyro_z * dt * gyro_yaw_sign
heading_new = normalize(heading_new)    # wrap to [0, 360)
```

### Velocity Update

**Mode A (sensor_speed):** Use `speed_mps` directly.

**Mode B (acceleration):**
```
v_new = v_old + corrected_forward_acceleration * dt
v_new = max(0, v_new)
```

**Mode C (hybrid):**
```
v_accel = v_old + acceleration * dt
v_new = alpha * sensor_speed + (1 - alpha) * v_accel
```

### Distance (Trapezoidal Integration)

```
distance = ((v_previous + v_current) / 2) * dt
```

### Position Propagation

```
delta_east  = distance * sin(heading)
delta_north = distance * cos(heading)

x_new = x_old + delta_east
y_new = y_old + delta_north
```

### Geographic Conversion

```
latitude  = ref_lat + degrees(y_north / R)
longitude = ref_lon + degrees(x_east / (R * cos(radians(ref_lat))))
```

Where `R = 6,371,000 m`.

### Why Drift Occurs

Dead Reckoning accumulates errors because:
- Gyroscope bias causes heading drift
- Accelerometer bias causes velocity drift
- Double integration of acceleration amplifies small errors
- Sensor noise from vibration and road shocks
- Unknown phone orientation makes gravity compensation impossible
- No external correction during GNSS outage

---

## 9. GNSS State Machine

```
UNINITIALIZED --[first valid GNSS]--> GNSS
GNSS --[GNSS lost]--> DEAD_RECKONING
DEAD_RECKONING --[GNSS restored]--> GNSS_REACQUIRED
GNSS_REACQUIRED --[next record]--> GNSS
```

### GNSS ON
- Use GNSS position directly.
- Estimate heading from bearing when displacement > threshold.
- Update velocity and prepare for possible outage.

### GNSS ON -> OFF
- Freeze last reliable GNSS coordinates.
- Keep current heading and velocity.
- Start outage timer.
- Switch to DEAD_RECKONING mode.

### GNSS OFF (Dead Reckoning)
1. Calculate dt
2. Integrate gyro_z to update heading
3. Update velocity (accelerometer or sensor speed)
4. Calculate distance (trapezoidal)
5. Compute East/North displacement
6. Update local coordinates
7. Convert to lat/lon

### GNSS OFF -> ON (Reacquisition)
1. Compute reconnection error (Haversine distance: DR estimate vs. GNSS)
2. Reset position to GNSS fix
3. Set mode to GNSS_REACQUIRED

---

## 10. No-GPS-Leakage Rule

**Critical requirement:** During GNSS OFF, the engine must NOT access:
- `gps_latitude`
- `gps_longitude`
- GPS-derived heading
- GPS-derived speed (if configured as GPS-derived)

GPS data remains in the CSV only for evaluation by other team members.

This is verified by `tests/test_no_gps_leakage.py`, which runs two
identical sensor sequences with corrupted GPS during GNSS OFF and asserts
bitwise-identical DR outputs.

---

## 11. How to Replace Test Data with Arsheen's CSV

1. Place Arsheen's CSV in the `data/` directory.
2. Ensure it has the required columns (see Section 4).
3. Run:

```bash
python dead_reckoning.py \
  --input data/arsheen_sensor_data.csv \
  --output outputs/estimated_trajectory.csv \
  --config config.yaml
```

If the CSV uses different column names, adjust `config.yaml` accordingly.

---

## 12. FastAPI Integration (for Nishitha)

```python
from src.dead_reckoning import DeadReckoningEngine, load_config

engine = DeadReckoningEngine(load_config("config.yaml"))

def process_live_sensor(sensor_record: dict) -> dict:
    """Process a single sensor reading and return estimated state."""
    return engine.process_sensor_record(sensor_record)
```

For batch processing:

```python
import pandas as pd

df = pd.read_csv("data/sensor_data.csv")
result = engine.process_dataframe(df)
```

All returned dictionaries are JSON-serializable (no NumPy types).

---

## 13. Output for Hari's Metrics

Hari can compare the estimated trajectory against ground truth:

```python
import pandas as pd

traj = pd.read_csv("outputs/estimated_trajectory.csv")
sensor = pd.read_csv("data/sample_sensor_data.csv")

# Merge on timestamp
merged = traj.merge(
    sensor[["timestamp", "ground_truth_latitude", "ground_truth_longitude"]],
    on="timestamp",
)

# Calculate error for each row
# ... (MAE, RMSE, drift analysis)
```

---

## 14. Known Limitations

- **Smartphone Orientation Assumption:** The prototype requires the smartphone to be mounted in a **known, fixed orientation** where the configured accelerometer axis (e.g. `accel_x`) points in the forward travel direction. Arbitrary phone orientations or shifts during driving will cause gravity contamination and projection errors.
- **Accelerometer Bias:** Uncorrected residual bias causes quadratic velocity drift over time.
- **Gyroscope Bias:** Residual bias causes linear heading drift over time.
- **Double Integration:** Double integrating accelerometer data amplifies high-frequency noise and road vibration.
- **No EKF / UKF:** No probabilistic sensor fusion filter is implemented during the outage.
- **No Map Matching:** Trajectory is calculated purely from inertial kinematics without road-network snapping.
- **No Vehicle Dynamics Model:** Does not use a non-holonomic bicycle or Ackermann steering constraint.
- **GPS-Derived Speed:** If speed is GPS-derived, it is disabled during outages to prevent data leakage, relying strictly on accelerometer integration.
- **Long Outages:** Outages exceeding 30–60 seconds will accumulate measurable drift in an open-loop system.

This is an honest baseline prototype (30-40%). Do not expect centimetre-level
or production-level accuracy.

---

## 15. Future Improvements

The architecture supports adding:
- CNN + GRU motion estimator
- EKF or UKF sensor fusion
- Smartphone orientation compensation
- Magnetometer heading
- Wheel-speed / OBD input
- Zero-velocity update (ZUPT)
- Map matching
- Advanced vehicle constraints
- FastAPI live streaming
- Web dashboard
- Mobile navigation

---

## 16. Project Structure

```
jeffryn-dead-reckoning/
  dead_reckoning.py         CLI entry point
  config.yaml               Engine configuration
  requirements.txt          Dependencies
  README.md                 This file
  .gitignore                Git ignore patterns
  src/dead_reckoning/       Core engine modules
  scripts/                  Fixture generator, demo, plotter
  data/                     Sensor data (sample + real)
  outputs/                  Generated trajectory & plots
  tests/                    Automated test suite (90 tests)
```

---

## License

Academic project -- PS 26168.
