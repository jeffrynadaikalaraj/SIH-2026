# Jeffryn Adaikalaraj A — Module Summary & Pitch Sheet

**Project:** PS 26168 — Intelligent Vehicle Navigation During GNSS Outages  
**Assigned Module:** Basic Dead Reckoning Engine (Baseline Prototype 30–40%)  
**Engine Version:** 1.0.0-PROTOTYPE  
**Location:** `D:\SIH\jeffryn-dead-reckoning\`

---

## 1. 2-Minute Evaluator Pitch

> *"In GPS-denied environments like tunnels and urban canyons, standard navigation fails. I designed, implemented, tested, and validated the **Dead Reckoning Engine** module for PS 26168.
>
> When satellite coverage is lost, my engine automatically detects the outage and takes over navigation using calibrated gyroscope integration for vehicle heading, combined with trapezoidal velocity integration to propagate local metric coordinates ($X, Y$), which are transformed to geographic Latitude and Longitude.
>
> On our 90-second benchmark journey featuring a 30-second outage with straight driving and turning maneuvers, the engine achieved a reconnection accuracy of **1.36 metres** with zero GPS leakage, verified by 90 automated pytest tests."*

---

## 2. Technical Architecture & State Machine

```
   [GNSS ON] ──(Signal Loss)──> [DEAD_RECKONING] ──(Signal Restored)──> [GNSS_REACQUIRED]
       │                               │                                     │
  Update GPS &                  Integrate Gyro (ψ)                    Calculate Reconnection
  Calibrate Biases              Integrate Velocity (v)                 Error (1.36 m) & Reset
                                Propagate Local (X, Y)                       │
                                Convert to Lat / Lon                         ▼
                                                                        [GNSS ON]
```

---

## 3. Key Technical Highlights

1. **Strict No-GPS-Leakage**:
   - `gps_latitude` and `gps_longitude` are strictly bypassed during GNSS outages.
   - Tested by corrupting GPS coordinates to Antarctica during an outage; Dead Reckoning output matched with zero divergence ($< 10^{-12}$).
2. **Automatic Stationary IMU Calibration**:
   - Automatically gathers 100 stationary samples during initial startup ($0\text{s}-10\text{s}$) to estimate $\text{gyro\_bias\_z}$ and $\text{accel\_bias\_forward}$, eliminating drift accumulation.
3. **Sub-Step Kinematic Propagation**:
   - Subdivides large time gaps ($\Delta t > 2.0\text{s}$) into stable internal integration steps while preserving total actual elapsed physical time.
4. **Three Configurable Velocity Modes**:
   - `sensor_speed` (wheel speed / CAN bus)
   - `acceleration` (pure accelerometer integration)
   - `hybrid` (blended $\alpha$-weighting)
5. **Dual Interface**:
   - Batch DataFrame processing for offline evaluation (`engine.process_dataframe`)
   - Streaming dictionary processing for live FastAPI web sockets (`engine.process_sensor_record`)

---

## 4. Key Performance Benchmark Results

| Parameter | Value |
|---|---|
| **Total Test Dataset Duration** | 90.0 seconds (901 records @ 10 Hz) |
| **Outage Duration** | 30.0 seconds (300 records @ 10 Hz) |
| **Outage Maneuvers** | 15s straight cruise + 15s right turn (3°/s) |
| **Total Estimated Distance** | 969.18 metres |
| **Peak Vehicle Speed** | 55.11 km/h |
| **Reconnection Error at $t=60\text{s}$** | **1.36 metres** |
| **Automated Test Coverage** | **90 / 90 tests passing (100%)** |

---

## 5. Team Integration Checklist

- **Input from Arsheen**: Drop `sensor_data.csv` into `data/` with the 11 standard columns.
- **Backend API for Nishitha**: Use `engine.process_sensor_record(sensor_dict)` returning clean JSON-ready dicts.
- **Metrics for Hari**: Provide `outputs/estimated_trajectory.csv` for MAE, RMSE, and drift % evaluation against ground truth.
- **Frontend for Charu**: Expose `estimated_latitude`, `estimated_longitude`, `estimated_speed_kmph`, `estimated_heading_deg`, `navigation_mode`, `outage_elapsed_seconds`, and `gnss_reconnection_error_m`.

---

## 6. Quick Execution Commands

```powershell
cd D:\SIH\jeffryn-dead-reckoning

# Run full demonstration
python scripts/run_demo.py

# Run live stream simulation
python scripts/simulate_live_stream.py --speed 20.0

# Run velocity modes benchmark
python scripts/compare_velocity_modes.py

# Run test suite
pytest tests/ -v
```
