# ISRO SIH26168: AI-ML based Intelligent Dead Reckoning System — GNSS Outage Simulation & Validation Suite

A modular Python framework for simulating GNSS signal loss, running Dead Reckoning (DR) navigation estimators during outages, seamlessly blending upon GNSS reacquisition, and computing validation error metrics.

---

## Architecture Overview

```
SIH26-Hari-gnsslosssim/
├── gnss_outage_simulator.py   # Configurable simulation engine & CLI interface
├── main.py                    # End-to-end primary benchmark demonstrator
├── requirements.txt           # Minimal dependencies (numpy, matplotlib)
├── README.md                  # System documentation & mathematical formulation
├── src/
│   ├── trajectory_generator.py # Synthetic 2D path generator (S-curve, urban, circular) & CSV loader
│   ├── sensor_simulator.py     # GNSS outage scheduler, Gaussian noise, IMU noise & gyro bias drift
│   ├── dead_reckoning.py       # Swappable DR base interface, Kinematic DR, and ML model placeholder
│   ├── fusion_engine.py        # Mode switching, seamless DR seeding & reacquisition blending
│   ├── metrics.py              # Euclidean error, MAE, RMSE, Drift %, and jump magnitude calculators
│   └── visualizer.py           # Publication-ready trajectory maps and multi-panel error dashboards
├── data/
│   └── sample_gps_log.csv     # Sample real-world vehicle GPS log for external data testing
├── output/                    # Generated high-resolution plots
│   ├── trajectory_plot.png    # 2D bird's-eye path overlay (GT vs GNSS vs DR vs Blending)
│   └── error_analysis_plot.png# Multi-panel temporal error, velocity & heading diagnostics
└── tests/
    └── test_simulation.py     # Automated unit and integration test suite
```

---

## Mathematical Formulation

### 1. Ground Truth & Kinematics
Given forward speed $v(t)$ and heading $\theta(t)$:
$$\dot{x}(t) = v(t) \cos(\theta(t)), \quad \dot{y}(t) = v(t) \sin(\theta(t)), \quad \dot{\theta}(t) = \omega(t)$$

Discrete integration with midpoint heading:
$$\theta_k = \theta_{k-1} + \frac{\omega_{k-1} + \omega_k}{2} \Delta t$$
$$x_k = x_{k-1} + \frac{v_{k-1} + v_k}{2} \cos\left(\frac{\theta_{k-1} + \theta_k}{2}\right) \Delta t$$
$$y_k = y_{k-1} + \frac{v_{k-1} + v_k}{2} \sin\left(\frac{\theta_{k-1} + \theta_k}{2}\right) \Delta t$$

### 2. Sensor Noise & Gyro Drift Modeling
- **GNSS Fix**:
  $$p_{\text{gnss}}(t) = p_{\text{true}}(t) + \mathcal{N}\left(0, \sigma_{\text{pos}}^2 \mathbf{I}_2\right)$$
- **IMU Odometry / Wheel Speed**:
  $$v_{\text{imu}}(t) = v_{\text{true}}(t) + b_v + \mathcal{N}\left(0, \sigma_v^2\right)$$
- **IMU Gyroscope Yaw Rate**:
  $$\omega_{\text{imu}}(t) = \omega_{\text{true}}(t) + b_\omega + \mathcal{N}\left(0, \sigma_\omega^2\right)$$

### 3. Reacquisition Blending (Smoothing Handover)
Upon GNSS reacquisition at $t = t_{\text{reacq}}$, rather than abruptly jumping the estimated position to the noisy GNSS fix, the navigation estimate smoothly blends over duration $\tau = 2.0\text{ s}$:
$$\alpha(t) = \min\left(1.0, \frac{t - t_{\text{reacq}}}{\tau}\right)$$
$$p_{\text{fused}}(t) = (1 - \alpha(t)) \cdot p_{\text{DR\_extrapolated}}(t) + \alpha(t) \cdot p_{\text{GNSS}}(t)$$

### 4. Validation Metrics (Computed during Outages)
- **Mean Absolute Error (MAE)**:
  $$\text{MAE} = \frac{1}{N} \sum_{k=1}^N \|p_{\text{est}}(t_k) - p_{\text{true}}(t_k)\|$$
- **Root Mean Squared Error (RMSE)**:
  $$\text{RMSE} = \sqrt{\frac{1}{N} \sum_{k=1}^N \|p_{\text{est}}(t_k) - p_{\text{true}}(t_k)\|^2}$$
- **Drift Percentage**:
  $$\text{Drift } \% = \frac{\text{Final Position Error at End of Outage}}{\text{Total Distance Traveled during Outage}} \times 100$$
- **Reacquisition Jump Magnitude**:
  $$\Delta_{\text{jump}} = \|p_{\text{DR}}(t_{\text{reacq}}) - p_{\text{GNSS}}(t_{\text{reacq}})\|$$

---

## Quick Start

### 1. Installation
Install minimal dependencies:
```bash
pip install -r requirements.txt
```

### 2. Run Benchmark Scenario (0-20s ON, 20-50s OFF, 50-70s ON)
```bash
python main.py
```

### 3. Run Configurable Simulator CLI
```bash
# Synthetic maneuver with custom outage window
python gnss_outage_simulator.py --duration 100 --outage 30 70 --speed 18.0 --profile urban_maneuver

# Load real GPS CSV log
python gnss_outage_simulator.py --csv data/sample_gps_log.csv --outage 20 50

# Test ML placeholder module
python gnss_outage_simulator.py --estimator ml_placeholder
```

### 4. Run Automated Tests
```bash
python -m unittest discover -s tests -p "test_*.py"
```

---

## Swapping in Machine Learning Models

The Dead Reckoning module is decoupled via the `BaseDeadReckoningEstimator` abstract interface in [`src/dead_reckoning.py`](file:///d:/SIH26-Hari-gnsslosssim/src/dead_reckoning.py).

To plug in a trained AI/ML model (e.g. PyTorch / ONNX / TensorFlow):
1. Inherit from `BaseDeadReckoningEstimator`.
2. Implement:
   - `reset(initial_state)`: resets neural network hidden states / buffers.
   - `update_gnss(gnss_meas)`: calibrates input history while GNSS is locked.
   - `step_dr(imu_meas, dt)`: executes neural network forward pass to predict $(\Delta x, \Delta y, \Delta \theta)$ and returns updated `NavState`.
3. Pass your model instance to `run_simulation(dr_estimator=YourMLModel())` without altering any other part of the pipeline.
