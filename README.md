# SIH-Prototype — Intelligent Vehicle Navigation During GNSS Outages

**Problem Statement:** PS 26168  
**Team Project Repository:** Smart India Hackathon Prototype

---

## 📌 Repository Structure & Modules

```
SIH-Prototype/
├── jeffryn-dead-reckoning/   # Module: Dead Reckoning Engine (Jeffryn Adaikalaraj A)
│   ├── src/dead_reckoning/  # Core physics engine, state machine, coordinate transforms
│   ├── scripts/             # Demonstration, benchmark, and visualization scripts
│   ├── tests/               # 90 automated pytest unit & integration tests
│   ├── config.yaml          # Engine tuning parameters
│   └── README.md            # In-depth module documentation
└── README.md                # Project repository overview
```

---

## 🚗 Completed Modules

### 1. Dead Reckoning Engine (`jeffryn-dead-reckoning/`)
- **Author:** Jeffryn Adaikalaraj A
- **Scope:** Baseline Prototype (30–40%)
- **Features:**
  - Automatic IMU stationary bias calibration ($0\text{s}–10\text{s}$).
  - Gyroscope yaw-rate integration for vehicle heading.
  - Three velocity modes: `sensor_speed`, `acceleration`, and `hybrid`.
  - Geodetic local metric $\leftrightarrow$ WGS-84 coordinate transformation.
  - Strict No-GPS-Leakage verification during outages.
  - Reconnection error measurement upon GNSS recovery (**1.36 m** benchmark).
  - 100% automated test coverage (90/90 tests passing).
  - Dual processing APIs: batch DataFrame and real-time streaming for FastAPI.

---

## 🚀 Quick Start (Dead Reckoning)

```bash
# Navigate to the module
cd jeffryn-dead-reckoning

# Install dependencies
pip install -r requirements.txt

# Run the full end-to-end demo
python scripts/run_demo.py

# Run automated tests
pytest tests/ -v
```