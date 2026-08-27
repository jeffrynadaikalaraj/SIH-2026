"""
config.py
=========
Centralized tunable parameters for the IDR-X navigation backend.
Adjust here to tune behavior without touching engine logic.
"""

# Vehicle Simulation
CRUISE_SPEED_MPS: float = 12.5
TURN_SPEED_MPS: float = 6.0
SPEED_APPROACH_RATE: float = 1.5
TURN_SLOW_THRESHOLD_M: float = 40.0

# GNSS Simulation
GNSS_NOISE_M: float = 0.000015
GNSS_DEFAULT_OUTAGE_SEC: float = 25.0

# Dead Reckoning
DR_DRIFT_EXPONENT: float = 1.3
DR_DRIFT_BASE: float = 0.0000008
DR_LATERAL_NOISE: float = 0.0000003
DR_LONG_NOISE: float = 0.0000005

# Turn-by-Turn Navigation
MANEUVER_ARRIVE_THRESHOLD_M: float = 18.0
MANEUVER_NOW_THRESHOLD_M: float = 25.0

# Rolling Route Cache
ROLLING_CACHE_KM: float = 5.0

# WebSocket / Real-Time
WS_UPDATE_HZ: float = 10.0
WS_STEP_DT: float = 0.1

# History Buffer
MAX_HISTORY_POINTS: int = 1000
