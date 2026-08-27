"""
Jeffryn Dead Reckoning Engine — public package interface.

Usage
-----
::

    from src.dead_reckoning import DeadReckoningEngine, load_config

    engine = DeadReckoningEngine(load_config("config.yaml"))
    result_df = engine.process_dataframe(sensor_df)
"""

from __future__ import annotations

from typing import Any

import yaml

from .engine import DeadReckoningEngine
from .exceptions import (
    ConfigurationError,
    DeadReckoningError,
    EngineError,
    PreprocessingError,
    ValidationError,
)
from .models import NavigationMode
from .state import VehicleState

__all__ = [
    "DeadReckoningEngine",
    "load_config",
    "NavigationMode",
    "VehicleState",
    "DeadReckoningError",
    "ConfigurationError",
    "ValidationError",
    "PreprocessingError",
    "EngineError",
]


def load_config(path: str) -> dict[str, Any]:
    """Load and return a YAML configuration file.

    Parameters
    ----------
    path : str
        Filesystem path to ``config.yaml``.

    Returns
    -------
    dict
        Parsed configuration dictionary.

    Raises
    ------
    ConfigurationError
        If the file cannot be read or parsed.
    """
    try:
        with open(path, "r", encoding="utf-8") as fh:
            config = yaml.safe_load(fh)
        if not isinstance(config, dict):
            raise ConfigurationError(
                f"Configuration file '{path}' did not parse as a dict."
            )
        return config
    except FileNotFoundError:
        raise ConfigurationError(f"Configuration file not found: '{path}'")
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"YAML parse error in '{path}': {exc}")
