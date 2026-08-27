"""
Custom exceptions for the Dead Reckoning engine.

Provides a structured exception hierarchy for configuration errors,
sensor validation failures, preprocessing issues, and engine faults.
"""


class DeadReckoningError(Exception):
    """Base exception for all Dead Reckoning engine errors."""
    pass


class ConfigurationError(DeadReckoningError):
    """Raised when the configuration file is missing, malformed, or contains
    invalid parameter values."""
    pass


class ValidationError(DeadReckoningError):
    """Raised when sensor data fails validation checks such as missing
    required columns, out-of-range values, or incompatible types."""
    pass


class PreprocessingError(DeadReckoningError):
    """Raised when data preprocessing encounters a fatal issue such as
    unparseable timestamps or entirely empty input."""
    pass


class EngineError(DeadReckoningError):
    """Raised when the Dead Reckoning engine encounters an unrecoverable
    internal fault during position propagation."""
    pass
