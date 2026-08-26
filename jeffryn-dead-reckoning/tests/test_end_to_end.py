"""
End-to-end integration test.

Processes the full 90-second generated test fixture and asserts:
  • No crash
  • Output length == input length
  • Required output columns exist
  • No infinite values in key columns
  • GNSS OFF records use DEAD_RECKONING mode
  • Position changes during moving outage
  • Reconnection occurs
  • Output CSV can be written and read back
"""

import math
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pandas as pd
import pytest

from scripts.generate_test_fixture import generate
from src.dead_reckoning import DeadReckoningEngine, load_config
from src.dead_reckoning.models import OUTPUT_COLUMNS


@pytest.fixture(scope="module")
def fixture_data():
    """Generate the 90-second test fixture once for all tests."""
    return generate()


@pytest.fixture(scope="module")
def config():
    config_path = os.path.join(
        os.path.dirname(__file__), "..", "config.yaml"
    )
    return load_config(config_path)


@pytest.fixture(scope="module")
def result(fixture_data, config):
    engine = DeadReckoningEngine(config)
    return engine.process_dataframe(fixture_data)


class TestEndToEnd:
    def test_no_crash(self, result):
        """Processing completes without exception."""
        assert result is not None

    def test_output_length(self, result, fixture_data):
        """Output row count matches input."""
        assert len(result) == len(fixture_data)

    def test_required_columns(self, result):
        for col in OUTPUT_COLUMNS:
            assert col in result.columns, f"Missing output column: {col}"

    def test_no_infinite_values(self, result):
        numeric_cols = [
            "estimated_latitude", "estimated_longitude",
            "estimated_x_m", "estimated_y_m",
            "estimated_velocity_mps", "estimated_heading_deg",
        ]
        for col in numeric_cols:
            vals = result[col].values
            assert np.all(np.isfinite(vals)), f"Infinite values in {col}"

    def test_dr_mode_during_outage(self, result):
        """Records where GNSS is off should use DEAD_RECKONING."""
        off_rows = result[result["gnss_available"] == False]
        if len(off_rows) > 0:
            # All should be DR or UNINITIALIZED (if before first fix)
            modes = off_rows["navigation_mode"].unique()
            for m in modes:
                assert m in ("DEAD_RECKONING", "UNINITIALIZED"), \
                    f"Unexpected mode during GNSS OFF: {m}"

    def test_position_changes_during_moving_outage(self, result):
        """During DR with non-zero speed, position should change."""
        dr = result[result["navigation_mode"] == "DEAD_RECKONING"]
        if len(dr) >= 2:
            x_range = dr["estimated_x_m"].max() - dr["estimated_x_m"].min()
            y_range = dr["estimated_y_m"].max() - dr["estimated_y_m"].min()
            total_change = math.sqrt(x_range ** 2 + y_range ** 2)
            assert total_change > 10.0, "Position barely moved during outage"

    def test_reconnection_occurs(self, result):
        """GNSS_REACQUIRED should appear at least once."""
        reacq = result[result["navigation_mode"] == "GNSS_REACQUIRED"]
        assert len(reacq) >= 1

    def test_output_csv_roundtrip(self, result, tmp_path):
        """Output can be saved and re-loaded without corruption."""
        path = os.path.join(str(tmp_path), "test_output.csv")
        result.to_csv(path, index=False)
        loaded = pd.read_csv(path)
        assert len(loaded) == len(result)
        for col in OUTPUT_COLUMNS:
            assert col in loaded.columns

    def test_heading_in_range(self, result):
        """All headings should be in [0, 360)."""
        h = result["estimated_heading_deg"]
        assert h.min() >= 0.0
        assert h.max() < 360.0

    def test_velocity_non_negative(self, result):
        """Velocity should never be negative."""
        assert result["estimated_velocity_mps"].min() >= 0.0

    def test_cumulative_distance_increases(self, result):
        """Cumulative distance should be monotonically non-decreasing."""
        dists = result["cumulative_distance_m"].values
        for i in range(1, len(dists)):
            assert dists[i] >= dists[i - 1] - 1e-9
