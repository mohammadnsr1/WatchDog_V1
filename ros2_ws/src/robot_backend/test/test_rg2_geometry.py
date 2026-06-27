from __future__ import annotations

import pytest

from robot_backend.utils.rg2_geometry import DEFAULT_RG2_CALIBRATION
from robot_backend.utils.rg2_geometry import RG2WidthLookup
from robot_backend.utils.rg2_geometry import joint_position_to_width
from robot_backend.utils.rg2_geometry import load_rg2_width_lookup
from robot_backend.utils.rg2_geometry import width_to_joint_position


def test_default_width_lookup_loads_monotonic_open_to_close_table() -> None:
    lookup = load_rg2_width_lookup()

    assert lookup is not None
    assert len(lookup.angles_rad) > 2
    assert list(lookup.angles_rad) == sorted(lookup.angles_rad)
    assert all(
        next_width <= width
        for width, next_width in zip(lookup.widths_m, lookup.widths_m[1:])
    )


def test_width_lookup_round_trip_matches_requested_width() -> None:
    lookup = load_rg2_width_lookup()
    assert lookup is not None

    for width_m in (0.0, 0.02, 0.05, 0.08):
        angle_rad = width_to_joint_position(
            width_m,
            DEFAULT_RG2_CALIBRATION,
            width_lookup=lookup,
        )
        reconstructed_width = joint_position_to_width(
            angle_rad,
            DEFAULT_RG2_CALIBRATION,
            width_lookup=lookup,
        )
        assert reconstructed_width == pytest.approx(width_m, abs=1e-6)


def test_width_lookup_overrides_linear_midstroke_mapping() -> None:
    synthetic_lookup = RG2WidthLookup(
        table_path='synthetic.csv',
        angles_rad=(-0.558505, -0.25, 0.7731809),
        widths_m=(0.1, 0.09, 0.0),
    )

    linear_angle = width_to_joint_position(0.05, DEFAULT_RG2_CALIBRATION)
    table_angle = width_to_joint_position(
        0.05,
        DEFAULT_RG2_CALIBRATION,
        width_lookup=synthetic_lookup,
    )

    assert table_angle != pytest.approx(linear_angle, abs=1e-3)
    assert joint_position_to_width(
        table_angle,
        DEFAULT_RG2_CALIBRATION,
        width_lookup=synthetic_lookup,
    ) == pytest.approx(0.05, abs=1e-6)
