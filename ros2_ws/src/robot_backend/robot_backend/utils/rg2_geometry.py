"""RG2 calibration helpers for width, joint position, and grasp offsets.

`/rg2_controller/set_position` expects the RG2 master joint target in
radians, while `robot_backend` exposes `GripperBackend.set_width(width_m)`
using physical gripper opening width in meters. This module bridges that
semantic gap.

The fixed TCP stays in the URDF. Width-dependent grasp geometry is handled in
software, and `tcp_offset_from_width` returns the transform from the fixed TCP
frame to the logical grasp center.
"""

from __future__ import annotations

import csv
import math
from bisect import bisect_left
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from .geometry import PoseDict
from .geometry import make_pose
from .grasp_offsets import compute_tcp_pose_from_grasp_center


@dataclass(frozen=True)
class RG2Calibration:
    """Calibration values for RG2 width, joint position, and grasp offset."""

    width_open_m: float
    width_closed_m: float
    position_open_rad: float
    position_closed_rad: float
    tcp_to_grasp_center_open: PoseDict
    tcp_to_grasp_center_closed: PoseDict


@dataclass(frozen=True)
class RG2WidthLookup:
    """Monotonic angle<->width lookup loaded from the RG2 calibration CSV."""

    table_path: str
    angles_rad: tuple[float, ...]
    widths_m: tuple[float, ...]


DEFAULT_RG2_CALIBRATION = RG2Calibration(
    width_open_m=0.10,
    width_closed_m=0.0,
    position_open_rad=-0.558505,
    position_closed_rad=0.7731809,
    tcp_to_grasp_center_open=make_pose(
        position={"x": 0.0, "y": 0.0, "z": 0.0256},
        orientation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    ),
    tcp_to_grasp_center_closed=make_pose(
        position={"x": 0.0, "y": 0.0, "z": 0.069},
        orientation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    ),
)


def _default_width_table_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("isaac_trajectory_bridge")
        return str(Path(share) / "config" / "rg2_width_table.csv")
    except Exception:
        return str(
            Path(__file__).resolve().parents[3]
            / "isaac_trajectory_bridge"
            / "config"
            / "rg2_width_table.csv"
        )


DEFAULT_RG2_WIDTH_TABLE_PATH = _default_width_table_path()


def clamp_width(
    width_m: float,
    calibration: RG2Calibration = DEFAULT_RG2_CALIBRATION,
) -> float:
    """Clamp a physical RG2 opening width to the calibrated range."""

    lower = min(calibration.width_closed_m, calibration.width_open_m)
    upper = max(calibration.width_closed_m, calibration.width_open_m)
    return min(max(float(width_m), lower), upper)


def _clamp_joint_position(
    position_rad: float,
    calibration: RG2Calibration,
) -> float:
    lower = min(calibration.position_open_rad, calibration.position_closed_rad)
    upper = max(calibration.position_open_rad, calibration.position_closed_rad)
    return min(max(float(position_rad), lower), upper)


def _interpolate(x: float, xp: tuple[float, ...], fp: tuple[float, ...]) -> float:
    """Linearly interpolate within sorted xp, clamping to the boundary values."""

    if len(xp) != len(fp):
        raise ValueError("Interpolation inputs must have the same length.")
    if len(xp) < 2:
        raise ValueError("Interpolation requires at least two samples.")
    if x <= xp[0]:
        return float(fp[0])
    if x >= xp[-1]:
        return float(fp[-1])

    idx = bisect_left(xp, x)
    x0 = xp[idx - 1]
    x1 = xp[idx]
    y0 = fp[idx - 1]
    y1 = fp[idx]
    if x1 == x0:
        return float(y1)

    fraction = (x - x0) / (x1 - x0)
    return float(y0 + fraction * (y1 - y0))


@lru_cache(maxsize=4)
def load_rg2_width_lookup(
    table_path: str = DEFAULT_RG2_WIDTH_TABLE_PATH,
) -> RG2WidthLookup | None:
    """Load the RG2 width table used by the controller, if available."""

    path = Path(table_path)
    if not path.exists():
        return None

    rows: list[tuple[float, float]] = []
    try:
        with path.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                direction = row.get("direction", "open_to_close")
                if direction not in ("open_to_close", ""):
                    continue
                try:
                    angle = float(row["master_angle_rad"])
                    width = float(row["true_width_m"])
                except (KeyError, TypeError, ValueError):
                    continue
                if math.isfinite(angle) and math.isfinite(width) and width >= 0.0:
                    rows.append((angle, width))
    except OSError:
        return None

    if len(rows) < 2:
        return None

    rows.sort(key=lambda item: item[0])
    angles = [angle for angle, _width in rows]
    widths = [width for _angle, width in rows]

    # Match the controller by smoothing out local reversals from noisy samples.
    for index in range(1, len(widths)):
        if widths[index] > widths[index - 1]:
            widths[index] = widths[index - 1]

    return RG2WidthLookup(
        table_path=str(path),
        angles_rad=tuple(angles),
        widths_m=tuple(widths),
    )


def closure_fraction_from_width(
    width_m: float,
    calibration: RG2Calibration = DEFAULT_RG2_CALIBRATION,
) -> float:
    """Return RG2 closure fraction for a physical width.

    The returned fraction is `0.0` at fully open and `1.0` at fully closed.
    """

    width_span = calibration.width_open_m - calibration.width_closed_m
    if width_span == 0.0:
        raise ValueError("RG2 calibration width_open_m and width_closed_m must differ.")

    width_clamped = clamp_width(width_m, calibration)
    return (calibration.width_open_m - width_clamped) / width_span


def width_to_joint_position(
    width_m: float,
    calibration: RG2Calibration = DEFAULT_RG2_CALIBRATION,
    width_lookup: RG2WidthLookup | None = None,
) -> float:
    """Convert physical RG2 width in meters to master joint radians."""

    width_clamped = clamp_width(width_m, calibration)

    if width_lookup is not None:
        return _clamp_joint_position(
            _interpolate(
                width_clamped,
                tuple(reversed(width_lookup.widths_m)),
                tuple(reversed(width_lookup.angles_rad)),
            ),
            calibration,
        )

    position_span = calibration.position_closed_rad - calibration.position_open_rad
    if position_span == 0.0:
        raise ValueError(
            "RG2 calibration position_open_rad and position_closed_rad must differ."
        )

    fraction = closure_fraction_from_width(width_clamped, calibration)
    return calibration.position_open_rad + fraction * position_span


def joint_position_to_width(
    position_rad: float,
    calibration: RG2Calibration = DEFAULT_RG2_CALIBRATION,
    width_lookup: RG2WidthLookup | None = None,
) -> float:
    """Convert RG2 master joint radians to physical width in meters."""

    position_clamped = _clamp_joint_position(position_rad, calibration)

    if width_lookup is not None:
        return clamp_width(
            _interpolate(
                position_clamped,
                width_lookup.angles_rad,
                width_lookup.widths_m,
            ),
            calibration,
        )

    position_span = calibration.position_closed_rad - calibration.position_open_rad
    if position_span == 0.0:
        raise ValueError(
            "RG2 calibration position_open_rad and position_closed_rad must differ."
        )

    fraction = (position_clamped - calibration.position_open_rad) / position_span
    width = calibration.width_open_m - fraction * (
        calibration.width_open_m - calibration.width_closed_m
    )
    return clamp_width(width, calibration)


def tcp_offset_from_width(
    width_m: float,
    calibration: RG2Calibration = DEFAULT_RG2_CALIBRATION,
) -> PoseDict:
    """Return the fixed-TCP to logical grasp-center transform for a width.

    Orientation interpolation is unnecessary for the current calibration
    because the logical grasp-center frame is assumed to remain aligned with
    the fixed TCP frame.
    """

    fraction = closure_fraction_from_width(width_m, calibration)
    open_position = calibration.tcp_to_grasp_center_open["position"]
    closed_position = calibration.tcp_to_grasp_center_closed["position"]
    position = {
        axis: open_position[axis] + fraction * (closed_position[axis] - open_position[axis])
        for axis in ("x", "y", "z")
    }
    orientation = dict(calibration.tcp_to_grasp_center_open["orientation"])
    frame_id = calibration.tcp_to_grasp_center_open.get(
        "frame_id",
        calibration.tcp_to_grasp_center_closed.get("frame_id"),
    )
    return make_pose(position=position, orientation=orientation, frame_id=frame_id)


def tcp_target_from_grasp_center(
    grasp_center_pose: PoseDict,
    width_m: float,
    calibration: RG2Calibration = DEFAULT_RG2_CALIBRATION,
) -> PoseDict:
    """Return the fixed TCP target pose for a desired logical grasp center.

    `grasp_center_pose` is the desired logical grasp center pose in the
    planning or world frame, i.e. `T_world_grasp_center`.
    `width_m` is the desired physical gripper opening in meters.

    This computes `T_tcp_grasp_center` from the width-dependent RG2
    calibration, then returns:

        T_world_tcp = T_world_grasp_center * inverse(T_tcp_grasp_center)

    The returned pose is the fixed TCP pose that should be sent to IK or
    MoveIt. The URDF TCP remains fixed; width only changes the software-side
    TCP-to-grasp-center offset.
    """

    tcp_offset = tcp_offset_from_width(width_m, calibration)
    return compute_tcp_pose_from_grasp_center(grasp_center_pose, tcp_offset)
