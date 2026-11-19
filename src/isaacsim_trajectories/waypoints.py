"""Convert timed SO-101 joint waypoints into a sampled, smooth reference.

Each segment is a rest-to-rest quintic, so position, velocity, and acceleration
are continuous at waypoint joins. This is an offline reference calculation;
collision, workspace, jerk, and measured robot dynamics are not validated.
"""
from __future__ import annotations

import math
from typing import Any

from .so101_demo import JOINT_NAMES

SCHEMA_VERSION = "so101-waypoints/1"
DEFAULT_MAX_SAMPLES = 1_000_000


def _number(value: Any, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    if positive and number <= 0:
        raise ValueError(f"{name} must be positive")
    return number


def _positive_integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    _number(value, name, positive=True)
    return value


def _read_waypoints(
    document: dict, control_hz: int, max_samples: int,
) -> tuple[list[str], list[int], list[list[float]]]:
    if not isinstance(document, dict) or set(document) != {"schema_version", "joint_names", "waypoints"}:
        raise ValueError("Waypoint input must contain exactly schema_version, joint_names, and waypoints")
    if document["schema_version"] != SCHEMA_VERSION:
        raise ValueError(f"Unsupported waypoint schema; expected {SCHEMA_VERSION}")
    names = document["joint_names"]
    if not isinstance(names, list) or len(names) != len(JOINT_NAMES) or not all(
        isinstance(name, str) for name in names
    ) or set(names) != set(JOINT_NAMES):
        raise ValueError("joint_names must contain each SO-101 joint exactly once")
    rows = document["waypoints"]
    if not isinstance(rows, list) or len(rows) < 2:
        raise ValueError("waypoints must contain at least two timed joint poses")

    steps: list[int] = []
    positions: list[list[float]] = []
    previous_time = -1.0
    gripper_column = names.index("gripper")
    for index, row in enumerate(rows):
        if not isinstance(row, dict) or set(row) != {"time_s", "positions_rad"}:
            raise ValueError(f"Waypoint {index} must contain exactly time_s and positions_rad")
        time = _number(row["time_s"], f"Waypoint {index}.time_s")
        if (index == 0 and time != 0) or (index > 0 and time <= previous_time):
            raise ValueError("Waypoint times must start at zero and strictly increase")
        grid_time = time * control_hz
        if not math.isfinite(grid_time) or not math.isclose(
            grid_time, round(grid_time), rel_tol=0, abs_tol=1e-8
        ):
            raise ValueError(f"Waypoint {index}.time_s must lie on the control_hz sample grid")
        step = round(grid_time)
        if steps and step <= steps[-1]:
            raise ValueError("Consecutive waypoints must span at least one control step")
        if step + 1 > max_samples:
            raise ValueError(f"Trajectory exceeds max_samples={max_samples}; shorten its duration or raise the limit explicitly")
        values = row["positions_rad"]
        if not isinstance(values, list) or len(values) != len(names):
            raise ValueError(f"Waypoint {index}.positions_rad must contain six joint positions")
        pose = [_number(value, f"Waypoint {index}.{name}") for name, value in zip(names, values)]
        if index == 0 and any(value != 0 for value in pose):
            raise ValueError("Replay currently requires a zero initial pose")
        if pose[gripper_column] != 0:
            raise ValueError("Replay currently requires the gripper held at zero")
        steps.append(step)
        positions.append(pose)
        previous_time = time
    return list(names), steps, positions


def _model_limits(
    model: dict, names: list[str], margin: float, velocity_limit: float,
) -> tuple[list[float], list[float], list[float]]:
    joints = model.get("joints") if isinstance(model, dict) else None
    if not isinstance(joints, dict) or set(joints) != set(JOINT_NAMES):
        raise ValueError("Model must define exactly the six SO-101 joints")
    lower, upper, velocity = [], [], []
    for name in names:
        joint = joints[name]
        if not isinstance(joint, dict) or joint.get("type") != "revolute":
            raise ValueError(f"Joint {name} must be revolute")
        lo = _number(joint.get("lower"), f"{name}.lower")
        hi = _number(joint.get("upper"), f"{name}.upper")
        speed = _number(joint.get("velocity"), f"{name}.velocity", positive=True)
        _number(joint.get("effort"), f"{name}.effort", positive=True)
        if lo >= hi or lo + margin > hi - margin:
            raise ValueError(f"Joint {name} has invalid position limits/margin")
        lower.append(lo + margin)
        upper.append(hi - margin)
        velocity.append(min(velocity_limit, speed))
    return lower, upper, velocity


def sample_waypoints(
    document: dict, model: dict, config: dict, *, max_samples: int = DEFAULT_MAX_SAMPLES,
) -> tuple[dict, dict]:
    """Validate the full piecewise quintic, then sample at the configured rate.

    Input poses are absolute joint angles in radians, ordered by joint_names.
    All waypoint times must align with the control grid (1e-8 tick tolerance
    for floating-point representation). Output preserves the input joint order.
    No poses are clipped, omitted, or added; no segment is silently retimed.
    """
    if not isinstance(config, dict):
        raise ValueError("config must be an object")
    physics_hz = _positive_integer(config.get("physics_hz"), "physics_hz")
    control_hz = _positive_integer(config.get("control_hz"), "control_hz")
    if physics_hz % control_hz:
        raise ValueError("physics_hz must be divisible by control_hz")
    max_samples = _positive_integer(max_samples, "max_samples")
    margin = _number(config.get("position_margin_rad"), "position_margin_rad")
    if margin < 0:
        raise ValueError("position_margin_rad must be nonnegative")
    speed_limit = _number(
        config.get("reference_velocity_limit_rad_s"), "reference_velocity_limit_rad_s", positive=True,
    )
    acceleration_limit = _number(
        config.get("reference_acceleration_limit_rad_s2"), "reference_acceleration_limit_rad_s2", positive=True,
    )
    names, steps, poses = _read_waypoints(document, control_hz, max_samples)
    lower, upper, speeds = _model_limits(model, names, margin, speed_limit)

    # The quintic blend is monotonic on [0, 1], so endpoint position bounds
    # also prove every intermediate position is within its joint envelope.
    for waypoint, pose in enumerate(poses):
        for column, name in enumerate(names):
            if not lower[column] <= pose[column] <= upper[column]:
                raise ValueError(f"Waypoint {waypoint}, joint {name}: position exceeds the applied limits/margin")

    peak_velocity = [0.0] * len(names)
    peak_acceleration = [0.0] * len(names)
    for segment in range(len(steps) - 1):
        duration = (steps[segment + 1] - steps[segment]) / control_hz
        for column, name in enumerate(names):
            displacement = abs(poses[segment + 1][column] - poses[segment][column])
            speed = 1.875 * (displacement / duration)
            acceleration = (10 / math.sqrt(3)) * ((displacement / duration) / duration)
            if not math.isfinite(speed) or speed > speeds[column]:
                raise ValueError(f"Segment {segment}, joint {name}: quintic peak velocity exceeds the applied limit; increase the segment duration")
            if not math.isfinite(acceleration) or acceleration > acceleration_limit:
                raise ValueError(f"Segment {segment}, joint {name}: quintic peak acceleration exceeds the applied limit; increase the segment duration")
            peak_velocity[column] = max(peak_velocity[column], speed)
            peak_acceleration[column] = max(peak_acceleration[column], acceleration)

    # Allocate control samples only after all path and size checks succeed.
    import numpy as np

    times = np.arange(steps[-1] + 1, dtype=float) / float(control_hz)
    positions = np.empty((len(times), len(names)), dtype=float)
    for segment in range(len(steps) - 1):
        first, last = steps[segment:segment + 2]
        u = np.arange(last - first + 1, dtype=float) / (last - first)
        # Evaluate near either endpoint without cancellation near u=1.
        half = np.minimum(u, 1 - u)
        blend = half**3 * (10 + half * (-15 + 6 * half))
        blend = np.where(u <= 0.5, blend, 1 - blend)
        start = np.asarray(poses[segment])
        end = np.asarray(poses[segment + 1])
        positions[first:last + 1] = start + blend[:, None] * (end - start)
        positions[first] = start
        positions[last] = end
    if not np.isfinite(positions).all() or not np.isfinite(times).all():
        raise ValueError("Sampled trajectory must contain finite values")

    validation = {
        "method": "analytic extrema of monotonic rest-to-rest quintic segments, including every waypoint join",
        "waypoint_count": len(poses),
        "waypoint_times_s": [step / control_hz for step in steps],
        "duration_seconds": steps[-1] / control_hz,
        "control_steps": steps[-1],
        "physics_steps_per_control": physics_hz // control_hz,
        "peak_velocity_rad_s": max(peak_velocity),
        "peak_acceleration_rad_s2": max(peak_acceleration),
        "peak_velocity_by_joint_rad_s": dict(zip(names, peak_velocity)),
        "peak_acceleration_by_joint_rad_s2": dict(zip(names, peak_acceleration)),
        "applied_position_limits_rad": {
            name: {"lower": lower[index], "upper": upper[index]} for index, name in enumerate(names)
        },
        "applied_velocity_limits_rad_s": dict(zip(names, speeds)),
        "applied_acceleration_limit_rad_s2": acceleration_limit,
        "position_margin_rad": margin,
        "waypoint_derivatives": "zero velocity and acceleration at every waypoint",
        "jerk": "unchecked; may be discontinuous at waypoint joins",
        "collision": "unchecked", "workspace": "unchecked", "dataset_eligible": False,
    }
    return validation, {"timestamps_s": times, "positions_rad": positions}
