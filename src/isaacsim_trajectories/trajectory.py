"""A simulator-independent file contract for sampled SO-101 reference motion.

The format contains commands, not measurements. Loading does not depend on the
generator that produced them. NumPy is imported only when an operation is used.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import re
from typing import Any, Mapping

from .so101_demo import JOINT_NAMES

SCHEMA_VERSION = "so101-trajectory/1"
ARRAY_NAMES = frozenset({"timestamps_s", "positions_rad"})


def _positive_rate(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _finite_number(value: Any, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if positive and value <= 0:
        raise ValueError(f"{name} must be positive")
    return float(value)


def validate_trajectory(metadata: Mapping[str, Any], arrays: Mapping[str, Any]) -> None:
    """Check the versioned reference-file structure without trusting validation claims."""
    import numpy as np

    if not isinstance(metadata, Mapping) or metadata.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Unsupported trajectory schema")
    if not isinstance(metadata.get("trajectory_id"), str) or not metadata["trajectory_id"].strip():
        raise ValueError("trajectory_id must be a nonempty string")
    if metadata.get("data_kind") != "reference":
        raise ValueError("Trajectory data_kind must be reference")
    if "seed" not in metadata or (metadata["seed"] is not None and (
        isinstance(metadata["seed"], bool) or not isinstance(metadata["seed"], int)
    )):
        raise ValueError("seed must be an integer or null")
    generator = metadata.get("generator")
    if not isinstance(generator, dict) or any(
        not isinstance(generator.get(key), str) or not generator[key].strip() for key in ("name", "version")
    ):
        raise ValueError("generator must identify its name and version")
    if not isinstance(metadata.get("resolved_generation_config"), dict):
        raise ValueError("resolved_generation_config must be an object")
    code = metadata.get("code")
    if not isinstance(code, dict) or "revision" not in code or "dirty" not in code:
        raise ValueError("code must include revision and dirty (null when unavailable)")
    if code["revision"] is not None and (not isinstance(code["revision"], str) or not code["revision"]):
        raise ValueError("code.revision must be a nonempty string or null")
    if code["dirty"] is not None and not isinstance(code["dirty"], bool):
        raise ValueError("code.dirty must be boolean or null")
    asset = metadata.get("asset")
    if not isinstance(asset, dict) or not isinstance(asset.get("source_sha256"), str) or not re.fullmatch(
        r"[0-9a-fA-F]{64}", asset["source_sha256"]
    ):
        raise ValueError("asset.source_sha256 must identify the source URDF")
    names = metadata.get("joint_names")
    if not isinstance(names, list) or len(names) != len(JOINT_NAMES) or not all(
        isinstance(name, str) for name in names
    ) or set(names) != set(JOINT_NAMES):
        raise ValueError("joint_names must contain each SO-101 joint exactly once")
    if metadata.get("joint_types") != ["revolute"] * len(names):
        raise ValueError("joint_types must identify all six revolute joints")
    if metadata.get("units") != {"time": "seconds", "positions": "radians"}:
        raise ValueError("Trajectory units must be seconds and radians")
    hz = _positive_rate(metadata.get("control_hz"), "control_hz")
    # Also reject non-JSON provenance and nonfinite numbers hidden in metadata.
    try:
        json.dumps(metadata, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ValueError("Trajectory metadata must be finite JSON data") from exc
    if not isinstance(arrays, Mapping) or set(arrays) != ARRAY_NAMES:
        raise ValueError(f"Expected exactly these trajectory arrays: {sorted(ARRAY_NAMES)}")
    values = {name: np.asarray(value) for name, value in arrays.items()}
    for name, array in values.items():
        if array.dtype.kind not in "fiu" or not np.isfinite(array).all():
            raise ValueError(f"{name} must contain finite real numeric values")
    times = values["timestamps_s"]
    if times.ndim != 1 or len(times) < 2:
        raise ValueError("timestamps_s must have shape (N,) with N >= 2")
    # Compare before differencing: unsigned integer subtraction can underflow.
    if times[0] != 0 or not np.all(times[1:] > times[:-1]):
        raise ValueError("timestamps_s must start at zero and strictly increase")
    # Permit representation roundoff, bounded by one microsecond and 0.01% of
    # one sample interval. Replay still uses its exact integer-step schedule.
    if not np.allclose(times, np.arange(len(times), dtype=float) / hz,
                       rtol=0, atol=min(1e-6, 1e-4 / hz)):
        raise ValueError("timestamps_s must lie on the uniform control_hz sample grid")
    if values["positions_rad"].shape != (len(times), len(names)):
        raise ValueError(f"positions_rad must have shape ({len(times)}, {len(names)})")


def save_trajectory(directory: Path, metadata: dict, arrays: dict) -> None:
    """Validate and create a new trajectory directory; never overwrite an existing one."""
    import numpy as np

    validate_trajectory(metadata, arrays)
    encoded_metadata = json.dumps(metadata, indent=2, allow_nan=False) + "\n"
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(directory / "trajectory.npz", **arrays)
    (directory / "metadata.json").write_text(encoded_metadata)


def load_trajectory(directory: Path) -> tuple[dict, dict]:
    """Read and validate reference data without enabling pickle deserialization."""
    import numpy as np

    directory = Path(directory)
    metadata = json.loads((directory / "metadata.json").read_text())
    with np.load(directory / "trajectory.npz", allow_pickle=False) as archive:
        if set(archive.files) != ARRAY_NAMES or len(archive.files) != len(ARRAY_NAMES):
            raise ValueError("Unexpected or duplicated arrays in trajectory.npz")
        arrays = {name: archive[name].copy() for name in archive.files}
    validate_trajectory(metadata, arrays)
    return metadata, arrays


def validate_for_replay(
    metadata: dict, arrays: dict, model: dict, physics_hz: int, control_hz: int,
    position_margin_rad: float, velocity_limit_rad_s: float, acceleration_limit_rad_s2: float,
) -> dict:
    """Validate stored commands against the selected model and replay schedule.

    Derivatives here are finite-difference screening estimates at the stored
    resolution, not proofs about an unknown continuous generator or measured
    robot acceleration. Zero velocities before and after the motion include
    start/stop transitions in the conservative discrete screening policy.
    Zero-order-held drive targets themselves have discrete updates.
    """
    import numpy as np

    validate_trajectory(metadata, arrays)
    physics_hz = _positive_rate(physics_hz, "physics_hz")
    control_hz = _positive_rate(control_hz, "control_hz")
    if physics_hz % control_hz or metadata["control_hz"] != control_hz:
        raise ValueError("Replay rates must match the trajectory and physics_hz must be divisible by control_hz")
    margin = _finite_number(position_margin_rad, "position_margin_rad")
    if margin < 0:
        raise ValueError("position_margin_rad must be nonnegative")
    velocity_limit = _finite_number(velocity_limit_rad_s, "velocity_limit_rad_s", positive=True)
    acceleration_limit = _finite_number(acceleration_limit_rad_s2, "acceleration_limit_rad_s2", positive=True)
    if metadata["asset"]["source_sha256"].lower() != model.get("source_sha256"):
        raise ValueError("Trajectory asset hash does not match the selected URDF")
    names = metadata["joint_names"]
    joints = model.get("joints", {})
    if set(joints) != set(JOINT_NAMES):
        raise ValueError("Model must define exactly the six SO-101 joints")
    lower, upper, applied_velocity = [], [], []
    for name in names:
        joint = joints[name]
        if joint.get("type") != "revolute":
            raise ValueError(f"Joint {name} must be revolute")
        lo = _finite_number(joint.get("lower"), f"{name}.lower")
        hi = _finite_number(joint.get("upper"), f"{name}.upper")
        if lo >= hi or lo + margin > hi - margin:
            raise ValueError(f"Joint {name} has invalid position limits/margin")
        speed = _finite_number(joint.get("velocity"), f"{name}.velocity", positive=True)
        _finite_number(joint.get("effort"), f"{name}.effort", positive=True)
        lower.append(lo + margin)
        upper.append(hi - margin)
        applied_velocity.append(min(velocity_limit, speed))
    positions = np.asarray(arrays["positions_rad"], dtype=np.float64)
    if not np.allclose(positions[0], 0.0, rtol=0, atol=1e-12):
        raise ValueError("Replay currently requires a zero initial pose")
    if not np.allclose(positions[:, names.index("gripper")], 0.0, rtol=0, atol=1e-12):
        raise ValueError("Replay currently requires the gripper held at zero")
    if np.any(positions < np.array(lower)) or np.any(positions > np.array(upper)):
        raise ValueError("Stored positions exceed the applied joint limits/margin")
    velocity = np.diff(positions, axis=0) * control_hz
    # Include startup from rest and the final held pose, even for two samples.
    # This estimates a velocity change per control step, not an analytic bound.
    stopped_velocity = np.vstack((np.zeros((1, len(names))), velocity, np.zeros((1, len(names)))))
    acceleration = np.diff(stopped_velocity, axis=0) * control_hz
    if not np.isfinite(velocity).all() or np.any(np.abs(velocity) > np.array(applied_velocity) + 1e-12):
        raise ValueError("Sampled reference velocity exceeds the applied limit")
    if not np.isfinite(acceleration).all() or np.any(np.abs(acceleration) > acceleration_limit + 1e-12):
        raise ValueError("Sampled reference acceleration exceeds the applied limit")
    return {
        "method": "finite differences of stored samples, padded with start/end rest velocities; no continuous derivative proof",
        "sample_dt_s": 1 / control_hz,
        "duration_seconds": (len(positions) - 1) / control_hz,
        "control_steps": len(positions) - 1,
        "physics_steps_per_control": physics_hz // control_hz,
        "peak_velocity_rad_s": float(np.max(np.abs(velocity))),
        "peak_acceleration_rad_s2": float(np.max(np.abs(acceleration))),
        "applied_velocity_limits_rad_s": dict(zip(names, applied_velocity)),
        "applied_acceleration_limit_rad_s2": acceleration_limit,
        "continuous_reference": "unchecked by replay; inspect generator validation separately",
        "collision": "unchecked", "workspace": "unchecked", "dataset_eligible": False,
    }
