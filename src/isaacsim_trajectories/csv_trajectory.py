"""Read joint CSVs offline and adapt named SO-101 data to the reference schema.

CSV angles are radians. Both source clocks and optional tool coordinates remain
raw input data; only robot_time_s schedules replay. Gen3 columns can be inspected
but cannot be mapped to the SO-101 articulation.
"""
from __future__ import annotations

import csv
import hashlib
import io
import json
import math
from pathlib import Path
import sys
from typing import Any

from .so101_demo import JOINT_NAMES
from .trajectory import _finite_number, _positive_rate, validate_trajectory

TIME_COLUMNS = ("source_time_s", "robot_time_s")
TOOL_COLUMNS = ("tool_x_m", "tool_y_m", "tool_z_m")
GEN3_COLUMNS = tuple(f"joint_{i}" for i in range(1, 8))
MAX_CSV_BYTES = 256 * 1024 * 1024


def _sample_bound(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 2:
        raise ValueError(f"{name} must be an integer >= 2")
    return value


def read_csv_trajectory(path: Path, *, max_rows: int = 1_000_000) -> tuple[dict, dict]:
    """Read finite numeric columns by header, preserving radians and both clocks.

    This validates the file structure, not its suitability for a robot. Optional
    tool XYZ has no declared frame or measurement provenance in this CSV format.
    """
    import numpy as np

    _sample_bound(max_rows, "max_rows")
    path = Path(path).resolve()
    with path.open("rb") as stream:
        source = stream.read(MAX_CSV_BYTES + 1)
    if len(source) > MAX_CSV_BYTES:
        raise ValueError(f"CSV exceeds the {MAX_CSV_BYTES}-byte input limit")
    try:
        reader = csv.reader(io.StringIO(source.decode("utf-8-sig"), newline=""), strict=True)
        header = [name.strip() for name in next(reader, [])]
        if not header or any(not name for name in header) or len(header) != len(set(header)):
            raise ValueError("CSV header must contain nonempty, unique column names (no duplicates)")
        fields = set(header)
        if not set(TIME_COLUMNS).issubset(fields):
            raise ValueError("CSV requires source_time_s and robot_time_s columns")
        tool_fields = fields.intersection(TOOL_COLUMNS)
        if tool_fields and tool_fields != set(TOOL_COLUMNS):
            raise ValueError("CSV tool coordinates require all of tool_x_m, tool_y_m, tool_z_m")
        joint_fields = fields.difference(TIME_COLUMNS, TOOL_COLUMNS)
        if joint_fields in (set(JOINT_NAMES[:-1]), set(JOINT_NAMES)):
            layout = "so101"
        elif joint_fields == set(GEN3_COLUMNS):
            layout = "gen3"
        else:
            raise ValueError("CSV has missing or unsupported joint columns; expected five named "
                             "SO-101 arm joints (optional gripper), or joint_1 through joint_7")
        rows = []
        for row in reader:
            if len(rows) >= max_rows:
                raise ValueError(f"CSV exceeds max_rows={max_rows}")
            if len(row) != len(header):
                raise ValueError(f"CSV row ending at line {reader.line_num} has {len(row)} fields; "
                                 f"expected {len(header)} columns")
            values = []
            for name, cell in zip(header, row):
                try:
                    number = float(cell)
                except ValueError as exc:
                    raise ValueError(f"CSV line {reader.line_num}, {name}: expected a finite number") from exc
                if not math.isfinite(number):
                    raise ValueError(f"CSV line {reader.line_num}, {name}: expected a finite number")
                values.append(number)
            rows.append(values)
    except (UnicodeError, csv.Error) as exc:
        raise ValueError(f"Invalid UTF-8 CSV: {exc}") from exc
    if len(rows) < 2:
        raise ValueError("CSV must contain at least two data rows")
    data = np.asarray(rows, dtype=np.float64)
    arrays = {}
    for name in TIME_COLUMNS:
        times = data[:, header.index(name)].copy()
        if times[0] < 0 or not np.all(times[1:] > times[:-1]):
            raise ValueError(f"{name} must be nonnegative and strictly increasing")
        arrays[name] = times
    names = [name for name in header if name in joint_fields]
    arrays["positions_rad"] = data[:, [header.index(name) for name in names]].copy()
    if tool_fields:
        arrays["tool_positions_m"] = data[:, [header.index(name) for name in TOOL_COLUMNS]].copy()
    metadata = {
        "source": {"path": str(path), "sha256": hashlib.sha256(source).hexdigest()},
        "format": "joint CSV with source and robot timestamps", "layout": layout,
        "columns": header, "joint_names": names, "sample_count": len(rows),
        "units": {"time": "seconds", "positions": "radians", "tool_positions": "meters"},
        "tool_coordinates": {"present": bool(tool_fields), "frame": "unspecified in CSV",
                             "role": "supplied context; not verified simulator measurements"},
        "source_asset_identity": "not supplied by CSV",
    }
    return metadata, arrays


def _fixed_gripper(config: dict) -> float:
    policy = config.get("gripper_policy")
    if policy == "hold zero radians":
        return 0.0
    if policy == "hold fixed radians":
        return _finite_number(config.get("fixed_gripper_rad"), "fixed_gripper_rad")
    raise ValueError("CSV replay requires a fixed gripper_policy")


def _so101_positions(metadata: dict, arrays: dict, config: dict) -> Any:
    import numpy as np

    if metadata["layout"] != "so101":
        raise ValueError("Gen3 joint_1 through joint_7 cannot be replayed on the SO-101; "
                         "use the CSV with named SO-101 joints. Automatic joint remapping is unsupported.")
    fixed_gripper = _fixed_gripper(config)
    names = metadata["joint_names"]
    raw = arrays["positions_rad"]
    return np.column_stack([
        raw[:, names.index(name)] if name in names else np.full(len(raw), fixed_gripper)
        for name in JOINT_NAMES
    ])


def csv_to_reference(
    metadata: dict, arrays: dict, model: dict, config: dict, *,
    code: dict | None = None, max_samples: int = 1_000_000,
) -> tuple[dict, dict]:
    """Resample robot-time joint angles; retain the first pose and final endpoint.

    This constructs a reference but does not assert replay feasibility. Call
    validate_csv_source and validate_for_replay before executing any motion.
    """
    import numpy as np

    _sample_bound(max_samples, "max_samples")
    hz = _positive_rate(config.get("control_hz"), "control_hz")
    physics_hz = _positive_rate(config.get("physics_hz"), "physics_hz")
    if physics_hz % hz:
        raise ValueError("physics_hz must be divisible by control_hz")
    raw = _so101_positions(metadata, arrays, config)
    times = arrays["robot_time_s"] - arrays["robot_time_s"][0]
    last_step = float(times[-1]) * hz
    if not math.isfinite(last_step) or last_step > max_samples - 1:
        raise ValueError(f"Resampled CSV exceeds max_samples={max_samples}")
    samples = math.ceil(last_step) + 1
    sampled_times = np.arange(samples, dtype=np.float64) / hz
    positions = np.column_stack([
        np.interp(sampled_times, times, raw[:, joint]) for joint in range(len(JOINT_NAMES))
    ])
    reference_arrays = {"timestamps_s": sampled_times, "positions_rad": positions}
    resolved_config = json.loads(json.dumps(config, allow_nan=False))
    source_csv = {
        **metadata, "time_column": "robot_time_s", "time_origin_s": float(arrays["robot_time_s"][0]),
        "source_duration_s": float(times[-1]),
        "gripper": "supplied CSV column" if "gripper" in metadata["joint_names"] else
                   f"inserted fixed {_fixed_gripper(config):g} radians",
        "asset_binding": "named joint angles interpreted using the configured SO-101 URDF; source calibration unverified",
        "raw_context": "both timestamps and tool XYZ preserved in source.csv when archived for replay",
    }
    reference_metadata = {
        "schema_version": "so101-trajectory/1", "trajectory_id": Path(metadata["source"]["path"]).stem,
        "data_kind": "reference", "seed": None,
        "generator": {"name": "csv-robot-time-linear", "version": "1"},
        "resolved_generation_config": {
            "replay_config": resolved_config, "time_column": "robot_time_s",
            "interpolation": "piecewise linear in joint radians", "max_samples": max_samples,
            "retiming": "none; subtract timestamp origin and pad final pose to next control tick",
        },
        "source_csv": source_csv, "initial_pose_policy": "first_sample",
        "fixed_gripper_rad": _fixed_gripper(config),
        "code": code if code is not None else {"revision": None, "dirty": None},
        "joint_names": list(JOINT_NAMES), "joint_types": ["revolute"] * len(JOINT_NAMES),
        "units": {"time": "seconds", "positions": "radians"}, "control_hz": hz,
        "asset": {**model, "urdf": config.get("urdf"), "repository": config.get("source_repository"),
                  "license": config.get("source_license")},
        "end_effector": {"urdf_link": "gripper_frame_link", "cartesian_poses": "not generated or measured"},
        "scene": {"id": "unspecified-in-csv", "coordinate_frame": "unspecified"},
        "timing": {
            "sample_dt_s": 1 / hz, "control_dt_s": 1 / hz, "intended_physics_dt_s": 1 / physics_hz,
            "resampling": "linear interpolation of robot_time_s onto the uniform control grid",
            "source_sample_count": len(raw), "source_duration_s": float(times[-1]),
            "end_padding_s": float(sampled_times[-1] - times[-1]),
            "initialization": "set first CSV pose before physics; no approach motion from zero",
            "terminal_hold_s": 1 / hz,
        },
        "validation": {
            "csv_structure": "passed", "collision": "unchecked", "workspace": "unchecked",
            "continuous_acceleration": "unchecked; linear segments can have velocity jumps at knots",
            "jerk": "unchecked", "measured_dynamics": "unchecked", "dataset_eligible": False,
        },
        "software": {"python": sys.version, "numpy": np.__version__, "simulator": None},
    }
    validate_trajectory(reference_metadata, reference_arrays)
    return reference_metadata, reference_arrays


def validate_csv_source(metadata: dict, arrays: dict, model: dict, config: dict) -> dict:
    """Check every original knot and interval, including features missed by resampling."""
    import numpy as np

    positions = _so101_positions(metadata, arrays, config)
    margin = _finite_number(config.get("position_margin_rad"), "position_margin_rad")
    if margin < 0:
        raise ValueError("position_margin_rad must be nonnegative")
    maximum_speed = _finite_number(config.get("reference_velocity_limit_rad_s"),
                                   "reference_velocity_limit_rad_s", positive=True)
    fixed_gripper = _fixed_gripper(config)
    if not np.allclose(positions[:, -1], fixed_gripper, rtol=0, atol=1e-12):
        raise ValueError(f"CSV replay requires the gripper held at {fixed_gripper:g} radians")
    with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
        speeds = np.abs(np.diff(positions, axis=0) / np.diff(arrays["robot_time_s"])[:, None])
    peaks = {}
    for column, name in enumerate(JOINT_NAMES):
        joint = model["joints"][name]
        lo = _finite_number(joint.get("lower"), f"{name}.lower") + margin
        hi = _finite_number(joint.get("upper"), f"{name}.upper") - margin
        limit = min(maximum_speed, _finite_number(joint.get("velocity"), f"{name}.velocity", positive=True))
        if lo > hi or np.any(positions[:, column] < lo) or np.any(positions[:, column] > hi):
            raise ValueError(f"CSV source positions for {name} exceed the applied joint limits/margin")
        peak = float(speeds[:, column].max())
        if not math.isfinite(peak) or peak > limit + 1e-12:
            raise ValueError(f"CSV source velocity for {name} reaches {peak:.6g} rad/s; "
                             f"applied limit is {limit:.6g} rad/s")
        peaks[name] = peak
    return {"method": "all source knots and piecewise linear interval slopes using robot_time_s",
            "position": "passed", "velocity": "passed", "peak_velocity_by_joint_rad_s": peaks,
            "continuous_acceleration": "unchecked; velocity can jump at source knots"}
