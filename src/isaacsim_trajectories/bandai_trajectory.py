"""Import Bandai V2's cubic joint path and independent local robot clock.

Only NumPy is needed, and it is imported at call time. The delivered package is
input data, not executable code. The resulting uniform reference still needs
the usual replay screening, including its transitions from and to rest.
"""
from __future__ import annotations

import hashlib
import io
import json
import math
from pathlib import Path
import re
import sys
import zipfile

from .so101_demo import JOINT_NAMES
from .trajectory import _finite_number, _positive_rate, validate_trajectory

NPZ_ARRAYS = frozenset({"path_c", "path_x", "grid", "squared_speed", "clock", "joint_names"})
MAX_FILE_BYTES = 64 * 1024 * 1024
MAX_ARRAY_BYTES = 128 * 1024 * 1024


def _read(path: Path, archive_name: str, sources: list) -> bytes:
    with path.open("rb") as stream:
        data = stream.read(MAX_FILE_BYTES + 1)
    if len(data) > MAX_FILE_BYTES:
        raise ValueError(f"Bandai source exceeds {MAX_FILE_BYTES} bytes: {path.name}")
    sources.append({"path": str(path), "archive_name": archive_name,
                    "sha256": hashlib.sha256(data).hexdigest()})
    return data


def _json(data: bytes) -> dict:
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Duplicate Bandai metadata key: {key}")
            result[key] = value
        return result

    result = json.loads(data, object_pairs_hook=unique)
    if not isinstance(result, dict):
        raise ValueError("Bandai metadata must be an object")
    json.dumps(result, allow_nan=False)
    return result


def _read_curve(source: bytes) -> dict:
    import numpy as np

    try:
        with zipfile.ZipFile(io.BytesIO(source)) as archive:
            entries = archive.infolist()
            if (len(entries) != len(NPZ_ARRAYS)
                    or {entry.filename for entry in entries} != {f"{key}.npy" for key in NPZ_ARRAYS}
                    or sum(entry.file_size for entry in entries) > MAX_ARRAY_BYTES):
                raise ValueError("Unexpected Bandai NPZ arrays or uncompressed size limit exceeded")
            # Check declared shapes before NumPy allocates from an NPY header.
            # A tiny damaged archive can otherwise declare an enormous array.
            for entry in entries:
                with archive.open(entry) as stream:
                    version = np.lib.format.read_magic(stream)
                    if version == (1, 0):
                        shape, _, dtype = np.lib.format.read_array_header_1_0(stream)
                    elif version == (2, 0):
                        shape, _, dtype = np.lib.format.read_array_header_2_0(stream)
                    else:
                        raise ValueError("Unsupported Bandai NPY header version")
                    expected_bytes = math.prod(shape) * dtype.itemsize
                    if (dtype.hasobject or expected_bytes > MAX_ARRAY_BYTES
                            or expected_bytes != entry.file_size - stream.tell()):
                        raise ValueError("Unsafe Bandai NPZ dtype, shape, or array size")
        with np.load(io.BytesIO(source), allow_pickle=False) as archive:
            arrays = {key: archive[key].copy() for key in NPZ_ARRAYS}
    except zipfile.BadZipFile as exc:
        raise ValueError("Invalid Bandai NPZ archive") from exc
    names = arrays["joint_names"]
    if (names.ndim != 1 or names.dtype.kind != "U" or len(names) != 5
            or set(names.tolist()) != set(JOINT_NAMES[:-1])):
        raise ValueError("Bandai NPZ must contain five unique named SO-101 joints; Gen3 is unsupported")
    for key in NPZ_ARRAYS - {"joint_names"}:
        value = arrays[key]
        if value.dtype.kind not in "fiu" or not np.isfinite(value).all():
            raise ValueError(f"Bandai {key} must contain finite real numbers")
        arrays[key] = value.astype(np.float64)
    x, c = arrays["path_x"], arrays["path_c"]
    grid, clock, speed = (arrays[key] for key in ("grid", "clock", "squared_speed"))
    if (x.ndim != 1 or len(x) < 2 or c.shape != (4, len(x) - 1, 5)
            or x[0] != 0 or np.any(np.diff(x) <= 0)):
        raise ValueError("Invalid Bandai cubic coefficient shape or path knots")
    if (grid.ndim != 1 or len(grid) < 2 or clock.shape != grid.shape or speed.shape != grid.shape
            or grid[0] != 0 or clock[0] != 0 or np.any(np.diff(grid) <= 0)
            or np.any(np.diff(clock) <= 0) or np.any(speed <= 0)
            or grid[-1] != x[-1]):
        raise ValueError("Invalid Bandai clock, positive squared speed, or path coverage")
    expected_dt = 2 * np.diff(grid) / (np.sqrt(speed[:-1]) + np.sqrt(speed[1:]))
    if not np.allclose(np.diff(clock), expected_dt, rtol=1e-8, atol=1e-10):
        raise ValueError("Bandai clock is inconsistent with its squared-speed law")
    if len(x) > 2:
        widths = np.diff(x)[:-1, None]
        previous_q = ((c[0, :-1] * widths + c[1, :-1]) * widths + c[2, :-1]) * widths + c[3, :-1]
        previous_dq = (3 * c[0, :-1] * widths + 2 * c[1, :-1]) * widths + c[2, :-1]
        if (not np.allclose(previous_q, c[3, 1:], rtol=1e-8, atol=1e-8)
                or not np.allclose(previous_dq, c[2, 1:], rtol=1e-8, atol=1e-8)):
            raise ValueError("Bandai cubic path must have continuous positions and first derivatives")
    return arrays


def _interior_roots(coefficients, width: float) -> list[float]:
    import numpy as np

    roots = np.roots(np.trim_zeros(coefficients, "f"))
    return [float(root.real) for root in roots
            if abs(root.imag) <= 1e-9 and 0 < root.real < width]


def _validate_continuous(curve: dict, model: dict, config: dict, gripper: float) -> dict:
    """Check analytic extrema on every intersection of path and clock pieces.

    In source coordinate s, q is cubic and squared progress speed u is linear.
    Velocity is q' sqrt(u); acceleration is q'' u + q' u'/2 (quadratic).
    Thus position, velocity and acceleration extrema use at most quadratic roots.
    """
    import numpy as np

    margin = _finite_number(config.get("position_margin_rad"), "position_margin_rad")
    if margin < 0:
        raise ValueError("position_margin_rad must be nonnegative")
    velocity_limit = _finite_number(config.get("reference_velocity_limit_rad_s"),
                                   "reference_velocity_limit_rad_s", positive=True)
    acceleration_limit = _finite_number(config.get("reference_acceleration_limit_rad_s2"),
                                       "reference_acceleration_limit_rad_s2", positive=True)
    names = curve["joint_names"].tolist()
    x, c, grid, speed = (curve[key] for key in ("path_x", "path_c", "grid", "squared_speed"))
    boundaries = np.union1d(x, grid)
    minima = np.full(5, np.inf)
    maxima = np.full(5, -np.inf)
    velocity = np.zeros(5)
    acceleration = np.zeros(5)
    for start, stop in zip(boundaries[:-1], boundaries[1:]):
        width = float(stop - start)
        p = min(int(np.searchsorted(x, start, side="right")) - 1, len(x) - 2)
        k = min(int(np.searchsorted(grid, start, side="right")) - 1, len(grid) - 2)
        offset = start - x[p]
        speed_slope = (speed[k + 1] - speed[k]) / (grid[k + 1] - grid[k])
        initial_speed = speed[k] + speed_slope * (start - grid[k])
        for j in range(5):
            c3, c2, c1, c0 = c[:, p, j]
            b3, b2 = c3, c2 + 3 * c3 * offset
            b1 = c1 + 2 * c2 * offset + 3 * c3 * offset**2
            b0 = ((c3 * offset + c2) * offset + c1) * offset + c0
            a0 = 2 * b2 * initial_speed + .5 * speed_slope * b1
            a1 = 6 * b3 * initial_speed + 3 * b2 * speed_slope
            a2 = 7.5 * b3 * speed_slope
            position_times = np.array([0., width, *_interior_roots([3*b3, 2*b2, b1], width)])
            q = ((b3 * position_times + b2) * position_times + b1) * position_times + b0
            velocity_times = np.array([0., width, *_interior_roots([a2, a1, a0], width)])
            v = ((3*b3*velocity_times + 2*b2)*velocity_times + b1) * np.sqrt(
                initial_speed + speed_slope * velocity_times)
            acceleration_times = np.array([0., width, *_interior_roots([2*a2, a1], width)])
            a = (a2 * acceleration_times + a1) * acceleration_times + a0
            minima[j] = min(minima[j], float(q.min()))
            maxima[j] = max(maxima[j], float(q.max()))
            velocity[j] = max(velocity[j], float(np.abs(v).max()))
            acceleration[j] = max(acceleration[j], float(np.abs(a).max()))
    report = {}
    for j, name in enumerate([*names, "gripper"]):
        joint = model["joints"][name]
        if joint.get("type") != "revolute":
            raise ValueError(f"Bandai replay requires revolute {name}")
        lower = _finite_number(joint.get("lower"), f"{name}.lower") + margin
        upper = _finite_number(joint.get("upper"), f"{name}.upper") - margin
        limit = min(velocity_limit, _finite_number(joint.get("velocity"), f"{name}.velocity", positive=True))
        lo, hi, v, a = (minima[j], maxima[j], velocity[j], acceleration[j]) if j < 5 else (gripper, gripper, 0., 0.)
        if lower > upper or lo < lower - 1e-10 or hi > upper + 1e-10:
            raise ValueError(f"Bandai continuous positions for {name} exceed the applied joint limits/margin")
        if not all(math.isfinite(value) for value in (lo, hi, v, a)):
            raise ValueError(f"Nonfinite Bandai continuous extrema for {name}")
        if v > limit + 1e-10:
            raise ValueError(f"Bandai continuous velocity for {name} reaches {v:.6g} rad/s; applied limit is {limit:.6g}")
        if a > acceleration_limit + 1e-10:
            raise ValueError(f"Bandai continuous acceleration for {name} reaches {a:.6g} rad/s^2; applied limit is {acceleration_limit:.6g}")
        report[name] = {"min_position_rad": float(lo), "max_position_rad": float(hi),
                        "peak_velocity_rad_s": float(v), "peak_acceleration_rad_s2": float(a),
                        "applied_lower_rad": lower, "applied_upper_rad": upper,
                        "applied_velocity_limit_rad_s": limit,
                        "applied_acceleration_limit_rad_s2": acceleration_limit}
    return {"status": "passed", "method": "analytic extrema on each cubic path / linear squared-speed intersection",
            "scope": "continuous interior only; start/stop transitions require sampled replay screening",
            "numerical_limit_tolerance": 1e-10, "continuity_rtol": 1e-8, "continuity_atol": 1e-8,
            "clock_rtol": 1e-8, "clock_atol_s": 1e-10,
            "interval_count": len(boundaries) - 1, "joints": report}


def _sample(curve: dict, times):
    import numpy as np

    clock, grid, speed = (curve[key] for key in ("clock", "grid", "squared_speed"))
    # Control ticks after the exact duration hold its endpoint. Never retime.
    t = np.minimum(times, clock[-1])
    k = np.minimum(np.searchsorted(clock, t, side="right") - 1, len(clock) - 2)
    elapsed = t - clock[k]
    source_acceleration = (speed[k + 1] - speed[k]) / (2 * (grid[k + 1] - grid[k]))
    source = grid[k] + np.sqrt(speed[k]) * elapsed + .5 * source_acceleration * elapsed**2
    # Algebraic clock inversion can stray a few ulps at a knot.
    source = np.minimum(np.maximum(source, grid[k]), grid[k + 1])
    x, c = curve["path_x"], curve["path_c"]
    p = np.minimum(np.searchsorted(x, source, side="right") - 1, len(x) - 2)
    delta = (source - x[p])[:, None]
    return ((c[0, p] * delta + c[1, p]) * delta + c[2, p]) * delta + c[3, p]


def bandai_to_reference(path: Path, model: dict, config: dict, *, code: dict | None = None,
                        max_samples: int = 1_000_000) -> tuple[dict, dict]:
    """Validate and sample a delivered SO-101 NPZ without changing its timing.

    Keep the package layout intact: motions/dataset_N/clip/so101.npz, its JSON
    sidecar, and provenance/robot_models/so101/{model.urdf,waypoint_model.json}.
    The source URDF bytes must match the selected replay model exactly.
    """
    import numpy as np

    path = Path(path).resolve()
    if (path.name != "so101.npz" or len(path.parents) < 4 or path.parents[2].name != "motions"
            or not re.fullmatch(r"dataset_\d+", path.parents[1].name)):
        raise ValueError("Expected Bandai motions/dataset_N/clip/so101.npz; Gen3 replay is unsupported")
    if isinstance(max_samples, bool) or not isinstance(max_samples, int) or max_samples < 2:
        raise ValueError("max_samples must be an integer >= 2")
    hz = _positive_rate(config.get("control_hz"), "control_hz")
    physics_hz = _positive_rate(config.get("physics_hz"), "physics_hz")
    if physics_hz % hz:
        raise ValueError("physics_hz must be divisible by control_hz")
    sources = []
    npz = _read(path, "source.npz", sources)
    sidecar = _json(_read(path.with_suffix(".json"), "source.json", sources))
    model_path = path.parents[3] / "provenance/robot_models/so101"
    declaration = _json(_read(model_path / "waypoint_model.json", "waypoint_model.json", sources))
    urdf = _read(model_path / "model.urdf", "source_model.urdf", sources)
    if (sidecar.get("format_version") != 1 or isinstance(sidecar.get("format_version"), bool)
            or sidecar.get("robot") != "so101" or declaration.get("robot") != "so101"
            or sidecar.get("joint_units") != "radians" or declaration.get("joint_units") != "radians"
            or sidecar.get("time_units") != "seconds" or sidecar.get("tool_units") != "meters"
            or sidecar.get("exact_trajectory") != path.name):
        raise ValueError("Unsupported Bandai metadata version, robot, units, or trajectory identity")
    for key, expected in (("root_frame", "base_link"), ("tool_frame", "gripper_frame_link")):
        if sidecar.get(key) != expected or declaration.get(key) != expected:
            raise ValueError(f"Unsupported Bandai {key}")
    digest = hashlib.sha256(urdf).hexdigest()
    if sidecar.get("urdf_sha256") != digest or model.get("source_sha256") != digest:
        raise ValueError("Bandai source URDF hash does not match the selected replay model")
    gripper = _finite_number(declaration.get("fixed_gripper_rad"), "fixed_gripper_rad")
    try:
        with np.errstate(over="raise", divide="raise", invalid="raise"):
            curve = _read_curve(npz)
            names = curve["joint_names"].tolist()
            if sidecar.get("joint_names") != names or declaration.get("joint_names") != names:
                raise ValueError("Bandai metadata joint order disagrees with the NPZ")
            continuous = _validate_continuous(curve, model, config, gripper)
            duration = float(curve["clock"][-1])
            last_step = duration * hz
            if not math.isfinite(last_step) or last_step > max_samples - 1:
                raise ValueError(f"Resampled Bandai trajectory exceeds max_samples={max_samples}")
            times = np.arange(math.ceil(last_step) + 1, dtype=np.float64) / hz
            raw = _sample(curve, times)
            positions = np.column_stack([raw[:, names.index(name)] for name in JOINT_NAMES[:-1]]
                                        + [np.full(len(times), gripper)])
    except (FloatingPointError, OverflowError) as exc:
        raise ValueError("Numerical overflow in Bandai curve or clock") from exc
    resolved_config = json.loads(json.dumps(config, allow_nan=False))
    resolved_config.update(fixed_gripper_rad=gripper, gripper_policy="hold fixed radians")
    metadata = {
        "schema_version": "so101-trajectory/1", "trajectory_id": path.parent.name,
        "data_kind": "reference", "seed": None,
        "generator": {"name": "bandai-v2-cubic-local-clock", "version": "1"},
        "resolved_generation_config": {"replay_config": resolved_config,
            "interpolation": "delivered cubic path composed with its local robot clock",
            "retiming": "none; final pose padded to the next control tick", "max_samples": max_samples},
        "source_bandai": {"source_files": sources, "metadata": sidecar, "model_declaration": declaration,
            "asset_binding": "source URDF SHA256 matches selected replay URDF",
            "upstream_validation": "source claims retained as provenance; replay validation computed independently",
            "package_root": str(path.parents[3])},
        "initial_pose_policy": "first_sample", "fixed_gripper_rad": gripper,
        "code": code if code is not None else {"revision": None, "dirty": None},
        "joint_names": list(JOINT_NAMES), "joint_types": ["revolute"] * len(JOINT_NAMES),
        "units": {"time": "seconds", "positions": "radians"}, "control_hz": hz,
        "asset": {**model, "urdf": config.get("urdf"), "repository": config.get("source_repository"),
                  "license": config.get("source_license")},
        "end_effector": {"urdf_link": "gripper_frame_link", "cartesian_poses": "not generated or measured",
                         "source_root_frame": "base_link"},
        "scene": {"id": "unspecified-in-bandai", "coordinate_frame": "source robot base_link"},
        "timing": {"sample_dt_s": 1 / hz, "control_dt_s": 1 / hz, "intended_physics_dt_s": 1 / physics_hz,
            "resampling": "evaluate exact cubic/local-clock curve on the uniform control grid",
            "source_duration_s": duration, "source_path_knot_count": len(curve["path_x"]),
            "source_clock_knot_count": len(curve["clock"]), "end_padding_s": float(times[-1] - duration),
            "initialization": "set first source pose before physics; no approach motion from zero",
            "terminal_hold_s": 1 / hz},
        "validation": {"continuous_reference": continuous, "collision": "unchecked", "workspace": "unchecked",
                       "jerk": "unchecked", "measured_dynamics": "unchecked", "dataset_eligible": False},
        "software": {"python": sys.version, "numpy": np.__version__, "simulator": None},
    }
    arrays = {"timestamps_s": times, "positions_rad": positions}
    validate_trajectory(metadata, arrays)
    return metadata, arrays
