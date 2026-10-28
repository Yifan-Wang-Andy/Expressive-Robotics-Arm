"""Validation and loading for the versioned SO-101 demonstration log."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping

from .so101_demo import JOINT_NAMES


SCHEMA_VERSION = "so101-control-log/1"
ARRAY_NAMES = frozenset({
    "timestamps_s", "commanded_positions_rad", "measured_positions_rad", "measured_velocities_rad_s"
})


def validate_log(metadata: Mapping[str, Any], arrays: Mapping[str, Any]) -> None:
    """Reject malformed logs; preserve nonfinite raw measurements in failed runs.

    Array columns are interpreted in metadata's joint_names order. No canonical
    numeric joint indices are assumed. NumPy is needed only when this is called.
    """
    import numpy as np

    if metadata.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Unsupported control-log schema")
    status = metadata.get("status")
    if status not in ("completed", "failed"):
        raise ValueError("Log status must be completed or failed")
    names = metadata.get("joint_names")
    if (
        not isinstance(names, list) or len(names) != len(JOINT_NAMES)
        or not all(isinstance(name, str) for name in names)
        or set(names) != set(JOINT_NAMES)
    ):
        raise ValueError("joint_names must contain each of the six SO-101 joints exactly once")
    if set(arrays) != ARRAY_NAMES:
        raise ValueError(f"Expected exactly these log arrays: {sorted(ARRAY_NAMES)}")
    values = {name: np.asarray(array) for name, array in arrays.items()}
    for name, array in values.items():
        if array.dtype.kind not in "fiu":
            raise ValueError(f"{name} must contain real numeric data")
    timestamps = values["timestamps_s"]
    if timestamps.ndim != 1 or len(timestamps) == 0:
        raise ValueError("timestamps_s must have shape (N,) with N > 0")
    count = len(timestamps)
    if not np.isfinite(timestamps).all() or not np.all(timestamps[1:] > timestamps[:-1]):
        raise ValueError("Log timestamps must be finite and strictly increasing")
    for name in ARRAY_NAMES - {"timestamps_s"}:
        if values[name].shape != (count, len(names)):
            raise ValueError(f"{name} must have shape ({count}, {len(names)})")
    if not np.isfinite(values["commanded_positions_rad"]).all():
        raise ValueError("Commanded positions must be finite")
    if status == "completed" and any(not np.isfinite(array).all() for array in values.values()):
        raise ValueError("Completed logs must contain finite measurements")
    if "samples" in metadata and metadata["samples"] != count:
        raise ValueError("Metadata sample count disagrees with the arrays")

    def check_error(value: Any) -> None:
        if value is None and status == "failed":
            return
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("Tracking-error summaries must be finite nonnegative numbers, or null for failed runs")
        if not math.isfinite(value) or value < 0:
            raise ValueError("Tracking-error summaries must be finite and nonnegative")

    if "max_tracking_error_rad" in metadata:
        check_error(metadata["max_tracking_error_rad"])
        if not np.isfinite(values["measured_positions_rad"]).all() and metadata["max_tracking_error_rad"] is not None:
            raise ValueError("Nonfinite measured positions require a null maximum tracking error")
    if "tracking_error_by_joint_rad" in metadata:
        errors = metadata["tracking_error_by_joint_rad"]
        if not isinstance(errors, dict) or set(errors) != set(names):
            raise ValueError("Per-joint tracking errors must use the recorded joint names")
        for value in errors.values():
            check_error(value)


def load_log(directory: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Load metadata.json and motion.npz without pickle, then validate the schema."""
    import numpy as np

    directory = Path(directory)
    metadata = json.loads((directory / "metadata.json").read_text())
    if not isinstance(metadata, dict):
        raise ValueError("Log metadata must be a JSON object")
    with np.load(directory / "motion.npz", allow_pickle=False) as archive:
        if set(archive.files) != ARRAY_NAMES or len(archive.files) != len(ARRAY_NAMES):
            raise ValueError("Unexpected or duplicated arrays in motion.npz")
        arrays = {name: archive[name].copy() for name in archive.files}
    validate_log(metadata, arrays)
    return metadata, arrays
