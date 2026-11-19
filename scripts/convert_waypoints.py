#!/usr/bin/env python3
"""Convert timed SO-101 joint waypoints to a validated replay directory.

Uses ordinary Python and NumPy; no Isaac Sim or GPU is needed. See
docs/trajectory-input.md and configs/example-waypoints.json for the input format.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from uuid import uuid4

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))

from isaacsim_trajectories.provenance import git_identity
from isaacsim_trajectories.so101_demo import load_model
from isaacsim_trajectories.trajectory import save_trajectory, validate_for_replay
from isaacsim_trajectories.waypoints import sample_waypoints

MAX_SAMPLES = 1_000_000


def _unique_fields(pairs: list[tuple[str, object]]) -> dict:
    """Avoid silently accepting the last of two conflicting JSON fields."""
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON field: {key}")
        result[key] = value
    return result


def convert(args: argparse.Namespace) -> tuple[Path, dict, dict]:
    """Load, interpolate, and validate; leave saving to the caller."""
    import numpy as np

    source = args.input.resolve()
    source_bytes = source.read_bytes()
    document = json.loads(source_bytes, object_pairs_hook=_unique_fields)
    config = json.loads(args.config.read_bytes(), object_pairs_hook=_unique_fields)
    if not isinstance(config, dict) or config.get("schema_version") != "so101-replay-config/1":
        raise ValueError("Unsupported replay configuration schema")
    for field in ("urdf", "source_repository", "source_license", "settings_provenance"):
        if not isinstance(config.get(field), str) or not config[field].strip():
            raise ValueError(f"Replay configuration requires a nonempty {field}")
    if config.get("gripper_policy") != "hold zero radians":
        raise ValueError("Converter requires gripper_policy: hold zero radians")
    # Reject nonfinite or otherwise unserializable provenance as well as motion.
    json.dumps(config, allow_nan=False)
    urdf = (args.urdf or PROJECT / config["urdf"]).resolve()
    config["urdf"] = str(urdf)
    model = load_model(urdf)
    continuous, arrays = sample_waypoints(document, model, config, max_samples=MAX_SAMPLES)
    prefix = "waypoints-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-")
    output = (args.output or PROJECT / "runs" / (prefix + uuid4().hex[:12])).resolve()
    names = document["joint_names"]
    metadata = {
        "schema_version": "so101-trajectory/1", "trajectory_id": output.name,
        "data_kind": "reference", "seed": None,
        "generator": {"name": "waypoint-quintic-rest-to-rest", "version": "1"},
        "sampling_distribution": "deterministic conversion of supplied waypoints",
        "source_waypoints": {"path": str(source), "sha256": hashlib.sha256(source_bytes).hexdigest()},
        "resolved_generation_config": {
            "waypoints": document, "replay_config": config, "max_samples": MAX_SAMPLES,
            "interpolation": {
                "method": "piecewise rest-to-rest quintic",
                "blend": "10*u^3 - 15*u^4 + 6*u^5",
                "waypoint_velocity_rad_s": 0.0, "waypoint_acceleration_rad_s2": 0.0,
                "retiming": "none; times must lie on the control grid within roundoff tolerance",
            },
        },
        "code": git_identity(PROJECT), "joint_names": names,
        "joint_types": ["revolute"] * len(names),
        "units": {"time": "seconds", "positions": "radians"}, "control_hz": config["control_hz"],
        "asset": {**model, "urdf": str(urdf), "git": git_identity(urdf.parent),
                  "repository": config["source_repository"], "license": config["source_license"]},
        "end_effector": {"urdf_link": "gripper_frame_link", "cartesian_poses": "not provided"},
        "scene": {"id": "so101-fixed-empty/1", "meters_per_unit": 1.0, "up_axis": "Z",
                  "fixed_base": True, "gravity_m_s2": [0.0, 0.0, -9.81], "obstacles": []},
        "timing": {
            "sample_dt_s": 1 / config["control_hz"], "control_dt_s": 1 / config["control_hz"],
            "intended_physics_dt_s": 1 / config["physics_hz"],
            "resampling": "timed waypoints sampled on the uniform control grid",
            "interpolation": "continuous piecewise quintic reference; replay holds targets",
            "initialization": "replay sets the required zero first pose before control",
            "terminal_hold_s": 1 / config["control_hz"],
        },
        "validation": {
            "joint_reference": continuous, "collision": "unchecked", "workspace": "unchecked",
            "jerk": "unchecked", "measured_dynamics": "unchecked",
            "dataset_eligible": False, "rejection_reasons": [],
        },
        "software": {"python": sys.version, "numpy": np.__version__, "simulator": None},
    }
    metadata["validation"]["replay_screen"] = validate_for_replay(
        metadata, arrays, model, physics_hz=config["physics_hz"], control_hz=config["control_hz"],
        position_margin_rad=config["position_margin_rad"],
        velocity_limit_rad_s=config["reference_velocity_limit_rad_s"],
        acceleration_limit_rad_s2=config["reference_acceleration_limit_rad_s2"],
    )
    return output, metadata, arrays


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="JSON file using the so101-waypoints/1 format")
    parser.add_argument("--config", type=Path, default=PROJECT / "configs/so101-demo.json",
                        help="Replay config supplying the model, rates, and reference limits")
    parser.add_argument("--urdf", type=Path, help="Override the source model path")
    parser.add_argument("--output", type=Path, help="New trajectory directory; default is unique under runs/")
    parser.add_argument("--check", action="store_true", help="Validate and report without saving files")
    args = parser.parse_args()
    try:
        output, metadata, arrays = convert(args)
        if args.check:
            print(json.dumps({"samples": len(arrays["timestamps_s"]), "joint_names": metadata["joint_names"],
                              "validation": metadata["validation"]}, indent=2, allow_nan=False))
            return
        save_trajectory(output, metadata, arrays)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(2, f"error: {exc}\n")
    print(f"Trajectory directory: {output}")
    print(f"Saved {len(arrays['timestamps_s'])} reference samples over {arrays['timestamps_s'][-1]:g} seconds.")


if __name__ == "__main__":
    main()
