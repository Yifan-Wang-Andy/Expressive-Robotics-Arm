#!/usr/bin/env python3
"""Generate a dummy SO-101 reference file without starting Isaac Sim.

Requires ordinary Python and NumPy. Pass the resulting directory to
control_so101.py --trajectory DIRECTORY using Isaac Sim's bundled Python.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from uuid import uuid4

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from isaacsim_trajectories.provenance import git_identity
from isaacsim_trajectories.so101_demo import JOINT_NAMES, load_model, sample_motion, validate_motion


def generate_dummy(config: dict, model: dict) -> tuple[dict, dict]:
    """Dummy method: sample one smooth out-and-back joint move, with others held.

    A later generator can replace this function while keeping the saved
    timestamps/positions interface unchanged. No simulator APIs are used here.
    """
    import numpy as np

    validation = validate_motion(
        model, config["joint"], config["offset_rad"], config["move_seconds"],
        config["physics_hz"], config["control_hz"], config["reference_velocity_limit_rad_s"],
        config["reference_acceleration_limit_rad_s2"], config["position_margin_rad"],
    )
    timestamps = np.arange(validation["control_steps"] + 1, dtype=float) / config["control_hz"]
    # Make the endpoint exactly the configured duration despite floating-point division.
    timestamps[-1] = validation["duration_seconds"]
    positions = np.zeros((len(timestamps), len(JOINT_NAMES)), dtype=float)
    index = JOINT_NAMES.index(config["joint"])
    positions[:, index] = [sample_motion(float(t), config["offset_rad"], config["move_seconds"])
                           for t in timestamps]
    return validation, {"timestamps_s": timestamps, "positions_rad": positions}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PROJECT / "configs/dummy-trajectory.json")
    parser.add_argument("--urdf", type=Path, help="Override the source model path")
    parser.add_argument("--joint", help="Arm joint to move; the gripper stays fixed")
    parser.add_argument("--offset-rad", type=float)
    parser.add_argument("--move-seconds", type=float, help="Duration of each direction")
    parser.add_argument("--output", type=Path, help="New output directory; existing directories are rejected")
    parser.add_argument("--check", action="store_true", help="Validate and report without saving files")
    args = parser.parse_args()
    from isaacsim_trajectories.trajectory import save_trajectory, validate_trajectory
    config = json.loads(args.config.read_text())
    if not isinstance(config, dict) or config.get("schema_version") != "dummy-trajectory-config/1":
        raise ValueError("Unsupported dummy generation configuration schema")
    for field in ("joint", "offset_rad", "move_seconds"):
        value = getattr(args, field)
        if value is not None:
            config[field] = value
    urdf = (args.urdf or PROJECT / config["urdf"]).resolve()
    config["urdf"] = str(urdf)
    model = load_model(urdf)
    validation, arrays = generate_dummy(config, model)
    prefix = "trajectory-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-")
    output = (args.output or PROJECT / "runs" / (prefix + uuid4().hex[:12])).resolve()
    import numpy as np
    metadata = {
        "schema_version": "so101-trajectory/1", "trajectory_id": output.name,
        "data_kind": "reference", "seed": None,
        "generator": {"name": "dummy-quintic-out-and-back", "version": "1"},
        "sampling_distribution": "deterministic", "resolved_generation_config": config,
        "code": git_identity(PROJECT), "joint_names": list(JOINT_NAMES),
        "joint_types": ["revolute"] * len(JOINT_NAMES),
        "units": {"time": "seconds", "positions": "radians"}, "control_hz": config["control_hz"],
        "asset": {**model, "urdf": str(urdf), "git": git_identity(urdf.parent),
                  "repository": config["source_repository"], "license": config["source_license"]},
        "end_effector": {"urdf_link": "gripper_frame_link", "cartesian_poses": "not provided"},
        "scene": {"id": "so101-fixed-empty/1", "meters_per_unit": 1.0, "up_axis": "Z",
                  "fixed_base": True, "gravity_m_s2": [0.0, 0.0, -9.81], "obstacles": []},
        "timing": {"sample_dt_s": 1 / config["control_hz"], "control_dt_s": 1 / config["control_hz"],
                   "intended_physics_dt_s": 1 / config["physics_hz"], "resampling": "none",
                   "interpolation": "continuous quintic reference sampled on the control grid; replay holds targets"},
        "validation": {"joint_reference": validation, "collision": "unchecked", "workspace": "unchecked",
                       "dataset_eligible": False, "rejection_reasons": []},
        "software": {"python": sys.version, "numpy": np.__version__, "simulator": None},
    }
    validate_trajectory(metadata, arrays)
    if args.check:
        print(json.dumps({"samples": len(arrays["timestamps_s"]), "joint_names": list(JOINT_NAMES),
                          "validation": validation}, indent=2))
        return
    save_trajectory(output, metadata, arrays)
    print(f"Trajectory directory: {output}")
    print(f"Saved {len(arrays['timestamps_s'])} reference samples over {validation['duration_seconds']:g} seconds.")


if __name__ == "__main__":
    main()
