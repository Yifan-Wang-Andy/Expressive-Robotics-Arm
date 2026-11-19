#!/usr/bin/env python3
"""Inspect a joint CSV without robot assets or Isaac Sim; angles are radians."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))

from isaacsim_trajectories.csv_trajectory import read_csv_trajectory


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="CSV path, for example data/my-trajectory.csv")
    args = parser.parse_args()
    try:
        metadata, arrays = read_csv_trajectory(args.input)
        summary = {
            **metadata,
            "timelines": {
                name: {"start_s": float(arrays[name][0]), "end_s": float(arrays[name][-1]),
                       "duration_s": float(arrays[name][-1] - arrays[name][0])}
                for name in ("source_time_s", "robot_time_s")
            },
            "replay_time_column": "robot_time_s",
            "joint_ranges_rad": {
                name: {"min": float(arrays["positions_rad"][:, index].min()),
                       "max": float(arrays["positions_rad"][:, index].max()),
                       "initial": float(arrays["positions_rad"][0, index])}
                for index, name in enumerate(metadata["joint_names"])
            },
            "validation": "CSV structure and finite values passed; robot replay limits not checked",
        }
    except (OSError, ValueError) as exc:
        parser.exit(2, f"error: {exc}\n")
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
