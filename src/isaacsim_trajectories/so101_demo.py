"""Inspect the SO-101 model and validate a deterministic quintic joint move.

This module needs neither Isaac Sim nor NumPy. It checks joint-space reference
motion only; workspace, collision, and measured-state checks are separate.
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


JOINT_NAMES = (
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"
)


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _positive(value: Any, name: str) -> float:
    result = _finite(value, name)
    if result <= 0:
        raise ValueError(f"{name} must be positive")
    return result


def load_model(urdf: Path) -> dict[str, Any]:
    """Read the selected follower URDF; preserve identities rather than DOF indices."""
    urdf = Path(urdf).resolve()
    data = urdf.read_bytes()
    try:
        root = ET.fromstring(data)
    except ET.ParseError as exc:
        raise ValueError(f"Invalid URDF XML: {exc}") from exc
    if root.tag != "robot" or root.get("name") != "so101_new_calib":
        raise ValueError("Expected the SO-101 follower so101_new_calib URDF")

    links = [element.get("name") for element in root.findall("link")]
    if not all(links) or len(links) != len(set(links)):
        raise ValueError("Link names must be present and unique")
    if not {"base_link", "gripper_frame_link"}.issubset(links):
        raise ValueError("Expected base_link and gripper_frame_link")

    mesh_files = set()
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename or "://" in filename:
            raise ValueError("Expected a local mesh filename")
        path = urdf.parent / filename
        if not path.is_file():
            raise ValueError(f"Mesh file does not exist: {path}")
        mesh_files.add(filename)
    if not mesh_files:
        raise ValueError("URDF does not contain any meshes")

    joints: dict[str, dict[str, Any]] = {}
    names: set[str] = set()
    for element in root.findall("joint"):
        name = element.get("name")
        kind = element.get("type")
        if not name or name in names:
            raise ValueError("Joint names must be present and unique")
        names.add(name)
        for relation in ("parent", "child"):
            link = element.find(relation)
            if link is None or link.get("link") not in links:
                raise ValueError(f"Joint {name} has an invalid {relation} link")
        if element.find("mimic") is not None:
            raise ValueError(f"Mimic joint {name} is unsupported by this demo")
        if kind == "fixed":
            continue
        if kind != "revolute":
            raise ValueError(f"Joint {name}: unsupported type {kind!r}")
        limit = element.find("limit")
        if limit is None:
            raise ValueError(f"Joint {name} is missing required limits")
        values = {
            key: _finite(limit.get(key), f"{name}.{key}")
            for key in ("lower", "upper", "velocity", "effort")
        }
        if values["lower"] >= values["upper"]:
            raise ValueError(f"Joint {name} requires lower < upper")
        if values["velocity"] <= 0 or values["effort"] <= 0:
            raise ValueError(f"Joint {name} requires positive velocity and effort limits")
        joints[name] = {"type": kind, **values}
    if set(joints) != set(JOINT_NAMES):
        raise ValueError(f"Expected exactly the six SO-101 joints: {JOINT_NAMES}")
    return {
        "name": root.get("name"),
        "source_sha256": hashlib.sha256(data).hexdigest(),
        "joints": joints,
        "links": links,
        "mesh_files": sorted(mesh_files),
    }


def validate_motion(
    model: dict[str, Any],
    joint: str,
    offset_rad: float,
    move_seconds: float,
    physics_hz: int,
    control_hz: int,
    velocity_limit: float,
    acceleration_limit: float,
    margin_rad: float,
) -> dict[str, Any]:
    """Check the full outbound/return quintic, including between control samples.

    All joints begin at zero. The gripper remains at zero. Experimental velocity
    and acceleration limits are explicit settings, not manufacturer ratings.
    """
    if joint not in JOINT_NAMES or joint == "gripper":
        raise ValueError("Select one of the five arm joints; the gripper remains fixed")
    offset = _finite(offset_rad, "offset_rad")
    duration = _positive(move_seconds, "move_seconds")
    velocity = _positive(velocity_limit, "velocity_limit")
    acceleration = _positive(acceleration_limit, "acceleration_limit")
    margin = _finite(margin_rad, "margin_rad")
    if margin < 0:
        raise ValueError("margin_rad must be nonnegative")
    for name, rate in (("physics_hz", physics_hz), ("control_hz", control_hz)):
        if isinstance(rate, bool) or not isinstance(rate, int) or rate <= 0:
            raise ValueError(f"{name} must be a positive integer")
    if physics_hz % control_hz:
        raise ValueError("physics_hz must be divisible by control_hz")
    segment_steps = duration * control_hz
    if not math.isfinite(segment_steps) or not math.isclose(
        segment_steps, round(segment_steps), rel_tol=0, abs_tol=1e-8
    ) or segment_steps < 1:
        raise ValueError("move_seconds must span an integer number of control steps")

    joints = model.get("joints", {})
    if set(joints) != set(JOINT_NAMES):
        raise ValueError("Model must define exactly the six SO-101 joints")
    for name, limits in joints.items():
        if limits.get("type") != "revolute":
            raise ValueError(f"Joint {name} must be revolute")
        lower = _finite(limits.get("lower"), f"{name}.lower")
        upper = _finite(limits.get("upper"), f"{name}.upper")
        _positive(limits.get("velocity"), f"{name}.velocity")
        _positive(limits.get("effort"), f"{name}.effort")
        if lower >= upper or lower + margin > 0 or upper - margin < 0:
            raise ValueError(f"Joint {name}: zero initial position violates limits/margin")
        target = offset if name == joint else 0.0
        if not lower + margin <= target <= upper - margin:
            raise ValueError(f"Joint {name}: target violates position limits/margin")

    # s(u)=10u^3-15u^4+6u^5 is monotonic on [0,1]. Its derivative extrema
    # prove validity over the whole continuous path, rather than at waypoints.
    peak_velocity = 1.875 * abs(offset) / duration
    peak_acceleration = (10 / math.sqrt(3)) * abs(offset) / duration**2
    applied_velocity = min(velocity, joints[joint]["velocity"])
    if not math.isfinite(peak_velocity) or peak_velocity > applied_velocity:
        raise ValueError("Quintic peak velocity exceeds the applied limit")
    if not math.isfinite(peak_acceleration) or peak_acceleration > acceleration:
        raise ValueError("Quintic peak acceleration exceeds the applied limit")
    return {
        "peak_velocity_rad_s": peak_velocity,
        "peak_acceleration_rad_s2": peak_acceleration,
        "applied_velocity_limit_rad_s": applied_velocity,
        "applied_acceleration_limit_rad_s2": acceleration,
        "duration_seconds": 2 * duration,
        "physics_steps_per_control": physics_hz // control_hz,
        "control_steps": 2 * round(segment_steps),
        "method": "analytic extrema of two rest-to-rest quintic segments",
    }


def sample_motion(t: float, offset_rad: float, move_seconds: float) -> float:
    """Evaluate an outbound/return move at simulation time t in [0, 2*T]."""
    time = _finite(t, "t")
    offset = _finite(offset_rad, "offset_rad")
    duration = _positive(move_seconds, "move_seconds")
    if not 0 <= time <= 2 * duration:
        raise ValueError("t must lie within the outbound/return duration")
    u = time / duration if time <= duration else (2 * duration - time) / duration
    return offset * (u**3 * (10 + u * (-15 + 6 * u)))
