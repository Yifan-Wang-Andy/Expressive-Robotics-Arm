"""USD scene composition, called only after Isaac Sim's SimulationApp starts.

Offline registry resolution lives in scenes.py. Environment sources are never
edited: the replay stage sublayers them and authors robot/settings in its own
root layer. This first slice supports static, meter/Z-up environments.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any


def compose_environment(stage: Any, scene: dict) -> None:
    """Validate and compose the selected environment into a new replay stage.

    Mutates only the new stage and the run's scene metadata. Fail before robot
    import/control for incompatible environments instead of changing their units
    or removing conflicting prims. USD imports stay behind the simulator entry.
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    if scene["stage"] is not None:
        source_path = Path(scene["stage"])
        if hashlib.sha256(source_path.read_bytes()).hexdigest() != scene["stage_sha256"]:
            raise ValueError("Scene source changed after selection; select it again before replay")
        source = Usd.Stage.Open(str(source_path))
        if source is None:
            raise ValueError(f"Could not open scene stage: {source_path}")
        if source.GetCompositionErrors():
            raise ValueError(f"Scene has USD composition errors: {source.GetCompositionErrors()}")
        if UsdGeom.GetStageMetersPerUnit(source) != 1.0 or UsdGeom.GetStageUpAxis(source) != "Z":
            raise ValueError("Scene source must use meters (metersPerUnit=1) and Z-up; no automatic conversion")
        for path in ("/World/so101", "/World/light", "/World/physicsScene"):
            if source.GetPrimAtPath(path):
                raise ValueError(f"Scene uses reserved replay path {path}")
        # An inherited World transform would also transform the imported robot
        # and its fixed joint, silently changing placement or scale.
        world = source.GetPrimAtPath("/World")
        if world:
            if not world.IsActive() or world.IsInstance() or world.IsA(UsdGeom.Gprim):
                raise ValueError("Scene /World must be an active, non-instanced container")
            if UsdGeom.XformCache().GetLocalToWorldTransform(world) != Gf.Matrix4d(1):
                raise ValueError("Scene /World transform must be identity; transform environment children instead")
        # Traverse instance proxies as well, so a nested instance cannot hide a
        # second robot or physics scene from the environment-only checks.
        for prim in Usd.PrimRange.Stage(source, Usd.TraverseInstanceProxies()):
            if (prim.HasAPI(UsdPhysics.ArticulationRootAPI) or prim.HasAPI(UsdPhysics.RigidBodyAPI)
                    or prim.IsA(UsdPhysics.Joint) or prim.IsA(UsdPhysics.Scene)):
                raise ValueError(f"Scene must be a static environment without robots, rigid bodies, joints, "
                                 f"or physics scenes: {prim.GetPath()}")
            if any(attribute.ValueMightBeTimeVarying() for attribute in prim.GetAttributes()):
                raise ValueError(f"Animated scene attributes are not supported: {prim.GetPath()}")
        stage.GetRootLayer().subLayerPaths.append(str(source_path))
        if stage.GetCompositionErrors():
            raise ValueError(f"Replay stage has USD composition errors: {stage.GetCompositionErrors()}")
        scene["stage_validation"] = "passed static environment, units, frame, and reserved-path checks"
        scene["composition"] = "source sublayer; replay edits authored in a separate root layer"
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
