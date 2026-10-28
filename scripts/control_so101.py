#!/usr/bin/env python3
"""Launch Isaac Sim 6.2 and replay a supplied SO-101 trajectory.

Use scripts/launch_isaacsim.sh --python, not system Python, for simulation.
System Python can run --check or --help without importing simulator packages.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from isaacsim_trajectories.so101_demo import load_model
from isaacsim_trajectories.demo_log import validate_log
from isaacsim_trajectories.provenance import git_identity
from isaacsim_trajectories.trajectory import load_trajectory, validate_for_replay


def prepare(args: argparse.Namespace) -> tuple:
    config = json.loads(args.config.read_text())
    if config.get("schema_version") != "so101-replay-config/1":
        raise ValueError("Unsupported replay configuration schema")
    urdf = (args.urdf or PROJECT / config["urdf"]).resolve()
    config["urdf"] = str(urdf)
    model = load_model(urdf)
    trajectory_metadata, trajectory_arrays = load_trajectory(args.trajectory)
    validation = validate_for_replay(
        trajectory_metadata, trajectory_arrays, model,
        physics_hz=config["physics_hz"], control_hz=config["control_hz"],
        position_margin_rad=config["position_margin_rad"],
        velocity_limit_rad_s=config["reference_velocity_limit_rad_s"],
        acceleration_limit_rad_s2=config["reference_acceleration_limit_rad_s2"],
    )
    for key in ("stiffness_nm_per_rad", "damping_nm_s_per_rad", "tracking_error_limit_rad",
                "measured_position_tolerance_rad"):
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    return config, urdf, model, trajectory_metadata, trajectory_arrays, validation


def run_simulation(args: argparse.Namespace, config: dict, urdf: Path, model: dict,
                   trajectory_metadata: dict, trajectory_arrays: dict,
                   validation: dict, output: Path) -> None:
    # Only SimulationApp is imported before the Omniverse extension system starts.
    from isaacsim import SimulationApp

    metadata = {
        "schema_version": "so101-control-log/1", "run_id": output.name,
        "status": "starting", "failure_reason": None, "seed": trajectory_metadata["seed"],
        "algorithm": trajectory_metadata["generator"], "resolved_config": config,
        "trajectory": {
            "directory": str(args.trajectory.resolve()), "metadata": trajectory_metadata,
            "files_sha256": {name: hashlib.sha256((args.trajectory / name).read_bytes()).hexdigest()
                             for name in ("metadata.json", "trajectory.npz")},
        },
        "asset": {"urdf": str(urdf), **model, "git": git_identity(urdf.parent)},
        "code": git_identity(PROJECT), "reference_validation": validation,
        "units": {"time": "seconds", "positions": "radians", "velocities": "radians/second"},
        "scene": {"id": "so101-fixed-empty/1", "meters_per_unit": 1.0, "up_axis": "Z",
                  "gravity_m_s2": [0, 0, -9.81]},
        "validation": {"collision": "unchecked", "workspace": "unchecked",
                       "measured_acceleration": "unchecked", "dataset_eligible": False},
        "timing": {"physics_dt_s": 1 / config["physics_hz"],
                   "control_dt_s": 1 / config["control_hz"],
                   "measurement_dt_s": 1 / config["physics_hz"],
                   "command_interpolation": "saved reference samples, zero-order hold; no resampling",
                   "reference_duration_s": float(trajectory_arrays["timestamps_s"][-1]),
                   "final_sample_hold_s": 1 / config["control_hz"],
                   "sample_alignment": "measured after step; command is target held during that step"},
    }
    metadata_path = output / "metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
    app = None
    samples: list[tuple] = []
    names: list[str] = []
    np = None
    try:
        print(f"Run directory: {output}", flush=True)
        app = SimulationApp({"headless": args.headless, "renderer": "MinimalRendering",
                             "disable_viewport_updates": args.headless, "extra_args": [
            "--/isaac/startup/ros_bridge_extension=", "--/isaac/startup/ros_sim_control_extension=false",
            "--/rtx/rendermode=MinimalRendering", "--/rtx-defaults/rendermode=MinimalRendering"
        ]})
        import numpy as np
        import omni.kit.app
        import omni.usd
        from pxr import UsdGeom, UsdLux, UsdPhysics, Gf
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.rendering_manager import RenderingManager, ViewportManager

        metadata["software"] = {"python": sys.version, "numpy": np.__version__,
                                "isaac_sim": (Path(os.environ["ISAAC_SIM_PATH"]) / "VERSION").read_text().strip(),
                                "physics_device": "cpu", "headless": args.headless,
                                "renderer": "MinimalRendering"}
        manager = omni.kit.app.get_app().get_extension_manager()
        for extension in ("omni.scene.optimizer.core", "isaacsim.robot.schema", "isaacsim.asset.importer.urdf"):
            manager.set_extension_enabled_immediate(extension, True)
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        from isaacsim.asset.importer.utils import fix_articulation_root_for_fixed_base

        print("Importing SO-101 URDF...", flush=True)
        imported = URDFImporter(URDFImporterConfig(
            urdf_path=str(urdf), usd_path=str(output / "robot"), fix_base=True,
            robot_type="Manipulator", merge_fixed_joints=True, allow_self_collision=False,
            joint_drive_type="force", joint_target_type="position",
            override_joint_stiffness=config["stiffness_nm_per_rad"],
            override_joint_damping=config["damping_nm_s_per_rad"],
        )).import_urdf()
        if not imported or not Path(imported).is_file():
            raise RuntimeError("URDF import did not produce a USD asset")
        metadata["asset"]["imported_usd"] = str(imported)
        stage_utils.create_new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        stage_utils.add_reference_to_stage(usd_path=str(imported), path="/World/so101", variants=[("Physics", "physx")])
        metadata["asset"]["physics_variant"] = "physx"
        # This importer build can retain the root API on base_link when a
        # root_joint already exists. NVIDIA's helper roots fixed articulations
        # at that world joint so the base is actually fixed in reduced coordinates.
        metadata["asset"]["fixed_base_root_relocations"] = fix_articulation_root_for_fixed_base(stage)
        UsdLux.DomeLight.Define(stage, "/World/light").CreateIntensityAttr(800)
        if UsdGeom.GetStageMetersPerUnit(stage) != 1.0 or UsdGeom.GetStageUpAxis(stage) != "Z":
            raise RuntimeError("Scene must be meters and Z-up")
        roots = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise RuntimeError(f"Expected one articulation, found {[str(p.GetPath()) for p in roots]}")
        root_path = str(roots[0].GetPath())
        fixed_to_world = []
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.FixedJoint):
                joint = UsdPhysics.FixedJoint(prim)
                bodies = (joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets())
                # The 6.2 importer may use the non-rigid robot Xform as body0,
                # instead of an empty relationship, to express the world anchor.
                def is_world_anchor(paths):
                    return not paths or (len(paths) == 1 and str(paths[0]) == "/World/so101"
                                         and not stage.GetPrimAtPath(paths[0]).HasAPI(UsdPhysics.RigidBodyAPI))
                def is_robot_body(paths):
                    return len(paths) == 1 and str(paths[0]).startswith("/World/so101/") and stage.GetPrimAtPath(paths[0]).HasAPI(UsdPhysics.RigidBodyAPI)
                if joint.GetJointEnabledAttr().Get() and (
                    (is_world_anchor(bodies[0]) and is_robot_body(bodies[1])) or
                    (is_world_anchor(bodies[1]) and is_robot_body(bodies[0]))
                ):
                    fixed_to_world.append({"path": str(prim.GetPath()), "bodies": [[str(p) for p in b] for b in bodies]})
        if not fixed_to_world:
            raise RuntimeError("Imported arm is not anchored to the world")
        metadata["articulation_root"] = root_path
        metadata["fixed_base_joints"] = fixed_to_world
        ee_paths = [str(p.GetPath()) for p in stage.Traverse() if p.GetName() == "gripper_frame_link"]
        metadata["end_effector"] = {"urdf_link": "gripper_frame_link", "usd_paths": ee_paths,
                                    "cartesian_measurements": "not recorded"}
        if not args.headless and stage.GetPrimAtPath("/OmniverseKit_Persp"):
            ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[0.6, 0.6, 0.45], target=[0, 0, 0.15])
        scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)
        SimulationManager.setup_simulation(dt=1 / config["physics_hz"], device="cpu")
        stage.GetRootLayer().Export(str(output / "scene.usda"))
        arm = Articulation(root_path)
        SimulationManager.initialize_physics()
        if not arm.is_physics_tensor_entity_valid():
            raise RuntimeError("Articulation physics did not initialize")
        # Tensor initialization can change DOF order. Map only after it completes.
        names = list(arm.dof_names)
        if len(names) != 6 or set(names) != set(model["joints"]):
            raise RuntimeError(f"Unexpected imported joint names: {names}")
        if any(t.name != "Rotation" for t in arm.dof_types):
            raise RuntimeError(f"Unexpected DOF types: {arm.dof_types}")
        lower, upper = [x.numpy()[0] for x in arm.get_dof_limits()]
        expected_lower = np.array([model["joints"][n]["lower"] for n in names])
        expected_upper = np.array([model["joints"][n]["upper"] for n in names])
        if not np.allclose(lower, expected_lower, atol=1e-4) or not np.allclose(upper, expected_upper, atol=1e-4):
            raise RuntimeError("Imported position limits disagree with URDF")
        velocity_limits = np.array([model["joints"][n]["velocity"] for n in names])
        arm.set_dof_max_velocities(velocity_limits)
        arm.set_dof_max_efforts([model["joints"][n]["effort"] for n in names])
        # The trajectory stores its own column order; the simulator supplies its
        # runtime DOF order. Map by name, independent of how either was generated.
        columns = [trajectory_metadata["joint_names"].index(name) for name in names]
        reference = trajectory_arrays["positions_rad"][:, columns]
        target = reference[0:1].copy()
        arm.set_default_state(dof_positions=target[0], dof_velocities=np.zeros(len(names)))
        arm.reset_to_default_state()  # Initialization only; motion uses drive targets.
        arm.set_dof_position_targets(target)
        initial = arm.get_dof_positions().numpy()[0].copy()
        base_position, base_orientation = [x.numpy().copy() for x in arm.get_world_poses()]
        metadata["initial_base_pose"] = {"position_m": base_position.tolist(), "orientation_wxyz": base_orientation.tolist()}
        if not np.allclose(initial, target[0], atol=1e-5):
            raise RuntimeError("Initial joint state did not match the first trajectory sample")
        metadata["joint_names"] = names
        metadata["joint_types"] = ["revolute"] * 6
        metadata["velocity_derivation"] = "simulator measured DOF velocity"
        metadata["initial_state_rad"] = initial.tolist()
        metadata["status"] = "running"
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
        samples.append((0.0, target[0].copy(), initial, arm.get_dof_velocities().numpy()[0].copy()))
        ticks_per_control = config["physics_hz"] // config["control_hz"]
        # Hold every saved sample for one control interval, including the last.
        # Uniform timestamps/rate compatibility were validated before startup.
        steps = len(reference) * ticks_per_control
        initial_time = SimulationManager.get_simulation_time()
        print(f"Replaying {trajectory_metadata['trajectory_id']}; order: {names}", flush=True)
        for tick in range(steps):
            if not app.is_running():
                raise RuntimeError("Application closed before motion completed")
            if tick % ticks_per_control == 0:
                row = tick // ticks_per_control
                target = reference[row:row + 1].copy()
                arm.set_dof_position_targets(target)
            SimulationManager.step()
            t = SimulationManager.get_simulation_time() - initial_time
            measured = arm.get_dof_positions().numpy()[0].copy()
            velocity = arm.get_dof_velocities().numpy()[0].copy()
            samples.append((t, target[0].copy(), measured, velocity))
            if not np.isfinite(measured).all() or not np.isfinite(velocity).all():
                raise RuntimeError("Nonfinite measured state")
            position, orientation = [x.numpy() for x in arm.get_world_poses()]
            if (not np.allclose(position, base_position, atol=1e-5) or
                    not np.allclose(np.abs(np.sum(orientation * base_orientation, axis=-1)), 1, atol=1e-5)):
                metadata["failed_base_pose"] = {"position_m": position.tolist(), "orientation_wxyz": orientation.tolist()}
                raise RuntimeError(f"Fixed robot base moved: position {base_position.tolist()} -> {position.tolist()}, orientation {base_orientation.tolist()} -> {orientation.tolist()}")
            if not math.isclose(t, (tick + 1) / config["physics_hz"], abs_tol=1e-5):
                raise RuntimeError("Unexpected physics timing; possible extra stepping")
            tolerance = config["measured_position_tolerance_rad"]
            if np.any(measured < lower - tolerance) or np.any(measured > upper + tolerance):
                raise RuntimeError("Measured position exceeded URDF limits")
            if np.any(np.abs(velocity) > velocity_limits + 1e-3):
                raise RuntimeError("Measured velocity exceeded URDF limits")
            if np.max(np.abs(measured - target[0])) > config["tracking_error_limit_rad"]:
                raise RuntimeError("Tracking error threshold exceeded")
            if not args.headless and tick % ticks_per_control == 0:
                RenderingManager.render()  # Renders without another physics step.
        expected_excursion = np.max(np.abs(reference - reference[0]), axis=0)
        measured_excursion = np.max(np.abs(np.array([row[2] for row in samples]) - initial), axis=0)
        if np.any((expected_excursion > 1e-6) & (measured_excursion < 0.5 * expected_excursion)):
            raise RuntimeError("Measured arm did not follow the supplied trajectory excursion")
        metadata["measured_excursion_by_joint_rad"] = dict(zip(names, measured_excursion.tolist()))
        metadata["commanded_excursion_by_joint_rad"] = dict(zip(names, expected_excursion.tolist()))
        metadata["status"] = "completed"
    except BaseException as exc:
        metadata["status"] = "failed"
        metadata["failure_reason"] = f"{type(exc).__name__}: {exc}"
        print(f"SO101_DEMO_FAILED: {metadata['failure_reason']}", file=sys.stderr, flush=True)
        raise
    finally:
        # Persist even failed or interrupted runs; never fabricate missing samples.
        try:
            if np is not None and samples:
                arrays = {
                    "timestamps_s": np.array([row[0] for row in samples]),
                    "commanded_positions_rad": np.array([row[1] for row in samples]),
                    "measured_positions_rad": np.array([row[2] for row in samples]),
                    "measured_velocities_rad_s": np.array([row[3] for row in samples]),
                }
                if any(a.shape != (len(samples), len(names)) for k, a in arrays.items() if k != "timestamps_s"):
                    raise RuntimeError("Invalid log array dimensions")
                if not np.all(np.diff(arrays["timestamps_s"]) > 0):
                    raise RuntimeError("Log timestamps are not strictly increasing")
                metadata["samples"] = len(samples)
                errors = np.max(np.abs(arrays["commanded_positions_rad"] - arrays["measured_positions_rad"]), axis=0)
                metadata["max_tracking_error_rad"] = float(errors.max()) if np.isfinite(errors).all() else None
                metadata["tracking_error_by_joint_rad"] = {
                    name: float(error) if np.isfinite(error) else None for name, error in zip(names, errors)
                }
                validate_log(metadata, arrays)
                np.savez_compressed(output / "motion.npz", **arrays)
        except BaseException as save_error:
            metadata["status"] = "failed"
            metadata["export_failure_reason"] = f"{type(save_error).__name__}: {save_error}"
            raise
        finally:
            try:
                metadata_path.write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
                if metadata["status"] == "completed":
                    print("SO101_DEMO_COMPLETED", flush=True)
            finally:
                if app is not None:
                    app.close(exit_code=0 if metadata["status"] == "completed" else 1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, required=True,
                        help="Directory containing trajectory.npz and metadata.json")
    parser.add_argument("--config", type=Path, default=PROJECT / "configs/so101-demo.json")
    parser.add_argument("--urdf", type=Path, help="Override the source URDF path")
    parser.add_argument("--headless", action="store_true", help="Run without a window")
    parser.add_argument("--check", action="store_true", help="Validate model and reference motion without Isaac Sim")
    args = parser.parse_args()
    config, urdf, model, trajectory_metadata, trajectory_arrays, validation = prepare(args)
    if args.check:
        print(json.dumps({"urdf": str(urdf), "trajectory_id": trajectory_metadata["trajectory_id"],
                          "validation": validation}, indent=2))
        return
    run_root = PROJECT / "runs"
    run_root.mkdir(exist_ok=True)
    prefix = "so101-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-")
    output = Path(tempfile.mkdtemp(prefix=prefix, dir=run_root))
    run_simulation(args, config, urdf, model, trajectory_metadata, trajectory_arrays, validation, output)


if __name__ == "__main__":
    main()
