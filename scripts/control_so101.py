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
import time
from typing import Callable

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from isaacsim_trajectories.so101_demo import load_model
from isaacsim_trajectories.demo_log import validate_log
from isaacsim_trajectories.provenance import git_identity
from isaacsim_trajectories.trajectory import load_trajectory, save_trajectory, validate_for_replay
from isaacsim_trajectories.csv_trajectory import (
    csv_to_reference, read_csv_trajectory, validate_csv_source,
)
from isaacsim_trajectories.bandai_trajectory import bandai_to_reference
from isaacsim_trajectories.scenes import load_scene_catalog, resolve_scene


def render_until(is_running: Callable[[], bool], render: Callable[[], None],
                 deadline: float | None) -> bool:
    """Keep the viewer responsive without physics steps; None waits for close.

    A deadline is monotonic wall time, used only for presentation. Always render
    at least once when running, even if a slow frame has missed the deadline.
    """
    while is_running():
        render()
        remaining = None if deadline is None else deadline - time.monotonic()
        if remaining is not None and remaining <= 0:
            return is_running()
        time.sleep(1 / 60 if remaining is None else min(1 / 60, remaining))
    return False


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path,
                        help="Reference directory, named SO-101 CSV, or Bandai V2 motions/.../so101.npz (exact curve and robot clock)")
    parser.add_argument("--csv-gripper-rad", type=float,
                        help="Fixed CSV gripper angle in radians (default: 0; Bandai V2 CSVs use 0.35). NPZ reads its package model.")
    parser.add_argument("--scene", default="empty", help="Scene name in the catalog (default: empty)")
    parser.add_argument("--scenes-config", type=Path, default=PROJECT / "configs/scenes.json",
                        help="Scene catalog JSON; stage paths are relative to this file")
    parser.add_argument("--list-scenes", action="store_true", help="List configured scenes without Isaac Sim")
    parser.add_argument("--config", type=Path, default=PROJECT / "configs/so101-demo.json")
    parser.add_argument("--urdf", type=Path, help="Override the source URDF path")
    parser.add_argument("--headless", action="store_true", help="Run without a window and exit after replay")
    parser.add_argument("--start-delay", type=float, default=3.0, metavar="SECONDS",
                        help="Windowed preview before motion (default: 3 seconds; no physics steps)")
    parser.add_argument("--playback-speed", type=float, default=1.0, metavar="FACTOR",
                        help="Windowed viewing speed (default: 1; 0.5 is half speed). Does not change simulation timing")
    parser.add_argument("--exit-on-complete", action="store_true",
                        help="Close the window after saving logs instead of keeping the final pose open")
    parser.add_argument("--check", action="store_true", help="Validate model and reference motion without Isaac Sim")
    args = parser.parse_args(argv)
    if args.trajectory is None and not args.list_scenes:
        parser.error("--trajectory is required unless --list-scenes is used")
    if args.csv_gripper_rad is not None:
        if not math.isfinite(args.csv_gripper_rad):
            parser.error("--csv-gripper-rad must be finite")
        if args.trajectory is None or args.trajectory.suffix.lower() != ".csv":
            parser.error("--csv-gripper-rad is only supported with a CSV trajectory")
    if not math.isfinite(args.start_delay) or args.start_delay < 0:
        parser.error("--start-delay must be finite and nonnegative")
    if not math.isfinite(args.playback_speed) or args.playback_speed <= 0:
        parser.error("--playback-speed must be finite and positive")
    return args


def prepare(args: argparse.Namespace) -> tuple:
    scene = resolve_scene(args.scenes_config, args.scene)
    config = json.loads(args.config.read_text())
    if config.get("schema_version") != "so101-replay-config/1":
        raise ValueError("Unsupported replay configuration schema")
    config["scene"] = scene
    urdf = (args.urdf or PROJECT / config["urdf"]).resolve()
    config["urdf"] = str(urdf)
    model = load_model(urdf)
    if args.trajectory.suffix.lower() == ".csv":
        if args.csv_gripper_rad is not None:
            config["gripper_policy"] = "hold fixed radians"
            config["fixed_gripper_rad"] = args.csv_gripper_rad
        csv_metadata, csv_arrays = read_csv_trajectory(args.trajectory)
        trajectory_metadata, trajectory_arrays = csv_to_reference(
            csv_metadata, csv_arrays, model, config, code=git_identity(PROJECT),
        )
        trajectory_metadata["validation"]["csv_source"] = validate_csv_source(
            csv_metadata, csv_arrays, model, config,
        )
    elif args.trajectory.suffix.lower() == ".npz":
        trajectory_metadata, trajectory_arrays = bandai_to_reference(
            args.trajectory, model, config, code=git_identity(PROJECT),
        )
    else:
        trajectory_metadata, trajectory_arrays = load_trajectory(args.trajectory)
    config["fixed_gripper_rad"] = trajectory_metadata.get("fixed_gripper_rad", 0.0)
    config["gripper_policy"] = "hold fixed radians" if config["fixed_gripper_rad"] else "hold zero radians"
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
    background = config.setdefault("background_color_rgb", [0.65, 0.65, 0.65])
    if not isinstance(background, list) or len(background) != 3 or not all(
        isinstance(value, (int, float)) and not isinstance(value, bool)
        and math.isfinite(value) and 0 <= value <= 1 for value in background
    ):
        raise ValueError("background_color_rgb must contain three finite numbers between 0 and 1")
    return config, urdf, model, trajectory_metadata, trajectory_arrays, validation


def archive_reference(input_path: Path, output_directory: Path, metadata: dict, arrays: dict) -> dict:
    """Archive external inputs and their exact prepared reference before startup."""
    input_path = Path(input_path).resolve()
    directory = input_path
    files = ["metadata.json", "trajectory.npz"]
    sources = []
    if input_path.suffix.lower() == ".csv":
        sources = [{"path": str(input_path), "archive_name": "source.csv",
                    "sha256": metadata["source_csv"]["source"]["sha256"]}]
    elif input_path.suffix.lower() == ".npz":
        sources = metadata["source_bandai"]["source_files"]
    if sources:
        contents = {}
        for item in sources:
            name = item["archive_name"]
            if Path(name).name != name or name in files or name in contents:
                raise ValueError("Invalid or duplicate source archive filename")
            source = Path(item["path"]).read_bytes()
            if hashlib.sha256(source).hexdigest() != item["sha256"]:
                raise ValueError(f"Source {name} changed after validation; check the input again before replay")
            contents[name] = source
        directory = output_directory / "reference"
        save_trajectory(directory, metadata, arrays)
        for name, source in contents.items():
            (directory / name).write_bytes(source)
        files.extend(contents)
    return {
        "input_path": str(input_path), "directory": str(directory), "metadata": metadata,
        "files_sha256": {name: hashlib.sha256((directory / name).read_bytes()).hexdigest() for name in files},
    }


def run_simulation(args: argparse.Namespace, config: dict, urdf: Path, model: dict,
                   trajectory_metadata: dict, trajectory_arrays: dict,
                   validation: dict, output: Path) -> None:
    # Minimal mode 2 accepts the background settings but renders black in this
    # build. Use the verified RTX renderer for both replay modes.
    renderer = "RaytracedLighting"
    metadata = {
        "schema_version": "so101-control-log/1", "run_id": output.name,
        "status": "starting", "failure_reason": None, "seed": trajectory_metadata["seed"],
        "algorithm": trajectory_metadata["generator"], "resolved_config": config,
        "trajectory": archive_reference(args.trajectory, output, trajectory_metadata, trajectory_arrays),
        "asset": {"urdf": str(urdf), **model, "git": git_identity(urdf.parent)},
        "code": git_identity(PROJECT), "reference_validation": validation,
        "units": {"time": "seconds", "positions": "radians", "velocities": "radians/second"},
        "scene": dict(config["scene"]),
        "validation": {"collision": "unchecked", "workspace": "unchecked",
                       "measured_acceleration": "unchecked", "dataset_eligible": False},
        "viewing": {"start_delay_s": 0.0 if args.headless else args.start_delay,
                    "playback_speed": None if args.headless else args.playback_speed,
                    "keep_open_after_completion": not (args.headless or args.exit_on_complete),
                    "pacing": "unpaced" if args.headless else
                              "wall-clock minimum per control interval; physics schedule unchanged"},
        "timing": {"physics_dt_s": 1 / config["physics_hz"],
                   "control_dt_s": 1 / config["control_hz"],
                   "measurement_dt_s": 1 / config["physics_hz"],
                   "command_interpolation": "prepared reference samples, zero-order hold; no resampling during physics",
                   "reference_preparation": trajectory_metadata.get("timing", {}),
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
        # Only SimulationApp is imported before the extension system starts.
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": args.headless, "renderer": renderer,
                             "minimal_shading_mode": 2,
                             "disable_viewport_updates": args.headless, "extra_args": [
            "--/isaac/startup/ros_bridge_extension=", "--/isaac/startup/ros_sim_control_extension=false",
            f"--/rtx/rendermode={renderer}", f"--/rtx-defaults/rendermode={renderer}"
        ]})
        import carb.settings
        import numpy as np
        import omni.kit.app
        import omni.usd
        from pxr import UsdGeom, UsdLux, UsdPhysics, Gf, Sdf
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
        from isaacsim_trajectories.sim_scene import compose_environment

        metadata["software"] = {"python": sys.version, "numpy": np.__version__,
                                "isaac_sim": (Path(os.environ["ISAAC_SIM_PATH"]) / "VERSION").read_text().strip(),
                                "physics_device": "cpu", "headless": args.headless,
                                "renderer": renderer,
                                "minimal_shading_mode": 2 if renderer == "MinimalRendering" else None}
        manager = omni.kit.app.get_app().get_extension_manager()
        for extension in ("omni.scene.optimizer.core", "isaacsim.robot.schema", "isaacsim.asset.importer.urdf"):
            manager.set_extension_enabled_immediate(extension, True)
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        from isaacsim.asset.importer.utils import fix_articulation_root_for_fixed_base

        print(f"Scene: {config['scene']['name']} ({config['scene']['id']})", flush=True)
        print("Importing SO-101 URDF...", flush=True)

        ## Import URDF to USD
        imported = URDFImporter(URDFImporterConfig(
            urdf_path=str(urdf), usd_path=str(output / "robot"), fix_base=True,
            robot_type="Manipulator", merge_fixed_joints=True, allow_self_collision=False,
            joint_drive_type="force", joint_target_type="position",
            override_joint_stiffness=config["stiffness_nm_per_rad"],
            override_joint_damping=config["damping_nm_s_per_rad"],
        )).import_urdf()
        if not imported or not Path(imported).is_file():
            raise RuntimeError("URDF import did not produce a USD asset")
        metadata["asset"]["imported_usd"] = str(imported)  # Save the path to the imported USD into metadata

        ## Initializing stage
        stage_utils.create_new_stage()
        app.reset_render_settings()
        stage = omni.usd.get_context().get_stage()
        compose_environment(stage, metadata["scene"])
        # Add robot to the stage
        stage_utils.add_reference_to_stage(usd_path=str(imported), path="/World/so101", variants=[("Physics", "physx")])
        metadata["asset"]["physics_variant"] = "physx"
        metadata["asset"]["fixed_base_root_relocations"] = fix_articulation_root_for_fixed_base(stage)
        # Add light to the stage
        light = UsdLux.DistantLight.Define(stage, "/World/light")
        light.CreateIntensityAttr(3000)
        UsdGeom.Xformable(light).AddRotateXYZOp().Set(Gf.Vec3f(-35, -25, 0))
        app.set_setting("/rtx/sceneDb/ambientLightIntensity", 0.3)
        carb.settings.get_settings().set_float_array("/rtx/sceneDb/ambientLightColor", [1.0, 1.0, 1.0])
        metadata["scene"]["lighting"] = {"type": "distant", "intensity": 3000,
                                         "rotation_xyz_deg": [-35, -25, 0],
                                         "ambient_intensity": 0.3, "ambient_color": [1, 1, 1]}

        ## Checking status of the stage
        if UsdGeom.GetStageMetersPerUnit(stage) != 1.0 or UsdGeom.GetStageUpAxis(stage) != "Z":
            raise RuntimeError("Scene must be meters and Z-up")
        roots = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise RuntimeError(f"Expected one articulation, found {[str(p.GetPath()) for p in roots]}")
        root_path = str(roots[0].GetPath())
        fixed_to_world = []
        # Check if robot is anchored to the world by a fixed joint
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.FixedJoint):
                joint = UsdPhysics.FixedJoint(prim)
                bodies = (joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets())
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
        # Set gravity
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)

        ## Setting up simulation iteration
        SimulationManager.setup_simulation(dt=1 / config["physics_hz"], device="cpu")
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
        # Load trajectory
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
        # Isaac Sim 6.2 stores viewport settings on the USD render product.
        # Apply to the active product after initialization rather than relying
        # on the deprecated global Carb settings to synchronize to the viewport.
        render_product = ViewportManager.get_render_product()
        if not args.headless and not render_product:
            raise RuntimeError("No active viewport render product for background configuration")
        if render_product:
            prim = render_product.GetPrim()
            prim.CreateAttribute("omni:rtx:background:source:type", Sdf.ValueTypeNames.Token, custom=False).Set("color")
            prim.CreateAttribute("omni:rtx:background:source:color", Sdf.ValueTypeNames.Color3f, custom=False).Set(
                Gf.Vec3f(*config["background_color_rgb"]))
            prim.CreateAttribute("omni:rtx:post:compositing:postTonemapBackgroundComposite",
                                 Sdf.ValueTypeNames.Bool, custom=False).Set(False)
        metadata["scene"]["background"] = {"source": "color", "rgb": config["background_color_rgb"],
                                           "composite_after_tonemapping": False,
                                           "method": "USD render product attributes after physics initialization",
                                           "render_product": str(render_product.GetPath()) if render_product else None}
        stage.GetRootLayer().Export(str(output / "scene.usda"))
        metadata["status"] = "running"
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
        samples.append((0.0, target[0].copy(), initial, arm.get_dof_velocities().numpy()[0].copy()))
        ticks_per_control = config["physics_hz"] // config["control_hz"]
        # Hold every saved sample for one control interval, including the last.
        # Uniform timestamps/rate compatibility were validated before startup.
        steps = len(reference) * ticks_per_control
        initial_time = SimulationManager.get_simulation_time()
        duration = steps / config["physics_hz"]
        expected_excursion = np.max(np.abs(reference - reference[0]), axis=0)
        moving = ", ".join(f"{name}: {math.degrees(value):.1f} degrees"
                           for name, value in zip(names, expected_excursion) if value > 1e-6)
        print(f"Motion: {moving or 'stationary'}; {duration:.3f} simulation seconds", flush=True)
        if not args.headless:
            print(f"Starting in {args.start_delay:g} seconds; viewing speed {args.playback_speed:g}x."
                  " Physics timing is unchanged.", flush=True)
            if not render_until(app.is_running, RenderingManager.render, time.monotonic() + args.start_delay):
                raise RuntimeError("Application closed before motion started")
        print(f"Replaying {trajectory_metadata['trajectory_id']}; order: {names}", flush=True)
        for tick in range(steps):
            if not app.is_running():
                raise RuntimeError("Application closed before motion completed")
            if tick % ticks_per_control == 0:
                # Per-interval pacing avoids racing to catch up after slow renders.
                # It never changes targets, dt, number of steps, or recorded time.
                frame_started = time.monotonic()
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
            if (tick + 1) % config["physics_hz"] == 0 or tick + 1 == steps:
                print(f"Replay: {t:.2f}/{duration:.2f} simulation seconds", flush=True)
            if not args.headless and (tick + 1) % ticks_per_control == 0:
                deadline = frame_started + 1 / config["control_hz"] / args.playback_speed
                if not render_until(app.is_running, RenderingManager.render, deadline):
                    raise RuntimeError("Application closed before motion completed")
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
                    print(f"Saved {metadata['samples']} samples to {output}\n"
                          f"Maximum tracking error: {metadata['max_tracking_error_rad']:.6f} rad", flush=True)
                    if not args.headless and not args.exit_on_complete:
                        print("Replay complete. Keeping the final pose open without advancing physics.\n"
                              "Close the Isaac Sim window or press Ctrl+C in this terminal to exit.", flush=True)
                        try:
                            render_until(app.is_running, RenderingManager.render, None)
                        except KeyboardInterrupt:
                            pass  # Logs are already saved; closing the viewer is not a failed replay.
                else:
                    print(f"Replay failed; diagnostic metadata and available samples: {output}",
                          file=sys.stderr, flush=True)
            finally:
                if app is not None:
                    app.close(exit_code=0 if metadata["status"] == "completed" else 1)


def main() -> None:
    args = parse_args()
    if args.list_scenes:
        for name, scene in load_scene_catalog(args.scenes_config).items():
            print(f"{name}: {scene['description']} [stage: {scene['stage'] or 'built-in empty'}]")
        return
    try:
        config, urdf, model, trajectory_metadata, trajectory_arrays, validation = prepare(args)
    except (OSError, ValueError, KeyError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2) from None
    if args.check:
        print(json.dumps({"urdf": str(urdf), "trajectory_id": trajectory_metadata["trajectory_id"],
                          "input_path": str(args.trajectory.resolve()),
                          "generator": trajectory_metadata["generator"],
                          "fixed_gripper_rad": trajectory_metadata.get("fixed_gripper_rad", 0.0),
                          "reference_preparation": trajectory_metadata.get("timing", {}),
                          "source_validation": trajectory_metadata.get("validation", {}),
                          "scene": config["scene"], "validation": validation}, indent=2))
        return
    run_root = PROJECT / "runs"
    run_root.mkdir(exist_ok=True)
    prefix = "so101-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-")
    output = Path(tempfile.mkdtemp(prefix=prefix, dir=run_root))
    run_simulation(args, config, urdf, model, trajectory_metadata, trajectory_arrays, validation, output)


if __name__ == "__main__":
    main()
