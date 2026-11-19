"""Real USD scene checks, separate from the ordinary offline test suite.

Run through the supported Isaac Sim launcher:
    bash scripts/launch_isaacsim.sh --python tests/simulator/test_scene_loading.py

These checks use Isaac Sim's USD runtime, but do not establish robot dynamics,
rendering, or collision validity. Ordinary Python reports an explicit skip.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from isaacsim_trajectories.scenes import resolve_scene
from isaacsim_trajectories.sim_scene import compose_environment


class SceneLoadingTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)

    def source(self, filename="environment.usda"):
        path = self.directory / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        stage = Usd.Stage.CreateNew(str(path))
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
        return stage, path

    def selection(self, stage, path):
        stage.GetRootLayer().Save()
        return {
            "stage": str(path),
            "stage_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }

    def reject(self, source, path, message):
        destination = Usd.Stage.CreateInMemory()
        with self.assertRaisesRegex(ValueError, message):
            compose_environment(destination, self.selection(source, path))
        self.assertEqual(list(destination.GetRootLayer().subLayerPaths), [])

    def test_builtin_empty_and_shipped_table(self):
        for name in ("empty", "table"):
            with self.subTest(scene=name):
                scene = resolve_scene(ROOT / "configs/scenes.json", name)
                stage = Usd.Stage.CreateInMemory()
                compose_environment(stage, scene)
                self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 1)
                self.assertEqual(UsdGeom.GetStageUpAxis(stage), "Z")
                self.assertFalse(stage.GetPrimAtPath("/World/so101"))
                if name == "table":
                    tabletop = stage.GetPrimAtPath("/World/Environment/Tabletop")
                    self.assertTrue(tabletop.IsA(UsdGeom.Cube))
                    self.assertTrue(tabletop.HasAPI(UsdPhysics.CollisionAPI))
                    self.assertIn("passed", scene["stage_validation"])

    def test_relative_nested_reference_and_source_preservation(self):
        component, component_path = self.source("parts/component.usda")
        cube = UsdGeom.Cube.Define(component, "/World/Marker")
        cube.CreateSizeAttr(0.04)
        component.GetRootLayer().Save()
        source, path = self.source()
        assembly = UsdGeom.Xform.Define(source, "/World/Environment/Assembly").GetPrim()
        assembly.GetReferences().AddReference("parts/component.usda")
        scene = self.selection(source, path)
        before = {file: file.read_bytes() for file in (path, component_path)}

        destination = Usd.Stage.CreateInMemory()
        compose_environment(destination, scene)
        marker = UsdGeom.Cube(destination.GetPrimAtPath("/World/Environment/Assembly/Marker"))
        self.assertTrue(marker)
        self.assertEqual(marker.GetSizeAttr().Get(), 0.04)
        marker.GetSizeAttr().Set(0.08)
        output = self.directory / "different-folder" / "replay.usda"
        output.parent.mkdir()
        self.assertTrue(destination.GetRootLayer().Export(str(output)))

        reopened = Usd.Stage.Open(str(output))
        self.assertEqual(reopened.GetCompositionErrors(), [])
        self.assertEqual(UsdGeom.Cube(reopened.GetPrimAtPath(
            "/World/Environment/Assembly/Marker")).GetSizeAttr().Get(), 0.08)
        self.assertEqual({file: file.read_bytes() for file in before}, before)
        self.assertEqual(cube.GetSizeAttr().Get(), 0.04)

    def test_units_and_axis_are_rejected_before_composition(self):
        for name, meters, axis in (("centimeters", 0.01, "Z"), ("y_up", 1, "Y")):
            with self.subTest(case=name):
                source, path = self.source(f"{name}.usda")
                UsdGeom.SetStageMetersPerUnit(source, meters)
                UsdGeom.SetStageUpAxis(source, axis)
                self.reject(source, path, "meters.*Z-up")

    def test_reserved_paths_are_rejected(self):
        for index, reserved in enumerate(("/World/so101", "/World/light", "/World/physicsScene")):
            with self.subTest(path=reserved):
                source, path = self.source(f"reserved-{index}.usda")
                UsdGeom.Xform.Define(source, reserved)
                self.reject(source, path, "reserved replay path")

    def test_world_transform_and_inactive_world_are_rejected(self):
        for case in ("translation", "scale", "inactive"):
            with self.subTest(case=case):
                source, path = self.source(f"world-{case}.usda")
                world = UsdGeom.Xform(source.GetPrimAtPath("/World"))
                if case == "translation":
                    world.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.5))
                elif case == "scale":
                    world.AddScaleOp().Set(Gf.Vec3f(2, 2, 2))
                else:
                    world.GetPrim().SetActive(False)
                self.reject(source, path, "Scene /World")

    def test_extra_robot_or_physics_objects_are_rejected(self):
        for case in ("articulation", "rigid_body", "joint", "physics_scene"):
            with self.subTest(case=case):
                source, path = self.source(f"physics-{case}.usda")
                prim = UsdGeom.Xform.Define(source, "/World/Environment/Object").GetPrim()
                if case == "articulation":
                    UsdPhysics.ArticulationRootAPI.Apply(prim)
                elif case == "rigid_body":
                    UsdPhysics.RigidBodyAPI.Apply(prim)
                elif case == "joint":
                    UsdPhysics.FixedJoint.Define(source, "/World/Environment/Joint")
                else:
                    UsdPhysics.Scene.Define(source, "/OtherPhysicsScene")
                self.reject(source, path, "static environment")

    def test_instance_proxy_cannot_hide_rigid_body(self):
        component, component_path = self.source("parts/robot.usda")
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Cube.Define(component, "/World/Body").GetPrim())
        component.GetRootLayer().Save()
        source, path = self.source()
        instance = UsdGeom.Xform.Define(source, "/World/Environment/Instance").GetPrim()
        instance.GetReferences().AddReference("parts/robot.usda")
        instance.SetInstanceable(True)
        self.assertTrue(instance.IsInstance())
        self.assertTrue(source.GetPrimAtPath("/World/Environment/Instance/Body").IsInstanceProxy())
        self.reject(source, path, "static environment")

    def test_animated_scene_attribute_is_rejected(self):
        source, path = self.source()
        translation = UsdGeom.Xform.Define(source, "/World/MovingObject").AddTranslateOp()
        translation.Set(Gf.Vec3d(0, 0, 0), Usd.TimeCode(0))
        translation.Set(Gf.Vec3d(0, 0, 1), Usd.TimeCode(1))
        self.reject(source, path, "Animated scene")

    def test_missing_reference_is_rejected(self):
        source, path = self.source()
        prim = UsdGeom.Xform.Define(source, "/World/MissingObject").GetPrim()
        prim.GetReferences().AddReference("does-not-exist.usda")
        self.reject(source, path, "composition errors")

    def test_malformed_stage_is_rejected(self):
        path = self.directory / "malformed.usda"
        path.write_text("#usda 1.0\ndef Xform \"Unclosed\" {\n")
        scene = {"stage": str(path), "stage_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        destination = Usd.Stage.CreateInMemory()
        with self.assertRaises((ValueError, Tf.ErrorException)):
            compose_environment(destination, scene)
        self.assertEqual(list(destination.GetRootLayer().subLayerPaths), [])

    def test_changed_source_hash_is_rejected(self):
        source, path = self.source()
        scene = self.selection(source, path)
        path.write_text(path.read_text() + "\n# edited after selection\n")
        destination = Usd.Stage.CreateInMemory()
        with self.assertRaisesRegex(ValueError, "changed after selection"):
            compose_environment(destination, scene)
        self.assertEqual(list(destination.GetRootLayer().subLayerPaths), [])


def main() -> int:
    # Checking the launcher environment keeps ordinary Python from accidentally
    # importing a partial Isaac installation or starting the simulator.
    if not os.environ.get("ISAAC_SIM_PATH"):
        print("SKIPPED: scene-loading simulator checks require ISAAC_SIM_PATH and the Isaac Sim launcher.")
        return 0
    try:
        from isaacsim import SimulationApp
    except ImportError as error:
        print(f"SKIPPED: Isaac Sim is unavailable in this Python runtime ({error}).")
        return 0

    app = None
    exit_code = 1
    try:
        app = SimulationApp({
            "headless": True, "renderer": "MinimalRendering", "disable_viewport_updates": True,
            "extra_args": ["--/isaac/startup/ros_bridge_extension=",
                           "--/isaac/startup/ros_sim_control_extension=false"],
        })
        global Gf, Tf, Usd, UsdGeom, UsdPhysics
        from pxr import Gf, Tf, Usd, UsdGeom, UsdPhysics

        result = unittest.TextTestRunner(verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromTestCase(SceneLoadingTests))
        exit_code = 0 if result.wasSuccessful() else 1
        return exit_code
    finally:
        if app is not None:
            app.close(exit_code=exit_code)


if __name__ == "__main__":
    sys.exit(main())
