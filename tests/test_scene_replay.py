"""Offline CLI scene/trajectory combinations; these do not validate USD physics."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

try:
    import numpy as np
except ImportError:
    np = None

from isaacsim_trajectories.so101_demo import JOINT_NAMES, load_model
from isaacsim_trajectories.trajectory import load_trajectory, save_trajectory


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/control_so101.py"


class SceneSelectionCliTests(unittest.TestCase):
    def test_list_scenes_needs_neither_site_packages_nor_trajectory_or_robot_assets(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            catalog = directory / "scenes.json"
            catalog.write_text(json.dumps({
                "schema_version": "so101-scenes/1",
                "scenes": {
                    "empty": {"id": "empty/1", "description": "Empty", "stage": None},
                    "remote-workstation": {
                        "id": "elsewhere/1", "description": "Unavailable local stage",
                        "stage": "missing.usd",
                    },
                },
            }))
            completed = subprocess.run(
                [sys.executable, "-S", str(SCRIPT), "--list-scenes", "--scenes-config", str(catalog),
                 "--config", "missing-config.json", "--urdf", "missing-robot.urdf"],
                cwd=directory, capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("empty: Empty [stage: built-in empty]", completed.stdout)
        self.assertIn("remote-workstation: Unavailable local stage [stage: missing.usd]", completed.stdout)

    def test_unknown_scene_fails_before_model_or_simulator_import(self):
        completed = subprocess.run(
            [sys.executable, "-S", str(SCRIPT), "--scene", "no-such-scene",
             "--trajectory", "missing-trajectory", "--config", "missing-config.json"],
            capture_output=True, text=True)
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("Unknown scene 'no-such-scene'", completed.stderr)
        self.assertIn("Available scenes: empty, table", completed.stderr)
        self.assertNotIn("ModuleNotFoundError", completed.stderr)
        self.assertNotIn("FileNotFoundError", completed.stderr)

    def test_missing_trajectory_is_an_explicit_cli_error(self):
        completed = subprocess.run(
            [sys.executable, "-S", str(SCRIPT), "--scene", "table"],
            capture_output=True, text=True)
        self.assertEqual(completed.returncode, 2)
        self.assertIn("--trajectory is required unless --list-scenes is used", completed.stderr)
        self.assertNotIn("Traceback", completed.stderr)


@unittest.skipIf(np is None, "Optional NumPy dependency is unavailable")
class SceneTrajectoryReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("scene_replay_script", SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.urdf = self.directory / "model.urdf"
        (self.directory / "mesh.stl").write_text("synthetic existence fixture")
        robot = ET.Element("robot", name="so101_new_calib")
        base = ET.SubElement(robot, "link", name="base_link")
        ET.SubElement(robot, "link", name="gripper_frame_link")
        geometry = ET.SubElement(ET.SubElement(base, "visual"), "geometry")
        ET.SubElement(geometry, "mesh", filename="mesh.stl")
        for name in JOINT_NAMES:
            joint = ET.SubElement(robot, "joint", name=name, type="revolute")
            ET.SubElement(joint, "parent", link="base_link")
            ET.SubElement(joint, "child", link="gripper_frame_link")
            ET.SubElement(joint, "limit", lower="-2", upper="2", velocity="1", effort="10")
        ET.ElementTree(robot).write(self.urdf)
        self.config = json.loads((ROOT / "configs/so101-demo.json").read_text())
        self.config["urdf"] = str(self.urdf)
        self.config_path = self.directory / "config.json"
        self.config_path.write_text(json.dumps(self.config))
        self.metadata = {
            "schema_version": "so101-trajectory/1", "trajectory_id": "scene-test",
            "data_kind": "reference", "seed": None,
            "generator": {"name": "synthetic-fixture", "version": "1"},
            "resolved_generation_config": {}, "code": {"revision": None, "dirty": None},
            "asset": {"source_sha256": load_model(self.urdf)["source_sha256"]},
            "joint_names": list(reversed(JOINT_NAMES)), "joint_types": ["revolute"] * 6,
            "control_hz": 60, "units": {"time": "seconds", "positions": "radians"},
            "scene": {"id": "so101-fixed-empty/1", "meters_per_unit": 1.0, "up_axis": "Z"},
        }
        self.arrays = {"timestamps_s": np.arange(3) / 60, "positions_rad": np.zeros((3, 6))}
        self.arrays["positions_rad"][1, self.metadata["joint_names"].index("shoulder_pan")] = 0.0001
        self.arrays["positions_rad"][1, self.metadata["joint_names"].index("wrist_roll")] = -0.00005
        self.trajectory = self.directory / "trajectory"
        save_trajectory(self.trajectory, self.metadata, self.arrays)

    def arguments(self, *extra):
        return ["--trajectory", str(self.trajectory), "--config", str(self.config_path), *extra]

    def check(self, *extra):
        return subprocess.run(
            [sys.executable, str(SCRIPT), *self.arguments(*extra), "--check"],
            cwd=self.directory, capture_output=True, text=True)

    def source_hashes(self):
        return {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                for path in self.trajectory.iterdir()}

    def test_default_scene_retains_empty_environment_in_prepare(self):
        args = self.module.parse_args(self.arguments())
        config, _, _, metadata, arrays, validation = self.module.prepare(args)
        self.assertEqual(config["scene"]["name"], "empty")
        self.assertEqual(config["scene"]["id"], "so101-fixed-empty/1")
        self.assertIsNone(config["scene"]["stage"])
        self.assertEqual(config["scene"]["stage_validation"], "built-in empty")
        self.assertEqual(metadata, self.metadata)
        for name, values in self.arrays.items():
            np.testing.assert_array_equal(arrays[name], values)
        self.assertEqual(validation["collision"], "unchecked")

    def test_table_check_and_prepare_preserve_reference_data_and_generation_scene(self):
        before = self.source_hashes()
        completed = self.check("--scene", "table")
        self.assertEqual(completed.returncode, 0, completed.stderr)
        report = json.loads(completed.stdout)
        self.assertEqual(report["scene"]["name"], "table")
        self.assertEqual(report["scene"]["id"], "so101-table/1")
        self.assertEqual(report["scene"]["stage_validation"], "pending simulator inspection")
        self.assertEqual(report["validation"]["collision"], "unchecked")
        self.assertFalse(report["validation"]["dataset_eligible"])
        prepared = self.module.prepare(self.module.parse_args(self.arguments("--scene", "table")))
        config, _, _, metadata, arrays, _ = prepared
        self.assertNotEqual(config["scene"]["id"], metadata["scene"]["id"])
        self.assertEqual(metadata, self.metadata)
        reloaded_metadata, reloaded_arrays = load_trajectory(self.trajectory)
        self.assertEqual(reloaded_metadata, self.metadata)
        for name, values in self.arrays.items():
            np.testing.assert_array_equal(arrays[name], values)
            np.testing.assert_array_equal(reloaded_arrays[name], values)
        self.assertEqual(self.source_hashes(), before)

    def test_custom_catalog_stage_path_resolves_from_catalog_with_external_cwd(self):
        catalog_directory = self.directory / "project" / "configs"
        catalog_directory.mkdir(parents=True)
        stage_directory = catalog_directory.parent / "assets"
        stage_directory.mkdir()
        stage = stage_directory / "room.usda"
        # Offline --check records a file identity; it deliberately cannot claim
        # that USD contents or stage units have been verified.
        stage.write_text("#usda 1.0\n# synthetic offline fixture\n")
        catalog = catalog_directory / "scenes.json"
        catalog.write_text(json.dumps({
            "schema_version": "so101-scenes/1", "scenes": {
                "room": {"id": "custom-room/1", "description": "Local room", "stage": "../assets/room.usda"},
            },
        }))
        completed = self.check("--scenes-config", str(catalog), "--scene", "room")
        self.assertEqual(completed.returncode, 0, completed.stderr)
        scene = json.loads(completed.stdout)["scene"]
        self.assertEqual(scene["stage"], str(stage.resolve()))
        self.assertEqual(scene["catalog_path"], str(catalog.resolve()))
        self.assertEqual(scene["stage_sha256"], hashlib.sha256(stage.read_bytes()).hexdigest())
        self.assertEqual(scene["stage_validation"], "pending simulator inspection")


if __name__ == "__main__":
    unittest.main()
