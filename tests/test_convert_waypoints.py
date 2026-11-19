"""Command-line conversion using temporary synthetic assets, without Isaac Sim."""
import hashlib
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
from isaacsim_trajectories.trajectory import load_trajectory, validate_for_replay


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(np is None, "Optional NumPy dependency is unavailable")
class ConverterTests(unittest.TestCase):
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
        self.input = self.directory / "waypoints.json"
        self.document = {
            "schema_version": "so101-waypoints/1",
            "joint_names": list(reversed(JOINT_NAMES)),
            "waypoints": [
                {"time_s": 0, "positions_rad": [0, 0, 0, 0, 0, 0]},
                {"time_s": 2, "positions_rad": [0, -0.1, 0, 0, 0, 0.2]},
                {"time_s": 4, "positions_rad": [0, 0.1, 0, 0, 0, 0.1]},
            ],
        }
        self.input.write_text(json.dumps(self.document, indent=2) + "\n")
        self.output = self.directory / "converted"

    def convert(self, *extra):
        return subprocess.run(
            [sys.executable, str(ROOT / "scripts/convert_waypoints.py"), str(self.input),
             "--config", str(self.config_path), "--output", str(self.output), *extra],
            cwd=self.directory, text=True, capture_output=True)

    def test_convert_round_trip_replay_screen_and_provenance(self):
        completed = self.convert()
        self.assertEqual(completed.returncode, 0, completed.stderr)
        metadata, arrays = load_trajectory(self.output)
        self.assertEqual(metadata["schema_version"], "so101-trajectory/1")
        self.assertEqual(metadata["generator"], {"name": "waypoint-quintic-rest-to-rest", "version": "1"})
        self.assertIsNone(metadata["seed"])
        self.assertEqual(metadata["joint_names"], self.document["joint_names"])
        self.assertEqual(metadata["resolved_generation_config"]["waypoints"], self.document)
        self.assertEqual(metadata["resolved_generation_config"]["replay_config"], self.config)
        self.assertEqual(metadata["source_waypoints"]["sha256"],
                         hashlib.sha256(self.input.read_bytes()).hexdigest())
        self.assertEqual(metadata["source_waypoints"]["path"], str(self.input))
        self.assertEqual(metadata["validation"]["collision"], "unchecked")
        self.assertEqual(metadata["validation"]["workspace"], "unchecked")
        self.assertFalse(metadata["validation"]["dataset_eligible"])
        self.assertEqual(arrays["positions_rad"].shape, (241, 6))
        np.testing.assert_array_equal(arrays["positions_rad"][[0, 120, 240]],
                                      [point["positions_rad"] for point in self.document["waypoints"]])
        report = validate_for_replay(
            metadata, arrays, load_model(self.urdf), self.config["physics_hz"],
            self.config["control_hz"], self.config["position_margin_rad"],
            self.config["reference_velocity_limit_rad_s"],
            self.config["reference_acceleration_limit_rad_s2"])
        self.assertEqual(metadata["validation"]["replay_screen"], report)

    def test_existing_directory_is_not_overwritten(self):
        completed = self.convert()
        self.assertEqual(completed.returncode, 0, completed.stderr)
        before = {path.name: path.read_bytes() for path in self.output.iterdir()}
        completed = self.convert()
        self.assertNotEqual(completed.returncode, 0)
        self.assertEqual({path.name: path.read_bytes() for path in self.output.iterdir()}, before)

    def test_check_does_not_write_and_urdf_override_is_resolved(self):
        self.config["urdf"] = "does-not-exist.urdf"
        self.config_path.write_text(json.dumps(self.config))
        completed = self.convert("--urdf", str(self.urdf), "--check")
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertFalse(self.output.exists())

    def test_invalid_motion_and_duplicate_json_fields_leave_no_output(self):
        self.document["waypoints"][1]["time_s"] = 0.1
        self.input.write_text(json.dumps(self.document))
        completed = self.convert()
        self.assertNotEqual(completed.returncode, 0)
        self.assertFalse(self.output.exists())
        self.input.write_text('{"schema_version":"so101-waypoints/1","schema_version":"so101-waypoints/1"}')
        completed = self.convert()
        self.assertNotEqual(completed.returncode, 0)
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
