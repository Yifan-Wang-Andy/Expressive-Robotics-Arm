"""Offline CSV CLI integration and source archival with synthetic robot assets."""
import copy
import csv
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

from isaacsim_trajectories.so101_demo import JOINT_NAMES
from isaacsim_trajectories.trajectory import load_trajectory, save_trajectory


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/control_so101.py"
READER = ROOT / "scripts/read_trajectory.py"


@unittest.skipIf(np is None, "Optional NumPy dependency is unavailable")
class CsvReplayIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("csv_replay_script", SCRIPT)
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
        # Keep synthetic rejection cases independent of the operating defaults.
        self.config["reference_velocity_limit_rad_s"] = 0.5
        self.config["reference_acceleration_limit_rad_s2"] = 1.0
        self.config_path = self.directory / "config.json"
        self.config_path.write_text(json.dumps(self.config))
        self.input = self.directory / "source.csv"
        # Deliberately put joints in reversed column order. The two clocks and
        # the tool coordinates have values that must not become joint commands.
        self.names = list(reversed(JOINT_NAMES[:-1]))
        self.header = ["source_time_s", "robot_time_s", *self.names,
                       "tool_x_m", "tool_y_m", "tool_z_m"]
        self.rows = [
            [0, 5, .01, .1, .4, -.3, .200, 20, 30, 40],
            [.03, 6, .01, .1, .4, -.3, .202, 21, 31, 41],
            [.06, 7, .01, .1, .4, -.3, .204, 22, 32, 42],
        ]
        self.write()

    def write(self, header=None, rows=None):
        with self.input.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(self.header if header is None else header)
            writer.writerows(self.rows if rows is None else rows)

    def arguments(self, path=None):
        return ["--trajectory", str(path or self.input), "--config", str(self.config_path)]

    def prepare(self, path=None):
        return self.module.prepare(self.module.parse_args(self.arguments(path)))

    def check(self):
        return subprocess.run(
            [sys.executable, str(SCRIPT), *self.arguments(), "--check"],
            cwd=self.directory, text=True, capture_output=True)

    def inspect(self):
        return subprocess.run(
            [sys.executable, str(READER), str(self.input)],
            cwd=self.directory, text=True, capture_output=True)

    def snapshot(self):
        return {str(path.relative_to(self.directory)): path.read_bytes()
                for path in self.directory.rglob("*") if path.is_file()}

    def test_direct_csv_check_preserves_input_and_maps_nonzero_pose_by_name(self):
        before = self.snapshot()
        completed = self.check()
        self.assertEqual(completed.returncode, 0, completed.stderr)
        report = json.loads(completed.stdout)
        self.assertEqual(report["scene"]["name"], "empty")
        self.assertEqual(report["validation"]["collision"], "unchecked")
        self.assertEqual(self.snapshot(), before)
        self.assertNotIn("isaacsim", sys.modules)
        _, _, _, metadata, arrays, _ = self.prepare()
        self.assertEqual(metadata["joint_names"], list(JOINT_NAMES))
        self.assertEqual(metadata["initial_pose_policy"], "first_sample")
        self.assertEqual(metadata["source_csv"]["time_column"], "robot_time_s")
        self.assertEqual(arrays["positions_rad"].shape, (121, 6))
        np.testing.assert_array_equal(arrays["positions_rad"][0], [.2, -.3, .4, .1, .01, 0])
        self.assertAlmostEqual(arrays["timestamps_s"][-1], 2.0)

    def test_raw_inspector_needs_no_model_and_gen3_cannot_be_mapped_to_so101(self):
        # The reader never needs even the synthetic model, so remove it here.
        model_bytes = self.urdf.read_bytes()
        self.urdf.unlink()
        completed = self.inspect()
        self.assertEqual(completed.returncode, 0, completed.stderr)
        report = json.loads(completed.stdout)
        self.assertEqual(report["sample_count"], 3)
        self.assertEqual(report["joint_names"], self.names)
        self.assertEqual(report["units"]["positions"], "radians")
        self.assertIn("source_time_s", completed.stdout)
        self.assertIn("robot_time_s", completed.stdout)
        self.assertIn("unspecified", completed.stdout)
        names = [f"joint_{i}" for i in range(1, 8)]
        self.write(["source_time_s", "robot_time_s", *names],
                   [[0, 0, 0, 0, 3.2, -2.3, 0, 1, 1.57],
                    [.03, .3, 0, 0, 3.2, -2.3, 0, 1, 1.57]])
        completed = self.inspect()
        self.assertEqual(completed.returncode, 0, completed.stderr)
        report = json.loads(completed.stdout)
        self.assertEqual(report["joint_names"], names)
        self.assertEqual(report["sample_count"], 2)
        self.urdf.write_bytes(model_bytes)
        completed = self.check()
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("Gen3", completed.stderr)
        self.assertIn("SO-101", completed.stderr)
        self.assertNotIn("ModuleNotFoundError", completed.stderr)

    def test_source_position_velocity_and_sampled_acceleration_failures_remain_errors(self):
        for value, expected in ((2.1, "positions"), (1.0, "velocity"), (.3, "acceleration")):
            with self.subTest(expected=expected):
                rows = copy.deepcopy(self.rows)
                rows[1][6] = value
                self.write(rows=rows)
                before = self.snapshot()
                completed = self.check()
                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(expected, completed.stderr)
                self.assertNotIn("ModuleNotFoundError", completed.stderr)
                self.assertEqual(self.snapshot(), before)

    def test_archive_preserves_raw_csv_and_normalized_reference_without_overwrite(self):
        _, _, _, metadata, arrays, _ = self.prepare()
        original = self.input.read_bytes()
        output = self.directory / "run"
        output.mkdir()
        provenance = self.module.archive_reference(self.input, output, metadata, arrays)
        reference = output / "reference"
        self.assertEqual((reference / "source.csv").read_bytes(), original)
        self.assertEqual(self.input.read_bytes(), original)
        saved_metadata, saved_arrays = load_trajectory(reference)
        self.assertEqual(saved_metadata, metadata)
        for name, array in arrays.items():
            np.testing.assert_array_equal(saved_arrays[name], array)
        self.assertEqual(Path(provenance["directory"]), reference)
        self.assertEqual(Path(provenance["input_path"]), self.input)
        for name in ("source.csv", "metadata.json", "trajectory.npz"):
            self.assertEqual(provenance["files_sha256"][name],
                             hashlib.sha256((reference / name).read_bytes()).hexdigest())
        before = self.snapshot()
        with self.assertRaises(FileExistsError):
            self.module.archive_reference(self.input, output, metadata, arrays)
        self.assertEqual(self.snapshot(), before)
        # Archived references remain usable through the original directory API.
        _, _, _, replay_metadata, replay_arrays, _ = self.prepare(reference)
        self.assertEqual(replay_metadata, metadata)
        np.testing.assert_array_equal(replay_arrays["positions_rad"], arrays["positions_rad"])

    def test_archive_rejects_csv_changed_after_prepare_before_creating_reference(self):
        _, _, _, metadata, arrays, _ = self.prepare()
        output = self.directory / "run"
        output.mkdir()
        self.input.write_bytes(self.input.read_bytes() + b"\n")
        with self.assertRaisesRegex(ValueError, "(?i)(changed|hash)"):
            self.module.archive_reference(self.input, output, metadata, arrays)
        self.assertFalse((output / "reference").exists())

    def test_directory_provenance_keeps_existing_reference_and_does_not_create_an_archive(self):
        _, _, _, metadata, arrays, _ = self.prepare()
        reference = self.directory / "existing"
        save_trajectory(reference, metadata, arrays)
        output = self.directory / "run"
        output.mkdir()
        before = self.snapshot()
        provenance = self.module.archive_reference(reference, output, metadata, arrays)
        self.assertEqual(Path(provenance["directory"]), reference)
        self.assertEqual(self.snapshot(), before)
        self.assertFalse((output / "reference").exists())
        for name in ("metadata.json", "trajectory.npz"):
            self.assertEqual(provenance["files_sha256"][name],
                             hashlib.sha256((reference / name).read_bytes()).hexdigest())


if __name__ == "__main__":
    unittest.main()
