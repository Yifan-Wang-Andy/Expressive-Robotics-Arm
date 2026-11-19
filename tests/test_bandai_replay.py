"""Bandai CLI and archival integration using temporary synthetic packages."""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET

import test_bandai_trajectory as fixtures

from isaacsim_trajectories.so101_demo import JOINT_NAMES


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/control_so101.py"
np = fixtures.np


@unittest.skipIf(np is None, "Optional NumPy dependency is unavailable")
class BandaiReplayIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("bandai_replay_script", SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def setUp(self):
        self.fixture = fixtures.BandaiTrajectoryTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.root = self.fixture.root
        # Keep the selected replay model separate from the input package, as
        # it is on the workstation. Deleting input provenance must not delete it.
        asset = self.root / "replay_asset"
        asset.mkdir()
        (asset / "mesh.stl").write_text("synthetic mesh existence fixture")
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
        urdf = asset / "model.urdf"
        ET.ElementTree(robot).write(urdf)
        source = urdf.read_bytes()
        (self.fixture.provenance / "model.urdf").write_bytes(source)
        self.fixture.sidecar["urdf_sha256"] = hashlib.sha256(source).hexdigest()
        self.fixture.write()
        config = json.loads((ROOT / "configs/so101-demo.json").read_text())
        config.update(self.fixture.config)
        config["urdf"] = str(urdf)
        self.config = self.root / "replay.json"
        self.config.write_text(json.dumps(config))

    def arguments(self, path=None):
        return ["--trajectory", str(path or self.fixture.path), "--config", str(self.config)]

    def prepare(self, path=None):
        return self.module.prepare(self.module.parse_args(self.arguments(path)))

    def snapshot(self):
        return {p.relative_to(self.root): p.read_bytes()
                for p in self.root.rglob("*") if p.is_file()}

    def test_check_prepares_exact_curve_without_simulator_or_source_changes(self):
        before = self.snapshot()
        completed = subprocess.run(
            [sys.executable, str(SCRIPT), *self.arguments(), "--check"],
            cwd=self.root, capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        report = json.loads(completed.stdout)
        self.assertEqual(report["generator"]["name"], "bandai-v2-cubic-local-clock")
        self.assertEqual(report["fixed_gripper_rad"], .35)
        self.assertEqual(report["source_validation"]["continuous_reference"]["status"], "passed")
        self.assertEqual(report["validation"]["collision"], "unchecked")
        self.assertFalse(report["validation"]["dataset_eligible"])
        self.assertEqual(self.snapshot(), before)
        self.assertNotIn("isaacsim", sys.modules)
        config, _, _, metadata, arrays, _ = self.prepare()
        self.assertEqual(config["fixed_gripper_rad"], .35)
        self.assertEqual(metadata["initial_pose_policy"], "first_sample")
        t = np.minimum(arrays["timestamps_s"], 2/3)
        expected = self.fixture.offsets + (t + .75*t*t)[:, None] * self.fixture.slopes
        np.testing.assert_allclose(arrays["positions_rad"][:, :-1], expected, atol=1e-14)

    def test_archive_preserves_every_source_and_replays_without_package(self):
        _, _, _, metadata, arrays, _ = self.prepare()
        sources = {item["archive_name"]: Path(item["path"]).read_bytes()
                   for item in metadata["source_bandai"]["source_files"]}
        output = self.root / "run"
        output.mkdir()
        provenance = self.module.archive_reference(self.fixture.path, output, metadata, arrays)
        reference = output / "reference"
        self.assertEqual(Path(provenance["directory"]), reference)
        self.assertEqual(set(provenance["files_sha256"]),
                         set(sources) | {"metadata.json", "trajectory.npz"})
        for name, source in sources.items():
            self.assertEqual((reference / name).read_bytes(), source)
            self.assertEqual(provenance["files_sha256"][name], hashlib.sha256(source).hexdigest())
        for name in ("metadata.json", "trajectory.npz"):
            self.assertEqual(provenance["files_sha256"][name],
                             hashlib.sha256((reference / name).read_bytes()).hexdigest())
        before = self.snapshot()
        with self.assertRaises(FileExistsError):
            self.module.archive_reference(self.fixture.path, output, metadata, arrays)
        self.assertEqual(self.snapshot(), before)
        shutil.rmtree(self.root / "motions")
        shutil.rmtree(self.root / "provenance")
        _, _, _, reloaded_metadata, reloaded_arrays, _ = self.prepare(reference)
        self.assertEqual(reloaded_metadata, metadata)
        for name, values in arrays.items():
            np.testing.assert_array_equal(reloaded_arrays[name], values)

    def test_source_mutations_are_rejected_before_archive_directory_creation(self):
        _, _, _, metadata, arrays, _ = self.prepare()
        for index, item in enumerate(metadata["source_bandai"]["source_files"]):
            with self.subTest(source=item["archive_name"]):
                path = Path(item["path"])
                original = path.read_bytes()
                output = self.root / f"rejected-{index}"
                output.mkdir()
                try:
                    path.write_bytes(original + b"\n")
                    with self.assertRaisesRegex(ValueError, "changed after validation"):
                        self.module.archive_reference(self.fixture.path, output, metadata, arrays)
                    self.assertFalse((output / "reference").exists())
                finally:
                    path.write_bytes(original)


if __name__ == "__main__":
    unittest.main()
