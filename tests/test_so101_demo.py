"""Offline tests; no Isaac Sim installation, GPU, NumPy, or upstream asset needed."""

import copy
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from isaacsim_trajectories.so101_demo import JOINT_NAMES, load_model, sample_motion, validate_motion


class DemoTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "model.urdf"
        (self.path.parent / "mesh.stl").write_text("synthetic mesh existence fixture")
        self.root = ET.Element("robot", name="so101_new_calib")
        base = ET.SubElement(self.root, "link", name="base_link")
        ET.SubElement(self.root, "link", name="gripper_frame_link")
        geometry = ET.SubElement(ET.SubElement(base, "visual"), "geometry")
        ET.SubElement(geometry, "mesh", filename="mesh.stl")
        for name in reversed(JOINT_NAMES):
            joint = ET.SubElement(self.root, "joint", name=name, type="revolute")
            ET.SubElement(joint, "parent", link="base_link")
            ET.SubElement(joint, "child", link="gripper_frame_link")
            ET.SubElement(joint, "limit", lower="-2", upper="2", velocity="1", effort="10")
        self.write_model()
        self.model = load_model(self.path)
        self.config = dict(model=self.model, joint="shoulder_pan", offset_rad=0.2,
                           move_seconds=2.0, physics_hz=120, control_hz=60,
                           velocity_limit=0.5, acceleration_limit=1.0, margin_rad=0.02)

    def write_model(self):
        ET.ElementTree(self.root).write(self.path)

    def test_model_identity_and_order_independence(self):
        self.assertEqual(set(self.model["joints"]), set(JOINT_NAMES))
        self.assertEqual(next(iter(self.model["joints"])), "gripper")
        self.assertEqual(len(self.model["source_sha256"]), 64)
        normal = validate_motion(**self.config)
        self.model["joints"] = dict(reversed(list(self.model["joints"].items())))
        self.assertEqual(normal, validate_motion(**self.config))

    def test_endpoints_return_and_symmetry(self):
        self.assertEqual(sample_motion(0, 0.2, 2), 0)
        self.assertAlmostEqual(sample_motion(2, 0.2, 2), 0.2)
        self.assertEqual(sample_motion(4, 0.2, 2), 0)
        self.assertAlmostEqual(sample_motion(0.7, 0.2, 2), sample_motion(3.3, 0.2, 2))
        self.assertAlmostEqual(sample_motion(1, -0.2, 2), -0.1)

    def test_interior_velocity_boundary_not_just_waypoints(self):
        # At endpoints speed is zero; the interior speed at t=T/2 is 0.1875.
        result = validate_motion(**{**self.config, "velocity_limit": 0.1875})
        self.assertEqual(result["peak_velocity_rad_s"], 0.1875)
        with self.assertRaisesRegex(ValueError, "peak velocity"):
            validate_motion(**{**self.config, "velocity_limit": 0.1874})

    def test_interior_acceleration_and_model_velocity(self):
        with self.assertRaisesRegex(ValueError, "peak acceleration"):
            validate_motion(**{**self.config, "acceleration_limit": 0.28})
        self.model["joints"]["shoulder_pan"]["velocity"] = 0.1
        with self.assertRaisesRegex(ValueError, "peak velocity"):
            validate_motion(**self.config)

    def test_zero_initial_pose_checked_on_stationary_joint(self):
        self.model["joints"]["wrist_roll"]["lower"] = 0.1
        with self.assertRaisesRegex(ValueError, "zero initial"):
            validate_motion(**self.config)

    def test_invalid_settings(self):
        cases = {"joint": ["gripper", "missing"], "offset_rad": [math.nan, math.inf, 2.0],
                 "move_seconds": [0, -1, 0.123, math.inf], "physics_hz": [119, 120.0, True, 0],
                 "control_hz": [0, -1, 61], "velocity_limit": [0, -1, math.nan],
                 "acceleration_limit": [0, math.inf], "margin_rad": [-0.1, 3, math.nan]}
        for name, values in cases.items():
            for value in values:
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    validate_motion(**{**self.config, name: value})

    def test_sample_rejects_out_of_interval_and_nonfinite(self):
        for t in (-0.001, 4.001, math.nan, math.inf):
            with self.subTest(t=t), self.assertRaises(ValueError):
                sample_motion(t, 0.2, 2)

    def test_missing_mesh_and_limits(self):
        (self.path.parent / "mesh.stl").unlink()
        with self.assertRaisesRegex(ValueError, "Mesh file"):
            load_model(self.path)
        (self.path.parent / "mesh.stl").touch()
        self.root.find("joint/limit").attrib.pop("velocity")
        self.write_model()
        with self.assertRaisesRegex(ValueError, "velocity"):
            load_model(self.path)

    def test_unsupported_model_variants(self):
        original = copy.deepcopy(self.root)
        for kind in ("continuous", "prismatic", "floating", "unknown"):
            with self.subTest(kind=kind):
                self.root = copy.deepcopy(original)
                self.root.find("joint").set("type", kind)
                self.write_model()
                with self.assertRaisesRegex(ValueError, "unsupported type"):
                    load_model(self.path)
        self.root = original
        ET.SubElement(self.root.find("joint"), "mimic", joint="shoulder_pan")
        self.write_model()
        with self.assertRaisesRegex(ValueError, "Mimic"):
            load_model(self.path)

    def test_duplicate_and_nonfinite_model_limits(self):
        self.root.append(copy.deepcopy(self.root.find("joint")))
        self.write_model()
        with self.assertRaisesRegex(ValueError, "unique"):
            load_model(self.path)
        self.root.remove(self.root.findall("joint")[-1])
        self.root.find("joint/limit").set("upper", "nan")
        self.write_model()
        with self.assertRaisesRegex(ValueError, "finite"):
            load_model(self.path)


if __name__ == "__main__":
    unittest.main()
