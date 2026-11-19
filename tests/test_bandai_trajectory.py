"""Synthetic Bandai package fixtures; independent of datasets and Isaac Sim."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

try:
    import numpy as np
except ImportError:
    np = None

from isaacsim_trajectories.bandai_trajectory import bandai_to_reference
from isaacsim_trajectories.so101_demo import JOINT_NAMES
from isaacsim_trajectories.trajectory import load_trajectory, save_trajectory, validate_for_replay


class BandaiOfflineImportTests(unittest.TestCase):
    def test_import_needs_no_numpy_or_simulator(self):
        source = Path(__file__).resolve().parents[1] / "src"
        code = ("import sys; sys.path.insert(0, sys.argv[1]); "
                "import isaacsim_trajectories.bandai_trajectory; "
                "assert 'numpy' not in sys.modules; assert 'isaacsim' not in sys.modules")
        subprocess.run([sys.executable, "-S", "-c", code, str(source)],
                       check=True, capture_output=True)


@unittest.skipIf(np is None, "Optional NumPy dependency is unavailable")
class BandaiTrajectoryTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.path = self.root / "motions/dataset_1/synthetic/so101.npz"
        self.path.parent.mkdir(parents=True)
        self.provenance = self.root / "provenance/robot_models/so101"
        self.provenance.mkdir(parents=True)
        self.urdf = b"synthetic model identity, not a simulator fixture"
        (self.provenance / "model.urdf").write_bytes(self.urdf)
        digest = hashlib.sha256(self.urdf).hexdigest()
        self.names = list(JOINT_NAMES[:-1])
        self.sidecar = {
            "format_version": 1, "robot": "so101", "joint_names": self.names,
            "joint_units": "radians", "tool_units": "meters", "time_units": "seconds",
            "root_frame": "base_link", "tool_frame": "gripper_frame_link",
            "urdf_sha256": digest, "exact_trajectory": "so101.npz",
            "method": "synthetic", "metrics": {"upstream_claim_only": True},
        }
        self.declaration = {
            "robot": "so101", "joint_names": self.names, "joint_units": "radians",
            "root_frame": "base_link", "tool_frame": "gripper_frame_link",
            "fixed_gripper_rad": .35,
        }
        self.model = {
            "name": "so101_new_calib", "source_sha256": digest,
            "joints": {name: {"type": "revolute", "lower": -2., "upper": 2.,
                              "velocity": 1., "effort": 10.} for name in JOINT_NAMES},
        }
        self.config = {
            "urdf": "synthetic.urdf", "physics_hz": 20, "control_hz": 10,
            "reference_velocity_limit_rad_s": .5, "reference_acceleration_limit_rad_s2": 1.,
            "position_margin_rad": .02, "gripper_policy": "hold zero radians",
        }
        self.offsets = np.array([.1, -.2, .3, .1, .01])
        self.slopes = np.array([.01, -.02, .03, -.01, .005])
        x = np.array([0., .4, 1.])
        c = np.zeros((4, 2, 5))
        c[2] = self.slopes
        c[3] = self.offsets + x[:-1, None] * self.slopes
        grid = np.array([0., .25, 1.])
        self.curve = {"path_c": c, "path_x": x, "grid": grid,
                      "squared_speed": 1 + 3*grid, "clock": 2/3*(np.sqrt(1+3*grid)-1),
                      "joint_names": np.array(self.names)}
        self.write()

    def write(self):
        np.savez_compressed(self.path, **self.curve)
        self.path.with_suffix(".json").write_text(json.dumps(self.sidecar))
        (self.provenance / "waypoint_model.json").write_text(json.dumps(self.declaration))

    def convert(self, **kwargs):
        return bandai_to_reference(self.path, self.model, self.config, **kwargs)

    def one_piece(self, coefficients):
        """Set one cubic in source time, with a unit-speed local clock."""
        self.curve = {
            "path_c": np.zeros((4, 1, 5)), "path_x": np.array([0., 1.]),
            "grid": np.array([0., 1.]), "squared_speed": np.ones(2),
            "clock": np.array([0., 1.]), "joint_names": np.array(self.names),
        }
        self.curve["path_c"][:, 0, 0] = coefficients
        self.write()

    def test_exact_local_clock_sampling_gripper_provenance_and_roundtrip(self):
        before = {p: p.read_bytes() for p in self.root.rglob("*") if p.is_file()}
        metadata, arrays = self.convert()
        times = arrays["timestamps_s"]
        t = np.minimum(times, 2/3)
        expected = self.offsets + (t + .75*t*t)[:, None]*self.slopes
        np.testing.assert_allclose(arrays["positions_rad"][:, :-1], expected, atol=1e-14)
        np.testing.assert_array_equal(arrays["positions_rad"][:, -1], .35)
        self.assertEqual(metadata["fixed_gripper_rad"], .35)
        self.assertEqual(metadata["resolved_generation_config"]["replay_config"]["gripper_policy"],
                         "hold fixed radians")
        self.assertEqual(metadata["initial_pose_policy"], "first_sample")
        self.assertAlmostEqual(times[-1], .7)
        self.assertAlmostEqual(metadata["timing"]["end_padding_s"], 1/30)
        bounds = metadata["validation"]["continuous_reference"]["joints"]
        for j, name in enumerate(self.names):
            self.assertAlmostEqual(bounds[name]["peak_velocity_rad_s"], abs(self.slopes[j])*2)
            self.assertAlmostEqual(bounds[name]["peak_acceleration_rad_s2"], abs(self.slopes[j])*1.5)
        self.assertFalse(metadata["validation"]["dataset_eligible"])
        self.assertEqual(len(metadata["source_bandai"]["source_files"]), 4)
        for item in metadata["source_bandai"]["source_files"]:
            self.assertEqual(item["sha256"], hashlib.sha256(Path(item["path"]).read_bytes()).hexdigest())
        self.assertEqual(before, {p: p.read_bytes() for p in before})
        output = self.root / "reference"
        save_trajectory(output, metadata, arrays)
        loaded_metadata, loaded_arrays = load_trajectory(output)
        self.assertEqual(loaded_metadata, metadata)
        for name, array in arrays.items():
            np.testing.assert_array_equal(loaded_arrays[name], array)
        validate_for_replay(metadata, arrays, self.model, 20, 10, .02, .5, 1.)

    def test_joint_order_is_mapped_by_name(self):
        self.curve["joint_names"] = np.array(list(reversed(self.names)))
        self.curve["path_c"] = self.curve["path_c"][:, :, ::-1]
        self.sidecar["joint_names"] = list(reversed(self.names))
        self.declaration["joint_names"] = list(reversed(self.names))
        self.write()
        metadata, arrays = self.convert()
        self.assertEqual(metadata["joint_names"], list(JOINT_NAMES))
        np.testing.assert_allclose(arrays["positions_rad"][0, :-1], self.offsets)

    def test_position_peak_between_control_samples_is_rejected(self):
        # The two stored samples would both be zero; the interior reaches 2.
        self.one_piece([0., -8., 8., 0.])
        self.config.update(control_hz=1, physics_hz=2, reference_velocity_limit_rad_s=100.,
                           reference_acceleration_limit_rad_s2=100.)
        self.model["joints"]["shoulder_pan"]["velocity"] = 100.
        with self.assertRaisesRegex(ValueError, "continuous positions.*shoulder_pan"):
            self.convert()

    def test_velocity_peak_between_control_samples_is_rejected(self):
        # q=.4*(3s^2-2s^3): sampled endpoint speed .4, interior peak .6.
        self.one_piece([-.8, 1.2, 0., 0.])
        self.config.update(control_hz=1, physics_hz=2, reference_acceleration_limit_rad_s2=10.)
        with self.assertRaisesRegex(ValueError, "continuous velocity.*shoulder_pan"):
            self.convert()

    def test_acceleration_peak_in_local_clock_is_rejected(self):
        self.config["reference_acceleration_limit_rad_s2"] = .04
        # Largest q'=.03 times source acceleration 1.5 = .045.
        with self.assertRaisesRegex(ValueError, "continuous acceleration.*elbow_flex"):
            self.convert()

    def test_interior_acceleration_extremum_with_decelerating_clock(self):
        self.one_piece([.01, 0., 0., 0.])
        self.curve["squared_speed"] = np.array([4., 1.])
        self.curve["clock"] = np.array([0., 2/3])
        self.write()
        metadata, _ = self.convert()
        peak = metadata["validation"]["continuous_reference"]["joints"]["shoulder_pan"]
        # q=.01*s^3 and u=4-3s give acceleration=.24*s-.225*s^2.
        self.assertAlmostEqual(peak["peak_acceleration_rad_s2"], .064)
        self.config["reference_acceleration_limit_rad_s2"] = .06
        with self.assertRaisesRegex(ValueError, "continuous acceleration.*shoulder_pan"):
            self.convert()

    def test_clock_integrity_and_coverage_are_required(self):
        original = copy.deepcopy(self.curve)
        for key, values in (("clock", [0., .2, .9]), ("squared_speed", [1., 0., 4.]),
                            ("grid", [0., .25, 1.1]), ("clock", [0., 0., 1.])):
            with self.subTest(key=key, values=values):
                self.curve = copy.deepcopy(original)
                self.curve[key] = np.array(values)
                self.write()
                with self.assertRaisesRegex(ValueError, "clock|speed|coverage"):
                    self.convert()

    def test_path_must_be_continuous_with_continuous_first_derivative(self):
        for coefficient in (3, 2):
            with self.subTest(coefficient=coefficient):
                original = self.curve["path_c"].copy()
                self.curve["path_c"][coefficient, 1, 0] += .01
                self.write()
                with self.assertRaisesRegex(ValueError, "continuous"):
                    self.convert()
                self.curve["path_c"] = original

    def test_source_identity_units_and_robot_are_not_inferred_from_filename(self):
        for key, value in (("urdf_sha256", "0"*64), ("robot", "gen3"),
                           ("joint_units", "degrees"), ("time_units", "milliseconds"),
                           ("root_frame", "world"), ("format_version", True)):
            with self.subTest(key=key):
                previous = self.sidecar[key]
                self.sidecar[key] = value
                self.write()
                with self.assertRaises(ValueError):
                    self.convert()
                self.sidecar[key] = previous
        self.write()
        (self.provenance / "model.urdf").write_bytes(b"changed model")
        with self.assertRaisesRegex(ValueError, "URDF hash"):
            self.convert()

    def test_malformed_arrays_and_unsupported_joints_are_rejected(self):
        original = copy.deepcopy(self.curve)
        cases = [("path_c", np.zeros((3, 2, 5))), ("path_x", np.array([0., .4, .4])),
                 ("clock", np.array([0., np.nan, 1.])),
                 ("joint_names", np.array(["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"])),
                 ("path_c", np.array([object()], dtype=object))]
        for key, value in cases:
            with self.subTest(key=key):
                self.curve = copy.deepcopy(original)
                self.curve[key] = value
                self.write()
                with self.assertRaises(ValueError):
                    self.convert()

    def test_missing_provenance_and_oversized_resampling_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "max_samples"):
            self.convert(max_samples=3)
        for invalid in (True, 1, 1.5):
            with self.assertRaisesRegex(ValueError, "max_samples"):
                self.convert(max_samples=invalid)
        (self.provenance / "waypoint_model.json").unlink()
        with self.assertRaises(FileNotFoundError):
            self.convert()

    def test_invalid_gripper_and_duplicate_json_keys_are_rejected(self):
        for value in (float("nan"), True, 3.):
            self.declaration["fixed_gripper_rad"] = value
            self.write()
            with self.assertRaises(ValueError):
                self.convert()
        self.declaration["fixed_gripper_rad"] = .35
        self.write()
        self.path.with_suffix(".json").write_text('{"robot":"so101","robot":"gen3"}')
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.convert()


if __name__ == "__main__":
    unittest.main()
