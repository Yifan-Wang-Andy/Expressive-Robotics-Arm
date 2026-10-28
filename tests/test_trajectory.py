"""Trajectory serialization and replay screening, independent of Isaac Sim."""
import copy
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

from isaacsim_trajectories.so101_demo import JOINT_NAMES
from isaacsim_trajectories.trajectory import (
    load_trajectory, save_trajectory, validate_for_replay, validate_trajectory,
)


class OfflineImportTests(unittest.TestCase):
    def test_import_without_site_packages_or_simulator(self):
        source = Path(__file__).resolve().parents[1] / "src"
        code = (
            "import sys; sys.path.insert(0, sys.argv[1]); "
            "import isaacsim_trajectories.trajectory; "
            "assert 'numpy' not in sys.modules; assert 'isaacsim' not in sys.modules"
        )
        subprocess.run([sys.executable, "-S", "-c", code, str(source)], check=True, capture_output=True)


@unittest.skipIf(np is None, "Optional NumPy dependency is unavailable")
class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.metadata = {
            "schema_version": "so101-trajectory/1", "trajectory_id": "synthetic",
            "data_kind": "reference", "seed": None,
            "generator": {"name": "synthetic-fixture", "version": "1"},
            "resolved_generation_config": {}, "code": {"revision": None, "dirty": None},
            "asset": {"source_sha256": "a" * 64}, "joint_names": list(JOINT_NAMES),
            "joint_types": ["revolute"] * 6, "control_hz": 10,
            "units": {"time": "seconds", "positions": "radians"},
        }
        self.arrays = {"timestamps_s": np.arange(5) / 10, "positions_rad": np.zeros((5, 6))}
        self.arrays["positions_rad"][:, 0] = [0, .01, .02, .01, 0]
        self.model = {"source_sha256": "a" * 64, "joints": {
            name: {"type": "revolute", "lower": -1.0, "upper": 1.0, "velocity": 2.0, "effort": 1.0}
            for name in JOINT_NAMES
        }}

    def replay(self, metadata=None, arrays=None, **kwargs):
        options = dict(physics_hz=20, control_hz=10, position_margin_rad=.02,
                       velocity_limit_rad_s=.5, acceleration_limit_rad_s2=3.0)
        options.update(kwargs)
        return validate_for_replay(metadata or self.metadata, arrays or self.arrays, self.model, **options)

    def test_round_trip_preserves_data_and_rejects_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "trajectory"
            save_trajectory(directory, self.metadata, self.arrays)
            metadata, arrays = load_trajectory(directory)
            self.assertEqual(metadata, self.metadata)
            for name in self.arrays:
                np.testing.assert_array_equal(arrays[name], self.arrays[name])
            with self.assertRaises(FileExistsError):
                save_trajectory(directory, self.metadata, self.arrays)
            self.assertEqual(json.loads((directory / "metadata.json").read_text()), self.metadata)

    def test_invalid_data_is_rejected_before_creating_directory(self):
        self.arrays["positions_rad"][1, 1] = np.nan
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "invalid"
            with self.assertRaises(ValueError):
                save_trajectory(directory, self.metadata, self.arrays)
            self.assertFalse(directory.exists())

    def test_joint_order_is_explicit_and_generator_is_not_special_cased(self):
        metadata = copy.deepcopy(self.metadata)
        metadata["generator"] = {"name": "an-unrelated-generator", "version": "2"}
        metadata["joint_names"] = list(reversed(JOINT_NAMES))
        arrays = {**self.arrays, "positions_rad": self.arrays["positions_rad"][:, ::-1]}
        normal = self.replay()
        permuted = self.replay(metadata, arrays)
        self.assertEqual(normal, permuted)
        # A tight shoulder bound must follow its name into the last column.
        self.model["joints"]["shoulder_pan"]["upper"] = .03
        with self.assertRaisesRegex(ValueError, "positions"):
            self.replay(metadata, arrays)

    def test_timestamp_grid_finiteness_and_dimensions(self):
        invalid = ([0, .1, .1, .3, .4], [0, .2, .1, .3, .4], [.1, .2, .3, .4, .5],
                   [0, .1, .201, .3, .4], [0, .1, np.nan, .3, .4],
                   np.array([0, 2, 1, 3, 4], dtype=np.uint64))
        for times in invalid:
            with self.subTest(times=times), self.assertRaises(ValueError):
                validate_trajectory(self.metadata, {**self.arrays, "timestamps_s": np.asarray(times)})
        for positions in (np.zeros((5, 5)), np.zeros((6, 6)), np.full((5, 6), np.inf),
                          np.zeros((5, 6), dtype=object), np.zeros((5, 6), dtype=bool),
                          np.zeros((5, 6), dtype=complex)):
            with self.subTest(shape=positions.shape), self.assertRaises(ValueError):
                validate_trajectory(self.metadata, {**self.arrays, "positions_rad": positions})
        for times in (np.zeros((5, 1)), np.zeros(0), np.zeros(1)):
            with self.subTest(shape=times.shape), self.assertRaises(ValueError):
                validate_trajectory(self.metadata, {**self.arrays, "timestamps_s": times})

    def test_metadata_requires_identity_units_and_schema(self):
        changes = ({"schema_version": "future/2"}, {"data_kind": "measured"},
                   {"joint_names": ["shoulder_pan"] * 6}, {"joint_types": ["continuous"] * 6},
                   {"units": {"time": "seconds", "positions": "degrees"}}, {"control_hz": True},
                   {"asset": {"source_sha256": "unknown"}}, {"code": {}}, {"generator": {}},
                   {"resolved_generation_config": {"bad": np.nan}}, {"seed": True})
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_trajectory({**self.metadata, **change}, self.arrays)

    def test_asset_schedule_and_initial_gripper_policy(self):
        with self.assertRaisesRegex(ValueError, "asset hash"):
            self.replay({**self.metadata, "asset": {"source_sha256": "b" * 64}})
        for options in ({"physics_hz": 21}, {"control_hz": 5}, {"physics_hz": 0},
                        {"velocity_limit_rad_s": np.nan}, {"acceleration_limit_rad_s2": -1},
                        {"position_margin_rad": -1}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.replay(**options)
        positions = self.arrays["positions_rad"].copy()
        positions[0, 1] = .01
        with self.assertRaisesRegex(ValueError, "initial pose"):
            self.replay(arrays={**self.arrays, "positions_rad": positions})
        positions[0, 1] = 0
        positions[2, -1] = .01
        with self.assertRaisesRegex(ValueError, "gripper"):
            self.replay(arrays={**self.arrays, "positions_rad": positions})

    def test_interior_spike_exceeds_derivative_limits_despite_valid_endpoints(self):
        positions = self.arrays["positions_rad"].copy()
        positions[:, 0] = [0, 0, .06, 0, 0]
        with self.assertRaisesRegex(ValueError, "velocity"):
            self.replay(arrays={**self.arrays, "positions_rad": positions})
        positions[:, 0] = [0, 0, .02, 0, 0]  # speed .2 < .5; acceleration 4 > 3.
        with self.assertRaisesRegex(ValueError, "acceleration"):
            self.replay(arrays={**self.arrays, "positions_rad": positions})
        self.model["joints"]["shoulder_pan"]["velocity"] = .05
        with self.assertRaisesRegex(ValueError, "velocity"):
            self.replay()

    def test_unsafe_archive_and_unexpected_arrays_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            (directory / "metadata.json").write_text(json.dumps(self.metadata))
            np.savez_compressed(directory / "trajectory.npz", **self.arrays, extra=np.zeros(1))
            with self.assertRaisesRegex(ValueError, "arrays"):
                load_trajectory(directory)
            np.savez_compressed(directory / "trajectory.npz", timestamps_s=self.arrays["timestamps_s"],
                                positions_rad=self.arrays["positions_rad"].astype(object))
            with self.assertRaises(ValueError):
                load_trajectory(directory)

    def test_boundary_acceleration_and_untrusted_validation_claims(self):
        arrays = {"timestamps_s": np.array([0, .1]), "positions_rad": np.zeros((2, 6))}
        arrays["positions_rad"][1, 0] = .001
        metadata = {**self.metadata, "validation": {"collision": "passed", "continuous_reference": "passed"}}
        report = self.replay(metadata, arrays)
        self.assertAlmostEqual(report["peak_acceleration_rad_s2"], .1)
        self.assertEqual(report["collision"], "unchecked")
        self.assertIn("unchecked", report["continuous_reference"])
        with self.assertRaisesRegex(ValueError, "acceleration"):
            self.replay(metadata, arrays, acceleration_limit_rad_s2=.05)
        # Constant interior velocity still requires accelerating from/to rest.
        self.arrays["positions_rad"][:, 0] = np.arange(5) * .01
        with self.assertRaisesRegex(ValueError, "acceleration"):
            self.replay(acceleration_limit_rad_s2=.5)

    def test_all_model_limits_are_required(self):
        for key in ("lower", "upper", "velocity", "effort"):
            for invalid in (None, np.nan, np.inf):
                model = copy.deepcopy(self.model)
                model["joints"]["wrist_roll"][key] = invalid
                with self.subTest(key=key, value=invalid), self.assertRaises(ValueError):
                    validate_for_replay(self.metadata, self.arrays, model, 20, 10, .02, .5, 3.0)


if __name__ == "__main__":
    unittest.main()
