"""Waypoint interpolation checks using synthetic limits, without Isaac Sim."""
import copy
from pathlib import Path
import subprocess
import sys
import unittest

try:
    import numpy as np
except ImportError:
    np = None

from isaacsim_trajectories.so101_demo import JOINT_NAMES
from isaacsim_trajectories.trajectory import validate_for_replay
from isaacsim_trajectories.waypoints import sample_waypoints


class WaypointImportTests(unittest.TestCase):
    def test_import_does_not_load_numpy_or_simulator(self):
        source = Path(__file__).resolve().parents[1] / "src"
        code = (
            "import sys; sys.path.insert(0, sys.argv[1]); "
            "import isaacsim_trajectories.waypoints; "
            "assert 'numpy' not in sys.modules; assert 'isaacsim' not in sys.modules"
        )
        subprocess.run([sys.executable, "-S", "-c", code, str(source)],
                       check=True, capture_output=True)


@unittest.skipIf(np is None, "Optional NumPy dependency is unavailable")
class WaypointTests(unittest.TestCase):
    def setUp(self):
        self.model = {"source_sha256": "a" * 64, "joints": {
            name: {"type": "revolute", "lower": -2.0, "upper": 2.0,
                   "velocity": 1.0, "effort": 10.0}
            for name in reversed(JOINT_NAMES)
        }}
        self.config = {"physics_hz": 120, "control_hz": 60,
                       "reference_velocity_limit_rad_s": 0.5,
                       "reference_acceleration_limit_rad_s2": 1.0,
                       "position_margin_rad": 0.02}
        self.document = {
            "schema_version": "so101-waypoints/1", "joint_names": list(JOINT_NAMES),
            "waypoints": [
                {"time_s": 0, "positions_rad": [0, 0, 0, 0, 0, 0]},
                {"time_s": 2, "positions_rad": [0.2, 0, 0, 0, -0.1, 0]},
                {"time_s": 3, "positions_rad": [0.2, 0, 0, 0, -0.1, 0]},
                {"time_s": 5, "positions_rad": [-0.1, 0, 0, 0, 0.15, 0]},
            ],
        }

    def sample(self, document=None, model=None, config=None, **kwargs):
        return sample_waypoints(self.document if document is None else document,
                                self.model if model is None else model,
                                self.config if config is None else config, **kwargs)

    def test_deterministic_grid_exact_waypoints_holds_and_nonzero_final_pose(self):
        report, arrays = self.sample()
        repeated_report, repeated = self.sample()
        self.assertEqual(report, repeated_report)
        self.assertEqual(report["duration_seconds"], 5)
        self.assertEqual(report["control_steps"], 300)
        self.assertEqual(arrays["positions_rad"].shape, (301, 6))
        np.testing.assert_array_equal(arrays["timestamps_s"], np.arange(301) / 60)
        for name, array in arrays.items():
            self.assertTrue(np.isfinite(array).all())
            np.testing.assert_array_equal(array, repeated[name])
        for waypoint in self.document["waypoints"]:
            np.testing.assert_array_equal(
                arrays["positions_rad"][round(waypoint["time_s"] * 60)], waypoint["positions_rad"])
        np.testing.assert_array_equal(
            arrays["positions_rad"][120:181],
            np.tile(self.document["waypoints"][1]["positions_rad"], (61, 1)))
        np.testing.assert_allclose(arrays["positions_rad"][60], [0.1, 0, 0, 0, -0.05, 0],
                                   rtol=0, atol=1e-15)

    def test_column_permutation_and_asymmetric_limits_follow_joint_names(self):
        _, normal = self.sample()
        reversed_document = copy.deepcopy(self.document)
        reversed_document["joint_names"].reverse()
        for waypoint in reversed_document["waypoints"]:
            waypoint["positions_rad"].reverse()
        _, reversed_arrays = self.sample(document=reversed_document)
        np.testing.assert_array_equal(normal["positions_rad"], reversed_arrays["positions_rad"][:, ::-1])
        self.model["joints"]["shoulder_pan"]["upper"] = 0.21
        with self.assertRaisesRegex(ValueError, "shoulder_pan"):
            self.sample(document=reversed_document)

    def test_continuous_limit_violations_hidden_between_two_saved_samples(self):
        document = copy.deepcopy(self.document)
        document["waypoints"] = [
            {"time_s": 0, "positions_rad": [0, 0, 0, 0, 0, 0]},
            {"time_s": 0.5, "positions_rad": [0.12, 0, 0, 0, 0, 0]},
        ]
        # These two endpoints pass finite-difference screening at 2 Hz.
        # Their intervening quintic peaks are 0.45 rad/s and ~2.77 rad/s^2.
        metadata = {
            "schema_version": "so101-trajectory/1", "trajectory_id": "boundary",
            "data_kind": "reference", "seed": None,
            "generator": {"name": "synthetic-fixture", "version": "1"},
            "resolved_generation_config": {}, "code": {"revision": None, "dirty": None},
            "asset": {"source_sha256": "a" * 64}, "joint_names": list(JOINT_NAMES),
            "joint_types": ["revolute"] * 6, "control_hz": 2,
            "units": {"time": "seconds", "positions": "radians"},
        }
        arrays = {"timestamps_s": np.array([0, 0.5]),
                  "positions_rad": np.array([point["positions_rad"] for point in document["waypoints"]])}
        for velocity, acceleration, error in ((0.3, 100, "velocity"), (1, 1, "acceleration")):
            with self.subTest(error=error):
                validate_for_replay(metadata, arrays, self.model, 4, 2, 0.02, velocity, acceleration)
                config = {**self.config, "physics_hz": 4, "control_hz": 2,
                          "reference_velocity_limit_rad_s": velocity,
                          "reference_acceleration_limit_rad_s2": acceleration}
                with self.assertRaisesRegex(ValueError, error):
                    self.sample(document=document, config=config)

    def test_urdf_velocity_applies_even_with_looser_project_limit(self):
        self.model["joints"]["wrist_roll"]["velocity"] = 0.1
        with self.assertRaisesRegex(ValueError, "wrist_roll"):
            self.sample()

    def test_zero_initial_pose_and_gripper_policy(self):
        for row, column in ((0, 0), (1, 5)):
            document = copy.deepcopy(self.document)
            document["waypoints"][row]["positions_rad"][column] = 0.001
            with self.subTest(row=row, column=column), self.assertRaises(ValueError):
                self.sample(document=document)

    def test_invalid_schema_names_and_row_structure(self):
        documents = [None, [], {}, {**self.document, "schema_version": "future/2"},
                     {**self.document, "extra": 1},
                     {**self.document, "joint_names": list(JOINT_NAMES[:-1])},
                     {**self.document, "joint_names": ["shoulder_pan"] * 6},
                     {**self.document, "joint_names": [None] + list(JOINT_NAMES[1:])},
                     {**self.document, "waypoints": []},
                     {**self.document, "waypoints": self.document["waypoints"][:1]}]
        for mutation in ({"extra": 0}, {"positions_rad": [0] * 5},
                         {"positions_rad": [[0] * 6]}, {"positions_rad": "0,0,0,0,0,0"}):
            document = copy.deepcopy(self.document)
            document["waypoints"][1].update(mutation)
            documents.append(document)
        document = copy.deepcopy(self.document)
        del document["waypoints"][1]["time_s"]
        documents.append(document)
        for document in documents:
            with self.subTest(document=document), self.assertRaises(ValueError):
                sample_waypoints(document, self.model, self.config)

    def test_nonfinite_boolean_and_string_values_rejected(self):
        for invalid in (True, "2", None, float("nan"), float("inf")):
            for field in ("time_s", "positions_rad"):
                document = copy.deepcopy(self.document)
                if field == "time_s":
                    document["waypoints"][1][field] = invalid
                else:
                    document["waypoints"][1][field][0] = invalid
                with self.subTest(invalid=invalid, field=field), self.assertRaises(ValueError):
                    self.sample(document=document)

    def test_time_order_and_grid_rejected_without_retiming(self):
        for times in ((1, 2, 3, 5), (0, 2, 2, 5), (0, 3, 2, 5),
                      (0, 2.001, 3, 5), (0, 1e-12, 3, 5), (0, -1, 3, 5)):
            document = copy.deepcopy(self.document)
            for point, time in zip(document["waypoints"], times):
                point["time_s"] = time
            with self.subTest(times=times), self.assertRaises(ValueError):
                self.sample(document=document)

    def test_float_roundoff_on_grid_is_accepted(self):
        document = copy.deepcopy(self.document)
        document["waypoints"][1]["time_s"] += 1e-12
        _, arrays = self.sample(document=document)
        np.testing.assert_array_equal(arrays["positions_rad"][120], document["waypoints"][1]["positions_rad"])

    def test_invalid_and_missing_settings_and_model_limits(self):
        changes = {"physics_hz": [119, True, 120.0, 0], "control_hz": [0, -1, 61],
                   "reference_velocity_limit_rad_s": [0, -1, float("nan"), True],
                   "reference_acceleration_limit_rad_s2": [0, float("inf")],
                   "position_margin_rad": [-0.1, 3]}
        for key, values in changes.items():
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    self.sample(config={**self.config, key: value})
            config = self.config.copy()
            del config[key]
            with self.subTest(missing=key), self.assertRaises(ValueError):
                self.sample(config=config)
        for key in ("lower", "upper", "velocity", "effort"):
            for value in (None, float("nan"), True):
                model = copy.deepcopy(self.model)
                model["joints"]["elbow_flex"][key] = value
                with self.subTest(limit=key, value=value), self.assertRaises(ValueError):
                    self.sample(model=model)
            model = copy.deepcopy(self.model)
            del model["joints"]["elbow_flex"][key]
            with self.subTest(missing_limit=key), self.assertRaises(ValueError):
                self.sample(model=model)
        model = copy.deepcopy(self.model)
        model["joints"]["elbow_flex"]["type"] = "continuous"
        with self.assertRaises(ValueError):
            self.sample(model=model)

    def test_sample_count_bound_and_stationary_path(self):
        with self.assertRaises(ValueError):
            self.sample(max_samples=300)
        _, arrays = self.sample(max_samples=301)
        self.assertEqual(len(arrays["timestamps_s"]), 301)
        for limit in (0, -1, True, 301.0):
            with self.subTest(limit=limit), self.assertRaises(ValueError):
                self.sample(max_samples=limit)
        document = copy.deepcopy(self.document)
        for point in document["waypoints"]:
            point["positions_rad"] = [0] * 6
        _, arrays = self.sample(document=document)
        np.testing.assert_array_equal(arrays["positions_rad"], np.zeros((301, 6)))


if __name__ == "__main__":
    unittest.main()
