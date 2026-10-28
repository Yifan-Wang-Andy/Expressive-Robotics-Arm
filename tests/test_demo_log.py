"""Log tests run offline; skipped when the optional NumPy extra is unavailable."""

import copy
import json
from pathlib import Path
import tempfile
import unittest

try:
    import numpy as np
except ImportError:
    np = None

from isaacsim_trajectories.demo_log import load_log, validate_log
from isaacsim_trajectories.so101_demo import JOINT_NAMES


@unittest.skipIf(np is None, "Optional NumPy dependency is unavailable")
class LogTests(unittest.TestCase):
    def setUp(self):
        self.metadata = {"schema_version": "so101-control-log/1", "status": "completed",
                         "joint_names": list(reversed(JOINT_NAMES)), "samples": 3,
                         "max_tracking_error_rad": 0.0}
        self.arrays = {"timestamps_s": np.array([0.0, 0.1, 0.2]),
                       "commanded_positions_rad": np.arange(18, dtype=float).reshape(3, 6),
                       "measured_positions_rad": np.arange(18, dtype=float).reshape(3, 6),
                       "measured_velocities_rad_s": np.zeros((3, 6))}

    def test_round_trip_preserves_joint_order_and_values(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            (directory / "metadata.json").write_text(json.dumps(self.metadata))
            np.savez_compressed(directory / "motion.npz", **self.arrays)
            metadata, arrays = load_log(directory)
        self.assertEqual(metadata, self.metadata)
        for name in self.arrays:
            np.testing.assert_array_equal(arrays[name], self.arrays[name])

    def test_nonmonotonic_or_nonfinite_timestamps(self):
        for times in ([0, 0, 1], [0, 2, 1], [0, np.nan, 1], [0, 1, np.inf]):
            with self.subTest(times=times), self.assertRaisesRegex(ValueError, "timestamps"):
                validate_log(self.metadata, {**self.arrays, "timestamps_s": np.array(times)})
        # Comparing adjacent values also catches unsigned underflow, unlike diff.
        with self.assertRaisesRegex(ValueError, "timestamps"):
            validate_log(self.metadata, {**self.arrays, "timestamps_s": np.array([2, 1, 3], dtype=np.uint64)})

    def test_joint_names_dimensions_and_sample_count(self):
        duplicate = list(JOINT_NAMES)
        duplicate[-1] = duplicate[0]
        for names in (duplicate, list(JOINT_NAMES[:-1]), [None] * 6):
            with self.subTest(names=names), self.assertRaises(ValueError):
                validate_log({**self.metadata, "joint_names": names}, self.arrays)
        with self.assertRaisesRegex(ValueError, "shape"):
            validate_log(self.metadata, {**self.arrays, "measured_positions_rad": np.zeros((3, 5))})
        with self.assertRaisesRegex(ValueError, "sample count"):
            validate_log({**self.metadata, "samples": 2}, self.arrays)

    def test_failed_raw_nonfinite_measurements_are_preserved(self):
        self.arrays["measured_positions_rad"][1, 2] = np.nan
        self.arrays["measured_velocities_rad_s"][2, 3] = np.inf
        with self.assertRaisesRegex(ValueError, "finite measurements"):
            validate_log(self.metadata, self.arrays)
        metadata = {**self.metadata, "status": "failed", "max_tracking_error_rad": None}
        validate_log(metadata, self.arrays)
        self.assertTrue(np.isnan(self.arrays["measured_positions_rad"][1, 2]))
        with self.assertRaisesRegex(ValueError, "null maximum"):
            validate_log({**metadata, "max_tracking_error_rad": 0.0}, self.arrays)
        self.arrays["commanded_positions_rad"][0, 0] = np.nan
        with self.assertRaisesRegex(ValueError, "Commanded"):
            validate_log(metadata, self.arrays)

    def test_schema_array_keys_and_unsafe_dtypes(self):
        with self.assertRaisesRegex(ValueError, "schema"):
            validate_log({**self.metadata, "schema_version": "future/2"}, self.arrays)
        with self.assertRaises(ValueError):
            validate_log(self.metadata, {**self.arrays, "extra": np.zeros(1)})
        arrays = copy.copy(self.arrays)
        arrays["measured_positions_rad"] = arrays["measured_positions_rad"].astype(object)
        with self.assertRaisesRegex(ValueError, "real numeric"):
            validate_log(self.metadata, arrays)
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            (directory / "metadata.json").write_text(json.dumps(self.metadata))
            np.savez_compressed(directory / "motion.npz", **arrays)
            with self.assertRaises(ValueError):
                load_log(directory)


if __name__ == "__main__":
    unittest.main()
