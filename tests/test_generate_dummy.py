"""Test the offline dummy method independently of a simulator or downloaded asset."""
import importlib.util
from pathlib import Path
import unittest

try:
    import numpy as np
except ImportError:
    np = None

from isaacsim_trajectories.so101_demo import JOINT_NAMES


@unittest.skipIf(np is None, "Optional NumPy dependency is unavailable")
class DummyGeneratorTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / "scripts/generate_dummy_trajectory.py"
        spec = importlib.util.spec_from_file_location("dummy_generator_script", path)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.model = {"joints": {name: {"type": "revolute", "lower": -2.0, "upper": 2.0,
                                       "velocity": 1.0, "effort": 10.0} for name in reversed(JOINT_NAMES)}}
        self.config = {"joint": "wrist_roll", "offset_rad": -0.3, "move_seconds": 5.0,
                       "physics_hz": 120, "control_hz": 60, "reference_velocity_limit_rad_s": 0.5,
                       "reference_acceleration_limit_rad_s2": 1.0, "position_margin_rad": 0.02}

    def test_deterministic_named_joint_and_inclusive_endpoints(self):
        validation, arrays = self.module.generate_dummy(self.config, self.model)
        _, repeated = self.module.generate_dummy(self.config, self.model)
        self.assertEqual(arrays["positions_rad"].shape, (601, 6))
        np.testing.assert_array_equal(arrays["positions_rad"], repeated["positions_rad"])
        np.testing.assert_array_equal(arrays["timestamps_s"], repeated["timestamps_s"])
        index = JOINT_NAMES.index("wrist_roll")
        self.assertEqual(arrays["positions_rad"][300, index], -0.3)
        np.testing.assert_array_equal(arrays["positions_rad"][[0, -1]], np.zeros((2, 6)))
        np.testing.assert_array_equal(np.delete(arrays["positions_rad"], index, axis=1), np.zeros((601, 5)))
        self.assertEqual(arrays["timestamps_s"][-1], validation["duration_seconds"])

    def test_invalid_motion_rejected_before_sampling(self):
        with self.assertRaisesRegex(ValueError, "velocity"):
            self.module.generate_dummy({**self.config, "move_seconds": 0.1}, self.model)
        with self.assertRaisesRegex(ValueError, "gripper"):
            self.module.generate_dummy({**self.config, "joint": "gripper"}, self.model)


if __name__ == "__main__":
    unittest.main()
