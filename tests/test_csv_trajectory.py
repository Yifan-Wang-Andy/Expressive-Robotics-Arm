"""CSV parsing and reference conversion with synthetic data; no simulator/assets."""
import copy
import csv
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

try:
    import numpy as np
except ImportError:
    np = None

from isaacsim_trajectories.csv_trajectory import (
    csv_to_reference, read_csv_trajectory, validate_csv_source,
)
from isaacsim_trajectories.so101_demo import JOINT_NAMES
from isaacsim_trajectories.trajectory import (
    load_trajectory, save_trajectory, validate_for_replay,
)


class CsvOfflineImportTests(unittest.TestCase):
    def test_import_does_not_require_simulator_or_numpy(self):
        source = Path(__file__).resolve().parents[1] / "src"
        code = (
            "import sys; sys.path.insert(0, sys.argv[1]); "
            "import isaacsim_trajectories.csv_trajectory; "
            "assert 'numpy' not in sys.modules; assert 'isaacsim' not in sys.modules"
        )
        subprocess.run([sys.executable, "-S", "-c", code, str(source)],
                       check=True, capture_output=True)


@unittest.skipIf(np is None, "Optional NumPy dependency is unavailable")
class CsvTrajectoryTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.path = self.directory / "synthetic.csv"
        self.header = ["source_time_s", "robot_time_s", *JOINT_NAMES[:-1],
                       "tool_x_m", "tool_y_m", "tool_z_m"]
        # The two clocks intentionally differ, and the source begins away from
        # zero pose. Tool coordinates are deliberately outside joint limits.
        self.rows = [
            [0.00, 4.00, .200, -.300, .400, .100, .010, -100, 50, 4],
            [0.05, 4.10, .201, -.298, .398, .101, .010, -101, 51, 5],
            [0.10, 4.25, .204, -.297, .397, .100, .010, -102, 52, 6],
        ]
        self.model = {
            "name": "so101_new_calib", "source_sha256": "a" * 64,
            "links": ["base_link", "gripper_frame_link"], "mesh_files": ["mesh.stl"],
            "joints": {
                name: {"type": "revolute", "lower": -2.0, "upper": 2.0,
                       "velocity": 1.0, "effort": 10.0}
                for name in JOINT_NAMES
            },
        }
        self.config = {
            "schema_version": "so101-replay-config/1", "urdf": "synthetic.urdf",
            "source_repository": "synthetic fixture", "source_license": "synthetic fixture",
            "physics_hz": 20, "control_hz": 10,
            "reference_velocity_limit_rad_s": .5,
            "reference_acceleration_limit_rad_s2": 1.0,
            "position_margin_rad": .02,
            "gripper_policy": "hold zero radians", "settings_provenance": "synthetic test limits",
        }
        self.write()

    def write(self, header=None, rows=None):
        with self.path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(self.header if header is None else header)
            writer.writerows(self.rows if rows is None else rows)

    def read(self):
        return read_csv_trajectory(self.path)

    def reference(self):
        metadata, arrays = self.read()
        return csv_to_reference(metadata, arrays, self.model, self.config,
                                code={"revision": "synthetic", "dirty": False})

    def screen(self, metadata, arrays):
        return validate_for_replay(
            metadata, arrays, self.model,
            self.config["physics_hz"], self.config["control_hz"],
            self.config["position_margin_rad"],
            self.config["reference_velocity_limit_rad_s"],
            self.config["reference_acceleration_limit_rad_s2"],
        )

    def test_raw_reader_preserves_radians_both_clocks_and_separate_tool_coordinates(self):
        before = self.path.read_bytes()
        metadata, arrays = self.read()
        self.assertEqual(metadata["layout"], "so101")
        self.assertEqual(metadata["joint_names"], list(JOINT_NAMES[:-1]))
        self.assertEqual(metadata["sample_count"], 3)
        self.assertEqual(metadata["source"]["path"], str(self.path.resolve()))
        self.assertEqual(metadata["source"]["sha256"], hashlib.sha256(before).hexdigest())
        self.assertEqual(metadata["units"],
                         {"time": "seconds", "positions": "radians", "tool_positions": "meters"})
        np.testing.assert_array_equal(arrays["source_time_s"], [0, .05, .1])
        np.testing.assert_array_equal(arrays["robot_time_s"], [4, 4.1, 4.25])
        np.testing.assert_array_equal(arrays["positions_rad"], np.array(self.rows)[:, 2:7])
        np.testing.assert_array_equal(arrays["tool_positions_m"], np.array(self.rows)[:, 7:])
        self.assertEqual(self.path.read_bytes(), before)

    def test_header_order_bom_and_spaces_do_not_change_joint_mapping(self):
        order = [9, 5, 1, 3, 8, 0, 6, 2, 7, 4]
        header = [" " + self.header[index] + " " for index in order]
        rows = [[row[index] for index in order] for row in self.rows]
        self.write(header, rows)
        self.path.write_bytes(b"\xef\xbb\xbf" + self.path.read_bytes())
        source_metadata, source_arrays = self.read()
        expected_names = [self.header[i] for i in order if self.header[i] in JOINT_NAMES]
        self.assertEqual(source_metadata["joint_names"], expected_names)
        for column, name in enumerate(expected_names):
            np.testing.assert_array_equal(
                source_arrays["positions_rad"][:, column],
                np.array(self.rows)[:, self.header.index(name)])
        metadata, arrays = self.reference()
        self.assertEqual(metadata["joint_names"], list(JOINT_NAMES))
        np.testing.assert_array_equal(arrays["positions_rad"][0, :-1], self.rows[0][2:7])
        np.testing.assert_array_equal(arrays["positions_rad"][:, -1], 0)

    def test_robot_timeline_nonzero_initial_pose_and_terminal_hold_survive_roundtrip(self):
        metadata, arrays = self.reference()
        self.assertEqual(metadata["initial_pose_policy"], "first_sample")
        self.assertEqual(metadata["code"], {"revision": "synthetic", "dirty": False})
        self.assertIn("source_csv", metadata)
        np.testing.assert_allclose(arrays["timestamps_s"], [0, .1, .2, .3], rtol=0, atol=1e-15)
        np.testing.assert_allclose(arrays["positions_rad"][:, 0], [.200, .201, .203, .204])
        np.testing.assert_array_equal(arrays["positions_rad"][0, :-1], self.rows[0][2:7])
        np.testing.assert_array_equal(arrays["positions_rad"][-1, :-1], self.rows[-1][2:7])
        self.screen(metadata, arrays)
        directory = self.directory / "reference"
        save_trajectory(directory, metadata, arrays)
        reloaded_metadata, reloaded_arrays = load_trajectory(directory)
        self.assertEqual(reloaded_metadata, metadata)
        for name, values in arrays.items():
            np.testing.assert_array_equal(reloaded_arrays[name], values)
        self.screen(reloaded_metadata, reloaded_arrays)
        with self.assertRaises(FileExistsError):
            save_trajectory(directory, metadata, arrays)

    def test_optional_tool_columns_and_explicit_gripper(self):
        header = ["source_time_s", "robot_time_s", *JOINT_NAMES]
        rows = [row[:7] + [0] for row in self.rows]
        self.write(header, rows)
        source_metadata, source_arrays = self.read()
        self.assertNotIn("tool_positions_m", source_arrays)
        self.assertEqual(source_metadata["joint_names"], list(JOINT_NAMES))
        metadata, arrays = self.reference()
        self.screen(metadata, arrays)
        rows[1][-1] = .01
        self.write(header, rows)
        with self.assertRaisesRegex(ValueError, "gripper"):
            metadata, arrays = self.reference()
            self.screen(metadata, arrays)

    def test_gen3_is_read_without_truncation_but_cannot_be_replayed_as_so101(self):
        names = [f"joint_{index}" for index in range(1, 8)]
        header = ["source_time_s", "robot_time_s", *names, "tool_x_m", "tool_y_m", "tool_z_m"]
        rows = [
            [0, 0, .1, .2, 3.2, -2.3, .5, 1.0, 1.57, .4, .1, .5],
            [.03, .3, .11, .21, 3.3, -2.2, .51, 1.01, 1.57, .41, .11, .51],
        ]
        self.write(header, rows)
        metadata, arrays = self.read()
        self.assertEqual(metadata["layout"], "gen3")
        self.assertEqual(metadata["joint_names"], names)
        np.testing.assert_array_equal(arrays["positions_rad"], np.array(rows)[:, 2:9])
        with self.assertRaisesRegex(ValueError, "(?i)(gen3|SO-101)"):
            csv_to_reference(metadata, arrays, self.model, self.config)

    def test_missing_duplicate_unknown_and_partial_headers_are_rejected(self):
        bad_headers = [
            self.header + ["shoulder_pan"],
            self.header + ["unexplained_column"],
            self.header + ["joint_1"],
            self.header[1:],
            self.header[:1] + self.header[2:],
            [name for name in self.header if name != "wrist_roll"],
            self.header[:-1],
            ["" if name == "shoulder_pan" else name for name in self.header],
        ]
        for header in bad_headers:
            with self.subTest(header=header):
                self.write(header, [[0] * len(header), [1] * len(header)])
                with self.assertRaises(ValueError):
                    self.read()

    def test_ragged_rows_and_insufficient_samples_are_rejected(self):
        bad_rows = [[], self.rows[:1], [self.rows[0], self.rows[1][:-1]],
                    [self.rows[0], self.rows[1] + [0]]]
        for rows in bad_rows:
            with self.subTest(rows=rows):
                self.write(rows=rows)
                with self.assertRaises(ValueError):
                    self.read()
        self.path.write_text("")
        with self.assertRaises(ValueError):
            self.read()

    def test_nonfinite_missing_and_nonnumeric_cells_are_rejected_in_every_column_group(self):
        for column in (0, 1, 2, 7):
            for value in ("nan", "inf", "-inf", "", "not-a-number", "True"):
                with self.subTest(column=column, value=value):
                    rows = copy.deepcopy(self.rows)
                    rows[1][column] = value
                    self.write(rows=rows)
                    with self.assertRaises(ValueError):
                        self.read()

    def test_each_clock_must_be_nonnegative_and_strictly_increasing(self):
        for column in (0, 1):
            for modification in ("negative", "duplicate", "decreasing"):
                with self.subTest(column=column, modification=modification):
                    rows = copy.deepcopy(self.rows)
                    if modification == "negative":
                        rows[0][column] = -1
                    elif modification == "duplicate":
                        rows[1][column] = rows[0][column]
                    else:
                        rows[1][column], rows[2][column] = rows[2][column], rows[1][column]
                    self.write(rows=rows)
                    with self.assertRaises(ValueError):
                        self.read()

    def test_row_and_resampling_limits_bound_allocations(self):
        with self.assertRaises(ValueError):
            read_csv_trajectory(self.path, max_rows=2)
        metadata, arrays = self.read()
        with self.assertRaises(ValueError):
            csv_to_reference(metadata, arrays, self.model, self.config, max_samples=3)
        rows = copy.deepcopy(self.rows)
        rows[-1][1] = 1e100
        self.write(rows=rows)
        metadata, arrays = self.read()
        with self.assertRaises(ValueError):
            csv_to_reference(metadata, arrays, self.model, self.config)

    def test_initial_pose_declaration_does_not_relax_joint_or_derivative_limits(self):
        metadata, arrays = self.reference()
        invalid = copy.deepcopy(arrays)
        invalid["positions_rad"][1, 0] = 2.1
        with self.assertRaisesRegex(ValueError, "positions"):
            self.screen(metadata, invalid)
        invalid = copy.deepcopy(arrays)
        invalid["positions_rad"][1, 0] += .1
        with self.assertRaisesRegex(ValueError, "velocity"):
            self.screen(metadata, invalid)
        invalid = copy.deepcopy(arrays)
        invalid["positions_rad"][1, 0] += .03
        with self.assertRaisesRegex(ValueError, "acceleration"):
            self.screen(metadata, invalid)

    def test_source_spike_between_control_samples_is_still_rejected(self):
        # A 60 ms-wide spike falls entirely between the 100 ms control samples.
        # Merely checking the resampled reference would erase this violation.
        rows = []
        for index, (time_s, shoulder) in enumerate(zip([0, .02, .05, .08, .2], [0, 0, .8, 0, 0])):
            rows.append([index * .01, time_s, shoulder, 0, 0, 0, 0, 0, 0, 0])
        self.write(rows=rows)
        metadata, arrays = self.read()
        with self.assertRaisesRegex(ValueError, "velocity"):
            validate_csv_source(metadata, arrays, self.model, self.config)
        rows[2][2] = 2.1
        self.write(rows=rows)
        metadata, arrays = self.read()
        with self.assertRaisesRegex(ValueError, "positions"):
            validate_csv_source(metadata, arrays, self.model, self.config)

    def test_source_limits_follow_names_and_model_velocity(self):
        source_metadata, source_arrays = self.read()
        validate_csv_source(source_metadata, source_arrays, self.model, self.config)
        self.model["joints"]["shoulder_pan"]["velocity"] = .001
        with self.assertRaisesRegex(ValueError, "velocity"):
            validate_csv_source(source_metadata, source_arrays, self.model, self.config)
        self.model["joints"]["shoulder_pan"]["velocity"] = 1.0
        self.model["joints"]["shoulder_pan"]["upper"] = .21
        with self.assertRaisesRegex(ValueError, "positions"):
            validate_csv_source(source_metadata, source_arrays, self.model, self.config)


if __name__ == "__main__":
    unittest.main()
