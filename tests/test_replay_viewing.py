"""Offline checks for viewer timing and CLI input; these do not test Isaac Sim."""

from contextlib import redirect_stderr
import importlib.util
import io
from pathlib import Path
import unittest
from unittest.mock import patch


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        if seconds < 0:
            raise AssertionError("Presentation must never request a negative sleep")
        self.sleeps.append(seconds)
        self.now += seconds


class ReplayViewingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / "scripts/control_so101.py"
        spec = importlib.util.spec_from_file_location("replay_viewing_script", path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_invalid_numeric_options_fail_before_loading_assets(self):
        cases = {
            "--start-delay": ("-1", "nan", "inf", "-inf"),
            "--playback-speed": ("0", "-1", "nan", "inf", "-inf"),
        }
        for option, values in cases.items():
            for value in values:
                with self.subTest(option=option, value=value):
                    stderr = io.StringIO()
                    with redirect_stderr(stderr), self.assertRaises(SystemExit) as error:
                        self.module.parse_args(["--trajectory", "unused", f"{option}={value}"])
                    self.assertEqual(error.exception.code, 2)
                    self.assertIn(f"{option} must be finite", stderr.getvalue())

    def test_expired_deadline_and_slow_frame_still_render_once_without_sleep(self):
        # Missing a presentation deadline must neither suppress the frame nor
        # attempt to catch up using a negative sleep.
        for initial_time, render_duration in ((2.0, 0.0), (0.0, 2.0)):
            with self.subTest(initial_time=initial_time, render_duration=render_duration):
                clock = FakeClock(initial_time)
                frames = []

                def render():
                    frames.append(clock.now)
                    clock.now += render_duration

                with patch.object(self.module, "time", clock):
                    running = self.module.render_until(lambda: True, render, 1.0)
                self.assertTrue(running)
                self.assertEqual(len(frames), 1)
                self.assertEqual(clock.sleeps, [])

    def test_wait_renders_during_delay_and_does_not_oversleep_deadline(self):
        clock = FakeClock()
        frames = []

        def render():
            frames.append(clock.now)
            clock.now += 0.002

        with patch.object(self.module, "time", clock):
            running = self.module.render_until(lambda: True, render, 0.05)
        self.assertTrue(running)
        self.assertGreater(len(frames), 1)
        self.assertTrue(all(0 < delay <= 1 / 60 for delay in clock.sleeps))
        # The last frame can exceed the deadline by its own rendering time.
        self.assertGreaterEqual(clock.now, 0.05)
        self.assertLessEqual(clock.now, 0.052 + 1e-12)

    def test_closed_viewer_never_renders(self):
        def unexpected_render():
            self.fail("A closed viewer must not be rendered")

        clock = FakeClock()
        with patch.object(self.module, "time", clock):
            self.assertFalse(self.module.render_until(lambda: False, unexpected_render, None))
        self.assertEqual(clock.sleeps, [])

    def test_close_during_frame_ends_both_timed_and_indefinite_waits(self):
        for deadline in (None, 10.0, 0.0):
            with self.subTest(deadline=deadline):
                clock = FakeClock()
                state = {"running": True, "frames": 0}

                def render():
                    state["frames"] += 1
                    state["running"] = False

                with patch.object(self.module, "time", clock):
                    running = self.module.render_until(lambda: state["running"], render, deadline)
                self.assertFalse(running)
                self.assertEqual(state["frames"], 1)
                self.assertLessEqual(clock.now, 1 / 60)


if __name__ == "__main__":
    unittest.main()
