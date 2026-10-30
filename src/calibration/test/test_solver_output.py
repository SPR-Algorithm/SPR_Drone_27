"""Log-only output regressions; no sensor or solver-result fixtures."""

from contextlib import redirect_stdout
from io import StringIO
import unittest
from unittest.mock import patch

from spr_calibration.solver import stream_kalibr_output


class SolverOutputTests(unittest.TestCase):
    def test_progress_flood_preserves_all_raw_lines_and_bounds_console_output(self):
        lines = [f"  Progress {index} / 3000\n" for index in range(3000)]
        log, console = StringIO(), StringIO()
        with patch("spr_calibration.solver.monotonic", side_effect=[index / 1000 for index in range(3000)]), \
                redirect_stdout(console):
            stream_kalibr_output(lines, log)
        self.assertEqual(log.getvalue(), "".join(lines))
        # Three seconds of periodic progress plus the latest end snapshot.
        self.assertEqual(console.getvalue(), "".join(lines[index] for index in (0, 1000, 2000, 2999)))

    def test_latest_progress_precedes_immediate_warning_and_next_stage_first_line(self):
        lines = ["  Progress 0 / 10\n", "\tProgress 9 / 10\n", "WARNING: next stage\n",
                 " Progress 0 / 20\n", " Progress 19 / 20\n", "ERROR: optimizer stopped\n"]
        log, console = StringIO(), StringIO()

        def source():
            for line in lines:
                yield line
                if line.startswith(("WARNING", "ERROR")):
                    self.assertTrue(console.getvalue().endswith(line))

        with patch("spr_calibration.solver.monotonic", side_effect=[0.0, 0.1, 0.2, 0.3]), redirect_stdout(console):
            stream_kalibr_output(source(), log)
        self.assertEqual(log.getvalue(), "".join(lines))
        self.assertEqual(console.getvalue(), "".join(lines))

    def test_last_suppressed_progress_is_visible_at_end(self):
        lines = ["  Progress 0 / 2\n", "  Progress 2 / 2\n"]
        log, console = StringIO(), StringIO()
        with patch("spr_calibration.solver.monotonic", side_effect=[10.0, 10.01]), redirect_stdout(console):
            stream_kalibr_output(lines, log)
        self.assertEqual(log.getvalue(), "".join(lines))
        self.assertEqual(console.getvalue(), "".join(lines))

    def test_regular_lines_are_never_throttled_or_modified(self):
        lines = ["Loading measurements\n", "\n", "WARNING: Progress stalled\n", "ERROR: failure\n", "traceback without newline"]
        log, console = StringIO(), StringIO()
        with patch("spr_calibration.solver.monotonic") as clock, redirect_stdout(console):
            stream_kalibr_output(lines, log)
        clock.assert_not_called()
        self.assertEqual(log.getvalue(), "".join(lines))
        self.assertEqual(console.getvalue(), "".join(lines))

    def test_periodically_shown_final_progress_is_not_duplicated(self):
        lines = ["Progress 0 / 2\n", "Progress 2 / 2\n"]
        log, console = StringIO(), StringIO()
        with patch("spr_calibration.solver.monotonic", side_effect=[0.0, 1.0]), redirect_stdout(console):
            stream_kalibr_output(lines, log)
        self.assertEqual(console.getvalue(), "".join(lines))

    def test_interrupted_stream_keeps_received_log_and_latest_progress(self):
        lines = ["  Progress 0 / 2\n", "  Progress 1 / 2\n"]
        log, console = StringIO(), StringIO()

        def source():
            yield from lines
            raise RuntimeError("stream interrupted")

        with patch("spr_calibration.solver.monotonic", side_effect=[0.0, 0.1]), redirect_stdout(console):
            with self.assertRaisesRegex(RuntimeError, "stream interrupted"):
                stream_kalibr_output(source(), log)
        self.assertEqual(log.getvalue(), "".join(lines))
        self.assertEqual(console.getvalue(), "".join(lines))


if __name__ == "__main__":
    unittest.main()
