"""Check error reporting without requiring Isaac Sim or a GPU."""

import contextlib
import io
import unittest
from unittest.mock import patch

import turtlebot3_scene as scene


class SceneDiagnosticsTests(unittest.TestCase):
    def test_traceback_is_printed_before_fast_shutdown_and_status_is_preserved(self):
        stderr = io.StringIO()
        test = self

        class FastShutdownApp:
            def close(self, exit_code=0):
                # Model Kit terminating immediately: no later Python code runs.
                test.assertIn("Scene failed", stderr.getvalue())
                test.assertIn("RuntimeError: missing test asset", stderr.getvalue())
                raise SystemExit(exit_code)

        with contextlib.redirect_stderr(stderr):
            with patch.object(scene, "run", side_effect=RuntimeError("missing test asset")):
                with self.assertRaises(SystemExit) as shutdown:
                    scene.run_with_diagnostics(FastShutdownApp(), None)
        self.assertEqual(shutdown.exception.code, 1)

    def test_success_reports_zero(self):
        class App:
            def close(self, exit_code=0):
                self.exit_code = exit_code

        app = App()
        with patch.object(scene, "run"):
            self.assertEqual(scene.run_with_diagnostics(app, None), 0)
        self.assertEqual(app.exit_code, 0)

    def test_older_close_signature_still_returns_failure(self):
        class OlderApp:
            def close(self):
                self.closed = True

        app = OlderApp()
        with contextlib.redirect_stderr(io.StringIO()):
            with patch.object(scene, "run", side_effect=ImportError("test missing API")):
                self.assertEqual(scene.run_with_diagnostics(app, None), 1)
        self.assertTrue(app.closed)

    def test_interrupt_is_not_reported_as_a_scene_error(self):
        class App:
            def close(self, exit_code=0):
                self.exit_code = exit_code

        app = App()
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), patch.object(
            scene, "run", side_effect=KeyboardInterrupt
        ):
            self.assertEqual(scene.run_with_diagnostics(app, None), 130)
        self.assertEqual(app.exit_code, 130)
        self.assertEqual(stderr.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
