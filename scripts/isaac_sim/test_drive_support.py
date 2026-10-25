"""Standard-library tests; also runnable without ROS or Isaac Sim installed."""

import math
import unittest

from drive_support import CommandWatchdog, robot_asset_candidates


class AssetCandidateTests(unittest.TestCase):
    def test_prefers_renamed_62_tutorial_asset(self):
        paths = robot_asset_candidates("https://example.com/Assets/Isaac/6.2/")
        self.assertEqual(
            paths[0],
            "https://example.com/Assets/Isaac/6.2/Isaac/Robots/"
            "Turtlebot/Turtlebot3/tb3_burger_processed.usda",
        )
        self.assertIn(
            "/Turtlebot/Turtlebot3/turtlebot3_burger/turtlebot3_burger.usda", paths[1]
        )
        self.assertTrue(paths[-1].endswith("/Turtlebot/turtlebot3_burger.usd"))

    def test_explicit_asset_never_falls_back_to_another_robot(self):
        self.assertEqual(
            robot_asset_candidates(None, "C:/robots/custom.usda"),
            ["C:/robots/custom.usda"],
        )

    def test_missing_asset_root_is_actionable(self):
        with self.assertRaisesRegex(ValueError, "--robot-usd"):
            robot_asset_candidates(None)


class WatchdogTests(unittest.TestCase):
    def test_initially_stopped(self):
        self.assertEqual(CommandWatchdog().command(1.0, 1.0), (0.0, 0.0))

    def test_fresh_command(self):
        watchdog = CommandWatchdog()
        self.assertTrue(watchdog.receive(0.1, -0.2, 1.0, 2.0))
        self.assertEqual(watchdog.command(1.1, 2.1), (0.1, -0.2))

    def test_each_clock_independently_expires_commands(self):
        for wall, sim in ((1.5, 2.1), (1.1, 2.5)):
            with self.subTest(wall=wall, sim=sim):
                watchdog = CommandWatchdog()
                watchdog.receive(0.1, 0.0, 1.0, 2.0)
                self.assertEqual(watchdog.command(wall, sim), (0.0, 0.0))
                self.assertEqual(watchdog.command(1.1, 2.1), (0.0, 0.0))

    def test_clock_reset_and_pause_clear_commands(self):
        watchdog = CommandWatchdog()
        watchdog.receive(0.1, 0.0, 1.0, 2.0)
        self.assertEqual(watchdog.command(1.1, 0.0), (0.0, 0.0))
        watchdog.receive(0.1, 0.0, 1.0, 2.0)
        watchdog.clear()
        self.assertEqual(watchdog.command(1.1, 2.1), (0.0, 0.0))

    def test_invalid_commands_stop_instead_of_preserving_last_motion(self):
        for linear, angular in ((math.nan, 0.0), (0.0, math.inf), (0.23, 0.0), (0.0, -1.1)):
            with self.subTest(linear=linear, angular=angular):
                watchdog = CommandWatchdog()
                watchdog.receive(0.1, 0.0, 1.0, 2.0)
                self.assertFalse(watchdog.receive(linear, angular, 1.1, 2.1))
                self.assertEqual(watchdog.command(1.2, 2.2), (0.0, 0.0))

    def test_invalid_timeouts(self):
        for timeout in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                CommandWatchdog(timeout)


if __name__ == "__main__":
    unittest.main()
