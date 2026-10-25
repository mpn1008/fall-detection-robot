"""Regress missing Isaac velocities and Stop/Play invalidation without Isaac/GPU."""

import math
import unittest

from drive_support import (
    CommandWatchdog,
    configure_drive,
    finite_vector,
    read_robot_sample,
    step_robot,
)


class FakeRobot:
    def __init__(self):
        self.handles_initialized = True
        self.initialization_succeeds = True
        self.initialize_calls = 0
        self.wheel_dof_indices = [0, 1]
        self.num_dof = 2
        self.control_mode = None
        self.actions = []
        self.position = [1.0, 2.0, 0.1]
        self.orientation = [1.0, 0.0, 0.0, 0.0]
        self.linear = [0.1, 0.0, 0.0]
        self.angular = [0.0, 0.0, 0.2]

    def initialize(self):
        self.initialize_calls += 1
        self.handles_initialized = self.initialization_succeeds

    def get_articulation_controller(self):
        return self

    def switch_control_mode(self, mode):
        self.control_mode = mode

    def apply_wheel_actions(self, action):
        if not self.handles_initialized:
            raise AssertionError("Attempted to control invalid physics handles")
        self.actions.append(action)

    def get_world_pose(self):
        return self.position, self.orientation

    def get_linear_velocity(self):
        return self.linear

    def get_angular_velocity(self):
        return self.angular


class FakeController:
    def __init__(self):
        self.reset_calls = 0

    def reset(self):
        self.reset_calls += 1

    def forward(self, command):
        return tuple(command)


class FakeWorld:
    def __init__(self):
        self.playing = True
        self.current_time = 2.0
        self.on_step = None
        self.reset_calls = 0

    def is_playing(self):
        return self.playing

    def step(self, render=True):
        callback, self.on_step = self.on_step, None
        if callback is not None:
            callback()
        if self.playing:
            self.current_time += 1.0 / 60.0

    def reset(self):
        self.reset_calls += 1
        self.playing = True
        self.current_time = 0.0


class DriveLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobot()
        self.world = FakeWorld()
        self.controller = FakeController()
        self.watchdog = CommandWatchdog()
        self.watchdog.receive(0.1, 0.0, 1.0, self.world.current_time)

    def step(self):
        return step_robot(self.world, self.robot, self.controller, self.watchdog, 1.1)

    def test_valid_handles_apply_fresh_command(self):
        self.assertTrue(self.step())
        self.assertEqual(self.robot.actions, [(0.1, 0.0)])
        self.assertEqual(self.world.reset_calls, 0)

    def test_configure_restores_velocity_control(self):
        self.assertEqual(configure_drive(self.robot, self.controller), (0, 1))
        self.assertEqual(self.robot.control_mode, "velocity")
        self.assertEqual(self.controller.reset_calls, 1)

    def test_failed_initialization_is_actionable(self):
        self.robot.initialization_succeeds = False
        with self.assertRaisesRegex(RuntimeError, "physics handles"):
            configure_drive(self.robot, self.controller)

    def test_invalid_wheel_indices_are_rejected(self):
        for indices in (None, [None, 1], [0, 0], [0], [-1, 1], [0, 2], [0.0, 1]):
            with self.subTest(indices=indices), self.assertRaisesRegex(RuntimeError, "indices"):
                self.robot.wheel_dof_indices = indices
                configure_drive(self.robot, self.controller)

    def test_pause_clears_motion_before_resume(self):
        self.world.playing = False
        self.assertFalse(self.step())
        self.assertEqual(self.robot.actions, [])
        self.world.playing = True
        self.assertTrue(self.step())
        self.assertEqual(self.robot.actions, [(0.0, 0.0)])

    def test_stop_during_gui_update_invalidates_state_without_crashing(self):
        def stop():
            self.world.playing = False
            self.robot.handles_initialized = False

        self.world.on_step = stop
        self.assertFalse(self.step())
        self.assertIsNone(read_robot_sample(self.robot))
        self.world.playing = True
        self.assertTrue(self.step())
        self.assertEqual(self.world.reset_calls, 1)
        self.assertEqual(self.robot.initialize_calls, 1)
        self.assertEqual(self.robot.control_mode, "velocity")
        self.assertEqual(self.robot.actions[-1], (0.0, 0.0))

    def test_rapid_stop_play_in_one_frame_never_replays_old_command(self):
        self.world.on_step = lambda: setattr(self.robot, "handles_initialized", False)
        self.assertFalse(self.step())
        self.assertTrue(self.step())
        self.assertTrue(self.step())
        self.assertEqual(self.robot.actions[-2:], [(0.0, 0.0), (0.0, 0.0)])


class OdometryStateTests(unittest.TestCase):
    def test_valid_sample_preserves_measurements(self):
        sample = read_robot_sample(FakeRobot())
        self.assertEqual(sample.position, (1.0, 2.0, 0.1))
        self.assertEqual(sample.linear_velocity, (0.1, 0.0, 0.0))
        self.assertEqual(sample.angular_velocity, (0.0, 0.0, 0.2))

    def test_missing_or_scalar_velocity_skips_instead_of_reaching_matmul(self):
        for field in ("linear", "angular"):
            for value in (None, 0.0, [], [0.0], [0.0, 0.0], [math.nan, 0.0, 0.0]):
                with self.subTest(field=field, value=value):
                    robot = FakeRobot()
                    setattr(robot, field, value)
                    self.assertIsNone(read_robot_sample(robot))

    def test_invalid_handles_skip_without_querying_physics(self):
        robot = FakeRobot()
        robot.handles_initialized = False
        robot.get_world_pose = lambda: self.fail("Queried invalid physics state")
        self.assertIsNone(read_robot_sample(robot))

    def test_handles_invalidated_during_read_are_rejected(self):
        robot = FakeRobot()

        def angular_velocity():
            robot.handles_initialized = False
            return [0.0, 0.0, 0.0]

        robot.get_angular_velocity = angular_velocity
        self.assertIsNone(read_robot_sample(robot))

    def test_invalid_pose_is_rejected(self):
        for field, value in (
            ("position", None),
            ("position", [math.inf, 0.0, 0.0]),
            ("orientation", None),
            ("orientation", [0.0, 0.0, 0.0, 0.0]),
        ):
            with self.subTest(field=field, value=value):
                robot = FakeRobot()
                setattr(robot, field, value)
                self.assertIsNone(read_robot_sample(robot))

    def test_quaternion_is_normalized(self):
        robot = FakeRobot()
        robot.orientation = [2.0, 0.0, 0.0, 0.0]
        self.assertEqual(read_robot_sample(robot).orientation, (1.0, 0.0, 0.0, 0.0))

    def test_vector_validation_rejects_scalars_and_wrong_dimensions(self):
        for value in (None, 0.0, [1.0, 2.0], [[1.0], [2.0], [3.0]], [math.inf, 0, 0]):
            with self.subTest(value=value):
                self.assertIsNone(finite_vector(value, 3))


if __name__ == "__main__":
    unittest.main()
